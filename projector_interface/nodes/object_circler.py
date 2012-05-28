#!/usr/bin/env python
import roslib; roslib.load_manifest('projector_interface')
from projector_calibration.msg import Homography
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import CameraInfo, PointCloud2
from std_msgs.msg import Empty
from geometry_msgs.msg import PointStamped
import image_geometry
from projector_interface._point_cloud import read_points_np
from pr2_python.pointclouds import xyz_array_to_pointcloud2
import projector_interface.srv
import tf

import cv2
import numpy as np
from collections import deque
from threading import RLock
import sys

import PySide
from PySide import QtGui, QtCore
from PySide.QtGui import QPalette

X_OFFSET =  25
Y_OFFSET = -25

CURSOR_OFFSET = np.array([0,25])

SELECT_DIST_THRESH = 50 #ugh, this is in pixels
SAME_OBJ_THRESH    = 0.03

class Colors:
    WHITE = QtGui.QColor(255,255,255)
    GREEN = QtGui.QColor(  0,255,0  )
    BLUE  = QtGui.QColor(  0,135,189)

class Circler(QtGui.QWidget):
    H = None
    objects = None  
    projected_objects = None
    int_objects = None  
    int_projected_objects = None
    int_ages = []
    click = rospy.Time(0)
    click_loc = np.float64([-1,-1,-1])
    click_duration = rospy.Duration(2.0)
    
    selected_pt = np.array([])
    
    use_selected_thresh = True
    
    cursor_pts = None
    projected_cursor = deque([], rospy.get_param('~/window_size', 10))
    cursor_pts_xyz = deque([], rospy.get_param('~/window_size', 10))
    
    model = None
    object_header = None
    cursor_header = None

    object_lock = RLock()
    intersected_lock = RLock()
    cursor_lock = RLock()
    
    hilights = []
    polygons = dict()
    
    def escHandler(self, e):
        sys.exit(0)

    def initUI(self):
        #self.addKeyHandler(16777216, self.escHandler)
        p = QPalette()
        p.setColor(QPalette.Background, QtGui.QColor(0,0,0))
        self.setPalette(p)
        self.showFullScreen()

    def info_cb(self, info):
        tmp_model = image_geometry.PinholeCameraModel()
        tmp_model.fromCameraInfo(info)
        self.model = tmp_model
        
    def click_cb(self, msg):
        self.click = rospy.Time.now()
        # self.sameObject(pt, self.click_loc)
        
        with self.cursor_lock:
            self.click_loc = self.selected_pt
            msg = xyz_array_to_pointcloud2(np.array(self.cursor_pts_xyz))
            msg.header.frame_id = '/table'
            msg.header.stamp = rospy.Time.now()
            self.click_stats_pub.publish(msg)
        
    def object_cb(self, msg):
        with self.object_lock:
            objects = read_points_np(msg, masked=False)
            if objects.shape[1] == 0: return
            
            self.object_header = msg.header
            self.objects = objects
            transformed_objects = self.projectPoints(self.objects, msg.header)
            self.projected_objects = cv2.perspectiveTransform(transformed_objects, self.H)
        
    def intersected_cb(self, msg):
        with self.intersected_lock:
            objects = read_points_np(msg, masked=False)
            if objects.shape[1] == 0: return

            self.int_object_header = msg.header
            self.int_objects = objects
            transformed_objects = self.projectPoints(objects, msg.header)
            self.int_projected_objects = cv2.perspectiveTransform(transformed_objects, self.H)
            for pt in transformed_objects[0]:
                self.int_ages.append(rospy.Time.now())

    def cursor_cb(self, msg):
        with self.cursor_lock:
            objects = read_points_np(msg, masked=False)
            if objects.shape[1] == 0: return
            
            self.cursor_header = msg.header
            self.cursor_pts = objects
            transformed_pts = self.projectPoints(self.cursor_pts, msg.header)
            #self.cursor_pts_xyz.extend(self.cursor_pts.tolist()[0])
            self.projected_cursor.extend(cv2.perspectiveTransform(transformed_pts, self.H)[0])

    def projectPoints(self, points, point_header):
        pts_out = []
        for point in points[0]:
            pt = PointStamped()
            pt.point.x, pt.point.y, pt.point.z = point.tolist()
            stamp = self.tfl.getLatestCommonTime(self.model.tf_frame, point_header.frame_id)
            pt.header = point_header
            pt.header.stamp = stamp
            # first, transform the point into the camera frame
            pt_out = self.tfl.transformPoint(self.model.tf_frame, pt).point
            # project it onto the image plane
            px = self.model.project3dToPixel((pt_out.x, pt_out.y, pt_out.z))
            pts_out.append(px)
        return np.array([pts_out])
    
    def isSelected(self, pt):
        with self.intersected_lock:
            if self.int_objects is not None and self.int_projected_objects is not None:
                for xformed in self.int_projected_objects[0]:
                    if np.sqrt(((pt-xformed)**2).sum()) < SELECT_DIST_THRESH: return True
        return False
            
    def isHilighted(self, pt):
        with self.intersected_lock:
            for hi in self.hilights:
                if self.sameObject(pt, hi[0]): return True
        return False
        
    def getHilightColor(self, pt):
        with self.intersected_lock:
            for hi in self.hilights:
                if self.sameObject(pt, hi[0]):
                    return hi[1]
    
    def dist(self, p1, p2):
        p1 = np.array(p1)
        return np.sum(np.sqrt((p1-p2)**2))
    
    def sameObject(self, p1, p2):
        return self.dist(p1,p2) < SAME_OBJ_THRESH
    
    def paintEvent(self, e):
        if rospy.is_shutdown(): sys.exit()
        # circle the objects
        r = 100
        
        with self.object_lock:
            if self.objects is not None and self.projected_objects is not None:
                # is dist(cursor,pt) <= dist(cursor,obj) for all obj?
                cursor = np.median(self.projected_cursor, 0)
                distances = [self.dist(pp,cursor) for pp in self.projected_objects[0]]
                closest_pt = self.projected_objects[0,np.argmin(distances)]
                
                for pt, xformed in zip(self.objects[0], self.projected_objects[0]):
                    qp = QtGui.QPainter()
                    qp.begin(self)
                    qp.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing);
                    color = Colors.WHITE
                    if self.isHilighted(pt):
                        # color = self.getHilightColor(pt)
                        inner_pen = qp.pen()
                        inner_pen.setWidth(6)
                        inner_pen.setColor(self.getHilightColor(pt))
                        qp.setPen(inner_pen)
                        inner_rect = QtCore.QRectF(800-xformed[1]-r/2 + X_OFFSET + 5, 600-xformed[0]-r/2 + Y_OFFSET + 5, r-10, r-10)
                        qp.drawArc(inner_rect, 0, 360*16)
                        
                    if (self.use_selected_thresh and self.isSelected(xformed)) or\
                       (not self.use_selected_thresh and all(xformed==closest_pt)):
                        self.selected_pt = pt
                        color = Colors.GREEN
                            
                            # if not self.sameObject(pt, self.click_loc):
                            #     sel_pt = PointStamped()
                            #     sel_pt.header.stamp = rospy.Time.now()
                            #     sel_pt.header.frame_id = self.object_header.frame_id
                            #     sel_pt.point.x, sel_pt.point.y, sel_pt.point.z = pt.tolist()
                            #     self.selected_pub.publish(sel_pt)
                            # self.click_loc = pt
                            
                    # is it the clicked point?
                    # if ((rospy.Time.now() - self.click) < self.click_duration) and\
                    #    (self.sameObject(pt, self.click_loc)):
                    if self.sameObject(pt, self.click_loc):
                        color = Colors.BLUE
                        # inner_pen = qp.pen()
                        # inner_pen.setWidth(6)
                        # inner_pen.setColor(Colors.BLUE)
                        # qp.setPen(inner_pen)
                        # inner_rect = QtCore.QRectF(800-xformed[1]-r/2 + X_OFFSET + 5, 600-xformed[0]-r/2 + Y_OFFSET + 5, r-10, r-10)
                        # qp.drawArc(inner_rect, 0, 360*16)
                    
                    
                    # elif ((rospy.Time.now() - self.click) < self.click_duration) and self.sameObject(pt, self.click_loc):
                    #     # color = Colors.BLUE
                    #     inner_pen = qp.pen()
                    #     inner_pen.setWidth(5)
                    #     inner_pen.setColor(Colors.BLUE)
                    #     qp.setPen(inner_pen)
                    #     inner_rect = QtCore.QRectF(800-xformed[1]-r/2 + X_OFFSET, 600-xformed[0]-r/2 + Y_OFFSET, r-5, r-5)
                    #     qp.drawArc(inner_rect, 0, 360*16)
                    #     self.click_loc = pt # maybe this shouldn't be here
                    qp.setPen(color)
                    pen = qp.pen()
                    pen.setWidth(5)
                    qp.setPen(pen)
                    rect = QtCore.QRectF(800-xformed[1]-r/2 + X_OFFSET, 600-xformed[0]-r/2 + Y_OFFSET, r, r)
                    #rect = QtCore.QRectF(800-xformed[1]-r/2 + 25, 600-xformed[0]-r/2, r, r)
                    #rect = QtCore.QRectF(xformed[1]-r/2 + 25, xformed[0]-r/2, r, r)
                    qp.drawArc(rect, 0, 360*16)
                    qp.end()
                   
            # draw the cursor 
            r = 10
            with self.object_lock:
                if self.cursor_pts is not None and len(self.projected_cursor) > 0:
                    xformed = np.median(self.projected_cursor, 0)
                    qp = QtGui.QPainter()
                    qp.begin(self)
                    qp.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing);
                    color = Colors.BLUE
                    qp.setPen(color)
                    pen = qp.pen()
                    pen.setWidth(5)
                    qp.setPen(pen)
                    rect = QtCore.QRectF(800-xformed[1]-r/2 + X_OFFSET, 600-xformed[0]-r/2 + Y_OFFSET, r, r)
                    qp.drawArc(rect, 0, 360*16)
                    
                    #rect = QtCore.QRectF(800-xformed[1]-r/2 + 25, 600-xformed[0]-r/2, r, r)
                    #rect = QtCore.QRectF(xformed[1]-r/2 + 25, xformed[0]-r/2, r, r)
                    
                    qp.end()
                    
            # draw any polygons
            
            qp = QtGui.QPainter()
            qp.begin(self)
            qp.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing);
            color = Colors.WHITE
            qp.setPen(color)
            pen = qp.pen()
            pen.setWidth(5)
            qp.setPen(pen)
            
            
            for name, polygon in self.polygons.iteritems():
                poly = PySide.QtGui.QPolygon()
                points = self.projectPoints(np.array([[(p.x,p.y,p.z) for p in polygon.polygon.points]]), polygon.header)
                points = cv2.perspectiveTransform(points, self.H)
                
                points[0] = ([600, 800] - points[0]) + [Y_OFFSET, X_OFFSET]
                
                for point in points[0]:
                    poly.push_back(PySide.QtCore.QPoint(point[1], point[0]))
                    # qp.drawPoint(point[1], point[0])
            
                # print points[0]
                #             
                #             
                qp.drawPolygon(poly)
                origin = points[0].min(0)
                size = points[0].max(0) - origin
                textRect = QtCore.QRectF(origin[1],origin[0],size[1],size[0])
                qp.drawText(textRect, QtCore.Qt.AlignCenter, name)
            
        # reset once the click has expired
        if (rospy.Time.now() - self.click) >= self.click_duration:
            self.click_loc = np.float64([[-1,-1,-1]])
            self.click_duration = rospy.Duration(2.0)
                 
    def handle_hilight(self, req):
        pt = self.tfl.transformPoint(self.object_header.frame_id, req.object).point
        color = QtGui.QColor(req.color.r,req.color.g,req.color.b)
        self.hilights.append(([pt.x,pt.y,pt.z], color))
        return projector_interface.srv.HilightObjectResponse()
        
    def handle_clear_hilight(self, req):
        self.hilights = []
        return projector_interface.srv.ClearHilightsResponse()

    def handle_get_cursor_stats(self, req):
        while not (rospy.Time.now() - self.click) < self.click_duration:
            rospy.sleep(0.05)
        with self.cursor_lock:
            msg = xyz_array_to_pointcloud2(np.array([list(p) + [0] for p in self.projected_cursor]))
            msg.header.frame_id = '/table'
            msg.header.stamp = rospy.Time.now()
            
            pt_msg = PointStamped()
            pt_msg.header = msg.header
            pt_msg.point.x, pt_msg.point.y, pt_msg.point.z = self.click_loc
            return projector_interface.srv.GetCursorStatsResponse(msg, pt_msg)
        
    def set_selection_method(self, req):
        if req.method == req.THRESH:
            self.use_selected_thresh = True
        elif req.method == req.CLOSEST:
            self.use_selected_thresh = False
        return projector_interface.srv.SetSelectionMethodResponse()

    def handle_draw_polygon(self, req):
        self.polygons[req.name] = req.polygon
        return projector_interface.srv.DrawPolygonResponse()

    def __init__(self):
        super(Circler, self).__init__()
        self.tfl = tf.TransformListener()
        r = rospy.Rate(10)
        rospy.loginfo('waiting for homography...')
        while (not rospy.has_param('/homography')) and (not rospy.is_shutdown()):
            r.sleep()
        self.H = np.float64(rospy.get_param('/homography')).reshape(3,3)
        rospy.loginfo('got homography')
        print self.H
        self.initUI()
        rospy.Subscriber('object_cloud', PointCloud2, self.object_cb)
        rospy.Subscriber('click', Empty, self.click_cb)
        rospy.Subscriber('intersected_points', PointCloud2, self.intersected_cb)
        rospy.Subscriber('intersected_points_cursor', PointCloud2, self.cursor_cb)
        rospy.Subscriber('camera_info', CameraInfo, self.info_cb)
        rospy.Service('hilight_object', projector_interface.srv.HilightObject, self.handle_hilight)
        rospy.Service('clear_hilights', projector_interface.srv.ClearHilights, self.handle_clear_hilight)
        rospy.Service('get_cursor_stats', projector_interface.srv.GetCursorStats, self.handle_get_cursor_stats)
        rospy.Service('set_selection_method', projector_interface.srv.SetSelectionMethod, self.set_selection_method)
        rospy.Service('draw_polygon', projector_interface.srv.DrawPolygon, self.handle_draw_polygon)
        self.selected_pub = rospy.Publisher('selected_point', PointStamped)
        self.click_stats_pub = rospy.Publisher('click_stats', PointCloud2)
        
        # testing
        # from geometry_msgs.msg import PolygonStamped, Point
        # polygon = PolygonStamped()
        # polygon.header.stamp = rospy.Time.now()
        # polygon.header.frame_id = 'table'
        # polygon.polygon.points.append(Point(0.0,0.0,0.0))
        # polygon.polygon.points.append(Point(0.2,0.0,0.0))
        # polygon.polygon.points.append(Point(0.2,0.2,0.0))
        # polygon.polygon.points.append(Point(0.0,0.2,0.0))
        # self.polygons['TEST'] = polygon
        
        timer = PySide.QtCore.QTimer(self)
        timer.setInterval(100)
        timer.timeout.connect(self.update)
        timer.start()
        rospy.loginfo('interface started')
        sys.exit(app.exec_())
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('object_circler')
    print 'Window size is %s' % rospy.get_param('~window_size', 1)
    app = PySide.QtGui.QApplication(sys.argv)
    c = Circler()
