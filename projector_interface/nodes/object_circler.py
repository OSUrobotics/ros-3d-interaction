#!/usr/bin/env python
# Copyright (c) 2013, Oregon State University
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Oregon State University nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL OREGON STATE UNIVERSITY BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author Dan Lazewatsky/lazewatd@engr.orst.edu

import roslib; roslib.load_manifest('projector_interface')
import rospy
from sensor_msgs.msg import CameraInfo, PointCloud2
from std_msgs.msg import Empty, String as String, Duration
from geometry_msgs.msg import PointStamped
import image_geometry
from projector_interface._point_cloud import read_points_np
from projector_interface.treeable import TreeCircleInfo, GraphicsItemInfo
from pr2_python.pointclouds import xyz_array_to_pointcloud2
from projector_interface import srv
import tf

import cv2
import numpy as np
from collections import deque
from threading import RLock
import sys


# import PySide
from PySide import QtGui, QtCore

from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from projector_interface.cfg import InterfaceConfig

import kdtree

from functools import partial

#X_OFFSET =  25
#Y_OFFSET = -25

X_OFFSET = 0
Y_OFFSET = 0

CURSOR_RADIUS = 10

SAME_OBJ_THRESH    = 0.03

class Colors:
    WHITE = QtGui.QColor(255, 255, 255)
    GREEN = QtGui.QColor(  0, 255, 0  )
    BLUE  = QtGui.QColor(  0, 135, 189)

class Circler(QtGui.QGraphicsView):
    H = None
    POLYGON_PEN = QtGui.QPen(Colors.WHITE, 5)
    ACTIVE_POLYGON_PEN = QtGui.QPen(Colors.GREEN, 5)
    CLICKED_OBJECT_PEN = QtGui.QPen(Colors.BLUE, 5)
    CIRCLE_RADIUS = 150

    objects = None  
    projected_objects = None
    int_objects = None  
    int_projected_objects = None
    click_loc = np.float64([-1, -1, -1])
    click_duration = rospy.Duration(1.0)
    
    selected_pt = np.array([])
    
    use_selected_thresh = True
    
    cursor_pts = None
    
    model = None
    object_header = None
    cursor_header = None

    object_lock = RLock()
    intersected_lock = RLock()
    cursor_lock = RLock()
    polygon_lock = RLock()
    
    polygons = dict()

    key_handlers = dict()

    click_stale = False
    config_inited = False

    click = QtCore.Signal()
    polygonAdded = QtCore.Signal(srv.DrawPolygonRequest)
    polygonsCleared = QtCore.Signal()
    cursorMoved = QtCore.Signal(PointCloud2)
    objectsChanged = QtCore.Signal(PointCloud2)
    objectHighlighted = QtCore.Signal(srv.HilightObject)
    hilightsCleared = QtCore.Signal()

    def keyPressEvent(self, e):
        for key, fn in self.key_handlers.items():
            if key == e.key():
                fn(e)
        
    def addKeyHandler(self, key, fn):
        self.key_handlers[key] = fn

    def escHandler(self, _):
        QtGui.QApplication.quit()

    def initUI(self):
        self.resetClickedObject()
        self.resetActiveObject()
        self.active_poly   = GraphicsItemInfo(QtGui.QGraphicsPolygonItem(), -1, label='\x00')
        self.active_object = TreeCircleInfo(QtCore.QRect(), self.POLYGON_PEN, (0, 0, 0))
        gfx_scene = QtGui.QGraphicsScene() 
        gfx_scene.setBackgroundBrush(QtGui.QColor(0, 0, 0))
        self.last_click_time = rospy.Time(0)

        # setup the cursor pen
        pen = QtGui.QPen(Colors.BLUE, 5)

        # add an ellipse to the scene
        cursor_rect = QtCore.QRect(self.width(), self.height(), CURSOR_RADIUS, CURSOR_RADIUS)
        self.obj_cursor = gfx_scene.addEllipse(cursor_rect, pen)   
        self.obj_cursor.setCacheMode(QtGui.QGraphicsItem.CacheMode.NoCache)
        self.obj_cursor.setZValue(2000)
        self.last_cursor_rect = self.obj_cursor.boundingRect()

        # add a line for the offscreen cursor hint
        self.obj_cursor_hint = gfx_scene.addLine(0, 0, 0, 0, pen=pen)
        self.obj_cursor_hint.hide()
        self.obj_cursor_hint.setZValue(2000)

        self.setScene(gfx_scene)

        self.setRenderHints(QtGui.QPainter.RenderHint.Antialiasing)

        self.addKeyHandler(16777216, self.escHandler)

        self.showFullScreen()

        # Manually set the scene rect to prevent the view
        # from scrolling to fit off-screen objects
        self.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignTop)
        self.setSceneRect(0, 0, self.width(), self.height())

        self.circles = kdtree.create(dimensions=3)

    def resetClick(self, obj):
        obj.item.setPen(self.POLYGON_PEN)
        obj.item.setZValue(0)
        obj.clicked = False
        self.scene().invalidate(obj.item.boundingRect())
        self.resetClickedObject()

    def resetClickedObject(self):
        self.clicked_object = GraphicsItemInfo(QtGui.QGraphicsPolygonItem(), -1, label='\x00')

    def resetActiveObject(self):
        self.active_poly = GraphicsItemInfo(QtGui.QGraphicsPolygonItem(), -1, label='\x00')

    def info_cb(self, info):
        tmp_model = image_geometry.PinholeCameraModel()
        tmp_model.fromCameraInfo(info)
        self.model = tmp_model 
        self.info_sub.unregister()
    

    def handleClick(self):
        # Make sure the click was on an object, and that nothing is already clicked
        self.last_click_time = rospy.Time.now()

        # Polygons
        if (self.active_poly.label != '\x00') and (self.clicked_object.label == '\x00'):
            self.clicked_object = self.active_poly
            self.clicked_object.item.setPen(self.CLICKED_OBJECT_PEN)
            self.clicked_object.item.setZValue(1000)
            self.clicked_object.clicked = True
            self.clicked_object_pub.publish(self.clicked_object.uid)
            self.scene().invalidate(self.clicked_object.item.boundingRect())
            QtCore.QTimer.singleShot(
                1000*self.click_duration.to_sec(),
                partial(self.resetClick, self.clicked_object)
            )

        # Circles
        if self.active_object.active:
            self.clicked_object = self.active_object
            self.clicked_object.item.setPen(self.CLICKED_OBJECT_PEN)
            self.clicked_object.item.setZValue(1000)
            self.clicked_object.clicked = True
            self.clicked_object_pub.publish(self.clicked_object.uid)
            self.scene().invalidate(self.clicked_object.item.boundingRect())
            QtCore.QTimer.singleShot(
                1000*self.click_duration.to_sec(),
                partial(self.resetClick, self.clicked_object)
            )

    def click_cb(self, msg):
        self.click_stale = False
        self.click.emit()

        with self.cursor_lock:
            self.click_loc = self.selected_pt
            msg = xyz_array_to_pointcloud2(np.array(self.cursor_pts_xyz))
            msg.header.frame_id = self.model.tf_frame
            msg.header.stamp = rospy.Time.now()
            self.click_stats_pub.publish(msg)
        
    def object_cb(self, msg):
        self.objectsChanged.emit(msg)

    def updateObjects(self, obj_msg):
        with self.object_lock:
            for obj in self.circles.inorder():
                self.scene().removeItem(obj.data)
                self.scene().invalidate(obj.data.boundingRect())
            self.circles = kdtree.create(dimensions=3)

            objects = read_points_np(obj_msg, masked=False)
            if objects.shape[1] == 0:
                return
            
            self.object_header = obj_msg.header
            self.objects = objects
            transformed_objects = self.projectPoints(self.objects, obj_msg.header)
            self.projected_objects = cv2.perspectiveTransform(transformed_objects, self.H)
            for pt, xformed in zip(self.objects[0], self.projected_objects[0]):
                coords = self.maybe_flip((xformed[1], xformed[0]))
                rect = QtCore.QRectF(
                    coords[0]-self.CIRCLE_RADIUS/2 + X_OFFSET,
                    coords[1]-self.CIRCLE_RADIUS/2 + Y_OFFSET,
                    self.CIRCLE_RADIUS,
                    self.CIRCLE_RADIUS
                )
                circle = TreeCircleInfo(rect, self.POLYGON_PEN, pt)
                if self.circles:
                    nearest = self.circles.search_nn(circle)
                    if np.sqrt(nearest.dist(circle)) > SAME_OBJ_THRESH:                    
                        self.circles.add(circle)
                        self.scene().addItem(circle)
                        self.scene().invalidate(rect)
                else:
                    self.circles.add(circle)
                    self.scene().addItem(circle)
                    self.scene().invalidate(rect)

    def intersected_cb(self, msg):
        with self.intersected_lock:
            objects = read_points_np(msg, masked=False)
            if objects.shape[1] == 0:
                return

            self.int_objects = objects
            transformed_objects = self.projectPoints(objects, msg.header)
            self.int_projected_objects = cv2.perspectiveTransform(transformed_objects, self.H)

    def cursor_cb(self, msg):
        self.cursorMoved.emit(msg)

    def hideCursor(self):
        self.obj_cursor.hide()
        self.scene().invalidate(self.obj_cursor.boundingRect())

    def showCursor(self):
        self.obj_cursor.show()
        self.scene().invalidate(self.obj_cursor.boundingRect())

    def hideCursorHint(self):
        self.obj_cursor_hint.hide()
        self.scene().invalidate(self.obj_cursor_hint.boundingRect())

    def showCursorHint(self):
        self.obj_cursor_hint.show()
        self.scene().invalidate(self.obj_cursor_hint.boundingRect())

    def update_cursor(self, cursor_msg):
        with self.cursor_lock:
            objects = read_points_np(cursor_msg, masked=False)
            if objects.shape[1] == 0:
                return
            
            self.cursor_header = cursor_msg.header
            self.cursor_pts = objects
            transformed_pts = self.projectPoints(self.cursor_pts, cursor_msg.header)
            self.projected_cursor.extend(cv2.perspectiveTransform(transformed_pts, self.H)[0])


        with self.cursor_lock:
            xformed = np.median(self.projected_cursor, 0)
            coords = self.maybe_flip((xformed[1], xformed[0]))
            cursor_x = coords[0]
            cursor_y = coords[1]

            # if the cursor is on-screen, draw it
            if    cursor_x > 0 and cursor_y > 0 \
              and cursor_x < self.width()       \
              and cursor_y < self.height():

                if self.obj_cursor_hint.isVisible():
                    self.hideCursorHint()

                cursor_rect = QtCore.QRectF(
                    cursor_x-CURSOR_RADIUS/2,# + X_OFFSET,
                    cursor_y-CURSOR_RADIUS/2,# + Y_OFFSET,
                    CURSOR_RADIUS,
                    CURSOR_RADIUS
                )
                self.showCursor()

                self.obj_cursor.setRect(cursor_rect)
                self.scene().invalidate(cursor_rect)
                self.scene().invalidate(self.last_cursor_rect)
                self.last_cursor_rect = self.obj_cursor.boundingRect()
            # otherwise, draw a hint
            else:
                if self.obj_cursor.isVisible:
                    self.hideCursor()

                hint_x = cursor_x
                if cursor_x < 0:
                    hint_x = 0
                if cursor_x > self.width():
                    hint_x = self.width()

                hint_y = cursor_y
                if cursor_y < 0:
                    hint_y = 0
                if cursor_y > self.height():
                    hint_y = self.height()

                head = np.array([hint_x, hint_y])# + [X_OFFSET, Y_OFFSET]
                tail_x, tail_y = 0, 0
                if hint_x == 0:
                    tail_x =  50
                if hint_x == self.width():
                    tail_x = -50
                if hint_y == 0:
                    tail_y =  50
                if hint_y == self.height():
                    tail_y = -50
                tail = head + [tail_x, tail_y]
                self.scene().invalidate(self.obj_cursor_hint.boundingRect())
                self.obj_cursor_hint.setLine(head[0], head[1], tail[0], tail[1])
                self.showCursorHint()

        self.updateIntersectedPolys()
        self.updateIntersectedCircles() 


    # TODO this may need a more efficient implementaion that doesn't look at all polygons
    def updateIntersectedPolys(self):
        with self.polygon_lock:
            with self.cursor_lock:
                cursor_center = self.last_cursor_rect.center()
            self.resetActiveObject()
            for info in self.polygons.values():
                if not info.clicked:
                    dirty = False
                    if info.item.polygon().containsPoint(cursor_center, QtCore.Qt.OddEvenFill):
                        dirty = not info.active 
                        info.active = True
                        info.item.setPen(self.ACTIVE_POLYGON_PEN)
                        info.item.setZValue(500)
                        self.active_poly = info
                    else:
                        dirty = info.active
                        info.active = False
                        info.item.setPen(self.POLYGON_PEN)
                        info.item.setZValue(0)
                        self.resetClickedObject()
                    if dirty:
                        self.scene().invalidate(info.item.boundingRect())

    def updateIntersectedCircles(self):
        if self.circles:
            # don't try to reset a clicked object
            if not self.active_object.clicked:
                self.active_object.active = False
                self.active_object.item.setPen(self.POLYGON_PEN)

            self.scene().invalidate(self.active_object.item.boundingRect())
            self.resetActiveObject()
            with self.object_lock:
                with self.cursor_lock:
                    cursor_center = self.last_cursor_rect.center()
                    cursor_center_3d = np.median(self.cursor_pts, 0).flatten().tolist()
                    nearest = self.circles.search_nn(cursor_center_3d)

                    # if we're requiring that the cursor be within the halo, do that check
                    if self.use_selected_thresh:
                        nearest = nearest if nearest.data.contains(cursor_center) else None

                    dirty = False 
                    # if the object has been clicked, don't touch it here
                    if nearest and not nearest.data.clicked: 
                        item = nearest.data
                        dirty = not item.active
                        item.active = True
                        item.setPen(self.ACTIVE_POLYGON_PEN)
                        item.setZValue(500)
                        self.active_object = item # TODO is this right?
                    if dirty:
                        self.scene().invalidate(item.boundingRect())


    def projectPoints(self, points, point_header):
        pts_out = []
        for point in points[0]:
            pt = PointStamped()
            try:
                pt.point.x, pt.point.y, pt.point.z = point.tolist()
            except Exception:
                print 'Error: ', point
            # first, transform the point into the camera frame
            if point_header.frame_id != self.model.tf_frame:
                stamp = rospy.Time(0)
                pt.header = point_header
                pt.header.stamp = stamp
                pt_out = self.tfl.transformPoint(self.model.tf_frame, pt).point
            else:
                pt_out = pt.point
            # project it onto the image plane
            px = self.model.project3dToPixel((pt_out.x, pt_out.y, pt_out.z))
            pts_out.append(px)
        return np.array([pts_out])
        
    def maybe_flip(self, coords):
        if self.flip:
            return self.width() - coords[0], self.height() - coords[1]
        return coords 
                 
    def hilight_object(self, req):
        pt = self.tfl.transformPoint(self.object_header.frame_id, req.object).point
        color = QtGui.QColor(req.color.r, req.color.g, req.color.b)
        if self.circles:
            pt_arr = [pt.x, pt.y, pt.z]
            to_hilight = self.circles.search_nn([pt.x, pt.y, pt.z])
            # np.spacing(1) is equivalent to eps in matlab
            if to_hilight.dist(pt_arr) < np.spacing(1):
                to_hilight.data.showHilight(color)

    def clear_hilights(self):
        for obj in self.circles.inorder():
            obj.data.clearHilight()

    def handle_hilight(self, req):
        self.objectHighlighted.emit(req)
        return srv.HilightObjectResponse()
        
    def handle_clear_hilight(self, _):
        self.hilightsCleared.emit()
        return srv.ClearHilightsResponse()

    def handle_get_cursor_stats(self, _):
        # wait until the click expires
        while not (rospy.Time.now() - self.last_click_time) < self.click_duration:
            rospy.sleep(0.05)
        with self.cursor_lock:
            msg = xyz_array_to_pointcloud2(np.array([list(p) + [0] for p in self.projected_cursor]))
            msg.header.frame_id = '/table_marker'
            msg.header.stamp = rospy.Time.now()
            
            pt_msg = PointStamped()
            pt_msg.header = msg.header
            pt_msg.point.x, pt_msg.point.y, pt_msg.point.z = self.clicked_object.point3d
            return srv.GetCursorStatsResponse(msg, pt_msg)
        
    def set_selection_method(self, req):
        if req.method == req.THRESH:
            self.use_selected_thresh = True
        elif req.method == req.CLOSEST:
            self.use_selected_thresh = False
        return srv.SetSelectionMethodResponse()

    def handle_draw_polygon(self, req):
        self.polygonAdded.emit(req)
        return srv.DrawPolygonResponse()

    def draw_polygon(self, req):
        with self.polygon_lock:
            # Project the polygon points onto the interface before saving them
            poly = QtGui.QPolygon()
            pts_arr = np.array([[(p.x, p.y, p.z) for p in req.polygon.polygon.points]])
            points = self.projectPoints(pts_arr, req.polygon.header)
            points = cv2.perspectiveTransform(points, self.H)
            if self.flip:
                points[0] = ([self.height(), self.width()] - points[0])
            for point in points[0]:
                poly.push_back(QtCore.QPoint(point[1], point[0]))

            poly_item = self.scene().addPolygon(poly, pen=self.POLYGON_PEN)

            textRect = poly.boundingRect()
            # Project the text rect points onto the interface
            if len(req.text_rect.points) > 0:
                text_rect_points = self.projectPoints(
                    np.array([[(p.x,p.y,p.z) for p in req.text_rect.points]]),
                    req.polygon.header
                )
                text_rect_points = cv2.perspectiveTransform(text_rect_points, self.H)
                if self.flip:
                    text_rect_points[0] = ([self.height(), self.width()] - text_rect_points[0]) 
                text_poly = QtGui.QPolygon()
                for point in text_rect_points[0]:
                    text_poly.push_back(QtCore.QPoint(point[1], point[0]))
                textRect = text_poly.boundingRect()

                if req.label:
                    font = QtGui.QFont('Decorative', 30)
                    text_item = QtGui.QGraphicsSimpleTextItem(
                        req.label, parent=poly_item,
                        scene=self.scene()
                    )
                    text_item.setFont(font)
                    text_item.setBrush(QtGui.QBrush(Colors.WHITE))
                    text_item.setPos(textRect.topLeft())


            self.polygons[req.id] = GraphicsItemInfo(poly_item, req.id, req.label)
            self.scene().invalidate(self.polygons[req.id].item.boundingRect())

    def clear_polygons(self):
        with self.polygon_lock:
            for poly in self.polygons.values():
                self.scene().removeItem(poly.item) 
                QtCore.QTimer.singleShot(100, partial(self.scene().invalidate, poly.item.boundingRect()))
                QtCore.QTimer.singleShot(100, partial(self.scene().invalidate, poly.item.childrenBoundingRect()))
            self.polygons.clear()

    def handle_clear_polygons(self, _):
        self.polygonsCleared.emit()
        return srv.ClearPolygonsResponse()

    def reconfig_cb(self, config, level):
        # ignore the first config we get
        if self.config_inited:
            self.projected_cursor = deque(self.projected_cursor, config['window_size'])
            self.cursor_pts_xyz   = deque(self.cursor_pts_xyz,   config['window_size'])

        else:
            config['window_size'] = self.projected_cursor.maxlen
            self.config_inited = True

        return config

    def checkShutdownRequest(self):
        if rospy.is_shutdown():
            QtGui.QApplication.quit()

    def __init__(self):
        super(Circler, self).__init__()
        self.tfl = tf.TransformListener()
        r = rospy.Rate(10)
        rospy.loginfo('waiting for homography...')
        while (not rospy.has_param('/homography')) and (not rospy.is_shutdown()):
            r.sleep()
        self.H = np.float64(rospy.get_param('/homography')).reshape(3, 3)
        self.flip = rospy.get_param('~flip', default=False)

        self.projected_cursor = deque([], rospy.get_param('~window_size', 10))
        self.cursor_pts_xyz   = deque([], rospy.get_param('~window_size', 10))
        self.circle_objects   = rospy.get_param('~circle_objects', True)

        # self.addKeyHandler(67, QtGui.QApplication.quit)

        self.click.connect(self.handleClick)
        self.polygonAdded.connect(self.draw_polygon)
        self.polygonsCleared.connect(self.clear_polygons)
        self.cursorMoved.connect(self.update_cursor)
        self.objectsChanged.connect(self.updateObjects)
        self.objectHighlighted.connect(self.hilight_object)
        self.hilightsCleared.connect(self.clear_hilights)

        rospy.loginfo('Window size = %s', rospy.get_param('~window_size', 10))
        rospy.loginfo('Flip        = %s', self.flip)
        rospy.loginfo('Circle      = %s', self.circle_objects)

        rospy.loginfo('got homography')
        print self.H
        self.initUI()
        rospy.Subscriber('object_cloud', PointCloud2, self.object_cb)
        rospy.Subscriber('click', Empty, self.click_cb)
        rospy.Subscriber('intersected_points', PointCloud2, self.intersected_cb, queue_size=1)
        rospy.Subscriber('intersected_points_cursor', PointCloud2, self.cursor_cb)
        self.info_sub = rospy.Subscriber('camera_info', CameraInfo, self.info_cb)
        rospy.Service('hilight_object', srv.HilightObject, self.handle_hilight)
        rospy.Service('clear_hilights', srv.ClearHilights, self.handle_clear_hilight)
        rospy.Service('get_cursor_stats', srv.GetCursorStats, self.handle_get_cursor_stats)
        rospy.Service('set_selection_method', srv.SetSelectionMethod, self.set_selection_method)
        rospy.Service('draw_polygon', srv.DrawPolygon, self.handle_draw_polygon)
        rospy.Service('clear_polygons', srv.ClearPolygons, self.handle_clear_polygons)
        self.selected_pub = rospy.Publisher('selected_point', PointStamped)

        self.rate_pub = rospy.Publisher('rate', Empty)
        self.dur_pub = rospy.Publisher('duration', Duration)

        self.click_stats_pub = rospy.Publisher('click_stats', PointCloud2)
        self.clicked_object_pub = rospy.Publisher('clicked_object', String)
        DynamicReconfigureServer(InterfaceConfig, self.reconfig_cb)

        self.interrupt_timer = QtCore.QTimer(self)
        self.interrupt_timer.setInterval(30)
        self.interrupt_timer.timeout.connect(self.checkShutdownRequest)
        self.interrupt_timer.start()
        
        rospy.loginfo('interface started')
        app.exec_()
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('object_circler')
    print 'Window size is %s' % rospy.get_param('~window_size', 1)
    app = QtGui.QApplication(sys.argv)
    c = Circler()
