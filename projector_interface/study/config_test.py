#!/usr/bin/env python
from __future__ import division

import roslib; roslib.load_manifest('projected_interface_builder')
from projected_interface_builder.projected_interface import ProjectedInterface
from projected_interface_builder.data_structures import PolygonInfo
from projected_interface_builder.convert_utils import QtPolyToROS, QtRectToPoly
import rospy

import sys

class ConfigTestInterface(ProjectedInterface):
    def __init__(self, polygon_file):
        super(ConfigTestInterface, self).__init__(polygon_file)

        # wait for the screen size params to become available
        while not rospy.has_param('/screen/height'):
            rospy.sleep(0.1)

        self.width = float(rospy.get_param('/screen/width'))
        self.height = float(rospy.get_param('/screen/height'))


    def updatePosition(self):
        square = self.polygons['square']
        ps = QtPolyToROS(square.polygon, square.id, interf.x, interf.y, interf.z, interf.res, '/bottom_left')

        # get the current bounds
        left = min(
            ps.polygon.points[0].x,
            ps.polygon.points[1].x,
            ps.polygon.points[2].x,
            ps.polygon.points[3].x,
        )

        right = max(
            ps.polygon.points[0].x,
            ps.polygon.points[1].x,
            ps.polygon.points[2].x,
            ps.polygon.points[3].x,
        )

        top = max(
            ps.polygon.points[0].y,
            ps.polygon.points[1].y,
            ps.polygon.points[2].y,
            ps.polygon.points[3].y,
        )

        bottom = min(
            ps.polygon.points[0].y,
            ps.polygon.points[1].y,
            ps.polygon.points[2].y,
            ps.polygon.points[3].y,
        )

        poly_width = abs(right-left)

        square.text_rect = square.polygon.boundingRect()
        square.text_rect.setTop(square.text_rect.top() + 20)
        self.x = (self.width/2)
        self.y =  self.height/2
        print self.x, self.y

        self.publish_polygons()

if __name__ == '__main__':
    rospy.init_node('config_test_interface')
    interf = ConfigTestInterface(sys.argv[1])
    interf.start()
    interf.updatePosition()
    rospy.spin()
    # interf.maybe_write_changes()