#!/usr/bin/env rosh
load('rosh_geometry', globals())

from lxml import etree
from lxml.etree import Element
import os
from tf.transformations import quaternion_from_euler
from math import pi

def make_tf_pub(parent, child, transform):
    attribs = dict(
        parent=parent,
        child=child,
        x=transform.pose.position.x,
        y=transform.pose.position.y,
        z=transform.pose.position.z,
        qx=transform.pose.orientation.x,
        qy=transform.pose.orientation.y,
        qz=transform.pose.orientation.z,
        qw=transform.pose.orientation.w
    )
    tr = Element(
        'node',
        type='static_transform_publisher',
        pkg='tf',
        name='%s_to_%s' % (parent, child),
        args='%(x)s %(y)s %(z)s %(qx)s %(qy)s %(qz)s %(qw)s %(parent)s %(child)s 100' % attribs
    )
    return tr

top_right = 'top_right'
bottom_left = 'bottom_left'

# top level launch file
launch_tree = Element('launch')
launch_tree.append(Element(
    'param',
    name='/screen/frame',
    value=bottom_left,
))

# make a static transform publisher for the bottom left marker
# this is the frame the interface is composed in
transforms._config.listener.waitForTransform(bottom_left, top_right, Time(0), Duration(5))
parent = transforms[bottom_left]._config.listener.chain(bottom_left, Time(0), '/', Time(0), '/')[0]
tr = transforms[parent](bottom_left)
elem = make_tf_pub(parent, bottom_left, tr)
launch_tree.append(elem)

# add some args for the optical rotate
elem_pi = Element('arg', name='pi/2', value='1.5707963267948966')
elem_optical_rotate = Element('arg', name='optical_rotate', value='0 0 0 -$(arg pi/2) 0 -$(arg pi/2)')

launch_tree.append(elem_pi)
launch_tree.append(elem_optical_rotate)

# first we need the camera link
parent = 'world_to_projector_cam_rgb_optical_frame'

transform = PoseStamped(frame_id='world', position=Point(0,0,0.5), orientation=Quaternion(0,0,0,1))
elem = make_tf_pub('world', 'projection_cam_link', transform)
launch_tree.append(elem)

# now we need to reorient things for the optical frame
# x = right     (forward)
# y = down      (left)
# z = forward   (up)
transform = PoseStamped(frame_id='world', position=Point(0,0,0.5), orientation=Quaternion(*quaternion_from_euler(-pi/2,0,-pi/2)))
# elem = make_tf_pub('projection_cam_link', parent, transform)
elem_projection_cam_optical_rotate = Element(
    'node',
    type='static_transform_publisher',
    pkg='tf',
    name='projector_cam_optical_rotate',
    args='$(arg optical_rotate) projection_cam_link %s 100' % parent
)

launch_tree.append(elem_projection_cam_optical_rotate)

# make a transform for the kinect
# same place as camera, but pointing backwards
orientation = quaternion_from_euler(0,0,pi)
transform.pose.orientation = Quaternion(*orientation)
transform.child_frame_id = 'face_cam_link'
elem = make_tf_pub('world', 'face_cam_link', transform)
launch_tree.append(elem)

# figure out how big the screen is
tr = transforms.bottom_left('top_right')
width, height = tr.pose.position.x, tr.pose.position.y

launch_tree.append(Element(
    'param',
    name='/screen/width',
    value=str(width),
    type='double'
))

launch_tree.append(Element(
    'param',
    name='/screen/height',
    value=str(height),
    type='double'
))

rospy.set_param('/screen/width', str(width))
rospy.set_param('/screen/height', str(height))

# wait for a homography
import rospy
while not rospy.has_param('/homography'):
    rospy.sleep(0.1)

homography = rospy.get_param('/homography')
elem_h = Element('rosparam', param='/homography')
elem_h.text = str(homography)
launch_tree.append(elem_h)

launch_path = os.path.join(roslib.packages.get_pkg_dir('projector_interface'), 'study', 'launch', 'setup.launch')
# with etree.xmlfile(launch_path) as lf:
with open(launch_path, 'w') as lf:
    lf.write(etree.tostring(launch_tree, pretty_print=True))
    print 'wrote', launch_path

# print etree.tostring(launch_tree, pretty_print=True)