#!/usr/bin/env rosh
load('rosh_geometry', globals())

from lxml import etree
from lxml.etree import Element
import os

def make_tf_pubs(parent, child, transform):
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
        args='%(x)s %(y)s %(z)s %(qx)s %(qy)s %(qz)s %(qw)s %(child)s_s %(parent)s 100' % attribs
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

# make a static transform publisher
transforms._config.listener.waitForTransform(bottom_left, top_right, Time(0), Duration(5))
parent = transforms[bottom_left]._config.listener.chain(bottom_left, Time(0), '/', Time(0), '/')[0]
tr = transforms[parent](bottom_left)
elem = make_tf_pubs(parent, bottom_left, tr)

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

launch_path = os.path.join(roslib.packages.get_pkg_dir('projector_interface'), 'study', 'launch', 'setup.launch')
# with etree.xmlfile(launch_path) as lf:
with open(launch_path, 'w') as lf:
    lf.write(etree.tostring(launch_tree, pretty_print=True))
    print 'wrote', launch_path

# print etree.tostring(launch_tree, pretty_print=True)