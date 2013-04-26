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
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty
import rospy

click_inhibit = False
last_val = 0

def uninhibit(args):
    global click_inhibit
    click_inhibit = False

def joy_cb(msg, args):
    global click_inhibit
    global last_val

    click = msg.buttons[2]
    if click and (last_val == 0) and not click_inhibit:
        click_inhibit = True
        args[0].publish()
        rospy.Timer(rospy.Duration(1.0), uninhibit, oneshot=True)
        last_val = 1
    elif click:
        pass
    else:
        last_val = 0
		

if __name__ == '__main__':
    rospy.init_node('point_to_click')
    click_pub = rospy.Publisher('click', Empty)
    rospy.Subscriber('joy', Joy, joy_cb, [click_pub])
    rospy.spin()
