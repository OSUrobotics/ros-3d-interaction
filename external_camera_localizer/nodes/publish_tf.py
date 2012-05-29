#!/usr/bin/env rosh
import roslib; roslib.load_manifest('tf')
import sys
import tf

load('rosh_robot', globals())

# if __name__ == '__main__':
    # if len(sys.argv) != 3:
        # logerr('Usage: %s topic1 topic2' % sys.argv[0])
        # exit(1)
    
topic1 = '/laser_cloud_converted'
topic2 = '/camera1/transformed'

print 'Waiting for', topic1
cloud1 = topics[topic1][0]
print 'Waiting for', topic2
cloud2 = topics[topic2][0]

print 'Waiting for service...'
wait_for_service('matchClouds')

print 'Calling service...'
transform = services.matchClouds(cloud1, cloud2)

print 'Got result...'
trans = transform.transform.translation
rot   = transform.transform.rotation

#t = tf.Transformer(True, Duration(10))


#listener = tf.TransformListener()
#cam2_rgb_offset = listener.lookupTransform('/camera2_link', cloud2.header.frame_id)

#trans, rot = listener.lookupTransform(


print '<node pkg="tf" type="static_transform_publisher" name="external_cam" args="%s %s %s %s %s %s %s %s %s 100" />' % (trans.x, trans.y, trans.z, rot.x, rot.y, rot.z, rot.w, cloud1.header.frame_id, cloud2.header.frame_id)
print '\n'
print 'rosrun tf static_transform_publisher %s %s %s %s %s %s %s %s %s 100' % (trans.x, trans.y, trans.z, rot.x, rot.y, rot.z, rot.w, cloud1.header.frame_id, cloud2.header.frame_id)
