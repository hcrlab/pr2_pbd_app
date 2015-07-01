#!/usr/bin/env python

import rospy
import tf
from pr2_pbd_interaction.srv import CurrentTf, CurrentTfResponse
import threading 

class tfPublisher:
    def __init__(self):
        self.tf = tf.TransformListener()
        self.current_transforms = []
        self.transform_names = []
        
        # self.tfPub = rospy.Publisher('current_tf_transforms', tf.msg.tfMessage, queue_size=10)
        self.thread = threading.Thread(target=self.tf_listener)
        self.thread.start()

        s = rospy.Service('tf_service', CurrentTf, self.service_callback)

    def service_callback(self, req):
        resp = CurrentTfResponse()
        msg = tf.msg.tfMessage()
        msg.transforms = self.current_transforms
        resp.current_transforms = msg
        return resp

    def tf_listener(self):
        rospy.Subscriber("tf", tf.msg.tfMessage, self.tf_callback)
        rospy.spin()
        
    def tf_callback(self, data):
        new_transforms = data.transforms
        self.transform_names = self.tf.getFrameStrings()
        current_transforms = []
        new_transform_names = []
        for transform in new_transforms:
            if transform.header.frame_id in self.transform_names:
                if not transform.header.frame_id in new_transform_names:
                    new_transform_names.append(transform.header.frame_id)
                    current_transforms.append(transform)

        for old_transform in self.current_transforms:
            if old_transform.header.frame_id in self.transform_names:
                if not old_transform.header.frame_id in new_transform_names:
                    current_transforms.append(old_transform)
                    new_transform_names.append(old_transform.header.frame_id)

        self.current_transforms = current_transforms
        # self.tfPub.publish(self.current_transforms)
        print "                  "
        print "                  "
        for transform in self.current_transforms:
            print transform.header.frame_id

            

if __name__ == '__main__':
    rospy.init_node('publish_tf')
    tfPublisher()
    rospy.spin()
        


