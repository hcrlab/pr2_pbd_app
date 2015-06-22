#!/usr/bin/env python

import rospy
import tf

class tfPublisher:
    def __init__(self):
        self.tf = tf.TransformListener()
        self.current_transforms = []
        self.transform_names = []
        rospy.Subscriber("tf", tf.msg.tfMessage, self.tf_callback)
        self.tfPub = rospy.Publisher('current_tf_transforms', tf.msg.tfMessage, queue_size=10)
        
    def tf_callback(self, data):
        new_transforms = data.transforms
        self.transform_names = self.tf.getFrameStrings()
        current_transforms = []
        new_transform_names = []
        for transform in new_transforms:
            if transform.header.frame_id in self.transform_names:
                new_transform_names.append(transform.header.frame_id)
                current_transforms.append(transform)

        for old_transform in self.current_transforms:
            if old_transform.header.frame_id in self.transform_names:
                if not old_transform.header.frame_id in new_transform_names:
                    current_transforms.append(old_transform)

        self.current_transforms = current_transforms
        self.tfPub.publish(self.current_transforms)

            

if __name__ == '__main__':
    rospy.init_node('publish_tf')
    tfPublisher()
    rospy.spin()
        


