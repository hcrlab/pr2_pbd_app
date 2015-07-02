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

        self.blacklist = ['bl_caster_rotation_link',
            'br_caster_rotation_link',
            'fl_caster_rotation_link',
            'fr_caster_rotation_link',
            'head_pan_link',
            'l_upper_arm_link',
            'l_elbow_flex_link',
            'l_gripper_r_finger_tip_link',
            'l_gripper_palm_link',
            'l_gripper_l_finger_link',
            'l_gripper_motor_slider_link',
            'l_gripper_r_finger_link',
            'l_shoulder_pan_link',
            'l_shoulder_lift_link',
            'l_forearm_link',
            'l_wrist_flex_link',
            'r_upper_arm_link',
            'r_elbow_flex_link',
            'r_gripper_r_finger_tip_link',
            'r_gripper_palm_link',
            'r_gripper_l_finger_link',
            'r_gripper_motor_slider_link',
            'r_gripper_r_finger_link',
            'r_shoulder_pan_link',
            'r_shoulder_lift_link',
            'r_forearm_link',
            'r_wrist_flex_link',
            'sensor_mount_link',
            'head_plate_frame',
            'head_mount_link',
            'head_mount_kinect_ir_link',
            'head_mount_kinect_rgb_link',
            'head_mount_prosilica_link',
            'head_tilt_link',
            'high_def_frame',
            'l_forearm_roll_link',
            'l_forearm_cam_frame',
            'l_upper_arm_roll_link',
            'laser_tilt_mount_link',
            'double_stereo_link',
            'narrow_stereo_link',
            'narrow_stereo_l_stereo_camera_frame',
            'narrow_stereo_r_stereo_camera_frame',
            'projector_wg6802418_frame',
            'r_forearm_roll_link',
            'r_forearm_cam_frame',
            'r_upper_arm_roll_link',
            'wide_stereo_link',
            'wide_stereo_l_stereo_camera_frame',
            'wide_stereo_r_stereo_camera_frame',
            'odom_combined']

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
                if not transform.header.frame_id in new_transform_names and not transform.header.frame_id in self.blacklist:
                    new_transform_names.append(transform.header.frame_id)
                    current_transforms.append(transform)

        for old_transform in self.current_transforms:
            if old_transform.header.frame_id in self.transform_names and not old_transform.header.frame_id in self.blacklist:
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
        


