#! /usr/bin/env python2

import rospy
import numpy as np
import tf_conversions
import tf2_ros
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import *
from std_msgs.msg import String
from subt_msgs.srv import *

class FixedTFBroadcaster:
    def __init__(self, robot_ns, parent_frame, child_frame):
        # Create a publisher to for tf
        self.pub_tf = rospy.Publisher('/'+robot_ns.data+'/tf_origin', TFMessage, queue_size=10)
        # Create a broadcaster for tf
        self.transform_broadcaster = tf2_ros.TransformBroadcaster()
        # Call the subt origin service
        self.tf = self.get_initial_pose_client(robot_ns, parent_frame, child_frame)

        # while ros exists
        while not rospy.is_shutdown():
            # Get the TF Message
            tfm = TFMessage([self.tf])
            # Broadcast and publish the TF
            self.transform_broadcaster.sendTransform(self.tf)
            self.pub_tf.publish(tfm)

    def get_initial_pose_client(self, robot_ns, parent_frame, child_frame):
        # Create dumby variables
        initial_transform = TransformStamped()
        initial_pose = Pose()

        # Wait for the service
        rospy.wait_for_service('/subt/pose_from_artifact_origin')
        try:
            # Create a service proxy
            pose_from_origin = rospy.ServiceProxy('/subt/pose_from_artifact_origin', PoseFromArtifact)
            # Call the proxy with the desired robot_ns
            response = pose_from_origin(robot_ns)
            # If the proxy failed return error
            if not response.success:
                rospy.logerror("Pose not found, check if robot is in starting area?")
                return
            # If the initial pose exits set it to the response
            initial_pose = response.pose
            # Set the transform stamped message with the pose and proper frames
            initial_transform.header.stamp = rospy.Time.now()
            initial_transform.header.frame_id = parent_frame
            initial_transform.child_frame_id = child_frame
            initial_transform.transform.translation.x = initial_pose.pose.position.x
            initial_transform.transform.translation.y = initial_pose.pose.position.y
            initial_transform.transform.translation.z = initial_pose.pose.position.z
            initial_transform.transform.rotation = initial_pose.pose.orientation
            # Return the tranform to be published
            return initial_transform

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

if __name__ == "__main__":
    rospy.init_node('intial_pose')

    # Get robot name from parameter and store in std_msgs string
    robot_ns = String()
    robot_ns.data = rospy.get_param('/robot_ns', 'X1')

    # Set up parent and child frame of TF
    parent_frame = robot_ns.data + '/map'
    child_frame = 'world'

    # Call TFBroadcaster Class
    tfb = FixedTFBroadcaster(robot_ns, parent_frame, child_frame)
    rospy.spin()
