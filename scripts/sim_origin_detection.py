#! /usr/bin/env python2

import rospy
import numpy as np
import tf_conversions
import tf2_ros
from geometry_msgs.msg import *

from subt_msgs.srv import *

def get_initial_pose_client(robot_ns, parent_frame, child_frame):
    transform_broadcaster = tf2_ros.TransformBroadcaster()
    initial_transform = TransformStamped()
    initial_pose = Pose()

    rospy.wait_for_service('/subt/pose_from_artifact_origin')
    try:
        pose_from_origin = rospy.ServiceProxy('/subt/pose_from_artifact_origin', PoseFromArtifact)
        response = pose_from_origin(robot_ns)
        if not response.success:
            rospy.logerror("Pose not found, check if robot is in starting area?")
            return
        initial_pose = response.pose
        initial_transform.header.stamp = rospy.Time.now()
        initial_transform.header.frame_id = parent_frame
        initial_transform.child_frame_id = child_frame
        initial_transform.translation.x = initial_pose.position.x
        initial_transform.translation.y = initial_pose.position.y
        initial_transform.translation.z = initial_pose.position.z
        initial_transform.rotation = initial_pose.orientation

        transform_broadcaster.sendTransform(initial_transform)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    rospy.init_node('intial_pose')
    robot_ns = rospy.get_param('/robot_ns', '/X1')
    parent_frame = rospy.get_param('/parent_frame','/world')
    child_frame = rospy.get_param('/child_frame','/base_imu_link')
    get_initial_pose_client()
    rospy.spinOnce()    

