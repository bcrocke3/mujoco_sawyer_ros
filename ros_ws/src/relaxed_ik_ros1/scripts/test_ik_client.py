#!/usr/bin/env python3


from relaxed_ik_ros1.srv import IKPose, IKPoseResponse
from geometry_msgs.msg import Pose, Point, Quaternion, Twist

import rospy
import sys

from typing import List

# set numpy print options to 2 decimal places, no scientific notation
import numpy as np
np.set_printoptions(precision=2, suppress=True)

class TestIKClient():

    def __init__(self):
        SERVICE_NAME: str = "/ik/solve_pose"

        rospy.wait_for_service(SERVICE_NAME)
        self.client = rospy.ServiceProxy(SERVICE_NAME, IKPose)

        print("IK Client connected to service: {}".format(SERVICE_NAME))
            

    def send_IK_pose(self, position: List[float], orientation: List[float]) -> None:
        # validate input
        assert(len(position) == 3)
        assert(len(orientation) == 4)

        # make a Pose message
        pose_msg = Pose()
        
        position_msg = Point()
        position_msg.x = position[0]
        position_msg.y = position[1]
        position_msg.z = position[2]
        pose_msg.position = position_msg

        quaternion_msg = Quaternion()
        quaternion_msg.x = orientation[0]
        quaternion_msg.y = orientation[1]
        quaternion_msg.z = orientation[2]
        quaternion_msg.w = orientation[3]
        pose_msg.orientation = quaternion_msg

        # make a Twist message for tolerance
        # twist_msgs = Twist()
        # twist_msgs.linear.x = 0.01
        # twist_msgs.linear.y = 0.01
        # twist_msgs.linear.z = 0.01
        # twist_msgs.angular.x = 0.01
        # twist_msgs.angular.y = 0.01
        # twist_msgs.angular.z = 0.01

        # send the request
        print("Sending IK Request")
        # result: IKPoseResponse = self.client([pose_msg], [twist_msgs])
        result: IKPoseResponse = self.client([pose_msg], [])
        resulting_joint_angles: List[float] = list(result.joint_state)

        # print results
        print("Response Recieved: {}".format(result))
        print("Joint Angles: {}".format(resulting_joint_angles))


    def printUsage(self):
        print("Usage: rosrun relaxed_ik_ros1 bre_test_script.py [x] [y] [z] [qx] [qy] [qz] [qw]")


if __name__ == "__main__":
    ik_client = TestIKClient()

    if len(sys.argv) == (1 + 3 + 4): # filename + 3 pos + 4 quat
        position = [float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])]
        orientation = [float(sys.argv[4]), float(sys.argv[5]), float(sys.argv[6]), float(sys.argv[7])]

        print("--- Input -- ")
        print("Position: {}".format(position))
        print("Orientation: {}".format(orientation))

        ik_client.send_IK_pose(position, orientation)
    else:
        ik_client.printUsage()
        sys.exit(1)
