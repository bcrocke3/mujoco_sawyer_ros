from mujoco_sawyer.srv import GetPose, GetPoseResponse, GetPoseRequest

import rospy
from typing import List
import numpy as np

class TestPoseClient():
    """ Class that sends requests to ROS services to test the simulation.
    """

    def __init__(self):
        """ Connects to ROS services for updating joint positions and solving IK.
        """
        rospy.wait_for_service("get_pose")
        self.pose_client = rospy.ServiceProxy("get_pose", GetPose)
        print("Client connected to GetPose service")

    def send_pose_request(self) -> List[float]:
        """ Sends a reques to the GetPose service to get the current pose of the end effector.
        
        :return: list of the current position and orientation of the end effector
        :rtype: list of floats
        """
        pose_response: GetPoseResponse = self.pose_client(GetPoseRequest())
        position: List[float] = [pose_response.pos_x, pose_response.pos_y, pose_response.pos_z]
        orientation: List[float] = [pose_response.quat_x, pose_response.quat_y, pose_response.quat_z, pose_response.quat_w]

        return position + orientation
            

    def printUsage(self):
        print("Usage: ")
        print("    rosrun mujoco_sawyer test_pose_client.py")


if __name__ == "__main__":
    # set numpy precision to 3 decimal places, no scientific notation
    np.set_printoptions(precision=3, suppress=True)
    
    client = TestPoseClient()
    result = np.array(client.send_pose_request())

    # print list with no commas
    print("Result: ", result)
