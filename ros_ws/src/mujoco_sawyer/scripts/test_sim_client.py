from mujoco_sawyer.srv import UpdateJoints
from sensor_msgs.msg import JointState

from relaxed_ik_ros1.srv import IKPose, IKPoseResponse
from geometry_msgs.msg import Pose, Point, Quaternion, Twist

import rospy
import sys
from typing import List

class TestSimClient():
    """ Class that sends requests to ROS services to test the simulation.
    """

    def __init__(self):
        """ Connects to ROS services for updating joint positions and solving IK.
        """
        rospy.wait_for_service("update_joints")
        self.joint_update_client = rospy.ServiceProxy("update_joints", UpdateJoints)
        print("Client connected to UpdateJoints service")


        rospy.wait_for_service("/ik/solve_pose")
        self.ik_client = rospy.ServiceProxy("/ik/solve_pose", IKPose)
        print("Client connected to IK service")
            

    def send_joint_request(self, joint_position: List[float]) -> None:
        """ Sends a request to the UpdateJoints service to update the joint positions of the simulated robot.

        :param joint_position: list of joint positions to send to the simulator
        :type joint_position: list of floats
        """
        joint_state_msg = JointState()
        joint_state_msg.position = joint_position

        result: bool = self.joint_update_client(joint_state_msg)
        print("Response Recieved: {}".format(result))

    def send_ik_request(self, position: List[float], orientation: List[float]) -> List[float]:
        """ Sends a request to the IK service to solve for the joint positions that will move the end effector to the
            specified position and orientation.

        :param position: x, y, z position of the end effector
        :type position: list of floats
        :param orientation: x, y, z, w quaternion orientation of the end effector
        :type orientation: list of floats

        :return: list of joint positions that will move the end effector to the specified position and orientation
        :rtype: list of floats
        """
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

        # send the request
        print("Sending IK Request")
        result: IKPoseResponse = self.ik_client([pose_msg], []) # tolerances empty, handled by IK; could be provided here
        resulting_joint_angles: List[float] = list(result.joint_state)

        return resulting_joint_angles


    def printUsage(self):
        print("Usage: ")
        print("    rosrun mujoco_sawyer test_sim_client.py --joints [joint_1] [joint_2] ... [joint_7]")
        print("      OR")
        print("    rosrun mujoco_sawyer test_sim_client.py --pose [x] [y] [z] [qx] [qy] [qz] [qw]")


if __name__ == "__main__":
    client = TestSimClient()

    if len(sys.argv) == (1 + 1 + 7) and sys.argv[1] == "--joints":
        # args: filename + --joints + 7 joint angles
        joint_position: List[float] = []
        for i in range(2, len(sys.argv)):
            joint_position.append(float(sys.argv[i]))

        client.send_joint_request(joint_position)

    elif len(sys.argv) == (1 + 1 + 3 + 4) and sys.argv[1] == "--pose":
        # args: filename + --pose + 3 pos + 4 quat
        position = [float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]) - 0.9] ## subtract 0.9 to account for different in origin in mujoco space and IK/urdf space
        orientation = [float(sys.argv[5]), float(sys.argv[6]), float(sys.argv[7]), float(sys.argv[8])]

        joint_positions: List[float] = client.send_ik_request(position, orientation)
        client.send_joint_request(joint_positions)
        
    else:
        client.printUsage()
        sys.exit(1)
