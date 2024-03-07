from mujoco_viz.mujoco_visualizer import MuJoCoVisualizer
from mujoco_sawyer.srv import UpdateJoints, UpdateJointsResponse, GetPose, GetPoseResponse
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Quaternion
from typing import List

import rospy
import numpy as np


class SimulationServer():
    """ ROS Node that manages simulation. It has a service that accepts joint commands and sends them to the 
        simulator to be executed.
    """

    def __init__(self):
        """ Initialize the simulation server node and create the service that accepts joint commands.
            Initialize the visualizer that will be used to simulate the robot.
        """

        self.move_joints_srv = rospy.Service("update_joints", UpdateJoints, self.handle_update_joints_request)
        print("Simulation server ready")

        self.pose_srv = rospy.Service("get_pose", GetPose, self.handle_get_pose_request)

        sawyer_basic_scene = "/home/breanne/repos/mujoco_menagerie/rethink_robotics_sawyer/scene.xml"
        self.visualizer = MuJoCoVisualizer(sawyer_basic_scene)


    def handle_get_pose_request(self, pose_request: GetPose) -> GetPoseResponse:
        """ Callback function for the service that returns current pose. It recieves an Empty message.
        """
        # print("Received request to get pose")
        pos, quat = self.visualizer.get_pose()

        pose_response = GetPoseResponse()
        pose_response.pos_x = pos[0]
        pose_response.pos_y = pos[1]
        pose_response.pos_z = pos[2]
        pose_response.quat_x = quat[0]
        pose_response.quat_y = quat[1]
        pose_response.quat_z = quat[2]
        pose_response.quat_w = quat[3]

        return pose_response


    def handle_update_joints_request(self, joints_request: UpdateJoints) -> UpdateJointsResponse:
        """ Callback function for the service that accepts joint commands. It receives a JointState message
            and sends it to the visualizer to be executed.
        """
        # print("Received request to update joints")
        joint_state: JointState = joints_request.joint_command
        target_position = np.array(list(joint_state.position)) # convert from tuple to list
        print("Target position: ", target_position)

        self.visualizer.add_target_to_trajectory(target_position)

        return UpdateJointsResponse(True)
    
    def start_simulator(self) -> None:
        self.visualizer.simulate()
        


if __name__ == "__main__":
    # set numpy print precision to 3 decimal places, no scientific notation
    np.set_printoptions(precision=3, suppress=True)

    rospy.init_node("simulation_server")
    server = SimulationServer()
    server.start_simulator()  # TODO: this should probably run in its own thread bc it blocks
    rospy.spin()