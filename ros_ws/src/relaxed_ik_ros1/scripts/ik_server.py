#! /usr/bin/env python3

import rospkg
import rospy
import sys
import os
import yaml

from relaxed_ik_ros1.srv import IKPose, IKPoseResponse

# import the Python wrapper for the Rust IK library
path_to_src: str = rospkg.RosPack().get_path('relaxed_ik_ros1') + '/relaxed_ik_core'
sys.path.insert(1, path_to_src + '/wrappers')
from python_wrapper import RelaxedIKRust

class IKServer:

    def __init__(self):
        """ Initialize the Rust IK library. Loads settings file for robot and IK solver.
            Initialize the IK Server node and create the service that accepts IK requests.
        """
        rospy.loginfo("Initializing IK Server...")

        rospy.sleep(1) #TODO: is this necessary? Copied from RelaxedIK ROS Example

        # Load the Settings
        # TODO: make a copy of the default settings file and load that instead
        # Sawyer is default so this is fine for now
        default_setting_file_path: str = path_to_src + '/configs/settings.yaml'

        setting_file_path: str = ""
        try: 
            setting_file_path = rospy.get_param('setting_file_path')
        except:
            pass

        if setting_file_path == "":
            print("Relaxed IK Rust: no setting file path is given, using the default setting file -- {}".format(default_setting_file_path))
            setting_file_path = default_setting_file_path

        os.chdir(path_to_src)

        # Load the infomation
        print("setting_file_path: ", setting_file_path)
        setting_file = open(setting_file_path, 'r')
        settings = yaml.load(setting_file, Loader=yaml.FullLoader)
       
        urdf_file = open(path_to_src + '/configs/urdfs/' + settings["urdf"], 'r')
        urdf_string = urdf_file.read()
        rospy.set_param('robot_description', urdf_string)

        self.relaxed_ik = RelaxedIKRust(setting_file_path)

        # Initialize Services
        self.ik_pose_service = rospy.Service('ik/solve_pose', IKPose, self.handle_ik_pose_request)
        rospy.loginfo("IK Server Initialized")


    def handle_ik_pose_request(self, ik_request: IKPose) -> IKPoseResponse:
        """ Handles requests to the IKPose service. Calls the Rust IK library and returns the result. 

        :param ik_request: The request to the IKPose service
        :type ik_request: IKPose (a ROS service, defined in srv folder)

        :return: The response to the IKPose service, a joint state message
        :rtype: IKPoseResponse (a ROS service, defined in srv folder)

        """
        # rospy.loginfo("Received IK Request")
        positions = []
        orientations = []
        tolerances = []
        for i in range(len(ik_request.ee_poses)):
            positions.append(ik_request.ee_poses[i].position.x)
            positions.append(ik_request.ee_poses[i].position.y)
            positions.append(ik_request.ee_poses[i].position.z)
            orientations.append(ik_request.ee_poses[i].orientation.x)
            orientations.append(ik_request.ee_poses[i].orientation.y)
            orientations.append(ik_request.ee_poses[i].orientation.z)
            orientations.append(ik_request.ee_poses[i].orientation.w)

            if i < len(ik_request.tolerances):
                tolerances.append(ik_request.tolerances[i].linear.x)
                tolerances.append(ik_request.tolerances[i].linear.y)
                tolerances.append(ik_request.tolerances[i].linear.z)
                tolerances.append(ik_request.tolerances[i].angular.x)
                tolerances.append(ik_request.tolerances[i].angular.y)
                tolerances.append(ik_request.tolerances[i].angular.z)
            else:
                for _ in range(6):
                    tolerances.append(0.0)
        
        # rospy.loginfo("Position: {}".format(positions))
        # rospy.loginfo("Orientation: {}".format(orientations))
        # rospy.loginfo("Tolerances: {}".format(tolerances))

        ik_solution = self.relaxed_ik.solve_position(positions, orientations, tolerances)

        res = IKPoseResponse()
        res.joint_state = ik_solution

        return res


if __name__ == '__main__':
    rospy.init_node('ik_server')
    server = IKServer()
    rospy.spin()