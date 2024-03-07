# sawyer_mujoco_ros

## System Requirements
 - OS: Ubuntu 20 (native or WSL2)
 - ROS: Noetic
 - Python: 3


## Setup
1. [Install Rust](https://www.rust-lang.org/tools/install) on your system
    - This is need by the IK
2. Clone this repo. `git clone <repo>`
3. Initialze the submodules. 
    - From the root of this repo, run `git submodule init && git submodule update --init --recursive`
    - This clones the IK package into the ROS workspace. You know that this worked if the `ros_ws/src/relaxed_ik_core` directory is not empty.
4. Build the Rust IK package.
    - `cd ros_ws/src/relaxed_ik_core`
    - `cargo build`
    - This might produce warnings, but should not produce errors.
5. Make a python virtual environment. `python3 -m venv venv`
6. Activate the virtual environment. `source venv/bin/activate`
7. Install the python dependencies. `pip install -r requirements-ros.txt`
8. Update the some file paths.
    - Open `ros_ws/src/mujoco_sawyer/scripts/simulation_server.py`. Change line 26 to be the path to the mujoco scene file on your system. You'll need to update the beginning of the path depending on where you cloned this repo.
    - Open `ros_ws/src/mujoco_sawyer/launch/simulator.launch`. Change line 6 to be the path to your python environment.
9. Build the ROS workspace. (Don't forget to source ROS first)
    - `cd ros_ws`
    - `catkin_make -DPYTHON_EXECUTABLE=<PATH_TO_YOUR_VENV>`
    - Replace `<PATH_TO_YOUR_VENV>` with the path to the python executable in your virtual environment, without quotes or brackets around the path.
    - This will build the ROS workspace with the python virtual environment as the default python interpreter.
    - You only need to specify the python environment the first time you catkin_make. After that, it will remember the environment and you can just `catkin_make`.
    - No guarantees that the requirements-ros.txt file is up to date. You may need to pip install additional packages.


## Test if you setup correctly
1. Source ROS. `source /opt/ros/noetic/setup.bash`
2. Build & source the ROS workspace. 
    - `catkin_make`
    - `source ros_ws/devel/setup.bash`
3. Launch the simulator. `roslaunch mujoco_sawyer simulator.launch`
    - This should open a GUI window with the sawyer robot in a mujoco environment.
4. Open a new terminal. Source ROS and the workspace. Activate your python environment.
    - `source /opt/ros/noetic/setup.bash`
    - `source ros_ws/devel/setup.bash`
    - `source venv/bin/activate`
5. Run a test command. 
    - Set the joint positions (params are joint positions in radians): `rosrun mujoco_sawyer test_sim_client.py --joints -1.58 0.0 0.0 -1.58 0.0 0.0 0.0`
    - Set the end effector pose (params are x, y, z  in meters and x, y, z, w quaternion): `rosrun mujoco_sawyer test_sim_client.py --pose 1.0 0.8 0.8 0.0 0.0 0.0 1.0`


## Notes
- The test section describes how to manually publish to the sim server to move the robot. You'll probably want to do this programmatically. Look at the `test_sim_client.py` file as an example. This is slightly different than moving the real sawyer robot. Real Sawyer publishes to a topic, this sim makes a service call.
- The simulator uses a PID controller to determine joint torques to move to the desired joint positions. The PID gains are set in the `ros_ws/src/mujoco_sawyer/src/mujoco_viz/mujoco_visualizer.py` file. Gains are not very well tuned. There might be a better option than a PID for this.