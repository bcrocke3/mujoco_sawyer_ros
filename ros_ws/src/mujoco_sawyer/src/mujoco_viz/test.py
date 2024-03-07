import mujoco
import mujoco.viewer


if __name__ == "__main__":
    # xml_path = "/home/breanne/repos/mujoco_menagerie/rethink_robotics_sawyer/sawyer_scene.xml"
    xml_path = "/home/breanne/repos/ipd-relax-project/ipd-relax-sim/assets/rethink_robotics_sawyer/sawyer_scene.xml"
    # Load the Mujoco model from the environment file
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    with mujoco.viewer.launch_passive(model, data) as viewer:
                while viewer.is_running():
                    pass




