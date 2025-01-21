import time
import mujoco
import mujoco.viewer
import numpy as np  # For sine wave calculations

# Load the model and data
model = mujoco.MjModel.from_xml_path("./stick_insect.xml")
data = mujoco.MjData(model)

# Get the joint index
joint_names = ['TR0','CR0','FR0','TL0','CL0','FL0','TR1','CR1','FR1','TL1','CL1','FL1','TR2','CR2','FR2','TL2','CL2','FL2']  # Replace with the actual joint name
joint_id_map = {}


for joint_name in joint_names:
    try:
        joint_id_map[joint_name] = (model.joint(name=joint_name).qposadr)[0] - 6
    except Exception as e:
        print(f"Error: Could not find joint {joint_name}. Exception: {e}")

# Print the resulting dictionary
print("Joint ID Map:", joint_id_map)



with mujoco.viewer.launch_passive(model, data) as viewer:
    # Close the viewer automatically after 30 wall-seconds.
    start = time.time()


    while viewer.is_running() : # and time.time() - start < 30
        step_start = time.time()


        if step_start - start < 5.0:
            for joint_name in joint_names:
                data.ctrl[joint_id_map[joint_name]] = 0.0
        elif step_start - start > 5.0:
            frequency = 2  # Hz
            amplitude = 30  # Radians
            desired_position = amplitude * np.sin(2 * np.pi * frequency * step_start)



            data.ctrl[joint_id_map['TR0']] = desired_position
            data.ctrl[joint_id_map['CR0']] = desired_position
            data.ctrl[joint_id_map['FR0']] = desired_position

            data.ctrl[joint_id_map['TL0']] = -desired_position
            data.ctrl[joint_id_map['CL0']] = -desired_position
            data.ctrl[joint_id_map['FL0']] = -desired_position

        
            data.ctrl[joint_id_map['TR1']] = desired_position
            data.ctrl[joint_id_map['CR1']] = 2*desired_position
            data.ctrl[joint_id_map['FR1']] = 0

            data.ctrl[joint_id_map['TL1']] = -2*desired_position
            data.ctrl[joint_id_map['CL1']] = -2*desired_position
            data.ctrl[joint_id_map['FL1']] = 0

            data.ctrl[joint_id_map['TR2']] = desired_position
            data.ctrl[joint_id_map['CR2']] = 2*desired_position
            data.ctrl[joint_id_map['FR2']] = 0

            data.ctrl[joint_id_map['TL2']] = -2*desired_position
            data.ctrl[joint_id_map['CL2']] = -2*desired_position
            data.ctrl[joint_id_map['FL2']] = 0


        # Step the simulation
        mujoco.mj_step(model, data)

        # Example modification of a viewer option: toggle contact points every two seconds
        with viewer.lock():
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(data.time % 2)

        # Sync the viewer with the simulation state
        viewer.sync()

        # Rudimentary time keeping to match the real-time simulation
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
