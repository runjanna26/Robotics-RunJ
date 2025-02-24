import time
import mujoco
import mujoco.viewer
import numpy as np  # For sine wave calculations

# Load the model and data
model = mujoco.MjModel.from_xml_path("./stick_insect.xml")
data = mujoco.MjData(model)

# Get the joint index
# Extract the joint names
joint_names = []
for joint_id in range(model.njnt):  # model.njnt gives the number of joints
    # Address of the joint name
    name_start = model.name_jntadr[joint_id]
    # Extract the name (null-terminated string)
    joint_name = model.names[name_start:].decode('utf-8').split('\x00', 1)[0]
    joint_names.append(joint_name)

# Print all joint names
print("Joint names:", joint_names)


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


        if step_start - start < 1.0:
            for joint_name in joint_names:
                data.ctrl[joint_id_map[joint_name]] = 0.0
        elif step_start - start > 1.0:
            frequency = 0.5  # Hz
            amplitude = np.pi/3  # Radians
            # desired_position = 



            data.ctrl[joint_id_map['TR0']] = np.pi/6 * np.sin(2 * np.pi * frequency * step_start)
            data.ctrl[joint_id_map['CR0']] = np.pi/3 * np.sin(2 * np.pi * frequency * step_start + np.pi/2)
            data.ctrl[joint_id_map['FR0']] = np.pi/4 * np.sin(2 * np.pi * frequency * step_start + np.pi/2)

            data.ctrl[joint_id_map['TL0']] = -np.pi/6 * np.sin(2 * np.pi * frequency * step_start)
            data.ctrl[joint_id_map['CL0']] = -np.pi/3 * np.sin(2 * np.pi * frequency * step_start + np.pi/2)
            data.ctrl[joint_id_map['FL0']] = -np.pi/4 * np.sin(2 * np.pi * frequency * step_start + np.pi/2)

        
            data.ctrl[joint_id_map['TR1']] = -np.pi/10 * np.sin(2 * np.pi * frequency * step_start)
            data.ctrl[joint_id_map['CR1']] =  np.pi/6 * np.sin(2 * np.pi * frequency * step_start - np.pi/2)
            data.ctrl[joint_id_map['FR1']] =  np.pi/4 * np.sin(2 * np.pi * frequency * step_start - np.pi/2)

            data.ctrl[joint_id_map['TL1']] = np.pi/10 * np.sin(2 * np.pi * frequency * step_start)
            data.ctrl[joint_id_map['CL1']] = -np.pi/6 * np.sin(2 * np.pi * frequency * step_start - np.pi/2)
            data.ctrl[joint_id_map['FL1']] = -np.pi/6 * np.sin(2 * np.pi * frequency * step_start - np.pi/2)

            data.ctrl[joint_id_map['TR2']] = np.pi/6 * np.sin(2 * np.pi * frequency * step_start)
            data.ctrl[joint_id_map['CR2']] = np.pi/3 * np.sin(2 * np.pi * frequency * step_start + np.pi/2)
            data.ctrl[joint_id_map['FR2']] = np.pi/4 * np.sin(2 * np.pi * frequency * step_start + np.pi/2)

            data.ctrl[joint_id_map['TL2']] = -np.pi/6 * np.sin(2 * np.pi * frequency * step_start)
            data.ctrl[joint_id_map['CL2']] = -np.pi/3 * np.sin(2 * np.pi * frequency * step_start + np.pi/2)
            data.ctrl[joint_id_map['FL2']] = -np.pi/4 * np.sin(2 * np.pi * frequency * step_start + np.pi/2)


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
