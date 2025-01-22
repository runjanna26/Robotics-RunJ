import time
import mujoco
import mujoco.viewer
import numpy as np  # For sine wave calculations

# Load the model and data
model = mujoco.MjModel.from_xml_path("./swerve_wheel_robot.xml")
data = mujoco.MjData(model)

# Extract the joint names
joint_names = []
for joint_id in range(model.njnt):  # model.njnt gives the number of joints
    # Address of the joint name
    name_start = model.name_jntadr[joint_id]
    # Extract the name (null-terminated string)
    joint_name = model.names[name_start:].decode('utf-8').split('\x00', 1)[0]
    joint_names.append(joint_name)

# Print all joint names
# print("Joint names:", joint_names)


joint_id_map = {}
for joint_name in joint_names:
    try:
        joint_id_map[joint_name] = (model.joint(name=joint_name).qposadr)[0] - 6
    except Exception as e:
        print(f"Error: Could not find joint {joint_name}. Exception: {e}")

# Print the resulting dictionary
print("Joint ID Map:", joint_id_map)


# Get the body IDs for the four magnet wheels
wheel_geoms = [
    mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'magnet_wheel_1'),
    mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'magnet_wheel_2'),
    mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'magnet_wheel_3'),
    mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'magnet_wheel_4')
]

# Get the ID for the wall geom (or surface you want to apply adhesive force to)
wall_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'wall')



with mujoco.viewer.launch_passive(model, data) as viewer:
    # Close the viewer automatically after 30 wall-seconds.
    start = time.time()


    while viewer.is_running() : # and time.time() - start < 30
        step_start = time.time()





        if step_start - start < 1.0:
            for joint_name in joint_names:
                data.ctrl[joint_id_map[joint_name]] = 0.0
        elif step_start - start > 1.0:
            desired_position = 0.0 
            data.ctrl[joint_id_map['direction_joint_1']] = 0
            data.ctrl[joint_id_map['direction_joint_2']] = 0
            data.ctrl[joint_id_map['direction_joint_3']] = 0
            data.ctrl[joint_id_map['direction_joint_4']] = 0

            data.ctrl[joint_id_map['driven_joint_1']] = 2
            data.ctrl[joint_id_map['driven_joint_2']] = 2
            data.ctrl[joint_id_map['driven_joint_3']] = 2
            data.ctrl[joint_id_map['driven_joint_4']] = 2

            for i in range(data.ncon):
                contact = data.contact[i]
                
                # Check if the contact involves any of the magnet wheel geoms
                for wheel_geom in wheel_geoms:
                    if contact.geom1 == wheel_geom or contact.geom2 == wheel_geom:
                        # Check if the contact is with the wall (you may need to adjust the logic based on your model)
                        if contact.geom1 == wall_geom_id or contact.geom2 == wall_geom_id:
                            # Apply adhesive force to the magnet wheel if it's in contact with the wall
                            data.ctrl[wheel_geom] = 1  # Apply full adhesion (adjust if needed)
                        else:
                            # No adhesion if the contact is not with the wall (e.g., contact with the ground)
                            data.ctrl[wheel_geom] = 0  # No adhesion


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
