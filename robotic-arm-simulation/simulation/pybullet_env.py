import pybullet as p
import pybullet_data
import numpy as np 

def sim_env(gui=True):
    # --- Connect to PyBullet ---
    if gui:
        physicsClient = p.connect(p.GUI)
    else:
        physicsClient = p.connect(p.DIRECT)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1./240.)

    # --- Plane ---
    plane_id = p.loadURDF("plane.urdf")

    # --- Custom Table with Legs ---
    table_thickness = 0.05
    table_size = [1.0, 1.0, table_thickness]  # Half extents (x, y, z)
    table_height = 0.7                        # Height from ground to tabletop surface
    table_top_z = table_height
    table_color = [0.8, 0.6, 0.4, 1]          # Light wood color

    # Create tabletop
    tabletop_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=table_size, rgbaColor=table_color)
    tabletop_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=table_size)
    table_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=tabletop_col,
        baseVisualShapeIndex=tabletop_vis,
        basePosition=[0.8, 0, table_top_z]
    )

    # --- Add 4 Legs ---
    leg_thickness = 0.05
    leg_height = table_height
    leg_half = [leg_thickness, leg_thickness, leg_height / 2]
    leg_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=leg_half, rgbaColor=table_color)
    leg_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=leg_half)

    # Leg positions (relative to table center)
    leg_offsets = [
        [ table_size[0] - leg_thickness,  table_size[1] - leg_thickness, leg_height / 2],
        [ table_size[0] - leg_thickness, -table_size[1] + leg_thickness, leg_height / 2],
        [-table_size[0] + leg_thickness,  table_size[1] - leg_thickness, leg_height / 2],
        [-table_size[0] + leg_thickness, -table_size[1] + leg_thickness, leg_height / 2],
    ]

    for offset in leg_offsets:
        leg_pos = [0.8 + offset[0], 0 + offset[1], offset[2]]
        p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=leg_col,
            baseVisualShapeIndex=leg_vis,
            basePosition=leg_pos
        )

    # --- Robot on Table ---
    robot_base_height = table_top_z + table_thickness
    robot_start_pos = [0.4, 0.0, robot_base_height]
    robot_start_ori = p.getQuaternionFromEuler([0, 0, 2*(np.pi)])
    robot_id = p.loadURDF("franka_panda/panda.urdf", robot_start_pos, robot_start_ori, useFixedBase=True)

    arm_joints = [0, 1, 2, 3, 4, 5, 6]
    gripper_joints = [9, 10]
    ee_link_index = 11

    # --- Table Surface Z ---
    top_z = table_top_z + table_thickness

    # --- Create Two Open Boxes (Trays) ---
    def create_open_box(center, color):
        half_size = [0.15, 0.15, 0.02]  # bottom area
        wall_thickness = 0.01
        wall_height = 0.04

        # Base
        base_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half_size, rgbaColor=color)
        base_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_size)
        base_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=base_col,
                                    baseVisualShapeIndex=base_vis, basePosition=center)

        # Walls (4 sides)
        wall_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[half_size[0], wall_thickness, wall_height])
        wall_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[half_size[0], wall_thickness, wall_height], rgbaColor=color)

        # Front & Back
        p.createMultiBody(0, wall_shape, wall_vis, [center[0], center[1] + half_size[1] + wall_thickness, center[2] + wall_height])
        p.createMultiBody(0, wall_shape, wall_vis, [center[0], center[1] - half_size[1] - wall_thickness, center[2] + wall_height])

        # Left & Right
        wall_shape_side = p.createCollisionShape(p.GEOM_BOX, halfExtents=[wall_thickness, half_size[1], wall_height])
        wall_vis_side = p.createVisualShape(p.GEOM_BOX, halfExtents=[wall_thickness, half_size[1], wall_height], rgbaColor=color)
        p.createMultiBody(0, wall_shape_side, wall_vis_side, [center[0] + half_size[0] + wall_thickness, center[1], center[2] + wall_height])
        p.createMultiBody(0, wall_shape_side, wall_vis_side, [center[0] - half_size[0] - wall_thickness, center[1], center[2] + wall_height])

        return base_id

    # Boxes farther away from the robot
    red_box_center = [1.1, 0.4, top_z + 0.02]
    blue_box_center = [1.1, -0.4, top_z + 0.02]
    red_box_id = create_open_box(red_box_center, [1, 0, 0, 1])
    blue_box_id = create_open_box(blue_box_center, [0, 0, 1, 1])

    # --- Objects ---
    # Bottle inside the blue box
    bottle_z = top_z + 0.02 + 0.025  # base of blue box + offset for bottle
    bottle_id = p.loadURDF("cube.urdf", [blue_box_center[0], blue_box_center[1], bottle_z], globalScaling=0.05)

    # Ball inside the red box
    ball_z = top_z + 0.02 + 0.025
    ball_id = p.loadURDF("sphere_small.urdf", [red_box_center[0], red_box_center[1], ball_z])

    # --- Camera Setup ---
    cam_width, cam_height = 640, 480
    cam_fov = 60
    cam_near_val = 0.01
    cam_far_val = 10.0
    cam_eye_pos = [1.8, 0.0, 1.3]
    cam_target_pos = [0.8, 0.0, 0.6]
    cam_up_vector = [0, 0, 1]

    view_matrix = p.computeViewMatrix(cam_eye_pos, cam_target_pos, cam_up_vector)
    projection_matrix = p.computeProjectionMatrixFOV(cam_fov, cam_width / cam_height, cam_near_val, cam_far_val)

    env = {
        "robot": robot_id,
        "plane": plane_id,
        "table": table_id,
        "bottle": bottle_id,
        "ball": ball_id,
        "red_box": red_box_id,
        "blue_box": blue_box_id,
        "arm_joints": arm_joints,
        "gripper_joints": gripper_joints,
        "ee_link": ee_link_index,
        "camera_params": {
            "width": cam_width,
            "height": cam_height,
            "view_matrix": view_matrix,
            "projection_matrix": projection_matrix
        }
    }
    return env
