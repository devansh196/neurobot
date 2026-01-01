from .pybullet_env import sim_env
import pybullet as p
import numpy as np
import time


SIM_SPEED = 0.6 # Lower = slower visualization; 1.0 = realtime
FPS = 60         # Visual smoothness

def slow_step(steps, speed_factor=1.0):
    """Step simulation with adjustable visualization speed."""
    for _ in range(steps):
        p.stepSimulation()
        time.sleep((1.0 / FPS) / (SIM_SPEED * speed_factor))



def pick_object(robot_id, object_id, ee_link_index, arm_joints, gripper_joints, object_pos, steps=240):
    """
    Robust pick function for Franka Panda:
    - Approaches above the object
    - Aligns wrist orientation correctly (facing down + yaw)
    - Descends smoothly
    - Grasps and lifts
    """

    # ---- Compute end-effector orientation ----
    # Franka default: gripper opens along Y, forward is +X
    # To face downward, pitch = -π/2
    # Add yaw rotation so the gripper can face the object naturally
    yaw = np.arctan2(object_pos[1], object_pos[0])  # rotation in table plane
    target_ori = p.getQuaternionFromEuler([0, -np.pi, yaw])

    # ---- Pre-grasp: move above object ----
    pre_grasp_height = 0.5 # height above object
    pre_grasp = [object_pos[0], object_pos[1], object_pos[2] + pre_grasp_height]
    joint_angles = p.calculateInverseKinematics(robot_id, ee_link_index, pre_grasp, target_ori)

    for j, a in zip(arm_joints, joint_angles):
        p.setJointMotorControl2(robot_id, j, p.POSITION_CONTROL, a)
    slow_step(steps, speed_factor=1.0)

    # ---- Open gripper ----
    for gj in gripper_joints:
        p.setJointMotorControl2(robot_id, gj, p.POSITION_CONTROL, targetPosition=0.08, force=200)
    slow_step(60, speed_factor=1.2)

    # ---- Descend to object ----
    grasp_height = object_pos[2]
    current_z = pre_grasp[2]
    descent_steps = 60
    dz = (current_z - grasp_height) / descent_steps

    for i in range(descent_steps):
        pos = [object_pos[0], object_pos[1], current_z - dz * (i + 1)]
        joint_angles = p.calculateInverseKinematics(robot_id, ee_link_index, pos, target_ori)
        for j, a in zip(arm_joints, joint_angles):
            p.setJointMotorControl2(robot_id, j, p.POSITION_CONTROL, a)
        slow_step(1, speed_factor=1.0)

    
    # ---- Attach object (simulate grasp) ----
    cid = p.createConstraint(
        parentBodyUniqueId=robot_id,
        parentLinkIndex=ee_link_index,
        childBodyUniqueId=object_id,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=[0, 0, 0],
        childFramePosition=[0, 0, 0]
    )
    slow_step(10, speed_factor=3.0)

    # ---- Close gripper ----
    for gj in gripper_joints:
        p.setJointMotorControl2(robot_id, gj, p.POSITION_CONTROL, targetPosition=0.0, force=500)
    slow_step(80, speed_factor=1.2)


    # ---- Lift ----
    lift_height = 0.2
    lift_pos = [object_pos[0], object_pos[1], object_pos[2] + lift_height]
    joint_angles = p.calculateInverseKinematics(robot_id, ee_link_index, lift_pos, target_ori)

    for j, a in zip(arm_joints, joint_angles):
        p.setJointMotorControl2(robot_id, j, p.POSITION_CONTROL, a)
    slow_step(steps, speed_factor=1.5)

    return cid



def place_object(robot_id, ee_link_index, arm_joints, gripper_joints, target_pos, constraint_id=None, steps=240):
    """Place object, fully detach, clear vertically, and return arm to home."""
    target_ori = p.getQuaternionFromEuler([0, -np.pi, 0])
    
    # ----------------------------------------------------
    # NEW: Define the final Z-position to ensure detachment
    place_surface_clearance = 0.1 # 5 mm clearance
    target_z = target_pos[2] + place_surface_clearance 
    # ----------------------------------------------------

    # Pre-place above target
    pre_place = [target_pos[0], target_pos[1], target_pos[2] + 0.15]
    joint_angles = p.calculateInverseKinematics(robot_id, ee_link_index, pre_place, target_ori)
    for j, a in zip(arm_joints, joint_angles):
        p.setJointMotorControl2(robot_id, j, p.POSITION_CONTROL, a)
    slow_step(steps, speed_factor=1.0)

    # Descend slowly to the clearance height (5mm above the surface)
    descent_steps = 60
    dz = (pre_place[2] - target_z) / descent_steps # Calculate step based on new target_z
    
    for i in range(descent_steps):
        pos = [target_pos[0], target_pos[1], pre_place[2] - dz * (i + 1)]
        joint_angles = p.calculateInverseKinematics(robot_id, ee_link_index, pos, target_ori)
        for j, a in zip(arm_joints, joint_angles):
            p.setJointMotorControl2(robot_id, j, p.POSITION_CONTROL, a)
        slow_step(1, speed_factor=1.2)
        
    # The arm is now holding the object 50mm above the surface.

    #  Fully release object & settle (FIXED)
    if constraint_id is not None: # <--- **FIXED THE MISSING IF**
        for gj in gripper_joints:
            p.setJointMotorControl2(robot_id, gj, p.POSITION_CONTROL, targetPosition=0.08, force=500) 
        
        p.removeConstraint(constraint_id)  # Detach constraint
        
        # Pause to let the object freefall and settle onto the surface
        slow_step(steps=120, speed_factor=1.0) # Increased steps for better settling

    # Lift straight up to clear object
    clearance_lift = 0.5 
    current_pos_info = p.getLinkState(robot_id, ee_link_index)[0] 
    lift_off_pos = [current_pos_info[0], current_pos_info[1], current_pos_info[2] + clearance_lift]

    joint_angles = p.calculateInverseKinematics(robot_id, ee_link_index, lift_off_pos, target_ori)
    for j, a in zip(arm_joints, joint_angles):
        p.setJointMotorControl2(robot_id, j, p.POSITION_CONTROL, a)
    slow_step(60, speed_factor=1.5) 

    # Move to neutral / home position
    home_pos = [0.5, 0, 1]  
    home_joint_angles = p.calculateInverseKinematics(robot_id, ee_link_index, home_pos, target_ori)
    for j, a in zip(arm_joints, home_joint_angles):
        p.setJointMotorControl2(robot_id, j, p.POSITION_CONTROL, a)
    slow_step(steps, speed_factor=1.2)



def run_pick_and_place(object_id, object_pos, place_target):
    """Run full smooth pick-and-place simulation."""
    env = sim_env(gui=True)
    robot_id = env["robot"]
    ee_link_index = env["ee_link"]
    arm_joints = env["arm_joints"]
    gripper_joints = env["gripper_joints"]

    # Pick phase
    cid = pick_object(robot_id, object_id, ee_link_index, arm_joints, gripper_joints, object_pos)

    # Place phase
    place_object(robot_id, ee_link_index, arm_joints, gripper_joints, place_target, constraint_id=cid)

    return env


def gesture_move(robot_id, speed=0.8):
    """
    BIG, CLEAR, HUMAN-LIKE arm gestures:
    - UP (Raise arm from shoulder)
    - DOWN
    - LEFT
    - RIGHT

    Movement is joint-based → NO IK → much more visible.
    """

    # Panda primary arm joints
    SHOULDER_PAN = 0   # rotates left/right
    SHOULDER_LIFT = 1  # raises/lowers arm
    ELBOW = 2

    # Current joint angles
    j0 = p.getJointState(robot_id, SHOULDER_PAN)[0]
    j1 = p.getJointState(robot_id, SHOULDER_LIFT)[0]
    j2 = p.getJointState(robot_id, ELBOW)[0]

    print("\nCommands: upar | nichay | baye | daye | exit\n")

    while True:
        cmd = input("Gesture command: ").strip().lower()

        if cmd == "exit":
            break

        # --- UP (arm raise) ---
        if cmd == "upar":
            target_j1 = j1 - np.deg2rad(35)      # raise shoulder
            target_j2 = j2 + np.deg2rad(20)      # bend elbow a bit

            p.setJointMotorControl2(robot_id, SHOULDER_LIFT, p.POSITION_CONTROL, target_j1, maxVelocity=speed)
            p.setJointMotorControl2(robot_id, ELBOW, p.POSITION_CONTROL, target_j2, maxVelocity=speed)

        # --- DOWN ---
        elif cmd == "nichay":
            target_j1 = j1 + np.deg2rad(35)
            target_j2 = j2 - np.deg2rad(20)

            p.setJointMotorControl2(robot_id, SHOULDER_LIFT, p.POSITION_CONTROL, target_j1, maxVelocity=speed)
            p.setJointMotorControl2(robot_id, ELBOW, p.POSITION_CONTROL, target_j2, maxVelocity=speed)

        # --- LEFT ---
        elif cmd == "baye":
            target_j0 = j0 + np.deg2rad(40)

            p.setJointMotorControl2(robot_id, SHOULDER_PAN, p.POSITION_CONTROL, target_j0, maxVelocity=speed)

        # --- RIGHT ---
        elif cmd == "daye":
            target_j0 = j0 - np.deg2rad(40)

            p.setJointMotorControl2(robot_id, SHOULDER_PAN, p.POSITION_CONTROL, target_j0, maxVelocity=speed)

        else:
            print("Unknown command")
            continue

        # apply movement
        for i in range(120):
            p.stepSimulation()
            time.sleep(1/240)

        # update current angles
        j0 = p.getJointState(robot_id, SHOULDER_PAN)[0]
        j1 = p.getJointState(robot_id, SHOULDER_LIFT)[0]
        j2 = p.getJointState(robot_id, ELBOW)[0]


def move_arm_direction(robot_id, ee_link_index, arm_joints, direction, angle_deg=35, steps=120):
    """
    Programmatic, big-gesture movement (callable from Flask).
    - direction: "upar" | "nichay" | "daye" | "baye"
    - Uses joint-based motion so movement is clearly visible.
    - arm_joints: list of the 7 arm joint indices (from env["arm_joints"])
    - ee_link_index is unused here but kept for compatibility
    """

    # Map Panda logical joints (if arm_joints = [0..6] this maps directly)
    # We'll use the first few joints for big gestures:
    # base (pan) -> arm_joints[0]
    # shoulder (lift) -> arm_joints[1]
    # elbow -> arm_joints[2]
    if len(arm_joints) < 3:
        print("move_arm_direction: arm_joints must contain at least 3 indices")
        return False

    base_j = arm_joints[0]
    shoulder_j = arm_joints[1]
    elbow_j = arm_joints[2]

    # Read current joint angles
    j_base = p.getJointState(robot_id, base_j)[0]
    j_shoulder = p.getJointState(robot_id, shoulder_j)[0]
    j_elbow = p.getJointState(robot_id, elbow_j)[0]

    # Convert degrees to radians
    ang = np.deg2rad(angle_deg)

    # Choose targets (shoulder-first human-like motion)
    if direction == "upar":
        target_shoulder = j_shoulder - ang          # raise shoulder (negative for Panda)
        target_elbow = j_elbow + ang * 0.5          # small elbow compensation
        target_base = j_base
    elif direction == "nichay":
        target_shoulder = j_shoulder + ang
        target_elbow = j_elbow - ang * 0.5
        target_base = j_base
    elif direction == "baye":   # left
        target_base = j_base + ang
        target_shoulder = j_shoulder
        target_elbow = j_elbow
    elif direction == "daye":   # right
        target_base = j_base - ang
        target_shoulder = j_shoulder
        target_elbow = j_elbow
    else:
        print("Unknown direction:", direction)
        return False

    # Create a smooth interpolation from current to target over `steps`
    for s in range(1, steps + 1):
        alpha = s / float(steps)
        set_base = j_base + (target_base - j_base) * alpha
        set_shoulder = j_shoulder + (target_shoulder - j_shoulder) * alpha
        set_elbow = j_elbow + (target_elbow - j_elbow) * alpha

        # Send POSITION_CONTROL for these joints; keep the other joints fixed at current positions
        p.setJointMotorControl2(robot_id, base_j, p.POSITION_CONTROL, targetPosition=set_base, maxVelocity=1.5, force=200)
        p.setJointMotorControl2(robot_id, shoulder_j, p.POSITION_CONTROL, targetPosition=set_shoulder, maxVelocity=1.5, force=200)
        p.setJointMotorControl2(robot_id, elbow_j, p.POSITION_CONTROL, targetPosition=set_elbow, maxVelocity=1.5, force=200)

        # Optionally keep other joints' targets at their current positions to avoid sudden flips
        # (This loop leaves them uncontrolled; PyBullet will hold previous commands.)
        p.stepSimulation()
        time.sleep(1.0 / 240.0)

    # final settle steps for visibility
    for _ in range(30):
        p.stepSimulation()
        time.sleep(1.0 / 240.0)

    return True
