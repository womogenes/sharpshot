"""
Helper functions for arc-based throwing motion.
Adapted from manipulation/ repository.
"""
import numpy as np
from pydrake.all import (
    PiecewiseQuaternionSlerp,
    RigidTransform,
    RotationMatrix,
    InverseKinematics,
    Solve,
    Quaternion,
)


def orientation_slerp(r_A, r_B):
    """Spherical linear interpolation between two rotations."""
    traj = PiecewiseQuaternionSlerp()
    traj.Append(0.0, r_A)
    traj.Append(1.0, r_B)
    return traj


def interpolate_poses_linear(T_world_poseA, T_world_poseB, t):
    """
    Linearly interpolate between two poses.
    t should be between 0 and 1.
    """
    p_A, r_A = T_world_poseA.translation(), T_world_poseA.rotation()
    p_B, r_B = T_world_poseB.translation(), T_world_poseB.rotation()

    # Rotation: spherical linear interpolation
    r_slerp = orientation_slerp(r_A, r_B)
    quat_array = r_slerp.value(t)  # Returns numpy array [w, x, y, z]
    # Drake's Quaternion expects [w, x, y, z] format
    quat = Quaternion(quat_array.flatten())
    r_curr = RotationMatrix(quat)

    # Position: straight line
    p_curr = p_A + t * (p_B - p_A)

    return RigidTransform(r_curr, p_curr)


def interpolate_poses_arc_motion(T_world_poseA, T_world_poseB, t):
    """
    Interpolate between two poses using arc motion.
    The position follows a circular arc, rotation uses slerp.
    t should be between 0 and 1.
    """
    p_A, r_A = T_world_poseA.translation(), T_world_poseA.rotation()
    p_B, r_B = T_world_poseB.translation(), T_world_poseB.rotation()

    # Rotation: spherical linear interpolation
    r_slerp = orientation_slerp(r_A, r_B)
    quat_array = r_slerp.value(t)  # Returns numpy array [w, x, y, z]
    quat = Quaternion(quat_array.flatten())
    r_curr = RotationMatrix(quat)

    # Position: arc motion (quarter circle from -90° to 0°)
    arc_radius = p_B[2] - p_A[2]  # Height difference
    phi = np.arctan2(p_B[1] - p_A[1], p_B[0] - p_A[0])  # XY direction
    theta = (t - 1) * np.pi / 2  # -90° to 0°
    
    p_curr = p_A + np.array([
        arc_radius * np.cos(theta) * np.cos(phi),
        arc_radius * np.cos(theta) * np.sin(phi),
        arc_radius * np.sin(theta) + arc_radius
    ])

    return RigidTransform(r_curr, p_curr)


def pose_to_jointangles(plant, T_world_target, q_nominal=None):
    """
    Solve inverse kinematics to find joint angles for a target pose.
    
    Args:
        plant: MultibodyPlant with the robot
        T_world_target: Target pose for the gripper
        q_nominal: Nominal joint configuration for initial guess
    
    Returns:
        Joint angles (7-DOF for iiwa)
    """
    if q_nominal is None:
        q_nominal = np.array([-1.57, 0.1, 0.0, -1.2, 0.0, 1.6, 0.0])
    
    plant_context = plant.CreateDefaultContext()
    world_frame = plant.world_frame()
    gripper_frame = plant.GetBodyByName("body").body_frame()
    
    ik = InverseKinematics(plant, plant_context)
    q_variables = ik.q()
    prog = ik.prog()
    
    # Get iiwa indices (assuming iiwa is the only actuated model)
    iiwa_instance = plant.GetModelInstanceByName("iiwa")
    iiwa_start = plant.GetJointByName("iiwa_joint_1").velocity_start()
    iiwa_end = plant.GetJointByName("iiwa_joint_7").velocity_start()
    q_iiwa = q_variables[iiwa_start:iiwa_end+1]
    
    # Position and orientation constraints
    p_WG = T_world_target.translation()
    R_WG = T_world_target.rotation()
    
    ik.AddPositionConstraint(
        frameA=world_frame,
        frameB=gripper_frame,
        p_BQ=np.zeros(3),
        p_AQ_lower=p_WG,
        p_AQ_upper=p_WG
    )
    
    ik.AddOrientationConstraint(
        frameAbar=world_frame,
        R_AbarA=R_WG,
        frameBbar=gripper_frame,
        R_BbarB=RotationMatrix(),
        theta_bound=0.0
    )
    
    # Initial guess and cost
    prog.SetInitialGuess(q_iiwa, q_nominal)
    diff = q_iiwa - q_nominal
    prog.AddCost(diff.dot(diff))
    
    result = Solve(prog)
    if not result.is_success():
        raise RuntimeError(f"IK failed to find solution for pose: {T_world_target}")
    
    return result.GetSolution(q_iiwa)


def create_q_knots(plant, pose_lst):
    """
    Convert list of end-effector poses to joint positions using IK.
    Uses previous solution as initial guess for smooth trajectories.
    
    Args:
        plant: MultibodyPlant with the robot
        pose_lst: List of RigidTransform poses
    
    Returns:
        List of joint angle arrays (7-DOF)
    """
    q_knots = []
    q_nominal = np.array([-1.57, 0.1, 0.0, -1.2, 0.0, 1.6, 0.0])
    
    for i, pose in enumerate(pose_lst):
        if i == 0:
            q_prev = q_nominal
        else:
            q_prev = q_knots[i - 1]
        
        try:
            q = pose_to_jointangles(plant, pose, q_nominal=q_prev)
            q_knots.append(q)
        except RuntimeError as e:
            print(f"IK failed at pose {i}/{len(pose_lst)}: {e}")
            break
    
    return q_knots


def joint_angles_to_pose(plant, joint_angles):
    """
    Forward kinematics: joint angles -> end-effector pose.
    
    Args:
        plant: MultibodyPlant with the robot
        jointangles: np.array of shape (7,) for iiwa joints
    
    Returns:
        RigidTransform of gripper body frame in world
    """
    plant_context = plant.CreateDefaultContext()
    iiwa = plant.GetModelInstanceByName("iiwa")
    plant.SetPositions(plant_context, iiwa, joint_angles)
    gripper_body = plant.GetBodyByName("body")
    return plant.EvalBodyPoseInWorld(plant_context, gripper_body)


def get_launch_speed_required(theta, x, y, g=9.81):
    """
    Calculate required launch speed for projectile motion.
    
    Args:
        theta: Launch angle (radians)
        x: Horizontal distance to target
        y: Vertical distance to target
        g: Gravity (m/s^2)
    
    Returns:
        Required launch speed (m/s)
    """
    assert 0 < theta < np.pi, "Launch angle must be between 0 and π"
    assert 0 < x, "Horizontal distance must be positive"
    assert y < np.tan(theta) * x, "Target must be reachable with given angle"
    
    time_to_target = np.sqrt(2.0 / g * (np.tan(theta) * x - y))
    speed_required = x / (np.cos(theta) * time_to_target)
    
    return speed_required
