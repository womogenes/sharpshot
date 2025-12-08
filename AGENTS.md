# AGENTS.md - Reference Implementation Documentation

## Overview
This document contains detailed information about the throwing implementation in `manipulation/` for future agents to reference when implementing the ball-throwing simulation.

---

## Project Goal
Simulate an IIWA robot arm throwing a ball into a target. The implementation should:
1. Pick up a ball from a known location
2. Move to a pre-throw pose
3. Execute a throwing motion (arc trajectory)
4. Release the ball at the optimal point to hit the target

---

## Reference Code Structure (`manipulation/`)

### Core Files
| File | Purpose |
|------|---------|
| `src/throw.py` | Main throwing logic: pickup planning, prethrow pose, throw trajectory |
| `src/trajectory.py` | Trajectory interpolation functions (linear, arc), projectile physics |
| `src/ik.py` | Inverse kinematics: pose-to-joint-angles, Jacobian calculations |
| `src/drake_helpers.py` | Drake utilities: plant setup, gripper controllers, visualization |

### Key Notebooks
| Notebook | Description |
|----------|-------------|
| `iiwa_toss.ipynb` | Basic throwing with pose-based trajectory |
| `iiwa_toss_jointangles.ipynb` | Throwing with joint-angle-based trajectory |
| `iiwa_toss_jointangles_v2.ipynb` | **Most complete**: optimization-based release timing, closed-loop gripper control |

---

## Key Functions and Their Signatures

### Trajectory Interpolation (`trajectory.py`)

```python
def interpolatePosesLinear(T_world_poseA, T_world_poseB, t):
    """
    Linear interpolation between two poses.
    - t: float in [0, 1]
    - Position: straight line
    - Rotation: spherical linear interpolation (slerp)
    Returns: RigidTransform
    """

def interpolatePosesArcMotion(T_world_poseA, T_world_poseB, t):
    """
    Arc motion interpolation for throwing.
    - t: float in [0, 1]
    - Position: quarter-circle arc from -90° to 0°
    - Arc radius = height difference (p_B[2] - p_A[2])
    - phi = arctan2(dy, dx) for XY heading
    - theta = (t - 1) * π/2
    Returns: RigidTransform
    """

def interpolatePosesArcMotion_pdot(T_world_poseA, T_world_poseB, t):
    """
    Derivative of arc motion (velocity at point t).
    Returns: np.array([vx, vy, vz])
    """

def get_launch_speed_required(theta, x, y, g=9.81):
    """
    Projectile physics: required launch speed to hit target.
    - theta: launch angle (radians), must be in (0, π)
    - x: horizontal distance (must be > 0)
    - y: vertical distance (must satisfy y < tan(theta) * x)
    - g: gravity (default 9.81)
    
    Formula:
        time_to_target = sqrt(2/g * (tan(theta)*x - y))
        speed = x / (cos(theta) * time_to_target)
    """

def get_launch_angle_required(approach_angle, x, y):
    """
    Calculate launch angle to hit (x, y) with given approach angle.
    Formula: arctan(2*y/x - tan(approach_angle))
    Note: Does NOT depend on gravity!
    """
```

### Inverse Kinematics (`ik.py`)

```python
def spatial_velocity_jacobian_at_jointangles(plant, context, jointangles, gripper_to_object_dist):
    """
    Compute 6x7 Jacobian at given joint configuration.
    - Returns [dw_x, dw_y, dw_z, dv_x, dv_y, dv_z] for each joint
    - gripper_to_object_dist: offset from gripper frame to object COM
    """

def jointangles_to_pose(plant, context, jointangles):
    """
    Forward kinematics: joint angles -> end-effector pose.
    - jointangles: np.array of shape (7,)
    Returns: RigidTransform
    """

def pose_to_jointangles(T_world_robotPose):
    """
    Inverse kinematics: pose -> joint angles.
    Uses optimization with joint-centering cost.
    - q_nominal = [-1.57, 0.1, 0.0, -1.2, 0.0, 1.6, 0.0, 0.0, 0.0]
    Returns: np.array of shape (9,) [7 iiwa + 2 gripper]
    """

def create_q_knots(pose_lst):
    """
    Batch IK: convert list of poses to joint configurations.
    - Uses previous solution as initial guess for continuity
    Returns: list of joint angle arrays
    """
```

### Throwing Logic (`throw.py`)

```python
def plan_pickup(T_world_robotInitial, T_world_gripperObject, t_goToObj, t_holdObj, t_goToPreobj):
    """
    Plan pickup trajectory.
    Returns: (t_lst, q_knots, total_pickup_time)
    """

def add_go_to_ja_via_jointinterpolation(ja1, ja2, t_start, t_lst, q_knots, time_interval_s, hold_time_s=0, num_samples=100, include_end=False):
    """
    Add joint-angle interpolation segment to trajectory.
    - Constant joint velocity over time interval
    - Optional hold time at end for stabilization
    Returns: (t_lst, q_knots) with new segment appended
    """

def plan_prethrow_pose(T_world_robotInitial, p_world_target, gripper_to_object_dist, throw_height, prethrow_height, prethrow_radius, throw_angle, meshcat=None):
    """
    Calculate prethrow and throw poses based on target direction.
    
    Key calculations:
    - theta = arctan2(target_y, target_x)  # heading to target
    - T_world_prethrow: positioned at (prethrow_radius * cos(theta), prethrow_radius * sin(theta), prethrow_height)
    - T_world_throw: offset from prethrow by throw_radius in same direction
    
    Returns: (T_world_prethrow, T_world_throw)
    """
```

---

## Throwing Motion Phases

### Phase 1: Pickup
1. Move from initial pose to object location
2. Hold for gripper to close
3. Return to initial pose (or waypoint)

### Phase 2: Pre-throw Setup
1. Move to waypoint (to clear obstacles)
2. Move to prethrow pose
3. Hold for stabilization (important for accurate throw)

### Phase 3: Throw Execution
1. Arc motion from prethrow to throw-end pose
2. Release at calculated optimal point
3. Follow-through (optional)

---

## Key Parameters

### Timing Parameters (from `iiwa_toss_jointangles_v2.ipynb`)
```python
t_goToObj = 1.0        # Time to reach object
t_holdObj = 2.0        # Time to hold at object (for gripper close)
t_goToPreobj = 1.0     # Time to return from object
t_goToWaypoint = 2.0   # Time to reach waypoint
t_goToPrethrow = 4.0   # Time to reach prethrow (includes 1s hold)
t_goToThrowEnd = 0.3-0.5  # Throw motion time (calculated from physics)
```

### Geometry Parameters
```python
GRIPPER_TO_OBJECT_COM_DIST = 0.11  # Distance from gripper frame to object COM
prethrow_height = 0.2              # Height of prethrow pose
throw_height = 0.5                 # Height of throw-end pose
prethrow_radius = 0.4              # Horizontal distance from origin
```

### Gripper States
```python
GRIPPER_OPEN = 0.5    # Reference uses 0.5
GRIPPER_CLOSED = 0.0
# Note: template_notebook uses opened=0.107, closed=0.0
```

---

## Joint-Angle Based Throwing (Preferred Method)

From `iiwa_toss_jointangles_v2.ipynb`:

### Prethrow and Throwend Joint Angles
```python
throw_heading = np.arctan2(P_WORLD_TARGET[1], P_WORLD_TARGET[0])
ja1 = throw_heading - np.pi

# These define the throw motion - joints 4 and 6 do most of the work
PRETHROW_JA = np.array([ja1, 0, 0, 1.9, 0, -1.9, 0, 0, 0])
THROWEND_JA = np.array([ja1, 0, 0, 0.4, 0, -0.4, 0, 0, 0])
```

### Release Point Optimization
```python
def throw_objective(inp, g=9.81):
    throw_motion_time, release_frac = inp
    
    release_ja = PRETHROW_JA + release_frac * (THROWEND_JA - PRETHROW_JA)
    
    # Get release pose and velocity via Jacobian
    T_world_releasePose = jointangles_to_pose(release_ja[:7])
    p_release = T_world_releasePose.translation() + rotation @ [0, GRIPPER_TO_OBJECT_COM_DIST, 0]
    
    J_release = spatial_velocity_jacobian_at_jointangles(release_ja[:7], GRIPPER_TO_OBJECT_COM_DIST)[3:6]
    v_release = J_release @ ((THROWEND_JA - PRETHROW_JA) / throw_motion_time)[:7]
    
    # Projectile physics
    x = horizontal_distance_to_target
    y = vertical_distance_to_target
    vx = np.linalg.norm(v_release[:2])
    vy = v_release[2]
    
    tta = x / vx  # time to target
    y_hat = vy * tta - 0.5 * g * tta**2  # predicted height at target
    phi_hat = np.arctan((vy - g * tta) / vx)  # landing angle
    
    # Minimize height error and landing angle constraint
    objective = (y_hat - y)**2 + max(phi_hat - MAX_APPROACH_ANGLE, 0)**2
    
    return objective
```

---

## Gripper Controllers

### V1: Pose-based release (`GripperControllerUsingIiwaState`)
- Releases when gripper reaches target release pose
- Simple but less accurate

### V2: Projectile prediction (`GripperControllerUsingIiwaStateV2`)
- Computes Jacobian in real-time
- Predicts projectile trajectory
- Releases when predicted landing is close to target

### V3: Angle-based release (`GripperControllerUsingIiwaStateV3`)
- Releases when launch angle exceeds threshold
- Most robust in practice
- Uses `launch_angle_thresh = 3 degrees` typically

---

## Variable Naming Conventions

### Reference (`manipulation/`) Style
```python
T_world_robotInitial      # RigidTransform: world frame to robot initial pose
T_world_gripperObject     # RigidTransform: world frame to gripper at object
p_world_target            # np.array: target position in world frame
PRETHROW_JA               # np.array: prethrow joint angles (uppercase for constants)
throw_heading             # float: heading angle to target
gripper_to_object_dist    # float: distance from gripper to object
```

### Template Notebook Style (to follow)
```python
X_WGinitial              # RigidTransform: world to gripper initial (X_ prefix)
X_WG_pick                # RigidTransform: world to gripper at pick
X_WG_prepick             # RigidTransform: world to gripper approach
p_world_target           # np.array: target position
T_world_prethrow         # RigidTransform: prethrow pose (T_ prefix also used)
```

### Key Differences to Note
1. Template uses `X_` prefix for RigidTransforms, reference uses `T_`
2. Template uses snake_case consistently
3. Template uses `opened/closed` for gripper, reference uses `GRIPPER_OPEN/GRIPPER_CLOSED`
4. Template passes `plant` to IK functions, reference creates plant internally

---

## Important Implementation Details

### Arc Motion Geometry
```
Arc is a quarter circle in 3D:
- Start: prethrow pose (lower, behind)
- End: throw pose (higher, forward)
- Arc radius = height difference
- Arc sweeps from -90° to 0° in the vertical plane aligned with target direction
```

### Jacobian Usage
```python
# Get spatial velocity Jacobian (6x7)
J_G = plant.CalcJacobianSpatialVelocity(
    context,
    JacobianWrtVariable.kQDot,
    gripper_frame,
    [0, gripper_to_object_dist, 0],  # Point offset from gripper
    world_frame,
    world_frame
)
# J_G[0:3] = angular velocity components
# J_G[3:6] = linear velocity components
```

### Trajectory Creation
```python
# Joint trajectory with smooth interpolation
q_traj = PiecewisePolynomial.CubicShapePreserving(t_lst, q_knots_array)

# Gripper trajectory with linear holds
g_traj = PiecewisePolynomial.FirstOrderHold(gripper_times, gripper_positions)
```

---

## Design Decisions (Resolved)

1. **Target specification**: Hard-coded for MVP. Use something like `p_world_target = np.array([0.5, -1.2, 0.5])`.

2. **Object type**: Ball only (`package://drake_models/manipulation_station/sphere.sdf`).

3. **Gripper control**: Open-loop (pre-planned release time based on fixed release fraction).

4. **Release fraction**: Fixed value (e.g., `release_frac = 0.5` or `0.7`). This determines where along the throw motion the gripper opens:
   - `release_ja = PRETHROW_JA + release_frac * (THROWEND_JA - PRETHROW_JA)`
   - No scipy optimization needed for MVP.

5. **Simulation timestep**: Use `time_step=1e-3` (default).

6. **Waypoint**: NOT needed for current setup. The template has ball at `[0, -0.5, 0.02]` with just a table—no obstacles between pickup and throw positions. Go directly from pickup to prethrow.

---

## Files to Create/Modify

Based on template_notebook.ipynb conventions:

1. **`src/throw_helpers.py`** - Already exists, may need:
   - `spatial_velocity_jacobian_at_jointangles()`
   - `jointangles_to_pose()`
   - `interpolate_joint_angle()`
   - Release optimization function

2. **Main notebook** - Should include:
   - Scenario YAML with ball and target
   - Pickup trajectory generation
   - Throw trajectory generation (joint-angle based)
   - Gripper trajectory
   - Simulation execution

---

## Code Snippets for Reference

### Prethrow/Throw Pose Calculation
```python
theta = np.arctan2(p_world_target[1], p_world_target[0])

T_world_prethrow = RigidTransform(
    p=np.array([
        prethrow_radius * np.cos(theta),
        prethrow_radius * np.sin(theta),
        prethrow_height
    ]),
    R=RotationMatrix.MakeXRotation(-np.pi/2).multiply(
        RotationMatrix.MakeYRotation(theta - np.pi/2)
    )
)

throw_radius = throw_height - prethrow_height
T_world_throw = RigidTransform(
    p=T_world_prethrow.translation() + np.array([
        throw_radius * np.cos(theta),
        throw_radius * np.sin(theta),
        throw_height - prethrow_height
    ]),
    R=RotationMatrix.MakeXRotation(-np.pi/2).multiply(
        RotationMatrix.MakeYRotation(theta - np.pi/2).multiply(
            RotationMatrix.MakeXRotation(-np.pi/2)
        )
    )
)
```

### Gripper Trajectory
```python
gripper_times = np.array([0, t_goToObj, t_holdObj, t_goToPreobj, t_goToWaypoint, t_goToPrethrow, t_release, t_end])
gripper_cumulative = np.cumsum(gripper_times)
gripper_positions = np.array([OPEN, OPEN, CLOSED, CLOSED, CLOSED, CLOSED, CLOSED, OPEN]).reshape(1, -1)
g_traj = PiecewisePolynomial.FirstOrderHold(gripper_cumulative, gripper_positions)
```

---

## Current Implementation Status (Dec 8, 2025)

### Active Notebook: `src/arc_trajectory_planning.ipynb`

**Approach**: Custom `AngleBasedGripperController` using `MakeHardwareStation` API.

### Implementation Steps Completed

#### Step 1: Define throw joint angles
```python
throw_heading = np.arctan2(P_TARGET[1], P_TARGET[0])  # -90° for target at [0, -1, 0]
ja1 = throw_heading  # Base joint points toward target

PRETHROW_JA = np.array([ja1, 0, 0, 1.9, 0, -1.9, np.pi])  # Wound up
THROWEND_JA = np.array([ja1, 0, 0, 0.4, 0, -0.4, np.pi])  # Extended
```

#### Step 2: Verify positions with FK
- Visualize triads at postpick, prethrow, throwend
- Run trajectory through all 3 positions to confirm alignment

#### Step 3: Compute planned launch angle
```python
RELEASE_FRAC = 0.5  # Release halfway through throw
THROW_DURATION = 0.4  # Fast throw motion
GRIPPER_TO_OBJECT_DIST = 0.12

release_ja = PRETHROW_JA + RELEASE_FRAC * (THROWEND_JA - PRETHROW_JA)
qdot_throw = (THROWEND_JA - PRETHROW_JA) / THROW_DURATION

# Jacobian at release point
J_G = plant.CalcJacobianTranslationalVelocity(
    context, JacobianWrtVariable.kQDot,
    gripper_body.body_frame(),
    [0, GRIPPER_TO_OBJECT_DIST, 0],  # Offset to ball COM
    plant.world_frame(), plant.world_frame()
)
v_release = J_iiwa @ qdot_throw
planned_launch_angle = np.arctan2(v_release[2], np.linalg.norm(v_release[:2]))
```

#### Step 4: AngleBasedGripperController
Custom LeafSystem adapted for `MakeHardwareStation`:
- **Inputs**: `iiwa_position` (7), `iiwa_velocity` (7)
- **Output**: `wsg_position` (1)
- **FSM states**: pickup → prethrow → release
- **Release condition**: `current_angle >= planned_launch_angle - threshold`

#### Step 5: Full simulation wiring
```python
builder = DiagramBuilder()
station = MakeHardwareStation(scenario, meshcat=meshcat)
builder.AddSystem(station)

q_source = builder.AddSystem(TrajectorySource(q_full_traj))
builder.Connect(q_source.get_output_port(), station.GetInputPort("iiwa.position"))

gripper_controller = AngleBasedGripperController(...)
builder.AddSystem(gripper_controller)
builder.Connect(station.GetOutputPort("iiwa.position_measured"), gripper_controller.GetInputPort("iiwa_position"))
builder.Connect(station.GetOutputPort("iiwa.velocity_estimated"), gripper_controller.GetInputPort("iiwa_velocity"))
builder.Connect(gripper_controller.GetOutputPort("wsg_position"), station.GetInputPort("wsg.position"))
```

### MakeHardwareStation Port Names (vs old ManipulationStation)
| New API | Old API |
|---------|---------|
| `iiwa.position` | `iiwa_position` |
| `iiwa.position_measured` | `iiwa_position_measured` |
| `iiwa.velocity_estimated` | `iiwa_velocity_estimated` |
| `wsg.position` | `wsg_position` |

### Key Parameters
```python
P_TARGET = np.array([0, -1.0, 0.0])
BALL_POS = [0, -0.5, 0.025]
GRIPPER_OPEN = 0.107
GRIPPER_CLOSED = 0.0
GRIPPER_TO_OBJECT_DIST = 0.12
RELEASE_FRAC = 0.5
THROW_DURATION = 0.4
launch_angle_thresh = np.radians(3)
```

### Next Steps
1. Test full simulation - verify ball releases and flies toward target
2. Tune parameters if needed (RELEASE_FRAC, THROW_DURATION, joint angles)
3. Add projectile prediction to verify landing position
