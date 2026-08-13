# Task 1 inverse-kinematics and trajectory audit

## Result

The simulation and hardware CSV files were already sending the same arm joint
coordinates. The maximum arm-coordinate difference between the two 4,461-sample
files was exactly `0 rad`. Therefore, the large pose mismatch was not caused by a
separate hardware trajectory. It was caused primarily by the geometric model used
to calculate those coordinates.

## Root causes

### 1. CAD mesh offsets were incorrectly part of the kinematic chain

The previous `lowerarm_joint` translation was:

```text
[0.170, 0.0189, -0.0805] m
```

Its length is 189.04 mm. The supplied robot drawing gives a physical upper-joint
to lower-joint axis distance of 170 mm. The extra `18.9 mm` and `-80.5 mm` are CAD
mesh-origin corrections, not physical joint-axis translations.

The previous IK targets had near-zero numerical residual because IK and simulation
both used the same incorrect URDF. When the old joint solutions are evaluated with
the corrected physical model, their TCP errors are between 80.35 mm and 82.06 mm.
That matches the observed symptom: hardware follows the requested joint motion, but
the resulting Cartesian poses are far away from the simulated target.

The fix separates geometry from visualization:

```xml
<!-- physical joint transform -->
<joint name="lowerarm_joint" ...>
  <origin xyz="0.170 0 0" rpy="0 1.5708 0" />
</joint>

<!-- CAD-origin correction only -->
<link name="upperarm">
  <visual>
    <origin xyz="0 -0.0189 0.0805" rpy="4.7124 0 0" />
  </visual>
</link>
```

### 2. The TCP length did not match the supplied robot and gripper dimensions

The supplied dimensions give:

```text
lower joint -> end-effector motor axis = 124.09 mm
approximate EOAT/gripper reach         =  70.00 mm
-------------------------------------------------
lower joint -> gripper TCP             = 194.09 mm
```

The fixed `gripper_tcp_joint` now uses `0.19409 m` along the lower-arm axis. This is
the position used by inverse kinematics. The value is recorded separately in
`config/kinematics_calibration.yaml` so future edits can be audited.

### 3. Approach clearance had silently changed from 60 mm to 100 mm

The confirmed Task 1 clearance is restored to `0.060 m`. The resulting target
heights are now generated consistently from:

- board surface: 8 mm;
- cube stack height: 40 mm;
- cube center at each stack level;
- 60 mm vertical approach clearance.

### 4. Vertical motions were interpolated in joint space

Interpolating two IK solutions directly in joint space does not produce a straight
TCP line. In the previous trajectory, one nominal vertical segment bowed by up to
9.216 mm.

Every descend, lift, and retreat segment is now marked:

```yaml
interpolation: cartesian_linear
```

The trajectory sampler creates a minimum-jerk scalar along the Cartesian line and
solves continuous IK for every sample. Long transfers remain joint-space
minimum-jerk motions. The independent audit reports effectively zero line deviation
for the generated 50 Hz trajectory.

### 5. Tiny IK residuals did not prove that the physical model was correct

The supplied generated waypoint file reported very small IK errors. Those errors
only showed that its joint values reproduced its targets inside the same model.
They did not validate link lengths, TCP placement, the physical homing zero, or
motor-coordinate signs. The supplied text also had indentation damage and was not
valid YAML, so it is retained only as diagnostic evidence and is not used at
runtime.

## Coordinate-frame review

The Task 1 board coordinates are converted to `base_link` as:

```text
x_base = y_board
y_base = x_board
z_base = z_board
```

This matches the established convention: board `+Y` points along the arm at
`shoulder_joint = 0`, while increasing the shoulder joint moves toward board `+X`.
The simulation and hardware generators read the same target configuration and the
kinematics audit rejects them if their Cartesian targets or arm waypoints differ.

## Supplied reach-angle observations

The supplied upper-arm values are referenced to a vertical/Y line, while the
lower-arm values are referenced to a horizontal/X line. They are therefore not
raw ROS relative-joint coordinates and must not be copied into URDF joint limits.
For the corrected URDF chain, using a signed X-Z plane convention:

```text
upper_reference_angle = pi/4 + q_upper
lower_reference_angle = -pi/4 - q_upper - q_lower
```

The supplied max-reach pair (`1.13097`, `-1.13097` rad) corresponds approximately
to:

```text
q_upper = +0.34557 rad
q_lower =  0.00000 rad
```

The sign of the supplied min-reach lower-arm value (`+1.22173 rad`) still needs one
physical confirmation. If it is a downward angle magnitude, its signed geometric
value is `-1.22173 rad`, which corresponds approximately to:

```text
q_upper = -0.26180 rad
q_lower = +0.69813 rad
```

These pairs are recorded as documentation-only calibration references. They are
not enforced as independent limits.

## Encoder scaling and post-homing coordinates

The hardware bridge continues to read the configured FAULHABER Factor Group values
from each drive (`0x608F`, `0x6091`, and `0x6092`) and derives PDO units per physical
output revolution at runtime. This path was reviewed but not changed as part of the
IK correction.

No model-to-drive affine mapping is applied in this revision. The hardware bridge,
homing searches, switch logic, method-37 zero assignment, and post-homing startup
flow are byte-for-byte unchanged from the accepted separated-workflow version.
`config/kinematics_calibration.yaml` records candidate sign/zero fields only as a
place to document measurements. A runtime mapping should be implemented only after
a low-speed, one-joint-at-a-time comparison proves a sign or zero mismatch between
RViz/model coordinates and the referenced drive coordinates.

## Files changed

- `rascl_description/urdf/rascl.urdf.xacro`
- `rascl_wp3_ss26_group11/config/task1_cube_poses.yaml`
- `rascl_wp3_ss26_group11/config/kinematics_calibration.yaml`
- `rascl_wp3_ss26_group11/scripts/generate_task1_waypoints_rtb.py`
- `rascl_wp3_ss26_group11/scripts/generate_min_jerk_task1.py`
- `rascl_wp3_ss26_group11/scripts/regenerate_task1_trajectory.py`
- `rascl_wp3_ss26_group11/scripts/audit_task1_kinematics.py`
- `rascl_wp3_ss26_group11/launch/wp3_prepare_task1.launch.py`
- both generated waypoint YAML files and both generated CSV files
- verification tests and documentation

No homing YAML, homing strategy, reference-switch sequence, or method-37 code was
changed.

## Required validation order on the robot

1. Rebuild and run `./verify_project.sh`.
2. Run `wp3_prepare_task1.launch.py` to regenerate both trajectories in the ROS
   container with Robotics Toolbox.
3. Run the RViz-only simulation and inspect the corrected link geometry and TCP
   paths.
4. Home normally with the separated homing launch.
5. Before a full Task 1 run, command a small positive motion on one arm joint at a
   time and confirm that the physical direction and RViz direction agree.
6. Record any demonstrated sign/zero mismatch, but do not guess or apply one during the first corrected-model test.
7. Perform the first Task 1 test at low speed with the emergency stop accessible.

The repository can verify geometry, IK, limits, continuity, and trajectory shape
offline. It cannot certify the final motor-coordinate sign/zero relationship
without that physical one-joint test.
