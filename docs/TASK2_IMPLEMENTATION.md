# Task 2 online motion planning — Group 11

This Task 2 implementation is added beside the frozen Task 1 workflow. It does
not modify homing, the PDO bridge, the URDF, Task 1 pose files, Task 1 trajectory
generation, or Task 1 execution.

## Runtime contract

The required topic remains:

```text
/goal_poses
geometry_msgs/msg/Point
```

`geometry_msgs/msg/Point` always contains `x`, `y`, and `z`, but this application
uses only:

```text
point.x = nominal cube centre X in base_link, metres
point.y = nominal cube centre Y in base_link, metres
```

`point.z` is ignored. The Task 2 TCP height is calculated as:

```text
board.surface_z_m + 0.5 * cube.height_m
```

With the current measured values, this is:

```text
0.008 m + 0.5 * 0.040 m = 0.028 m
```

The configured `board.target_xy_correction_base_m` is then added to the runtime
X/Y position. This empirical pair must be recalibrated whenever the movable board
has shifted.

## Workspace placeholders

The current configuration intentionally contains the requested temporary values:

```yaml
workspace:
  units: m
  min_radius_m: 0.11
  max_radius_m: 0.36108
  values_are_placeholders: true
```

Simulation is allowed with these values. Hardware execution is deliberately
blocked while `values_are_placeholders` is `true`. After the physical feasible
radii are supplied, replace the two metre values and set the flag to `false`.

## Fixed goal

Task 2 reuses the exact Task 1 goal in board coordinates:

```yaml
goal:
  board_xy_m: [0.100, 0.070]
```

It uses the same board-to-base mapping and the same empirical XY correction as
Task 1. The goal is on the right side and is only required to lie inside the
verified feasible region. It is not forced to the maximum feasible radius.

## One online cycle

For every accepted runtime cube position, `wp3_tsk2` creates and validates the
complete trajectory before publishing any motion:

1. Verify hardware/session state and pick-ready.
2. Apply the empirical X/Y correction.
3. Reject positions outside the configured radius range and reject IK solutions outside the required shoulder-joint range.
4. Solve IK for above-cube, cube-centre, above-goal, and goal-centre.
5. Move above the cube with the gripper open.
6. Descend vertically to the cube centre with the gripper open.
7. Close the gripper while the arm remains at the cube centre.
8. Lift vertically, transfer to the fixed Task 1 goal, and descend vertically.
9. Open the gripper while the arm remains at the goal centre.
10. Retreat and return to pick-ready/open.
11. Wait for the next cube.

New points received while planning or executing are rejected rather than queued,
so stale cube positions cannot be executed later.

## Simulation

After rebuilding and sourcing:

```bash
ros2 launch rascl_wp3_ss26_group11 sim_wp3_tsk2.launch.py
```

In another terminal, publish only X/Y:

```bash
ros2 run rascl_wp3_ss26_group11 publish_task2_cube --x 0.18 --y 0.06
```

The helper sends `z=0.0`, prints the corrected X/Y, fixed Z, radius, and angle,
and the Task 2 node ignores the message Z field.

You can also use the required topic directly:

```bash
ros2 topic pub --once /goal_poses geometry_msgs/msg/Point \
  "{x: 0.18, y: 0.06, z: 0.0}"
```

## Hardware, after verified radii are entered

Terminal 1 remains the accepted frozen homing/controller workflow:

```bash
ros2 launch rascl_wp3_ss26_group11 wp3_homing_hardware.launch.py
```

Leave it running. In Terminal 2:

```bash
ros2 launch rascl_wp3_ss26_group11 wp3_tsk2.launch.py
```

Then publish one cube position at a time. `wp3_tsk2.launch.py` starts only the
online planner; it does not own EtherCAT, homing, trajectory regeneration, or
Task 1.

## Files added for Task 2

```text
config/task2_online_planning.yaml
launch/sim_wp3_tsk2.launch.py
launch/wp3_tsk2.launch.py
rascl_wp3_ss26_group11/task2_config.py
rascl_wp3_ss26_group11/task2_kinematics.py
rascl_wp3_ss26_group11/task2_trajectory.py
rascl_wp3_ss26_group11/task2_planner.py
rascl_wp3_ss26_group11/task2_cube_publisher.py
rascl_wp3_ss26_group11/wp3_tsk2.py
test/test_task2_contract.py
```

## Not yet claimed as physically validated

The online planner has been integrated and statically/unit checked in this
revision. The temporary 0.11 m and 0.30 m radii are not claimed as the final
physical feasible region, and no hardware Task 2 run is claimed until those
values are replaced and supervised testing is completed.
