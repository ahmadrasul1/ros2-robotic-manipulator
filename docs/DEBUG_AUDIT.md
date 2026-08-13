# RASCL WP3 Group 11 — current execution and safety audit

## Scope

The repository was reviewed across the active ROS 2 execution chain:

1. Docker and Python dependencies.
2. Package installation rules.
3. URDF, ros2_control interfaces, controller order, and joint limits.
4. Profile Position homing and pick-ready movement.
5. EtherCAT PDO mapping and CSP operation.
6. C++ ros2_control to Python bridge IPC.
7. Task 1 IK, minimum-jerk sampling, CSV validation, and runtime streaming.
8. Launch ownership and failure separation.

## Current workflow architecture

The previous all-in-one hardware launch was removed. The hardware workflow now
has three explicit owners:

| Phase | Launch file | Owns | Must not own |
|---|---|---|---|
| Prepare | `wp3_prepare_task1.launch.py` | IK, waypoint output, minimum-jerk CSV, validation | EtherCAT, homing, controllers, task playback |
| Home/runtime | `wp3_homing_hardware.launch.py` | EtherCAT, four-axis homing, pick-ready, CSP, ros2_control, controllers | IK, CSV generation, task playback |
| Execute | `wp3_tsk1_hardware.launch.py` | Loading and streaming one validated hardware CSV | EtherCAT startup, homing, trajectory generation |

The homing/runtime launch is intentionally persistent. It remains alive in one
terminal while the Task 1 player is started from another terminal. This retains
the current PDO session and the reference facts established during the same
startup run.

Stopping the homing/runtime launch invalidates that session. The next hardware
run must home again; the Task 1 launch does not preserve or invent homing state.

## Important corrected issues

| Area | Previous problem | Current correction |
|---|---|---|
| Launch coupling | Task 1 generation, homing, EtherCAT startup, and execution happened in one launch. | Three independent launch files now have non-overlapping responsibilities. |
| Repeated homing | Starting Task 1 automatically reran homing. | The Task 1 launch only starts the player and waits for an existing homed PDO stack. |
| Hidden rebuild | Hardware execution regenerated the trajectory immediately before motion. | Preparation is an offline command that exits before hardware is started. |
| Reference lifetime | A separate process could have claimed old homing state after a hardware restart. | The current-run ready marker is owned by the still-running bridge and removed on shutdown. |
| Start-pose jump | A task could begin without checking sample zero against the homed pose. | The player checks the ready marker and CSV start pose; the PDO bridge independently checks the first command against live feedback. |
| Gripper trajectory limits | Stored simulation data still used the obsolete `1.5708`/`1.3879` rad gripper values. | Simulation and hardware trajectories now use the current `0.0` open and `-0.083333333` hold values within `[-0.12, 0.02]` rad. |
| Stale pick-ready | Stored trajectories began at `[0.0, -0.4, 0.3, 1.5708]`. | Both current trajectories begin and end at the configured all-zero pick-ready pose. |
| Manual reference bypass | YAML could have been used to assert a valid gripper reference. | Runtime reference validity is produced only by successful homing and final switch verification. |
| Unsafe drive transition | A retained target could exist while enabling a drive. | The measured position is written as the target before mode enable and CSP transition. |
| Blocking executor | A timer callback previously could block through the full trajectory. | The executor publishes one resampled command per timer tick at 100 Hz. |

## Homing sequence

The configured dependency-aware order is:

1. Home the shoulder using its A/B reference procedure.
2. Park the lower arm at its negative clearance boundary.
3. Home the upper arm from C toward B while the lower arm remains parked.
4. Home the lower arm directly toward B.
5. Home the end effector toward its single reference switch using the configured
   `search_direction`.
6. Verify all required references and zero tolerances.
7. Move to the explicit pick-ready pose.
8. Enter CSP and publish the ready marker.

Method 37 is used only after the corresponding reference input is confirmed. It
sets the position counter; it is not treated as a search routine.

## Runtime gates before Task 1

The Task 1 player publishes no controller command until:

- the joint position controller has a subscriber;
- `/tmp/rascl_pdo_ready` contains `PDO_READY`;
- `allow_motion=true`;
- arm reference validity is true;
- end-effector reference validity is true;
- the marker contains a valid measured four-joint pose;
- the measured startup pose matches CSV sample zero within the configured
  tolerances.

The PDO bridge separately rejects:

- non-finite commands;
- wrong joint count;
- position-limit violations;
- first-command mismatch against live feedback;
- per-cycle velocity jumps;
- command-watchdog expiry;
- unstable working counter or EtherCAT state;
- drive fault, following-error, internal-limit, or warning conditions;
- repeated cycle lateness.

## Current trajectory validation

Both current files pass the strict validator:

- `task1_full_simulation_ik.csv`
- `task1_full_hardware.csv`

Each contains 4,461 samples over 223.000 seconds and starts/ends at:

```text
[0.0, 0.0, 0.0, 0.0] rad
```

Hardware trajectory ranges and observed peaks:

| Joint | Position range (rad) | Peak speed (rad/s) | Peak acceleration (rad/s²) |
|---|---:|---:|---:|
| shoulder | `-1.127025 … 0.845785` | `0.369877` | `0.113888` |
| upper arm | `-1.391290 … 0.000000` | `0.282250` | `0.173810` |
| lower arm | `0.000000 … 1.300604` | `0.217272` | `0.066900` |
| end effector | `-0.083333 … 0.000000` | `0.026037` | `0.013360` |

These checks establish numerical consistency only. They do not prove collision
clearance, payload capability, cube friction, switch wiring, board coordinates,
or safe physical torque.

## Verification performed here

Passed:

- Python syntax parsing for all source files.
- YAML and XML/xacro XML syntax parsing.
- Eight unit tests.
- Strict validation of both simulation and hardware trajectories.
- Cross-file audit of joint order, limits, PDO startup modes, reference gates,
  pick-ready consistency, and launch separation.
- Shell syntax check.

Not available in this environment:

- ROS Jazzy/colcon C++ build.
- Docker engine execution.
- EtherCAT hardware and the four physical drives.
- Physical switch, collision, payload, and cube tests.

Run a clean build inside the provided Docker image before robot testing:

```bash
rm -rf build install log
colcon build --symlink-install
source install/local_setup.bash
./verify_project.sh
```

## Physical checks still required

1. Confirm slave order is shoulder, upper arm, lower arm, end effector.
2. Confirm all configured digital-input numbers and active-low polarity.
3. Confirm cable and mechanical clearance over every homing search.
4. Verify the all-zero pick-ready pose is safe on the assembled robot.
5. Test `hardware_hold_rad=-0.083333333` at low force with the actual cube.
6. Verify the board frame and all cube coordinates physically.
7. Run an empty-arm trajectory before the first cube-stacking attempt.

## Files that matter

- Offline preparation: `src/rascl_wp3_ss26_group11/launch/wp3_prepare_task1.launch.py`
- Persistent homing/runtime: `src/rascl_wp3_ss26_group11/launch/wp3_homing_hardware.launch.py`
- Task-only execution: `src/rascl_wp3_ss26_group11/launch/wp3_tsk1_hardware.launch.py`
- Homing sequence: `src/rascl_wp3_ss26_group11/config/homing.yaml`
- Task geometry and gripper targets: `src/rascl_wp3_ss26_group11/config/task1_cube_poses.yaml`
- Joint limits: `src/rascl_wp3_ss26_group11/config/robot_limits.yaml`
- Task runtime PDO: `src/rascl_hardware_interface/config/ethercat_pdo_task1.yaml`
- Hardware trajectory: `src/rascl_wp3_ss26_group11/trajectories/task1/task1_full_hardware.csv`
