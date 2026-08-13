# RASCL-Bot ROS 2 Robotic Manipulator

ROS 2 control, robot-description, kinematics, trajectory-generation, and hardware-integration project for a multi-DOF robotic manipulator with a custom parallel gripper.

The system connects a ROS 2 motion layer to physical servo drives through `ros2_control`, EtherCAT/PDO communication, and CiA 402 drive control. It supports RViz simulation, hardware commissioning, calibrated pick-and-place trajectories, and online Cartesian target planning.

> **Project context:** This was developed as a university team project. The repository is currently prepared as a personal Git candidate, but the package metadata remains **Proprietary**. Check `PUBLICATION_NOTICE.md` before making it public.


## What the system does

- Models the manipulator and gripper in **URDF/Xacro**.
- Exposes the robot through a custom **`ros2_control` hardware interface**.
- Communicates with FAULHABER drives over **EtherCAT** using cyclic PDOs and CiA 402 control states.
- Converts between drive/encoder coordinates and ROS joint coordinates.
- Generates inverse-kinematics waypoints from Cartesian task targets.
- Samples **minimum-jerk trajectories** for smooth robot motion.
- Executes pre-generated Task 1 pick-and-place trajectories in simulation and hardware.
- Implements Task 2 online planning from incoming Cartesian XY targets.
- Applies joint/workspace limits, reference checks, start-pose checks, and trajectory validation before hardware motion.

## System architecture

```text
Cartesian task target
        |
        v
Inverse kinematics
        |
        v
Waypoint / trajectory generation
        |
        v
Minimum-jerk sampling
        |
        v
ROS 2 task node
        |
        v
ros2_control position controller
        |
        v
Custom hardware interface
        |
        v
EtherCAT / PDO / CiA 402 bridge
        |
        v
Servo drives -> physical robot
```

For RViz-only simulation, the trajectory can instead be published directly as joint states without opening the EtherCAT hardware stack.

## My main contributions

This was a team project. My strongest contributions were on the software, robot-model, integration, and debugging side rather than the mechanical CAD/3D-printing work.

- **URDF/Xacro robot integration:** worked on the robot link/joint hierarchy, physical joint transforms, joint limits, mesh alignment/origins, gripper integration, TCP definition, and `ros2_control` interfaces in the robot description.
- **Kinematics and calibration:** worked on reconciling the mathematical robot model with the physical manipulator, including link dimensions, coordinate-frame/sign issues, zero/reference offsets, TCP placement, and simulation-to-hardware position errors.
- **Inverse kinematics and Task 1 motion generation:** contributed to Cartesian waypoint generation, IK integration, minimum-jerk trajectory generation, approach/lift/transfer/retreat motion structure, and validation of generated joint trajectories.
- **Task 2 online motion planning:** worked on the runtime planner that accepts Cartesian XY targets, checks the workspace, solves IK, creates a trajectory, and executes the motion against the existing controller stack.
- **ROS 2 and hardware integration/debugging:** worked with launch files, `ros2_control`, controller topics, joint states, EtherCAT/PDO communication, CiA 402/CSP behavior, drive feedback, joint limits, and the simulation-to-real execution pipeline.
- **Gripper software integration and calibration:** integrated the custom gripper into the URDF/control stack and tuned its software-side open/contact/hold coordinates for physical cube manipulation.
- **Testing and fault investigation:** investigated trajectory rejection, following errors, coordinate inversions, encoder/radian discrepancies, controller startup problems, joint-limit mismatches, and hardware-vs-simulation behavior.

### Contribution boundaries

I was **not the primary contributor for the CAD design or 3D-printing/manufacturing of the gripper**, and I do not present those parts as my own mechanical-design work.

I also worked with and debugged the homing/reference workflow, but **homing implementation was not one of my main contributions**; it is therefore intentionally not highlighted as a primary portfolio claim.

## Repository structure

```text
.
├── Dockerfile
├── rosws.sh
├── verify_project.sh
├── docs/
│   ├── DEBUG_AUDIT.md
│   ├── KINEMATICS_AUDIT.md
│   ├── TASK2_IMPLEMENTATION.md
│   └── TASK2_TEST_REPORT.md
└── src/
    ├── rascl_description/
    │   ├── config/
    │   ├── launch/
    │   ├── meshes/
    │   ├── rviz/
    │   └── urdf/
    ├── rascl_hardware_interface/
    │   ├── config/
    │   ├── include/
    │   ├── scripts/
    │   └── src/
    └── rascl_wp3_ss26_group11/
        ├── config/
        ├── launch/
        ├── rascl_wp3_ss26_group11/
        ├── scripts/
        ├── test/
        └── trajectories/
```

Generated colcon directories (`build/`, `install/`, `log/`), Python caches, local shell history, and backup `old_*` source copies are intentionally excluded from Git.

## Main software stack

- ROS 2 Jazzy
- Python and C++
- `ros2_control` / `ros2_controllers`
- EtherCAT / SOEM / PySOEM
- CiA 402 drive control
- Robotics Toolbox for Python
- NumPy / Matplotlib
- URDF / Xacro
- Docker

## Build environment

The supplied Docker image installs ROS 2 Jazzy, `ros2_control`, RViz, PySOEM, Robotics Toolbox for Python, and SOEM.

From the repository root:

```bash
REBUILD=true ./rosws.sh
```

Inside the container:

```bash
cd /root/ws
rm -rf build install log
colcon build --symlink-install
source install/local_setup.bash
./verify_project.sh
```

After source-code/configuration changes:

```bash
cd /root/ws
colcon build --symlink-install
source install/local_setup.bash
```

## Simulation

### Task 1

```bash
ros2 launch rascl_wp3_ss26_group11 sim_wp3_tsk1.launch.py
```

### Task 2

```bash
ros2 launch rascl_wp3_ss26_group11 sim_wp3_tsk2.launch.py
```

Publish an example Task 2 target:

```bash
ros2 run rascl_wp3_ss26_group11 publish_task2_cube --x 0.18 --y 0.06
```

## Hardware workflow

> Hardware execution can move a physical robot. Keep the emergency stop accessible and use the laboratory's approved commissioning procedure.

The hardware workflow is deliberately separated so trajectory generation, homing/reference establishment, and task execution cannot be accidentally coupled.

### 1. Prepare and validate Task 1

This stage performs trajectory generation/validation without opening EtherCAT:

```bash
ros2 launch rascl_wp3_ss26_group11 wp3_prepare_task1.launch.py
```

The generated trajectories are stored under:

```text
src/rascl_wp3_ss26_group11/trajectories/task1/
```

Regenerate the trajectory after changing task poses, robot limits, URDF geometry, gripper calibration, or trajectory-generation logic.

### 2. Establish the hardware/controller session

In a dedicated terminal:

```bash
source /root/ws/install/local_setup.bash
ros2 launch rascl_wp3_ss26_group11 wp3_homing_hardware.launch.py
```

This starts the EtherCAT/controller stack and establishes the required reference state before task execution. Leave it running while a hardware task is executed.

### 3. Execute Task 1

In another terminal:

```bash
source /root/ws/install/local_setup.bash
ros2 launch rascl_wp3_ss26_group11 wp3_tsk1_hardware.launch.py
```

The task node is gated by hardware readiness/reference checks, controller availability, trajectory freshness, and startup-pose agreement before publishing motion commands.

### Task 2 hardware execution

After the workspace and hardware configuration have been verified:

```bash
# Terminal 1: persistent hardware/controller session
ros2 launch rascl_wp3_ss26_group11 wp3_homing_hardware.launch.py

# Terminal 2: online planner
ros2 launch rascl_wp3_ss26_group11 wp3_tsk2.launch.py
```

## Kinematics and trajectory generation

The current kinematic model separates physical joint transforms from CAD mesh-origin corrections. This is important because a mesh origin is not necessarily the physical rotation axis of the joint.

Task 1 uses:

- Cartesian pick/place targets,
- inverse kinematics to generate joint-space waypoints,
- minimum-jerk Cartesian-line sampling for local approach/lift/retreat motions,
- minimum-jerk joint-space interpolation for larger transfers,
- joint-limit and kinematic validation before execution.

The project also compares simulation and hardware trajectory representations to ensure the arm trajectory stays consistent while allowing hardware-specific gripper coordinates.

See `docs/KINEMATICS_AUDIT.md` for the detailed kinematic/debugging notes.

## Hardware interface and EtherCAT

The `rascl_hardware_interface` package provides a custom `ros2_control` system interface and a cyclic PDO bridge for the physical servo drives.

The stack includes:

- command/state interfaces for the robot joints,
- EtherCAT adapter selection,
- PDO process-data exchange,
- CiA 402 drive-state handling,
- CSP position commands,
- drive feedback and status words,
- joint/velocity/tracking limits,
- runtime readiness and safety checks.

The Ethernet adapter is machine-specific. The bridge supports overriding it through:

```bash
export RASCL_ETHERCAT_INTERFACE=<your-interface-name>
```

Do not assume the adapter name stored in a lab configuration matches another computer.

## Verification

Run:

```bash
cd /root/ws
./verify_project.sh
```

The verification scripts cover syntax/configuration checks, trajectory validation, kinematic endpoint checks, Cartesian approach paths, joint ordering/limits, simulation-vs-hardware arm consistency, and hardware-startup safety contracts.


## Attribution and license

This repository contains work produced in a university team setting and may include course-provided or teammate-authored components. The ROS package metadata currently declares the code **Proprietary**. See `PUBLICATION_NOTICE.md` before public redistribution.
