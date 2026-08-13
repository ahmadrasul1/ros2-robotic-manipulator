# Task 2 verification report

## Checks completed for this revision

- Parsed every Python source file with the Python AST parser.
- Parsed every YAML file and every XML/Xacro file.
- Ran all seven Task 2 contract tests successfully.
- Verified that the Task 2 configuration:
  - uses only `Point.x` and `Point.y`;
  - ignores `Point.z` and computes the fixed centre height from the board and cube;
  - keeps the empirical base-frame X/Y correction;
  - uses the temporary `0.11 m` and `0.30 m` radius placeholders;
  - reuses the frozen Task 1 goal;
  - keeps hardware execution blocked while the radii are placeholders.
- Verified by source-level contract test that the first gripper-closing segment is
  `close_gripper_at_cube_center`, after the open-gripper vertical descent and
  before the lift.
- Verified that the Task 2 hardware launch does not start homing, EtherCAT, Task 1
  regeneration, or Task 1 playback.

## Checks requiring the supplied ROS container

The current analysis environment does not contain ROS Jazzy or
`roboticstoolbox-python`. Therefore, run the clean colcon build and the Task 2 RViz
simulation inside the supplied Docker image before hardware use.

## Existing unrelated baseline failure

Running the complete repository unit-test suite still reaches one failure that is
already present in the exact uploaded baseline project:

```text
lowerarm_joint physical translation:
[0.17, 0.0189, -0.0805] differs from [0.17, 0.0, 0.0] by 82.689 mm
```

The failure comes from the pre-existing Task 1 kinematics audit expecting a
`lowerarm_joint` translation different from the current frozen URDF. Task 2 did
not modify the URDF, Task 1 audit, Task 1 trajectories, or any Task 1/homing
runtime source to conceal this mismatch.

## Required next verification

1. Replace the two placeholder radii with the physically verified values and set
   `workspace.values_are_placeholders: false` only after they are confirmed.
2. Run a clean build in the project container.
3. Run the Task 2 simulation and publish representative X/Y positions, including
   both radial boundaries and shoulder-sector boundaries.
4. Test on hardware only after simulation and with the existing homing stack
   already running.
