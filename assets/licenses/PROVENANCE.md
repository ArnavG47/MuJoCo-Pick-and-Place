# Gripper Model Provenance

Public community-maintained descriptions (PickNik Robotics), not official Robotiq specifications.
Both repos are git submodules under `assets/grippers/` and are never edited directly.
Edits are made in working copies under `assets/grippers/work/`.

## Robotiq 2F-85 (parallel-jaw baseline)
- Source: https://github.com/PickNikRobotics/ros2_robotiq_gripper
- Package used: `robotiq_description`
- Commit: a74d007d8f2f06dc6a503ad21038ba869d4999a6   Commit date: 2026-09-08   Recorded: 2026-10-23
- License: BSD 3-Clause, Copyright (c) 2022 PickNik Robotics (`robotiq_2f85_LICENSE`)
  - Redistribution allowed if the copyright notice and license text are kept.
- Working copy: `assets/grippers/work/robotiq_description/`
- Edits:
  1. `urdf/robotiq_2f_85_gripper.urdf.xacro` line 8: `$(find robotiq_description)` replaced with the absolute working-copy path (no ROS package lookup available).
  2. `urdf/robotiq_2f_85_macro.urdf.xacro` line 14: `include_ros2_control:=true` changed to `false` (ROS hardware interface; MuJoCo control comes from actuators instead).
  3. `urdf/robotiq_2f_85_macro.urdf.xacro` line 24: `$(find robotiq_description)` replaced with the absolute working-copy path.
- Generated files:
  - `robotiq_2f85.urdf`: produced by `xacro urdf/robotiq_2f_85_gripper.urdf.xacro` (default args: `use_fake_hardware=true`, `com_port=/dev/ttyUSB0`; both unused with ros2_control disabled). Verified: no `xacro:`, `$(`, or `ros2_control` left.

## EPick (dual-suction baseline)
- Source: https://github.com/PickNikRobotics/ros2_epick_gripper
- Package used: `epick_description`
- Commit: d07391307d2d16cacf88daa168ce02839589397e   Commit date: 2026-09-08   Recorded: 2026-10-23
- License: BSD 3-Clause, Copyright (c) 2022 PickNik Robotics (`epick_LICENSE`)
- Working copy: not created yet (Lesson 8)
- Edits: none yet
