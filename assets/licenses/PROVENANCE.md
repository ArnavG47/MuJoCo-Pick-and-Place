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
  - `robotiq_2f85.urdf` hand edit: 18 `package://robotiq_description` mesh paths replaced with the absolute working-copy path. Redo this if xacro is rerun.

## EPick (dual-suction baseline)
- Source: https://github.com/PickNikRobotics/ros2_epick_gripper
- Package used: `epick_description`
- Commit: d07391307d2d16cacf88daa168ce02839589397e   Commit date: 2026-09-08   Recorded: 2026-10-23
- License: BSD 3-Clause, Copyright (c) 2022 PickNik Robotics (`epick_LICENSE`)
- Working copy: none. The URDF is not loaded; the EPick is rebuilt in code (`make_epick_spec` in `src/build_models.py`).
- Values taken from the source:
  - `EPICK_LEN = 0.1023`, `EPICK_R = 0.044` (m): `epick_description/urdf/epick_body.xacro`
  - `CUP_H = 0.015` (m): `epick_description/urdf/epick_single_suction_cup.xacro`
- Own choices (not in the source): a crossbar (`BAR_T = 0.01`, `BAR_W = 0.02` m) holding two cups, `cup_r = 0.012` m,
  `spacing = 0.025` m, adhesion `f_cup = 10` N per cup. Simplified cylinder/box geoms, no meshes.

## Test objects (Google Scanned Objects, pre-converted to MJCF)
- Source: https://github.com/kevinzakka/mujoco_scanned_objects (MJCF made with obj2mjcf; collision = 32 V-HACD convex hulls per object)
- Commit: 6ff8d275cebfd5b47e49685e3cfbe64b20e49a3c   Commit date: 2022-07-07   Recorded: 2026-11-06
- License: meshes and textures (obj, png) CC-BY 4.0 (Google Scanned Objects, Downs et al. 2022);
  MJCF (xml) files MIT (`scanned_objects_MJCF_LICENSE`). CC-BY requires attribution: keep this section.
- Copied unchanged into `assets/objects/scanned/` (8 of 1030 objects, sparse checkout):
  ACE_Coffee_Mug_Kristen_16_oz_cup, Android_Lego, Beta_Glucan, Canon_Pixma_Ink_Cartridge_8,
  Great_Dinos_Triceratops_Toy, JarroSil_Activated_Silicon, Jarrow_Formulas_Glucosamine_Hci_Mega_1000_100_ct,
  Nestle_Nips_Hard_Candy_Peanut_Butter
- Changes made in code at load time (`src/objects.py`), files on disk untouched:
  - absolute mesh/texture paths, a freejoint, prefix `obj_`
  - visual mesh density 0; collision-hull density scaled so the body mass equals a nominal mass
  - nominal masses (0.05-0.35 kg) are rough estimates of the real products, not measured
  - yaw 0 or 90 deg and an xy shift so the bounding-box centre sits at world x = y = 0
