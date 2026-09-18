# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

ROS2 Jazzy workspace for an autonomous boat with RTK-GNSS positioning. This repository contains workspace-level configuration (deploy, docs, vcstool .repos); source packages live in `src/` and are managed by vcstool via `minix_ws.repos` (gitignored).

## Build Commands

```bash
# Import source packages
vcs import src < minix_ws.repos

# Build (bme_common_msgs must be built first)
source /opt/ros/jazzy/setup.bash
source ~/minix_ws/.venv/bin/activate
cd ~/minix_ws
colcon build --symlink-install --packages-select bme_common_msgs
source install/setup.bash
colcon build --symlink-install
source install/setup.bash

# Build single package
colcon build --symlink-install --packages-select <package_name>

# Packages needing pyproj (look_ahead_control, lg580p_gnss_driver, ublox_gnss_driver) require venv python:
/home/kariko/minix_ws/.venv/bin/python3 -m colcon build --symlink-install --packages-select <package_name>
```

## Running

```bash
# Hardware (each in separate terminal):
ros2 launch lg580p_gnss_driver lg580p_gnss.launch.py
ros2 launch ntrip_client ntrip_client.launch.py
ros2 launch mavlink_ros2_bridge mavlink_bridge.launch.py
ros2 launch look_ahead_control look_ahead_following.launch.py
ros2 run maestro_driver maestro_driver_node

# Legacy GNSS (u-blox 2-receiver setup, superseded by lg580p_gnss_driver):
# ros2 launch ublox_gnss_driver ublox_gnss.launch.py

# Simulation:
ros2 launch bme_simplerover_gazebo sim.launch.py
ros2 launch mavlink_ros2_bridge mavlink_bridge.launch.py
ros2 launch look_ahead_control look_ahead_following.launch.py
```

## Lint / Test

```bash
colcon test --packages-select <package_name>
colcon test-result --verbose
```

No functional unit tests — only ament lint tests (flake8, pep257, copyright).

## Repository Structure

- `minix_ws.repos` — vcstool definitions for source packages (`iori-N/` forks are self-authored/modified; others reference upstream, mainly IgarashiBME GitHub, as-is)
- `deploy/` — systemd service files, launch scripts, installer for auto-start on boot
- `doc/` — system-overview.md (full architecture), deployment.md, operation.md
- `src/` — gitignored; populated by `vcs import`

## Key Conventions

- **Coordinate frame**: ENU per REP-103 (East=X, North=Y, Up=Z). Yaw: East=0, North=90, CCW positive. mavlink_ros2_bridge converts ENU↔NED for QGC.
- **pyproj / venv**: Installed in `~/minix_ws/.venv`, not via rosdep. Do not add pyproj to package.xml.
- **Message naming (ROS2 Jazzy)**: Files PascalCase (`PVT.msg`), fields snake_case. Violations cause build failures.
- **PWM**: Center=1500us, range=1000-2000us. `UInt16MultiArray` on `/rc_pwm` with `[ch1_pwm, ch2_pwm]`.
- **ArduPilot mode constants**: Armed=217 (base_mode), Disarmed=89. Auto custom_mode=10.
- **Python threading**: All Python nodes use `rclpy.spin()` on a daemon thread; main thread handles blocking I/O or control loops.
- **Parameter persistence**: mavlink_ros2_bridge saves QGC-tunable params to `~/.ros/mavlink_bridge_params.yaml`.
- **Documentation language**: Japanese.
