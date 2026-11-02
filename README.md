# Rosmaster R2 Simulation

A Gazebo simulation of the Yahboom **Rosmaster R2**, the small Ackermann-steering (car-like) robot in this lab. The goal is a simulated twin of the robot. It should use the same ROS 2 topics and frames as the real one, so mapping and navigation code can be developed and tested in simulation first, then run unchanged on the robot.

It is a companion to [Autonomous-Mapping-and-Navigation](https://github.com/NotAScratch/Autonomous-Mapping-and-Navigation), which builds SLAM and navigation for the same robot.

> **Status:** Phase 3 done. The simulated R2 has its lidar, IMU and depth camera, publishing the real drivers' topics, frames and rates (17/17 automatic checks pass; the lidar matches the test room to 7 mm). Known gaps: camera rates need a PC-class GPU to verify at 30 fps; sensor rates are only checked at real-time factor ≥ 0.9. Next: parity with the real stack. See [`docs/research/03-phase3-sensors.md`](docs/research/03-phase3-sensors.md).

---

## Goals

1. **Drive:** spawn the R2 in Gazebo and drive it with `/cmd_vel`, steering like the real car (it cannot turn on the spot).
2. **Sense:** simulate the lidar (YDLIDAR 4ROS), IMU and depth camera (Orbbec Astra Plus), and publish them to ROS 2 under the same topic names and frames as the real robot.
3. **Match the real robot:** reproduce the real topic and TF layout (`/odom_raw`, `/imu/data_raw`, `/scan`, `odom → base_footprint → base_link`), so the real stack's EKF, SLAM and Nav2 configs work unchanged.
4. **Worlds:** a few test worlds, from an empty room to a copy of the real room the robot maps.
5. **Run anywhere:** run on an Ubuntu 22.04 PC, and in headless mode on the Jetson itself.

## Software environment

| Item | Version |
|------|---------|
| OS | Ubuntu 22.04 (Jetson Linux R36.4.3 on the robot) |
| ROS 2 | Humble |
| Simulator | Gazebo Fortress (`ign gazebo`, gz-sim 6.18) |
| ROS ↔ Gazebo bridge | `ros_gz` (Humble binaries) |
| Robot computer | Jetson Orin NX 8 GB (Super), `MAXN_SUPER` power mode |

## Quick start

Needs Ubuntu 22.04, ROS 2 Humble, `ros-humble-ros-gz`, and Yahboom's `yahboomcar_description` package (for the meshes).

```bash
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
colcon build --symlink-install && source install/setup.bash

export ROS_DOMAIN_ID=77 IGN_PARTITION=r2sim   # keep away from the real robot
ros2 launch rosmaster_r2_sim sim.launch.py                 # PC, with the Gazebo window
ros2 launch rosmaster_r2_sim sim.launch.py headless:=true  # Jetson, server only
```

Drive it with any `/cmd_vel` publisher, for example `python3 ~/teleop_wasd.py`.

Automatic checks: `ros2 run rosmaster_r2_sim phase2_check.py` (drive), `phase3_check.py` (sensors, in the `test_room` world), or launch with `test_actuators:=true` and run `phase1_check.py` (model).

## Roadmap

- [x] **Phase 0, research:** survey the real robot's software, the robot model and the Gazebo plugins available ([survey](docs/research/00-system-survey.md))
- [x] **Phase 1, model:** a simulation-ready robot model: simple collision shapes, fixed wheel axes, a real mass for the lidar link, a `base_footprint` root link ([write-up](docs/research/01-phase1-model.md))
- [x] **Phase 2, drive:** Ackermann steering plugin, odometry, joint states, a teleop test ([write-up](docs/research/02-phase2-drive.md))
- [x] **Phase 3, sensors:** lidar, IMU, depth camera, bridged to ROS 2 ([write-up](docs/research/03-phase3-sensors.md))
- [ ] **Phase 4, parity:** match the real robot's topics and TF tree, run the real EKF config against the simulation
- [ ] **Phase 5, use it:** run `slam_toolbox` and Nav2 in simulation, then the same launch files on the real robot

## Licence

GPL-3.0, the same as Autonomous-Mapping-and-Navigation. See [`LICENSE`](LICENSE).

### Third-party files

The robot meshes and base model come from Yahboom's `yahboomcar_description` package. They are not copied into this repo. The simulation will reference that package as it is installed on the robot. See the survey for details.
