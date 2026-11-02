# Rosmaster R2 Simulation

A Gazebo simulation of the Yahboom **Rosmaster R2**, the small Ackermann-steering (car-like) robot in this lab. The goal is a simulated twin of the robot. It should use the same ROS 2 topics and frames as the real one, so mapping and navigation code can be developed and tested in simulation first, then run unchanged on the robot.

It is a companion to [Autonomous-Mapping-and-Navigation](https://github.com/NotAScratch/Autonomous-Mapping-and-Navigation), which builds SLAM and navigation for the same robot.

> **Status:** Phase 0, research. The real robot's software stack has been surveyed and the gaps in Yahboom's model are listed. No simulation code yet. See [`docs/research/00-system-survey.md`](docs/research/00-system-survey.md).

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

## Roadmap

- [x] **Phase 0, research:** survey the real robot's software, the robot model and the Gazebo plugins available ([survey](docs/research/00-system-survey.md))
- [ ] **Phase 1, model:** a simulation-ready robot model: simple collision shapes, fixed wheel axes, a real mass for the lidar link, a `base_footprint` root link
- [ ] **Phase 2, drive:** Ackermann steering plugin, odometry, joint states, a teleop test
- [ ] **Phase 3, sensors:** lidar, IMU, depth camera, bridged to ROS 2
- [ ] **Phase 4, parity:** match the real robot's topics and TF tree, run the real EKF config against the simulation
- [ ] **Phase 5, use it:** run `slam_toolbox` and Nav2 in simulation, then the same launch files on the real robot

## Licence

GPL-3.0, the same as Autonomous-Mapping-and-Navigation. See [`LICENSE`](LICENSE).

### Third-party files

The robot meshes and base model come from Yahboom's `yahboomcar_description` package. They are not copied into this repo. The simulation will reference that package as it is installed on the robot. See the survey for details.
