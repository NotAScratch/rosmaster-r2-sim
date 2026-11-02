# 01: Phase 1, a simulation-ready model

*2026-11-02. Built and tested on the Jetson Orin NX, Gazebo Fortress 6.18, server only (no window).*

Goal: a model of the R2 that Gazebo can simulate. It should rest on its wheels, roll straight, and steer within its real limits. There is no drive plugin yet (that is Phase 2), so a test rig drives each joint directly.

## What was built

Package `rosmaster_r2_sim`:

| File | What it is |
|------|------------|
| `urdf/r2.urdf.xacro` | The robot. Same frame and joint names as Yahboom's model, so the TF tree matches the real robot. Reuses Yahboom's meshes for looks |
| `urdf/r2.gazebo.xacro` | Gazebo additions: tyre friction, joint state publisher, ground-truth odometry, and the test rig (`test_actuators:=true`) |
| `worlds/empty.sdf` | Flat ground, 1 ms physics step |
| `config/bridge.yaml` | `/clock`, `/joint_states`, `/ground_truth/odom` from Gazebo to ROS |
| `config/bridge_test.yaml` | Test rig commands from ROS to Gazebo |
| `launch/sim.launch.py` | Starts Gazebo (window or `headless:=true`), robot_state_publisher, spawns the robot, starts the bridges |
| `scripts/phase1_check.py` | Automatic pass/fail checks |

## Changes from Yahboom's model

These fix the problems listed in [00-system-survey.md §4](00-system-survey.md#4-problems-in-yahbooms-model-that-stop-it-working-in-gazebo).

| Yahboom's model | This model | Why |
|-----------------|------------|-----|
| Full-detail mesh collisions (11 MB chassis) | Two boxes for the chassis, cylinders for wheels, simple shapes for camera and lidar | Fast, stable contact. Mesh wheels are not truly round |
| Front-left wheel axis `0 −1 0` | All four wheel axes `0 1 0` | A positive wheel speed now rolls every wheel forward |
| Front wheels toed by −0.010 and −0.013 rad (both pointing right) | No toe | The CAD export left a slight right pull |
| Steering links 2 g with ~1e-7 inertia | 20 g, box inertia | Tiny inertias make the solver unstable |
| Lidar mass 0, inertia 0 | 0.1 kg (estimate), box inertia | Zero-mass links are invalid in Gazebo |
| Wheel inertia from CAD | Uniform cylinder (same mass) | Clean, symmetric values. Within 15% of the CAD values |
| No Gazebo settings | Tyres μ = 1.0 | A car-like robot needs grip to steer |

Joint positions, the steering limit (±0.6 rad), ride height (`base_footprint → base_link` z = 0.0215) and the chassis and camera inertias are Yahboom's values, unchanged.

Known gap: the chassis keeps Gazebo's default friction. Gazebo merges fixed links (camera, lidar, IMU) into `base_footprint`, and sdformat ignores `<gazebo reference>` friction tags on merged collisions. This doesn't matter unless the chassis scrapes something.

## Results

```
ros2 launch rosmaster_r2_sim sim.launch.py headless:=true test_actuators:=true
ros2 run rosmaster_r2_sim phase1_check.py
```

| Check | Result |
|-------|--------|
| Rests on the ground | PASS, `base_footprint` z = 0.7 mm |
| Level | PASS, roll 0.00°, pitch −0.10° |
| No creep when stopped | PASS, 0.0 mm in 2 s |
| Drives forward (5 rad/s on both rear wheels, 4 s) | PASS, 0.694 m (expected 0.690 m) |
| Stays straight | PASS, 0.0 mm sideways, 0.00° yaw drift |
| Steering follows +0.4 / −0.4 rad | PASS, both wheels at ±0.400 |
| Steering stops at the limit (asked for 1.2 rad) | PASS, both at 0.600 |
| Steering left turns left | PASS, +41.2° |

**9/9 passed.**

The turn test is only a sign check. Both front wheels steer to the same angle (no Ackermann geometry yet) and both rear wheels are forced to the same speed (no differential). The wheels scrub, so the turn is tighter than an ideal bicycle model. The Ackermann plugin in Phase 2 fixes both, and the turn radius will be checked against L / tan δ then.

### Performance on the Jetson

Server only, 1 ms step, no sensors: **real-time factor ≈ 0.87**. The robot runs at 87% of real speed. Sensors (Phase 3) will lower this. Options if it gets too slow: a 2 ms step (the robot is small but not stiff), or run on the PC.

## Running it

On the Jetson (no window), or on a PC with `headless:=false` (the default) to get the Gazebo window. The PC needs Ubuntu 22.04, ROS 2 Humble, `ros-humble-ros-gz`, and Yahboom's `yahboomcar_description` package built in a workspace. That is the only Yahboom package needed, for the meshes.

```bash
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash   # for yahboomcar_description
cd ~/rosmaster-r2-sim && colcon build --symlink-install && source install/setup.bash

export ROS_DOMAIN_ID=77          # keep away from the real robot (domain 28)
export IGN_PARTITION=r2sim       # keep Gazebo topics private too
ros2 launch rosmaster_r2_sim sim.launch.py headless:=true
```

**Stopping:** Ctrl-C stops the ROS nodes, but in testing the `ign gazebo` server kept running. Check with `pgrep -af "ign gazebo"` and stop it with `kill <pid>`.

## Next step

Phase 2: replace the test rig with Gazebo's `AckermannSteering` system on `/cmd_vel`, using wheelbase 0.2353 m, kingpin spacing 0.130 m, wheel radius 0.0345 m and steering limit 0.6 rad. Publish `/odom_raw` (odom → base_footprint, no TF by default) to match the real `base_node_R2`. Then check the turn radius against L / tan δ and drive with `teleop_wasd.py`.
