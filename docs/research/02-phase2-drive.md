# 02: Phase 2, Ackermann drive

*2026-11-02. Built and tested on the Jetson Orin NX, Gazebo Fortress 6.18, with the Gazebo window open.*

Goal: drive the simulated R2 with `/cmd_vel`, the way teleop and Nav2 drive the real one, and publish wheel odometry the way the real robot does.

## What was added

| File | Change |
|------|--------|
| `urdf/r2.gazebo.xacro` | Gazebo's `AckermannSteering` system, with geometry taken from `r2.urdf.xacro`. Used unless `test_actuators:=true` |
| `urdf/r2.urdf.xacro` | `max_speed` 1.0 m/s and `max_accel` 1.0 m/s² properties |
| `config/bridge_drive.yaml` | `/cmd_vel` in, `/odom_raw` out |
| `config/bridge_odom_tf.yaml` | `odom → base_footprint` on `/tf`, only with `publish_odom_tf:=true` |
| `launch/sim.launch.py` | Picks the bridges; new `publish_odom_tf` option |
| `scripts/phase2_check.py` | Automatic checks through `/cmd_vel` |

### Plugin settings

| Setting | Value | Source |
|---------|-------|--------|
| `wheel_base` | 0.23529 m | rear axle to kingpins, Yahboom's model |
| `wheel_separation` | 0.1685 m | rear track |
| `kingpin_width` | 0.130 m | distance between the steering pivots |
| `wheel_radius` | 0.0345 m | wheel mesh |
| `steering_limit` | 0.6 rad | steering joint limit |
| `max_velocity` | ±1.0 m/s | the real driver clamps to 1.0 (`Ackman_driver_R2` `xlinear_limit`) |
| `max_acceleration` | ±1.0 m/s² | **guess**, not yet measured on the robot |

### Topics, matching the real robot

| Topic | Real robot | Simulation |
|-------|-----------|------------|
| `/cmd_vel` | `Ackman_driver_R2` → motor board | Ackermann plugin |
| `/odom_raw` | `base_node_R2`: integrates speed and steering angle | plugin: integrates rear wheel joints and steering angle |
| `odom → base_footprint` TF | the EKF publishes it | not published (`publish_odom_tf:=true` turns it on, for runs without the EKF) |
| `/joint_states` | steering angle, plus random wheel angles | real joint positions from Gazebo |
| `/vel_raw` | speed and steering angle (degrees, in `linear.y`) | **not published**; only `base_node_R2` reads it, and the sim replaces that node |

## Results

```
ros2 launch rosmaster_r2_sim sim.launch.py
ros2 run rosmaster_r2_sim phase2_check.py
```

| Check | Result |
|-------|--------|
| `/odom_raw` frames | PASS, `odom → base_footprint` |
| No odom TF by default | PASS |
| Straight at 0.2 m/s | PASS, 0.200 m/s, 0.0 mm sideways |
| Stops on a zero command | PASS, 0.0000 m/s after 1.5 s |
| Turn radius (asked for 0.6 m at the rear axle) | PASS, 0.612 m at `base_footprint` (expected 0.611 m: `base_footprint` sits 0.116 m ahead of the axle) |
| Yaw rate follows `angular.z` | PASS, 0.326 rad/s for 0.333 commanded (−2%, see below) |
| Ackermann wheel angles | PASS, inner 0.414 / outer 0.340 rad, exactly the geometric values |
| Right turn mirrors left | PASS |
| Steering stays within ±0.6 rad | PASS, 0.590 / 0.454 |
| Tightest turn (asked for 0.2 m) | PASS, 0.433 m. The inner wheel at the limit gives ≈0.425 m |
| Reverse | PASS |
| Top speed (asked for 3.0 m/s) | PASS, 1.001 m/s |
| `/odom_raw` distance over the run | PASS, 6.648 m vs 6.643 m true (+0.1%) |
| `/odom_raw` turning over the run | PASS, 477.2° vs 479.8° true (−0.5%) |

**15/15 passed.** I also drove the full test with the Gazebo window open on the Jetson.

### What the simulated robot gets wrong

**1. Steering is too slow (known gap, not fixable with this plugin).** From straight to a 0.6 m turn, the steering took **2.5 s to reach 90%** of its angle, with a time constant of about 1.2 s. Fortress's `AckermannSteering` moves the steering joints at a speed proportional to the remaining angle, with a gain fixed in its code. Newer Gazebo versions have a setting for it; Fortress does not. A hobby steering servo typically takes around 0.2 s. Until this is fixed, the simulated robot will cut corners late compared with the real one. Nav2 path tracking in simulation will look worse than it should.

Found by: the first turn check failed with a radius of 0.623 m. Raising tyre friction 3× changed nothing. A sweep over speeds and radii showed exact radii once settled (0.601 m for 0.6, 0.501 m for 0.5). Logging the steering angle then showed the slow approach.

Options:

| Option | Work | Result |
|--------|------|--------|
| A. Keep the plugin (now) | none | Correct geometry and odometry; slow steering |
| B. Small ROS node in place of the plugin: `/cmd_vel` → steering angles (Gazebo `JointPositionController`, fast) and rear wheel speeds (`JointController`), with `/odom_raw` worked out from `/joint_states` the way `base_node_R2` does it | about 150 lines | Steering speed is a setting and can be matched to the real servo. Copies the real driver + base_node split |
| C. `gz_ros2_control` + `ackermann_steering_controller` | apt install of `ros-humble-ign-ros2-control` and the controllers, plus configs | The standard ROS 2 route. Heavier, more to tune |

**Recommendation: B, in Phase 4 (parity),** after timing the real servo. One way to time it: film the wheels while sending a steering step.

**2. About 2% wheel slip in turns.** In a steady turn the robot moves about 2% slower than its wheels turn (1.3% at a 1 m radius, 2.3% at 0.5 m). The rear wheels are forced to exact Ackermann speeds and scrub slightly. Odometry, which counts wheel turns, reads +0.1% on distance over the full test. A real robot shows the same kind of error.

### Odometry error metric

The first version of the check compared final positions and failed with 18%. That metric is misleading: a heading error of a few degrees early on gets multiplied by every metre driven afterwards. Per manoeuvre, the odometry was within 1–4° per half-circle and within millimetres on distance. The check now uses the standard calibration measures, total distance and total turning, compared separately.

## Driving it yourself

```bash
export ROS_DOMAIN_ID=77 IGN_PARTITION=r2sim   # after sourcing; your .bashrc sets 28
ros2 launch rosmaster_r2_sim sim.launch.py
# second terminal, same exports:
python3 ~/teleop_wasd.py
```

`teleop_wasd.py` publishes plain `/cmd_vel` (0.2 m/s, 0.5 rad/s), so it works unchanged.

## Next step

Phase 3: sensors. A lidar matching the real `/scan` (frame `laser`, 10 Hz, front 180°), an IMU on `/imu/data_raw` in `imu_link`, and the Astra Plus depth camera. First check whether `gpu_lidar` renders on the Jetson without the window.
