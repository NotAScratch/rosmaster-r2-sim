# 04: Phase 4, parity with the real stack

*2026-11-03. Built and tested headless on the Jetson Orin NX, Gazebo Fortress 6.18.*

Goal: the real robot's IMU filter and EKF run unchanged against the simulation, and the simulation offers them the same topics, frames, TF tree and odometry the real drivers do. Phase 2's slow-steering gap is closed on the way.

## What was added

| File | Change |
|------|--------|
| `scripts/r2_drive.py` | New drive node, in place of Gazebo's `AckermannSteering` (Phase 2, option B). Stands in for `Ackman_driver_R2` + motor board + `base_node_R2`: `/cmd_vel` → steering angles and rear wheel speeds; `/joint_states` → `/vel_raw`, `/odom_raw`, optional odom TF |
| `scripts/ackermann.py`, `test/test_ackermann.py` | The geometry and odometry maths, with 9 unit tests (`python3 -m pytest rosmaster_r2_sim/test`) |
| `config/r2_drive.yaml` | Drive limits: speed, acceleration, **steering speed**, rates |
| `urdf/r2.gazebo.xacro` | Joint controllers always on (speed on the rear wheels, position on the steering); the plugin is gone |
| `urdf/r2.urdf.xacro` | Steering joint velocity limit 3 → 20 rad/s, so `steer_rate` is what limits the steering |
| `config/bridge_drive.yaml`, `bridge_test.yaml` | The four joint commands; `bridge_odom_tf.yaml` removed (`r2_drive.py` sends that TF) |
| `launch/sim.launch.py` | Starts `r2_drive.py` with geometry read from the URDF; `localization:=true` starts the real Madgwick filter and EKF; refuses `localization:=true publish_odom_tf:=true` |
| `scripts/phase2_check.py` | Three new checks: steering speed, reversing with a turn, `/vel_raw` |
| `scripts/phase4_check.py` | Parity check; `--passive` for the real robot, `--no-camera` |

### The real stack, as run here

| Node | Config | Changes |
|------|--------|---------|
| `imu_filter_madgwick` | `yahboomcar_bringup/param/imu_filter_param.yaml` | `use_sim_time: true` only |
| `robot_localization` `ekf_filter_node`, `/odometry/filtered` → `/odom` | `robot_localization/params/ekf_x1_x3.yaml` | `use_sim_time: true` only (the file says `false`; a later parameter dict overrides it). Not started through `ekf_x1_x3_launch.py`, which has no way to set it |

### Drive settings

| Setting | Value | Source |
|---------|-------|--------|
| Geometry | wheel base 0.23529 m, track 0.1685 m, kingpins 0.130 m, wheel radius 0.0345 m, steering ±0.6 rad | read from the URDF at launch, so Gazebo and the node cannot disagree |
| `max_speed` | 1.0 m/s | real driver's `xlinear_limit` |
| `max_accel` | 1.0 m/s² | **guess** |
| `steer_rate` | 5.0 rad/s | **guess** for a hobby servo, not yet measured |
| `/odom_raw`, `/vel_raw` | 10 Hz | the real driver's 0.1 s timer |
| `/odom_raw` covariances | pose 0.001, twist 0.0001 (x, y, yaw), all else 0 | `base_node_R2.cpp` |
| `/vel_raw` | `linear.x` speed, `linear.y` steering angle in degrees, `angular.z` yaw rate | `Ackman_driver_R2` |
| `/cmd_vel` with `linear.x` below 1 mm/s | steering straight, no motion | a car cannot turn on the spot |
| `/cmd_vel` timeout | none | the real driver holds the last command too |

## Results

### `phase4_check.py`, the real Madgwick + EKF against the simulation

`ros2 launch rosmaster_r2_sim sim.launch.py headless:=true localization:=true`, then `ros2 run rosmaster_r2_sim phase4_check.py`:

| Check | Result |
|-------|--------|
| The real robot's topics and types (`/cmd_vel`, `/vel_raw`, `/odom_raw`, `/imu/data_raw`, `/imu/data`, `/odom`, `/scan`, `/joint_states`, `/tf`, `/tf_static`, camera images) | PASS |
| Frames of `/odom_raw`, `/odom` (`odom → base_footprint`), `/imu/data_raw`, `/imu/data` (`imu_link`), `/scan` (`laser`) | PASS |
| TF `odom → base_footprint → base_link → laser / imu_link / camera_link` | PASS |
| `odom → base_footprint` comes only from the EKF | PASS, 50 transforms over 5 s, all equal to `/odom` at the same stamp |
| `/odom_raw` covariances | PASS, exactly `base_node_R2`'s |
| `/odom_raw` rate | PASS, 10.0 Hz |
| `/imu/data` orientation, unit quaternion | PASS |
| `base_link` level when still, from Madgwick | PASS, worst roll/pitch 0.66° |
| **EKF `/odom` distance** over the drive sequence | PASS, 6.900 m vs 6.884 m true (+0.2%) |
| **EKF `/odom` turning** | PASS, 488.6° vs 477.2° true (+2.4%) |
| `/odom_raw` alone (information) | 6.825 m (−0.9%), 481.4° (+0.9%) |
| EKF yaw jitter while straight or stopped (information) | **59.5° of back-and-forth for a net +7.8°**, see below |

**16/16 checks passed.**

### Drive, `phase2_check.py` on the new drive node

**18/18 passed** (15 from Phase 2, 3 new). The new ones:

| Check | Phase 2 (plugin) | Now |
|-------|------------------|-----|
| Steering speed, 0 → 90% of 0.374 rad | 2.5 s | **0.10 s** (`steer_rate` 5.0 rad/s allows 0.22 s) |
| `/vel_raw` in a steady 0.6 m turn | not published | 0.200 m/s, 21.41° (joints: 21.41°) |
| Reversing while asking to turn left | not checked | yaw rate +0.295 rad/s for +0.300 commanded |

The rest match Phase 2: turn radius 0.612 m (0.611 expected), wheel angles 0.414 / 0.340 rad exactly, tightest turn 0.425 m, top speed 1.000 m/s. `/odom_raw` over the run: distance −0.5%, turning +1.9% (Phase 2: +0.1%, −0.5%). `r2_drive.py` integrates like `base_node_R2`, with the speed applied at `base_footprint` rather than the rear axle, which makes it slightly worse than the plugin's odometry, and more like the real robot's.

### Regression

`phase1_check.py` (with `test_actuators:=true`): 9/9. `phase3_check.py` in `test_room` with `localization:=true`: 17/17.

## What the simulation shows about the real stack, and how it was found

**1. The real EKF config makes `/odom` yaw zig-zag.** The first run of `phase4_check.py` failed with the EKF turning **+20.1%** too far, while `/odom_raw` was within 1.7%. Logging every boundary in a steady turn ruled out a frame or sign error: true yaw rate 0.326 rad/s, `/odom_raw` 0.333, Madgwick (as `base_link`) 0.324, raw gyro z −0.327 (the frame is upside down), EKF 0.338. Logging per manoeuvre then showed where the excess came from. With the robot reversing straight after the turns, the EKF yaw moved back and forth 22.5° for a net −1.3°, while the ground truth moved 1.4°. On standing still at the end it moved 22.6° for a net +0.6°.

The cause is in `ekf_x1_x3.yaml`. It fuses **absolute yaw from two sources**, `/odom_raw` (`odom0_config` yaw true) and the IMU (`imu0_config` yaw true, `relative`). After a few turns the two disagree by a few degrees (here `/odom_raw` +132.8° against the true +130.5°), and the filter swings between them at every update. The check now samples its distance and turning sums once per sim second, the same way for every source. It reports the swing separately, as information.

This is the real robot's config, run unchanged, so the real robot should do the same, and probably worse. Its `base_node_R2` uses a 0.25 m wheel base against the model's 0.2353 m (survey §3, point 3), so its `/odom_raw` under-turns by about 6% and the two yaws drift apart faster. SLAM and Nav2 will see a robot whose heading jitters by a few degrees while it drives straight. The usual fix is to fuse only one absolute yaw: drop yaw from `odom0_config`, or set `odom0_differential: true`. Choosing that is a change to the real robot, so it is left to you, not made here.

**2. `imu_link` is mounted upside down.** The IMU frame sits at `rpy (0, π, π/2)` (Yahboom's model). The simulated raw accelerometer reads z = −9.77 m/s² standing still, and Madgwick reports the orientation of that frame, roll 180° (quaternion (0, −1, 0, 0)). That is correct for the frame. The first "level" check judged `imu_link` instead of `base_link` and failed with 180°. It now composes the orientation with TF. Whether the real board's raw data is really in this flipped frame is a parity question: `phase4_check.py --passive` on the real robot answers it (it will fail "base_link level" if the real data are z-up).

**3. The steering was 25× too slow, now a setting.** Phase 2's plugin took 2.5 s to steer; `r2_drive.py` moves the steering at `steer_rate` like a servo, 0.10 s for the same step. A 3 rad/s velocity limit on the steering joints in the model would have capped it, so it was raised for the simulation.

**4. `r2_drive.py` crashed on Ctrl-C.** The launch file stopping it raised `ExternalShutdownException` (exit −11 under launch). It now exits cleanly.

## Remaining known gaps

- **Not yet measured on the real robot** (the motor board was not connected during this phase): the steering speed (`steer_rate` 5.0 rad/s is a guess), IMU noise (σ 0.002 rad/s, 0.02 m/s², estimates), and a `phase4_check.py --passive` run on the robot. How to do each is in the Phase 4 plan, Task 5.
- The real EKF's yaw zig-zag (above), until its config is changed on the robot.
- The real `base_node_R2` wheel base, 0.25 m against 0.2353 m.
- No `/imu/mag`: the real driver publishes it, nothing uses it (`use_mag: false`).
- No `/cmd_vel` timeout, like the real driver. If teleop dies mid-drive, the robot keeps going, in both.
- `max_accel` 1.0 m/s² is a guess.

## Running it yourself

```bash
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/software/library_ws/install/setup.bash   # robot_localization
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=77 IGN_PARTITION=r2sim
ros2 launch rosmaster_r2_sim sim.launch.py headless:=true localization:=true
# second terminal, same setup:
ros2 run rosmaster_r2_sim phase4_check.py

# On the real robot (simulation stopped, domain 28, bringup and lidar running):
ros2 run rosmaster_r2_sim phase4_check.py --passive --no-camera
```

## Next step

Phase 5: run `slam_toolbox` and Nav2 in the simulation on top of this stack, then the same launch files on the real robot. Before that, measure the steering servo and run the passive check on the robot, and decide on the EKF's yaw sources.
