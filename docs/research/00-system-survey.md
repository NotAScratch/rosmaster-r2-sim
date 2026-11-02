# 00: System survey

*2026-11-02. Read from the robot itself (Jetson Orin NX, `ROBOT_TYPE=r2`, `RPLIDAR_TYPE=4ROS`, `CAMERA_TYPE=astraplus`, `ROS_DOMAIN_ID=28`).*

This entry records what the simulation has to copy: the real robot's geometry, its ROS 2 topics and frames, and the software on the Jetson. It also lists the problems in Yahboom's robot model that stop it from working in Gazebo as it is.

All file paths are on the robot, under `~/yahboomcar_ros2_ws/yahboomcar_ws/src/` unless stated otherwise.

---

## 1. Software on the robot

| Item | Found |
|------|-------|
| OS | Ubuntu 22.04.5, Jetson Linux R36.4.3 |
| Board | Jetson Orin NX 8 GB, Super dev kit, `MAXN_SUPER` power mode. 7.4 GB RAM shared between CPU and GPU |
| ROS 2 | Humble (`/opt/ros/humble`) |
| Gazebo | Fortress, `libignition-gazebo6` 6.18.0, CLI `ign gazebo` |
| Bridge | `ros-humble-ros-gz`, `ros-gz-bridge`, `ros-gz-sim`, `ros-gz-image`, `ros-gz-interfaces`, `ros-gz-sim-demos` |
| Rendering | `ignition-rendering6` with both `ogre` and `ogre2` (OGRE-Next 2.2.5). `ogre2` is needed for `gpu_lidar` |
| Already installed | `slam_toolbox`, `cartographer_ros`, `nav2_bringup`, `robot_localization`, `imu_filter_madgwick`, `xacro` |
| **Not** installed | `gazebo_ros` / Gazebo Classic. Fine: Classic is end-of-life and Fortress is the Humble default |

Gazebo system plugins present in `/usr/lib/aarch64-linux-gnu/ign-gazebo-6/plugins/`:

- `ackermann-steering-system`: drive and steering for the R2
- `odometry-publisher-system`: ground-truth odometry, if wanted
- `imu-system`, `sensors-system`: IMU, lidar, camera
- `diff-drive-system`: not needed (the R2 is not differential drive)

Nothing needs to be installed to start. `ros2_control`/`gz_ros2_control` is **not** installed. The plan below avoids it on purpose (see §6).

## 2. Robot geometry

Source: `yahboomcar_description/urdf/yahboomcar_R2.urdf.xacro`. It was exported from SolidWorks, then edited by hand. The **xacro** is what the bringup launch loads. The plain `yahboomcar_R2.urdf` next to it is older, and its values are different (for example, its steering limits are 0/0). Always use the xacro.

### Measured from the model

| Quantity | Value | How |
|----------|-------|-----|
| Wheelbase (rear axle to front steering pivots) | **0.2353 m** | 0.11922 + 0.11607 (joint origins) |
| Rear track | 0.1685 m | 2 × 0.08425 |
| Front steering pivot spacing (kingpins) | 0.130 m | 2 × 0.065 |
| Front track (wheel centres) | ≈ 0.172 m | pivot ± hub offsets (0.0228, 0.0195) |
| Wheel radius | **0.0345 m** | wheel mesh bounding box 69 × 26.5 × 69 mm |
| Wheel width | 0.0265 m | same |
| Steering limit | ±0.6 rad (±34.4°) | `front_*_steer_joint` `<limit>` |
| Minimum turning radius (rear axle centre) | ≈ 0.34 m | L / tan(0.6) |
| `base_footprint → base_link` | z = 0.0215 m | `base_joint` |
| Chassis mesh bounding box | 0.342 × 0.165 × 0.231 m | `base_link.STL` (11 MB) |
| Total mass in model | **0.99 kg** | sum of the `<inertial>` masses |

The wheels sit on the ground to within 1 mm (rear wheel bottom at z = 0.0215 + 0.0125 − 0.0345 ≈ 0). The ride height is consistent.

**To verify on the real robot:** total mass (0.99 kg seems light for a chassis with a Jetson, battery, lidar and camera. Weigh it), real maximum steering angle, and maximum speed.

### Frames in the model

```
base_footprint
└── base_link                      (z +0.0215)
    ├── back_left_link / back_right_link      continuous (rear wheels)
    ├── front_left_steer  → front_left_wheel  revolute ±0.6 → continuous
    ├── front_right_steer → front_right_wheel revolute ±0.6 → continuous
    ├── camera_link   fixed  (0.095, 0, 0.0996)
    ├── laser_link    fixed  (0.0125, 0, 0.131), yaw π
    └── imu_link      fixed  (−0.05, 0, 0.05), rpy (0, π, π/2)
```

## 3. How the real robot's software fits together

Started by `yahboomcar_bringup/launch/yahboomcar_bringup_R2_launch.py`. The lidar comes from `yahboomcar_nav/launch/laser_bringup_launch.py`.

```
/cmd_vel (Twist)
   │
   ▼
Ackman_driver_R2 (Python, talks to the motor board over serial)
   ├── /vel_raw       Twist: linear.x = speed (m/s), linear.y = steering angle in DEGREES
   ├── /imu/data_raw  Imu, frame imu_link
   ├── /imu/mag       MagneticField
   ├── /joint_states  steering angle; wheel angles are random numbers (only for show)
   └── /voltage, /edition
   │
   ▼
base_node_R2 (C++): integrates /vel_raw with a bicycle model, wheelbase parameter default 0.25
   └── /odom_raw      Odometry odom → base_footprint (pub_odom_tf=false in bringup)

/imu/data_raw ─► imu_filter_madgwick ─► /imu/data
/odom_raw + /imu/data ─► robot_localization EKF (ekf_x1_x3.yaml, two_d_mode) ─► /odom + TF odom → base_footprint

ydlidar_ros2_driver (ydlidar.yaml) ─► /scan, frame "laser"
static_transform_publisher base_link → laser  (0.0435, 0, 0.11), yaw 0 for the 4ROS
robot_state_publisher ─► TF for everything in the model
```

What this means for the simulation:

1. **What to replace.** The simulation stands in for `Ackman_driver_R2` + `base_node_R2` + the lidar driver. It must publish `/odom_raw`, `/imu/data_raw` and `/scan` in the real frames. Then Madgwick, the EKF, SLAM and Nav2 can all run unchanged. The simulation must **not** publish `odom → base_footprint` itself, or it will fight the EKF. Add a launch option to turn that on when running without the EKF.
2. **`/cmd_vel` meaning.** On the real robot, `linear.x` is speed and `angular.z` is a turn-rate request that the motor board firmware turns into a steering angle. The Gazebo Ackermann plugin also takes a `Twist` with `linear.x` + `angular.z` and works out the steering angle itself. So the two should behave alike, but the conversion constants should be compared with drive tests.
3. **Odometry wheelbase mismatch (affects the real robot too).** `base_node_R2` uses `wheelbase = 0.25` (not set in the launch file), but the model says 0.2353. That makes the real `/odom_raw` underestimate the turn rate by about 6%. The EKF partly hides this with the IMU yaw rate. Worth fixing on the real robot. The simulation should use 0.2353.
4. **Two lidar frames.** The model has `laser_link` at (0.0125, 0, 0.131) turned 180°. The real scans are stamped `laser`, placed by a separate static transform at (0.0435, 0, 0.11) with no turn. The real stack uses `laser`, so the simulated lidar should publish in `laser` at that pose. Which pose is physically right should be measured on the robot. The mapping repo's Phase 1 calibration found the 4ROS's raw "front" is at ≈303° and its angles run clockwise. The ROS driver's `reversion`/`inverted` flags correct for that, so `/scan` should already be in the standard ROS direction (counter-clockwise, x forward). Check this before trusting the static transform.

### Lidar settings (real `/scan`)

From `ydlidar_ros2_driver/params/ydlidar.yaml`: frame `laser`, **10 Hz**, angle −90° … +90° (**front half only**), `range_min` 0.01, `range_max` 50 (the driver's limit, not the sensor's real range). `fixed_resolution: true`, `sample_rate: 20`.

To match the real robot, the simulated lidar should start with a 180° field of view and a realistic range. The real 4ROS range and resolution must come from its datasheet. The half-circle view is a driver setting and can be widened on both the real and simulated robot later.

### Camera

`CAMERA_TYPE=astraplus` (Orbbec Astra Plus, driver `ros2_astra_camera`). It should be simulated as an `rgbd_camera` on `camera_link`. Resolution and depth range are to be taken from the `ros2_astra_camera` launch files for `astraplus`. They were not surveyed yet.

## 4. Problems in Yahboom's model that stop it working in Gazebo

| # | Problem | Effect in Gazebo | Fix |
|---|---------|------------------|-----|
| 1 | The Gazebo include is commented out, and `yahboomcar_R2.gazebo.xacro` does not exist | No plugins, sensors, friction or colours | Write our own `r2.gazebo.xacro` |
| 2 | Every collision is a full-detail mesh (chassis STL is 11 MB) | Slow, jittery physics; mesh wheels are not round | Box for the chassis, cylinders for the wheels |
| 3 | Front-left wheel axis is `0 −1 0`, the other three are `0 1 0` | One front wheel spins backwards for a "forward" command | Flip it so all four axes match. Only for the sim wrapper; do not edit Yahboom's package |
| 4 | `laser_link` has mass 0 and inertia 0 | sdformat warnings; Gazebo may drop the link | Small real mass (≈0.1 kg) |
| 5 | Total mass ≈ 1 kg, steering links weigh 2 g | Tiny inertias make the simulation unstable at large time steps | Use the measured mass and set a minimum inertia for the steering links |
| 6 | `imu_link` has no inertial and sits under a `π` roll | Fine for TF, but the IMU sensor must be attached to `base_link` with that pose | Put the IMU sensor on `base_link` with the `imu_link` pose |
| 7 | No friction settings | Wheels slide; a car-like robot needs grip to steer | `mu1/mu2` ≈ 1.0 on the wheels, low friction on the chassis |

**Approach:** do not edit Yahboom's package. This repo gets its own `rosmaster_r2_sim` package with a wrapper xacro. It reuses Yahboom's visual meshes through `package://yahboomcar_description/...` and supplies its own collision shapes, inertials and Gazebo tags. Yahboom's files then stay out of this repo (they carry no licence file), and the real robot's bringup is untouched.

## 5. Running on the Jetson

- Gazebo Fortress's window runs on the Orin's GPU, but slowly. 8 GB shared RAM is tight with RViz, SLAM and the Jetson desktop all running.
- Plan: **run the simulation without its window** on the Jetson (`ign gazebo -s -r`) and watch it in RViz. Check whether `gpu_lidar`/camera sensors render without a display on JetPack 6 (Fortress has a `--headless-rendering` EGL option). If they don't, use a CPU-free fallback (keep the display and hide the window, or run on a PC).
- Use the **same `ROS_DOMAIN_ID=28`** only when the real robot is off, or use a different domain for the simulation, so simulated and real `/cmd_vel` and `/scan` never mix.
- Every node in the simulation needs `use_sim_time: true`. The real EKF config has `use_sim_time: false` written in, so it must be overridden from the launch file.

## 6. Design decisions so far

| Decision | Choice | Why |
|----------|--------|-----|
| Simulator | Gazebo Fortress | It's the Humble default and already installed; Classic is end-of-life |
| Drive | Gazebo's own `AckermannSteering` system | Already installed, no extra packages. `gz_ros2_control` + `ackermann_steering_controller` would match some tutorials but isn't installed and adds a lot for little gain at this stage. Revisit if Nav2 needs controller feedback |
| Model | Wrapper xacro around Yahboom's meshes | Keeps vendor files out of this repo and the real robot's software untouched |
| Interface | Copy the **real** topics (`/odom_raw`, `/imu/data_raw`, `/scan`, frame `laser`) | Lets the real EKF/SLAM/Nav2 configs run unchanged |

## 7. Open questions

1. Real mass, max steering angle and max speed of the R2 (weigh and measure).
2. Which lidar pose is right: the model's `laser_link` or the bringup's `laser` transform? (Measure with a ruler.)
3. 4ROS datasheet: range, angular resolution, noise, so the simulated lidar matches.
4. Astra Plus resolution, field of view and depth range from `ros2_astra_camera`.
5. Do `gpu_lidar` and `rgbd_camera` render without a display on JetPack 6 / Fortress?
6. Simulation on the Jetson, on a PC, or both? This decides how much effort goes into running without a window.

## Next step

Phase 1: create the `rosmaster_r2_sim` package with the wrapper xacro (§4), spawn it in an empty Fortress world and check it sits still, rolls straight, and its steering joints move.
