# 03: Phase 3, sensors

*2026-11-02. Built and tested headless on the Jetson Orin NX, Gazebo Fortress 6.18.*

Goal: simulate the R2's lidar, IMU and depth camera, publishing the same topics, frames and rates as the real drivers, so the real EKF, SLAM and Nav2 configs run unchanged (Phase 4).

## What was added

| File | Change |
|------|--------|
| `urdf/r2.sensors.xacro` | The three sensors: `gpu_lidar`, `imu`, `depth_camera` + `camera`, with the real drivers' topic names and frames |
| `urdf/r2.urdf.xacro` | Astra frame chain (`camera_link`, `camera_color/depth_frame`, optical frames), `imu_link` |
| `config/bridge_sensors.yaml` | `/scan`, `/imu/data_raw` over `ros_gz_bridge` |
| `config/bridge_camera.yaml` | colour/depth images, `camera_info`, `/camera/depth/points` |
| `worlds/test_room.sdf` | A 4 × 4 m room with a crate and a post, known to the checks; also `empty.sdf` got the sensors systems |
| `launch/sim.launch.py` | Sensor/camera bridges, `camera:=` option; spawns from a converted SDF file (see below) |
| `scripts/phase3_check.py` | Automatic checks: rates, TF, lidar accuracy against the room's geometry, IMU against gravity and the true turn rate, camera format and depth |

### Sensor settings

| Setting | Value | Source |
|---------|-------|--------|
| Lidar FOV | −90 … +90°, 1001 beams (0.18° apart) | `ydlidar.yaml` (front half only), 20 kHz / 10 Hz over 360° |
| Lidar range | 0.01 – 12 m | driver `range_min` 0.01; 12 m estimate (9.6 m seen in a room) |
| Lidar noise | Gaussian, σ 10 mm | estimate, typical time-of-flight |
| IMU rate | 10 Hz | the driver's 0.1 s timer |
| IMU noise | σ 0.002 rad/s, 0.02 m/s² | estimate, MPU-class MEMS |
| Camera | 640 × 480 @ 30 fps, FOV 58.4° | `astro_pro_plus.launch.xml`; FOV estimate for the Astra series |
| Depth range | 0.6 – 8 m | Astra series estimate |

### Topics and frames, matching the real robot

| Real robot | Simulation |
|-----------|------------|
| `/scan` frame `laser`, 10 Hz | same (the `laser` frame, at the bringup's static-transform pose) |
| `/imu/data_raw` frame `imu_link`, 10 Hz | same |
| `/camera/color/image_raw` etc., optical frames | same names, via `ignition_frame_id` |
| `odom → base_footprint → base_link → laser/imu/camera` | same tree (`robot_state_publisher` from the same URDF) |

## Results

`ros2 launch rosmaster_r2_sim sim.launch.py world:=test_room`, then `ros2 run rosmaster_r2_sim phase3_check.py`:

| Check | Result |
|-------|--------|
| All topics present | PASS |
| TF to `laser`, `imu_link`, camera optical frames | PASS |
| Scan frame, field of view | PASS, 1001 beams, 0.180° apart |
| **Scan vs. the room, beam by beam** | PASS, 997 beams compared, median error **7.4 mm**, 95% within 20.7 mm |
| Beams blocked by the robot itself | PASS, 4 of 1001 |
| IMU gravity, in `base_link` | PASS, (+0.01, +0.02, +9.80) m/s² |
| IMU still when stopped | PASS, 0.003 rad/s |
| IMU yaw rate while turning | PASS, gyro 0.244 vs true 0.245 rad/s |
| Camera formats, frames, calibration | PASS, 640×480 rgb8 / 32FC1, fx 572.6 |
| Point cloud | PASS, 307200 points |
| Depth straight ahead | PASS, 1.072 m vs 1.072 m expected |

**17/17 checks passed.**

### Rates

Configured like the real drivers: 10 Hz lidar and IMU, 30 fps cameras. Delivery rates are only meaningful when the simulation keeps real time; the check verifies them strictly at a real-time factor ≥ 0.9, and reports them as information below that. On this Jetson, with all sensors, the sim runs at RTF 0.73 and delivers: `/scan` and `/imu/data_raw` 7.3 Hz wall = **exactly 10 Hz in sim time**; the cameras 18–19 Hz wall ≈ 25 Hz sim time (the Orin's GPU cannot render two 640×480 sensors at 30 fps); `/camera/depth/points` ≈ 12 Hz sim time (the bridge converts 307k points per message). Verify 30 fps on a PC-class GPU.

## What the simulation gets wrong, and what it took to find it

**1. The lidar saw its own housing and camera.** The first full run failed "matches the room" with 520 beams of 1001 returning nothing at all, in the whole forward sector. Lowering `range_min` from 0.05 to the real driver's 0.01 turned those into returns of 0.02–0.07 m: the lidar's own housing rim (the scan origin sits inside the housing, like the real 4ROS, which sees out through its window) and the camera body, whose mesh crosses the scan plane about 5 cm ahead of the origin.

The real robot's own scans (mapping repo, Phase 1 CSVs) settle what should happen instead: **86% of the real front half are room returns**, with only small near-field patches (deck edges, camera sides, ~14%). So the real lidar's plane clears the camera; the simulated one must too.

Fortress has the mechanism for this: visuals carry `visibility_flags`, the lidar has a `visibility_mask`, and a standalone world test confirmed flagged visuals are ignored. The housing and the camera body now carry `visibility_flags` 1 and the lidar's mask `4294967294` excludes bit 0.

**2. `create -topic` drops `visibility_flags`.** With the flags in the model, the lidar *still* saw the housing. The flags survive `ign sdf -p` conversion, they are in the SDF the spawn should use — but spawning through the `robot_description` **topic** loses them, while loading the same model from a **file** (a world file, or `create -file`) keeps them, verified each way with a minimal world. So `sim.launch.py` now converts the xacro to SDF at launch time and spawns from the file. `robot_state_publisher` still gets the URDF, so RViz and TF are unchanged.

**3. Sensor rates are meaningless below real time.** At RTF 0.5, Fortress delivers the sensors at wall rates unrelated to the configured ones (measured: `/scan` anywhere from 7 to 17 Hz wall across runs, while sim-time stamps suggested 19 Hz). The check therefore counts messages per wall-clock second and only enforces the configured rates at RTF ≥ 0.9. A leftover Gazebo from an earlier experiment also stole GPU for a while and produced the wildest numbers of the day.

### Remaining known gaps

- The real robot's scan has near-field patches (deck edges, camera sides) in ~14% of the front-half beams; the simulated robot, with its housing and camera masked, has 4 such beams. SLAM filters these anyway (`min_range`), so the difference is accepted for now.
- Camera noise, and IMU/lidar noise values generally, are estimates; check against datasheets or measured data (Phase 4).
- The 4ROS's true scan plane height relative to the camera should be measured on the robot (a ruler settles it), along with the survey's other open poses.
- 30 fps camera delivery and `/camera/depth/points` at 30 Hz need a PC-class computer; the Jetson tops out around 25 and 12 Hz in sim time.

## Running it yourself

```bash
export ROS_DOMAIN_ID=77 IGN_PARTITION=r2sim   # after sourcing; your .bashrc sets 28
ros2 launch rosmaster_r2_sim sim.launch.py headless:=true world:=test_room
# second terminal, same exports:
ros2 run rosmaster_r2_sim phase3_check.py
```

`camera:=false` drops the depth camera and its bridges (the sim then runs much faster), `test_actuators:=true` keeps the Phase 1 rig.

## Next step

Phase 4, parity: run the real EKF config against the simulation, match `/vel_raw` if anything needs it, and time the real steering servo to decide on the Phase 2 plugin replacement.
