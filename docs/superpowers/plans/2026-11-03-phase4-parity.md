# Phase 4: Parity with the Real Stack — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The real robot's Madgwick filter and EKF run unchanged against the simulation, fed by a drive node that copies the real driver + `base_node_R2` split (fast, settable steering; `/odom_raw` and `/vel_raw` like the real ones), and a check proves the simulated topics, frames and TF tree match the real robot's.

**Architecture:** Phase 2's option B. Gazebo's `AckermannSteering` system is replaced by two always-on joint controllers per axle (`JointPositionController` on the steering joints, `JointController` on the rear wheels) and a ROS node, `r2_drive.py`, that turns `/cmd_vel` into rate-limited steering angles and wheel speeds and integrates `/odom_raw` from `/joint_states` the way `base_node_R2` does. Pure math lives in `ackermann.py` with pytest unit tests; ROS behaviour is proven by the existing check-script pattern (`phase2_check.py`, new `phase4_check.py`). A `localization:=true` launch option starts the real configs straight from their installed packages.

**Tech Stack:** ROS 2 Humble, Gazebo Fortress 6.18, `ros_gz_bridge`, rclpy, numpy, pytest 6.2.5, `robot_localization` (from `~/yahboomcar_ros2_ws/software/library_ws`), `imu_filter_madgwick` (`/opt/ros/humble`).

**Spec:** README roadmap, Phase 4 line: *"match the real robot's topics and TF tree, run the real EKF config against the simulation"*; `docs/research/02-phase2-drive.md` §"What the simulated robot gets wrong" 1 (option B, recommended for Phase 4); `docs/research/03-phase3-sensors.md` §"Remaining known gaps" and §"Next step"; `docs/research/00-system-survey.md` §3 (real data flow), §5 (`use_sim_time` override).

## Global Constraints

- Do not edit Yahboom's or `robot_localization`'s files. The real configs are loaded from their installed share directories: `yahboomcar_bringup/param/imu_filter_param.yaml`, `robot_localization/params/ekf_x1_x3.yaml`.
- The real EKF config hard-codes `use_sim_time: false`; override it to `true` from the launch file (a later parameter dict wins).
- The EKF owns `odom → base_footprint`. Nothing else may publish it unless `publish_odom_tf:=true`, and that option is refused together with `localization:=true`.
- Geometry (from `r2.urdf.xacro`): `wheel_base` 0.23529 m, `wheel_separation` 0.1685 m, `kingpin_width` 0.130 m, `wheel_radius` 0.0345 m, `steer_limit` 0.6 rad, `max_speed` 1.0 m/s, `max_accel` 1.0 m/s² (guess).
- `/odom_raw` like `base_node_R2`: frame `odom` → `base_footprint`; pose covariance `[0]=[7]=[35]=0.001`, twist covariance `[0]=[7]=[35]=0.0001`, all else 0; `twist.linear.y = 0`; published once per driver tick at **10 Hz** (the driver's 0.1 s timer).
- `/vel_raw` like `Ackman_driver_R2`: `geometry_msgs/Twist`, `linear.x` speed m/s, `linear.y` steering angle in **degrees**, `angular.z` the yaw rate.
- Every node gets `use_sim_time: true`. Every run uses `export ROS_DOMAIN_ID=77 IGN_PARTITION=r2sim`, except the real-robot task (domain 28, sim stopped).
- Sourcing for every command: `source /opt/ros/humble/setup.bash && source ~/yahboomcar_ros2_ws/software/library_ws/install/setup.bash && source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash && source install/setup.bash`.
- Existing checks must keep passing: `phase1_check.py` (with `test_actuators:=true`), `phase2_check.py`, `phase3_check.py` 17/17.
- Write-ups follow the existing style in `docs/research/0N-*.md`: plain sentences, tables, measured numbers, "what the simulation gets wrong".

## Review Focus

1. `/cmd_vel` with `linear.x = 0`, `angular.z ≠ 0` (Nav2 rotate-in-place, teleop "turn" key alone): no division by zero, steering goes to 0, the robot does not move. → `test_zero_speed_turn_request` (Task 1).
2. Reverse with a turn request (`v < 0`, `w > 0`): the robot's yaw rate must have the sign of `w`, as with the plugin. → `test_reverse_keeps_yaw_sign` (Task 1), `reverse turn` check (Task 3).
3. Huge `w` at tiny `v` (e.g. `v = 1e-4`, `w = 1.0`): steering saturates so the *inner* wheel is at ±0.6 rad, no NaN. → `test_saturation_inner_wheel` (Task 1).
4. `/joint_states` with a repeated or older stamp (sim paused, bridge restart): odometry must not integrate a zero or negative `dt`. → `test_odometry_ignores_nonpositive_dt` (Task 1).
5. `localization:=true publish_odom_tf:=true` together: two publishers would fight over `odom → base_footprint`; the launch must stop with a clear message. → Task 4 step 4.

---

## File Structure

| File | Responsibility |
|------|----------------|
| `rosmaster_r2_sim/scripts/ackermann.py` (create) | Pure math: wheel angles, wheel speeds, rate limit, bicycle-equivalent angle, `base_node_R2`-style odometry. No ROS imports |
| `rosmaster_r2_sim/test/test_ackermann.py` (create) | pytest unit tests for `ackermann.py` |
| `rosmaster_r2_sim/scripts/r2_drive.py` (create) | ROS node: `/cmd_vel` → joint commands; `/joint_states` → `/odom_raw`, `/vel_raw`, optional odom TF |
| `rosmaster_r2_sim/urdf/r2.gazebo.xacro` (modify) | Drop `AckermannSteering`; joint controllers always on |
| `rosmaster_r2_sim/config/bridge_drive.yaml` (modify) | Four joint command topics ROS → GZ (replaces `/cmd_vel` and odometry) |
| `rosmaster_r2_sim/config/bridge_test.yaml` (modify) | Same Gazebo topics, under the old `/test/...` ROS names |
| `rosmaster_r2_sim/config/bridge_odom_tf.yaml` (delete) | The odom TF now comes from `r2_drive.py` |
| `rosmaster_r2_sim/launch/sim.launch.py` (modify) | Starts `r2_drive.py`; `localization:=` option; refuses the TF conflict |
| `rosmaster_r2_sim/scripts/phase2_check.py` (modify) | Adds steering speed, reverse-turn and `/vel_raw` checks |
| `rosmaster_r2_sim/scripts/phase4_check.py` (create) | Parity check: topics, types, frames, covariances, TF ownership; EKF vs ground truth; `--passive` mode for the real robot |
| `rosmaster_r2_sim/CMakeLists.txt`, `package.xml` (modify) | Install new scripts; `exec_depend` on `robot_localization`, `imu_filter_madgwick`, `yahboomcar_bringup`, `geometry_msgs`, `tf2_ros` |
| `docs/research/04-phase4-parity.md` (create), `README.md` (modify) | Write-up, status line, roadmap tick |

---

### Task 1: Ackermann math, unit tested

**Files:**
- Create: `rosmaster_r2_sim/scripts/ackermann.py`
- Test: `rosmaster_r2_sim/test/test_ackermann.py`

**Interfaces:**
- Produces (all floats in SI units, radians):
  - `bicycle_angle_for(v: float, w: float, wheel_base: float) -> float` — `atan(wheel_base * w / v)`; returns `0.0` when `abs(v) < 1e-3`.
  - `wheel_angles(delta: float, wheel_base: float, kingpin_width: float, limit: float) -> tuple[float, float, float]` — returns `(left, right, delta_used)`. Clamps `delta` so `max(abs(left), abs(right)) <= limit`; `delta_used` is the clamped bicycle angle. Positive = left turn.
  - `rear_wheel_speeds(v: float, delta: float, wheel_base: float, wheel_separation: float, wheel_radius: float) -> tuple[float, float]` — `(left, right)` joint speeds in rad/s; inner wheel slower: `v * (1 ∓ (sep/2)·tan(delta)/wheel_base) / radius`.
  - `bicycle_angle_from(left: float, right: float, kingpin_width: float, wheel_base: float) -> float` — inverse of `wheel_angles` (use `cot δ = (cot δl + cot δr)/2`, and `0.0` when both are ~0).
  - `step_toward(current: float, target: float, max_rate: float, dt: float) -> float` — rate limit.
  - `class BicycleOdometry(wheel_base: float)` with attributes `x, y, yaw, w` and `update(v: float, delta: float, stamp: float) -> bool` — integrates like `base_node_R2.cpp` (`w = v·tan(δ)/L`, then `x += v cos(yaw) dt`, `y += v sin(yaw) dt`, `yaw += w dt`); the first call only stores the stamp; returns `False` (and changes nothing) when `stamp <= last stamp`.

- [ ] **Step 1: Write the failing tests** in `rosmaster_r2_sim/test/test_ackermann.py`. Import with `sys.path.insert(0, <repo>/rosmaster_r2_sim/scripts)`. Constants `L=0.23529, K=0.130, SEP=0.1685, R=0.0345, LIM=0.6`.

```python
def test_straight():
    assert bicycle_angle_for(0.2, 0.0, L) == 0.0
    assert wheel_angles(0.0, L, K, LIM) == (0.0, 0.0, 0.0)
    l, r = rear_wheel_speeds(0.2, 0.0, L, SEP, R)
    assert l == pytest.approx(0.2 / R) and r == pytest.approx(0.2 / R)

def test_phase2_turn_geometry():
    # Phase 2 measured these exact values for a 0.6 m rear-axle radius
    d = bicycle_angle_for(0.2, 0.2 / 0.6, L)
    left, right, used = wheel_angles(d, L, K, LIM)
    assert left == pytest.approx(0.414, abs=1e-3) and right == pytest.approx(0.340, abs=1e-3)
    assert bicycle_angle_from(left, right, K, L) == pytest.approx(used, abs=1e-9)

def test_zero_speed_turn_request():
    assert bicycle_angle_for(0.0, 0.5, L) == 0.0
    assert rear_wheel_speeds(0.0, 0.0, L, SEP, R) == (0.0, 0.0)

def test_reverse_keeps_yaw_sign():
    d = bicycle_angle_for(-0.2, 0.3, L)
    odo = BicycleOdometry(L); odo.update(-0.2, d, 0.0); odo.update(-0.2, d, 1.0)
    assert odo.w == pytest.approx(0.3, rel=1e-6) and odo.yaw > 0

def test_saturation_inner_wheel():
    left, right, used = wheel_angles(bicycle_angle_for(1e-4, 1.0, L), L, K, LIM)
    assert left == pytest.approx(LIM) and 0 < right < LIM and not math.isnan(used)
    left, right, _ = wheel_angles(-1.5, L, K, LIM)
    assert right == pytest.approx(-LIM)

def test_step_toward():
    assert step_toward(0.0, 0.5, 5.0, 0.02) == pytest.approx(0.1)
    assert step_toward(0.45, 0.5, 5.0, 0.02) == pytest.approx(0.5)
    assert step_toward(0.0, -0.5, 5.0, 0.02) == pytest.approx(-0.1)

def test_odometry_matches_base_node():
    odo = BicycleOdometry(L)
    for i in range(101):                       # 10 s at 10 Hz, 0.2 m/s straight
        odo.update(0.2, 0.0, i * 0.1)
    assert (odo.x, odo.y, odo.yaw) == pytest.approx((2.0, 0.0, 0.0), abs=1e-9)

def test_odometry_ignores_nonpositive_dt():
    odo = BicycleOdometry(L); odo.update(0.2, 0.0, 1.0); odo.update(0.2, 0.0, 2.0)
    assert odo.update(0.2, 0.0, 2.0) is False and odo.update(0.2, 0.0, 1.5) is False
    assert odo.x == pytest.approx(0.2)
```

- [ ] **Step 2: Run, expect failure**
Run: `python3 -m pytest rosmaster_r2_sim/test -v`
Expected: collection error, `ModuleNotFoundError: No module named 'ackermann'`.

- [ ] **Step 3: Implement the interfaces above in `rosmaster_r2_sim/scripts/ackermann.py`.** For `wheel_angles`: `left = atan(L / (L/tan δ − K/2))`, `right = atan(L / (L/tan δ + K/2))` for δ ≠ 0; the clamp limit on δ is the δ that puts the inner wheel at `limit`, i.e. `atan(L / (L/tan(limit) + K/2))`. Module docstring: "Ackermann geometry and base_node_R2-style odometry for r2_drive.py. No ROS imports, so it can be unit tested."

- [ ] **Step 4: Run, expect pass**
Run: `python3 -m pytest rosmaster_r2_sim/test -v`
Expected: 8 passed.

- [ ] **Step 5: Commit**
```bash
git add rosmaster_r2_sim/scripts/ackermann.py rosmaster_r2_sim/test/test_ackermann.py
git commit -m "Phase 4: Ackermann math with unit tests"
```

---

### Task 2: `r2_drive.py` node and joint controllers in Gazebo

**Files:**
- Create: `rosmaster_r2_sim/scripts/r2_drive.py`
- Modify: `rosmaster_r2_sim/urdf/r2.gazebo.xacro` (drive and test-rig blocks), `rosmaster_r2_sim/urdf/r2.urdf.xacro` (new `steer_rate` property)
- Modify: `rosmaster_r2_sim/config/bridge_drive.yaml`, `rosmaster_r2_sim/config/bridge_test.yaml`
- Delete: `rosmaster_r2_sim/config/bridge_odom_tf.yaml`
- Modify: `rosmaster_r2_sim/launch/sim.launch.py`, `CMakeLists.txt`, `package.xml`

**Interfaces:**
- Consumes: everything in Task 1's Produces block (import `ackermann` from the script's own directory; `install(PROGRAMS ...)` must also install `ackermann.py` next to it).
- Produces:
  - Gazebo topics (`ignition.msgs.Double`): `/model/r2/joint/front_left_steer_joint/cmd_pos`, `/model/r2/joint/front_right_steer_joint/cmd_pos`, `/model/r2/joint/back_left_joint/cmd_vel`, `/model/r2/joint/back_right_joint/cmd_vel`. ROS names in `bridge_drive.yaml`: `/r2_drive/<joint>/cmd_pos|cmd_vel` (`std_msgs/Float64`). `bridge_test.yaml` keeps its ROS names `/test/<joint>/...` and only changes `gz_topic_name`, so `phase1_check.py` is untouched.
  - Node `r2_drive` (executable `r2_drive.py`). Parameters (defaults): `wheel_base` 0.23529, `wheel_separation` 0.1685, `kingpin_width` 0.130, `wheel_radius` 0.0345, `steer_limit` 0.6, `max_speed` 1.0, `max_accel` 1.0, `steer_rate` 5.0 rad/s (guess, until Task 5), `control_rate` 50.0 Hz, `odom_rate` 10.0 Hz, `publish_odom_tf` false.
  - Topics: subscribes `/cmd_vel` (Twist), `/joint_states`; publishes `/odom_raw`, `/vel_raw` (Global Constraints), `/tf` `odom → base_footprint` only if `publish_odom_tf`.
  - Launch: `r2_drive` starts unless `test_actuators:=true`; `publish_odom_tf:=` is passed to it as a parameter.

- [ ] **Step 1: Gazebo side.** In `r2.gazebo.xacro`, remove the `AckermannSteering` block and make the four controllers from the test rig unconditional, on the `/model/r2/joint/<joint>/...` topics. Steering controller: `use_velocity_commands` true, `p_gain` 20, `cmd_max` 20 / `cmd_min` -20 (fast enough that `steer_rate` in the node is what limits the steering, not the controller). Add `<xacro:property name="steer_rate" value="5.0"/>` beside `max_accel` in `r2.urdf.xacro` with the comment "rad/s, guess; measure the real servo (Phase 4 Task 5)" — the launch file reads it into the node parameter so there is one source of truth.

- [ ] **Step 2: Write `r2_drive.py`.** Control timer at `control_rate`: speed = `step_toward(speed, clamp(cmd.v, ±max_speed), max_accel, dt)`; δ target from `bicycle_angle_for` then `wheel_angles`; current δ = `step_toward(δ, target, steer_rate, dt)`; publish `wheel_angles(δ)` to the steering topics and `rear_wheel_speeds(speed, δ)` to the wheel topics. On `/joint_states`: v = mean rear wheel velocity × radius, δ = `bicycle_angle_from(steer joint positions)`, `odometry.update(v, δ, msg stamp)`. Odometry timer at `odom_rate` publishes `/odom_raw`, `/vel_raw` (δ in degrees) and the optional TF, all stamped with the latest `/joint_states` stamp. No `/cmd_vel` timeout: the real `Ackman_driver_R2` holds the last command too (note it in the write-up). Docstring names the real nodes it replaces.

- [ ] **Step 3: Bridges, launch, build files.** `bridge_drive.yaml`: the four ROS → GZ joint topics only. Delete `bridge_odom_tf.yaml` and its use in the launch file. Launch file: add the `r2_drive` Node with `use_sim_time` true, the geometry from the xacro (read `steer_rate`, `max_speed`, `max_accel` by evaluating the xacro properties, or pass them as literal defaults that match — pick one and comment it). `CMakeLists.txt`: install `ackermann.py`, `r2_drive.py`, `phase4_check.py` (Task 4 creates the last; add it then). `package.xml`: `geometry_msgs`, `tf2_ros`.

- [ ] **Step 4: Build and run the existing checks**
```bash
colcon build --symlink-install && source install/setup.bash
export ROS_DOMAIN_ID=77 IGN_PARTITION=r2sim
ros2 launch rosmaster_r2_sim sim.launch.py headless:=true camera:=false &
ros2 run rosmaster_r2_sim phase2_check.py
```
Expected: `15/15 checks passed`. The turn radius check may now pass with less margin or need its settle time shortened — do not loosen any tolerance; if one fails, debug with superpowers:systematic-debugging. Then stop the sim, relaunch with `test_actuators:=true` and run `phase1_check.py`: all pass.

- [ ] **Step 5: Commit**
```bash
git add -A rosmaster_r2_sim
git commit -m "Phase 4: r2_drive node replaces the AckermannSteering plugin"
```

---

### Task 3: Drive checks for the new behaviour

**Files:**
- Modify: `rosmaster_r2_sim/scripts/phase2_check.py`

**Interfaces:**
- Consumes: `/vel_raw`, `/odom_raw`, `/joint_states` from Task 2; the existing `drive()` / `check` helpers in `phase2_check.py`.

- [ ] **Step 1: Add three checks.**
  - `steering speed`: from straight, command the 0.6 m turn; log `bicycle_angle_from` of the steer joints at every tick; PASS if 90% of the final angle is reached within `0.9·Δ/steer_rate + 0.15` s (Δ = final angle; `steer_rate` read from the running `r2_drive` node's parameter via `rclpy` parameter client, so Task 5's change keeps the check valid). Report the measured time. Phase 2's plugin took 2.5 s.
  - `reverse turn`: `v = -0.2`, `w = +0.3` for 4 s; PASS if the ground-truth yaw rate is positive and within 10% of 0.3.
  - `/vel_raw`: during the steady 0.6 m turn, PASS if `linear.x` within 0.01 m/s of 0.2, and `linear.y` within 0.5° of `degrees(bicycle_angle_from(joint angles))`.
- [ ] **Step 2: Run** (sim as in Task 2 step 4).
Expected: `18/18 checks passed`, steering 90% time ≈ 0.1 s at `steer_rate` 5.0.
- [ ] **Step 3: Commit** — `git commit -am "Phase 4: drive checks for steering speed, reverse turns and /vel_raw"`

---

### Task 4: Real Madgwick + EKF against the simulation, and the parity check

**Files:**
- Modify: `rosmaster_r2_sim/launch/sim.launch.py`, `package.xml`, `CMakeLists.txt`
- Create: `rosmaster_r2_sim/scripts/phase4_check.py`

**Interfaces:**
- Consumes: Task 2's `/odom_raw`, `/vel_raw`, launch `publish_odom_tf`; Phase 3's `/imu/data_raw`, `/scan`, camera topics; `/ground_truth/odom`.
- Produces: launch option `localization` (default `false`); with it, nodes `imu_filter_madgwick` and `ekf_filter_node` (names as on the real robot), `/imu/data`, `/odom`, TF `odom → base_footprint` from the EKF. `phase4_check.py [--passive]`.

- [ ] **Step 1: Launch option.** With `localization:=true`, add:
  - `imu_filter_madgwick_node` (package `imu_filter_madgwick`, name `imu_filter_madgwick`), `parameters=[<yahboomcar_bringup share>/param/imu_filter_param.yaml, {'use_sim_time': True}]`.
  - `ekf_node` (package `robot_localization`, name `ekf_filter_node`), `parameters=[<robot_localization share>/params/ekf_x1_x3.yaml, {'use_sim_time': True}]`, `remappings=[('/odometry/filtered', '/odom')]` — the same as `ekf_x1_x3_launch.py`, which cannot be included as-is because it does not take `use_sim_time`.
  - `package.xml`: `exec_depend` on `robot_localization`, `imu_filter_madgwick`, `yahboomcar_bringup`.

- [ ] **Step 2: Write `phase4_check.py`**, same structure as `phase3_check.py` (`check()`, `info()`, results tally, exit code 1 on any FAIL). Checks:
  - **Topics and types** (both modes): `/cmd_vel` Twist, `/vel_raw` Twist, `/odom_raw` Odometry, `/imu/data_raw` Imu, `/imu/data` Imu, `/odom` Odometry, `/scan` LaserScan, `/joint_states` JointState, `/tf`, `/tf_static`, `/camera/color/image_raw`, `/camera/depth/image_raw` Image. Missing or wrong type = FAIL. Report (INFO) every other topic each side has, so a later diff of the two runs shows extras such as the real `/imu/mag`.
  - **Frames** (both): `/odom_raw` and `/odom` `odom → base_footprint`; `/imu/data_raw` and `/imu/data` `imu_link`; `/scan` `laser`.
  - **TF tree** (both): `odom → base_footprint → base_link → {laser, imu_link, camera_link}` resolvable; exactly one publisher node of `odom → base_footprint`: for 5 s, every such transform on `/tf` must equal the latest `/odom` pose within 1 mm / 0.001 rad at the same stamp (a second publisher shows up as mismatches).
  - **`/odom_raw` covariances and rate** (both): the Global Constraints values; rate 10 Hz ± 1 in message stamps.
  - **EKF follows the truth** (active mode only, skipped with `--passive`): drive the Phase 2 manoeuvre sequence (reuse the list by copying it; do not import from `phase2_check.py`); compare `/odom` with `/ground_truth/odom` using Phase 2's metrics: total distance within 2%, total turning within 3%; also INFO the same numbers for `/odom_raw`, so the write-up can say whether the IMU helps.
  - **Madgwick orientation** (both): `/imu/data` orientation is a unit quaternion and, standing still for 3 s, roll and pitch within 2°.

- [ ] **Step 3: Run**
```bash
ros2 launch rosmaster_r2_sim sim.launch.py headless:=true world:=test_room localization:=true &
ros2 run rosmaster_r2_sim phase4_check.py
```
Expected: all checks PASS. If the EKF warns about the clock or drops messages (`sensor_timeout` 0.1 at RTF < 1), note the numbers, try `camera:=false`, and record the outcome in the write-up — do not edit the real EKF config.

- [ ] **Step 4: Refuse the TF conflict.** In `launch_setup`, if both `localization` and `publish_odom_tf` are `true`, raise `RuntimeError("publish_odom_tf:=true and localization:=true would both publish odom -> base_footprint; pick one")`. Verify: `ros2 launch rosmaster_r2_sim sim.launch.py headless:=true localization:=true publish_odom_tf:=true` exits with that message and starts no Gazebo.

- [ ] **Step 5: Regression.** `phase3_check.py` in `test_room` with `localization:=true`: 17/17.

- [ ] **Step 6: Commit** — `git add -A rosmaster_r2_sim && git commit -m "Phase 4: real Madgwick and EKF run against the sim; parity check"`

---

### Task 5: Measure the real robot (needs the person at the robot)

The sim runs on the robot's own Jetson, so these use the same machine with the simulation **stopped** and the real bringup on domain 28. Each item ends in a number written into the sim; nothing is changed on the real robot.

**Files:**
- Modify: `rosmaster_r2_sim/urdf/r2.urdf.xacro` (`steer_rate`), `rosmaster_r2_sim/urdf/r2.sensors.xacro` (IMU noise), only if the numbers differ.

- [ ] **Step 1: Passive parity on the real robot.** Robot powered, `ros2 launch yahboomcar_bringup yahboomcar_bringup_R2_launch.py` and the lidar driver running, `ROS_DOMAIN_ID=28`, then `ros2 run rosmaster_r2_sim phase4_check.py --passive`. Expected: same PASS list as the sim's passive checks. Any FAIL is a real difference: record it in the write-up and decide per item whether the sim or the plan's assumption (e.g. `/odom_raw` at 10 Hz) changes.
- [ ] **Step 2: IMU noise.** Robot still on the floor, `ros2 topic echo --csv /imu/data_raw > imu_still.csv` for 60 s; compute the standard deviation of gyro and accel per axis (numpy, one-off, no new script in the repo). Replace the Phase 3 estimates (σ 0.002 rad/s, 0.02 m/s²) if they differ by more than 2×.
- [ ] **Step 3: Steering servo speed.** Wheels off the ground, film the front wheels at 60 fps or more while publishing one `/cmd_vel` step (`v` 0.2, `w` 0.333, i.e. the 0.6 m turn) after 2 s of straight. Count frames from the step to the wheels stopping: `steer_rate ≈ 0.34 rad / time`. Set `steer_rate` in `r2.urdf.xacro`.
- [ ] **Step 4: Rerun** Task 3's `phase2_check.py` and Task 4's `phase4_check.py` in the sim with the new values; all pass.
- [ ] **Step 5: Commit** — `git commit -am "Phase 4: steering speed and IMU noise from the real robot"`

If the person cannot do this now, skip it, keep the guesses, and list it under "Remaining known gaps" in the write-up.

---

### Task 6: Write-up and README

**Files:**
- Create: `docs/research/04-phase4-parity.md`
- Modify: `README.md` (status block, roadmap Phase 4 `[x]`, quick-start line for `localization:=true` and `phase4_check.py`)

- [ ] **Step 1: Write `04-phase4-parity.md`** with the sections of `03-phase3-sensors.md`: What was added (table), settings with sources, topics/frames real vs sim, Results (every check, measured numbers, pasted from the runs above), What the simulation gets wrong and how it was found, Remaining known gaps (at least: no `/imu/mag`; no `/cmd_vel` timeout, like the real driver; Task 5 items if skipped; the real `base_node_R2` wheelbase 0.25 vs 0.2353), Running it yourself, Next step (Phase 5: `slam_toolbox` and Nav2).
- [ ] **Step 2: Verify** every number in the write-up appears in a check output from this phase (superpowers:verification-before-completion).
- [ ] **Step 3: Commit** — `git add docs/research/04-phase4-parity.md README.md && git commit -m "Phase 4: parity with the real stack"`; push the branch and open a PR into `main` only when the person asks.
