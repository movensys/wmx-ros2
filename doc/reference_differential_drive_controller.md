# Differential Drive Controller Reference

Standalone rclcpp node (`wmx_ros2_package/src/differential_drive_controller.cpp`)
that drives two WMX3 wheel axes directly via CoreMotion `StartVel` and exposes the
Nova autonomy contract (command velocity in, odometry feedback out). The WMX/ROS-free
math (kinematics, dead-reckoning, deltas, accel EMA) lives in the unit-tested
`nova_diff_drive_logic` package; the node is the ROS/WMX wiring around it.

```
/cmd_vel_safe (Twist) ──▶ ┌──────────────────────────────┐ ──▶ /odom_enc    (Odometry)
                          │ differential_drive_controller │ ──▶ /odom_deltas (TwistStamped)
wmx/engine/ready (Bool) ─▶│  single loop @ rate (100 Hz)  │ ──▶ /odom_accel  (AccelStamped)
                          │  WMX3 CoreMotion StartVel /   │ ──▶ /omega_enc   (Float64MultiArray)
                          │  GetStatus (one per cycle)    │ ──▶ /tf odom→base_link (optional)
                          └──────────────────────────────┘
```

---

## Parameters

All parameters are declared with defaults; the defaults already match the Nova
autonomy contract, so a deployment only *needs* to override the per-robot
hardware values (group A below).

All parameters are read **once at node construction**; there is no
parameter-set callback. A runtime `ros2 param set` is accepted by rclcpp but
has no effect on behaviour — restart the node to apply new values.

### A. Per-robot hardware (must be set per machine)

| Parameter | Type | Default | Unit | Description |
|---|---|---|---|---|
| `left_axis` | int | `0` | – | WMX3 axis index of the left wheel. **Not validated**: must be a valid axis index and ≠ `right_axis`; an out-of-range value causes an out-of-bounds status read (garbage odometry / undefined behaviour), not an error message. |
| `right_axis` | int | `1` | – | WMX3 axis index of the right wheel. Same caveat as `left_axis`. |
| `wheel_radius` | double | `0.095` | m | Drive-wheel radius `R`. Guarded: values ≤ 0 fall back to the default (warn). |
| `wheel_to_wheel` | double | `0.55` | m | Wheel separation `L` (distance between the two drive wheels). Guarded: ≤ 0 falls back to the default (warn). |
| `wmx_param_file_path` | string | `/diff_drive/no_param` | – | WMX parameter XML imported at init via `config->ImportAndSetAll()` (axis gear/feedback/limit setup). The default is a deliberate non-path: the import fails with an ERROR log but the node keeps running with whatever parameters the engine already has. The diffbot launch file overrides it with `config/diffbot_wmx_parameters.xml` resolved at launch time. |

### B. Motion profile / loop

| Parameter | Type | Default | Unit | Description |
|---|---|---|---|---|
| `rate` | int | `100` | Hz | Control-loop rate. One loop does **one** `GetStatus` and drives both the odometry and the command path. Guarded: ≤ 0 falls back to 100. The timer period is `1000 / rate` truncated to whole milliseconds, so prefer rates that divide 1000 (100, 125, 200, 250, 500). |
| `acc_time` | double | `1.0` | **ms** | `StartVel` trapezoidal profile acceleration time (`profile.accTimeMilliseconds`, `ProfileType::TimeAccTrapezoidal`). Note the unit: milliseconds — the default 1.0 ms is effectively an instant ramp; the WMX-side axis limits do the real shaping. Not guarded: passed to WMX unvalidated. |
| `dec_time` | double | `1.0` | **ms** | Same as `acc_time` for deceleration. Also applies to the stale-command stop (see `cmd_vel_timeout`). |

The control timer is a **wall timer**: under `use_sim_time:=true` the loop still
ticks at wall-clock rate while all stamps, integration `dt` and the
`cmd_vel_timeout` check follow the ROS clock. A paused `/clock` therefore
freezes odometry integration *and* disables the stale-command stop while the
last wheel target keeps being held.

### C. Behaviour / safety

| Parameter | Type | Default | Unit | Description |
|---|---|---|---|---|
| `cmd_vel_timeout` | double | `0.25` | s | Stale-command safety: if no command arrives on `cmd_vel_topic` within this window (or none has ever arrived), the wheel target is forced to zero. The stop decelerates over `dec_time` through the normal trapezoid — it is **not** an emergency stop; a true e-stop must go through the WMX hardware-level stop path. Not guarded: a negative value makes every cycle stale (permanent zero target, no warning). |
| `accel_publish_rate` | double | `10.0` | Hz | Rate limit for `/odom_accel` relative to the control loop. `0` = publish every control cycle. Guarded: negative values fall back to 10.0. |
| `accel_alpha` | double | `0.3` | – | EMA weight of the newest raw acceleration sample, valid range (0, 1]; higher = more responsive, lower = smoother. The estimator snaps to zero when both the current and previous velocity samples are ~0 (kills the EMA tail at standstill). Not guarded: the range is not enforced (0 pins `/odom_accel` to zero; >1 destabilizes the EMA — validate in the config layer). |
| `publish_tf` | bool | `false` | – | Publish `odom_frame → base_frame` TF from the integrated pose. Keep **false** when a localization EKF owns that TF (Nova: EKF is launched when an IMU is configured). Enable only as the fallback for IMU-less / no-EKF configs where this node is the sole odometry source. |
| `odom_frame` | string | `odom` | – | `frame_id` for `/odom_enc`, `/odom_deltas` and the optional TF parent. |
| `base_frame` | string | `base_link` | – | `child_frame_id` for `/odom_enc`/TF and `frame_id` for `/odom_accel`. |

### D. Topic names (defaults = Nova contract)

| Parameter | Default | Description |
|---|---|---|
| `cmd_vel_topic` | `/cmd_vel_safe` | Command input (subscription). |
| `encoder_odometry_topic` | `/odom_enc` | Encoder odometry output (EKF `odom0` input). |
| `encoder_omega_topic` | `/omega_enc` | Per-wheel encoder velocity output. |
| `odom_deltas_topic` | `/odom_deltas` | Accumulated travel output (DistanceTraveled monitor). |
| `odom_accel_topic` | `/odom_accel` | Body acceleration output (Motion monitor). |

Topic names are plain parameters (not ROS remap-only), so the Toolkit can set them
in the generated node config like any other value.

---

## Topics

| Topic (default) | Dir | Type | QoS | Rate | Notes |
|---|---|---|---|---|---|
| `wmx/engine/ready` | sub | `std_msgs/Bool` | reliable, transient_local, depth 1 | 1 Hz | Init gate from `wmx_engine_node`, re-published every second (`false` until engine communication starts, `true` afterwards; transient_local so late joiners get the last sample). The controller acts on the first `true` and ignores the rest; the subscription is dropped after successful init. Name is fixed (not a parameter). |
| `/cmd_vel_safe` | sub | `geometry_msgs/Twist` | default (reliable, volatile), depth 1 | producer | **Plain** `Twist`, not `TwistStamped` (the Nova EKF runs `stamped_control: false`). Uses `linear.x` [m/s] and `angular.z` [rad/s]. |
| `/odom_enc` | pub | `nav_msgs/Odometry` | default, depth 1 | `rate` | `header.frame_id = odom_frame`, `child_frame_id = base_frame`. Pose = dead-reckoned (exact-arc integration); twist = `vx`, `vy`(=0), `vyaw` from forward kinematics. Covariance: see below. |
| `/odom_deltas` | pub | `geometry_msgs/TwistStamped` | default, depth 1 | `rate` | Accumulated `Σ|v|·dt` (in `twist.linear.x`, m) and `Σ|ω|·dt` (in `twist.angular.z`, rad) since the previous publish; resets each publish. `frame_id = odom_frame`. |
| `/odom_accel` | pub | `geometry_msgs/AccelStamped` | default, depth 1 | `accel_publish_rate` | EMA-filtered derivative of body velocity over the actual inter-publish interval. `frame_id = base_frame`. |
| `/omega_enc` | pub | `std_msgs/Float64MultiArray` | default, depth 1 | `rate` | `data = [left, right]` wheel angular velocity [rad/s] (`actualVelocity` from `GetStatus`). |
| `/tf` (`odom_frame → base_frame`) | pub | TF | tf2 default | `rate` | Only when `publish_tf: true`. |

**Namespaces.** The five data-topic defaults are *absolute* names, so launching
the node in a ROS namespace does **not** namespace them — override the topic
parameters explicitly for multi-robot/namespaced deployments. The fixed
`wmx/engine/ready` gate is *relative* (it follows the namespace, on both the
engine and controller side): the controller must run in the **same namespace
as `wmx_engine_node`**, or the ready signal never arrives and the node waits
forever.

### `/odom_enc` covariance (fixed, not parameterized)

The Jetstream localization EKF fuses **only twist `vx`, `vy`, `vyaw`** from this
source (`robot_localization` `odom0_config`). Pose x/y/yaw are nevertheless kept
authoritative for the no-EKF fallback where this odometry feeds Nav2 directly.

| Block | Authoritative (variance `0.01`) | Non-authoritative (variance `99999`) |
|---|---|---|
| pose | x, y, yaw | z, roll, pitch |
| twist | vx, vy, vyaw | vz, v_roll, v_pitch |

---

## Units and conventions

- Command: `linear.x` [m/s], `angular.z` [rad/s]; positive `angular.z` = CCW (REP-103).
- Wheel velocity (`StartVel` target and `actualVelocity` feedback) is the wheel
  angular velocity in **rad/s**. The WMX axis user-unit scaling (encoder counts,
  gear ratio) must be configured WMX-side — via the `wmx_param_file_path` XML —
  so that one axis velocity unit = 1 rad/s at the wheel. There is no gear-ratio
  parameter in the node.
- Kinematics (`nova_diff_drive_logic::DiffDriveModel`):
  - inverse: `ωl = (2v − ωL)/(2R)`, `ωr = (2v + ωL)/(2R)`
  - forward: `v = R(ωr + ωl)/2`, `ω = R(ωr − ωl)/L`
- Odometry pose integration uses exact-arc integration for `|ω| ≥ 1e-3` rad/s and
  straight-line otherwise; non-positive/non-finite `dt` steps are ignored.

---

## Lifecycle and runtime behaviour

**Startup.** The node starts idle and waits for `wmx/engine/ready` (latched `Bool`
from `wmx_engine_node`). On `true` it runs the init sequence on a dedicated thread
(so blocking retries never stall the executor):

1. `CreateDevice(WMX3_SDK_PATH, DeviceTypeNormal, 10 s)` — on `StartProcessLockError`
   retries every 1 s up to 30 times (another process holding the device lock);
   any other error aborts this init attempt.
2. `SetDeviceName("differential_drive_controller")`.
3. `ImportAndSetAll(wmx_param_file_path)` — failure is logged but non-fatal.
4. Create publishers/subscriber and the control timer; drop the ready subscription.

A failed init attempt is not terminal: it resets the init guard, and since the
engine re-publishes `ready=true` at 1 Hz (and the ready subscription is only
dropped on success), the full sequence is automatically re-attempted on the
next ready message (~1 s later) until it succeeds. The process never exits
with an error code on init failure — it stays alive and inert (no topics, no
timer). Supervise via logs or topic liveness (`/odom_enc` at `rate` Hz), not
exit codes.

The node does **not** start communication, clear alarms, or switch servos on —
that is owned by the engine/general nodes (see
`reference_wmx_ros2_general_nodes.md` for the service sequence).

**Control loop** (every `1/rate`, single `GetStatus` per cycle):

1. *Engine gate* — if `engineState != Communicating`: warn (1 s throttle), publish
   nothing, command nothing, and drop the loop clock + resend cache so `dt` and the
   `StartVel` state re-baseline cleanly on recovery.
2. *Odometry path* — runs even with servo off (encoder feedback stays valid):
   integrate pose, accumulate deltas, publish `/omega_enc`, `/odom_enc`,
   `/odom_deltas`, `/odom_accel` (rate-limited), optional TF.
3. *Command path* — skipped (with 1 s-throttled warn, resend cache invalidated)
   while an amp alarm is active or either servo is off; recovery therefore always
   re-sends the current target.
4. *Stale-command check* — no fresh command within `cmd_vel_timeout` ⇒ target zero.
5. *Resend-on-change* — `StartVel` is re-sent only when the wheel target changes,
   so the velocity trapezoid is not restarted every cycle. The "last sent" cache
   commits only if **both** axes accept the command; a failed `StartVel` (e.g. a
   transient motion-state conflict) is retried next cycle — this guarantees a
   timeout→zero stop can never be swallowed by a failed send.

**Shutdown.** Destructor cancels the timer, commands both wheels to zero, and
closes the WMX device.

**Process model.** Ships as a standalone executable on the default
single-threaded executor; the command state shared between the `cmd_vel`
callback and the control loop is unsynchronized by design (single-threaded
callback execution is a correctness assumption). It is not built as a
composable component — run it as its own process, one per robot.

---

## Configuration files

A deployment consists of two files plus the launch wiring
(example: `launch/wmx_ros2_diffbot_navigation.launch.py`):

1. **ROS parameter YAML** — `config/diffbot_navigation_config.yaml`, key
   `differential_drive_controller.ros__parameters` (all tables above).
2. **WMX parameter XML** — `config/diffbot_wmx_parameters.xml`: axis-level
   gear/feedback/limit/e-stop setup imported at node init. This is where the
   "axis unit = wheel rad/s" scaling and the hardware-level motion limits live.
3. **Launch** — starts the general WMX nodes (engine etc.), the
   `joint_state_broadcaster`, and this node; injects `wmx_param_file_path`
   (resolved from the package share at launch time) and `use_sim_time`.

```yaml
differential_drive_controller:
  ros__parameters:
    left_axis: 0
    right_axis: 1
    rate: 100
    acc_time: 1.0        # ms (StartVel trapezoid)
    dec_time: 1.0        # ms
    wheel_radius: 0.095  # m
    wheel_to_wheel: 0.55 # m
    cmd_vel_timeout: 0.25     # s — stale-command stop window
    accel_publish_rate: 10.0  # Hz — /odom_accel rate limit (0 = every cycle)
    accel_alpha: 0.3
    publish_tf: false    # true only for IMU-less / no-EKF configs
    odom_frame: odom
    base_frame: base_link
    cmd_vel_topic: /cmd_vel_safe
    encoder_odometry_topic: /odom_enc
    encoder_omega_topic: /omega_enc
    odom_deltas_topic: /odom_deltas
    odom_accel_topic: /odom_accel
    wmx_param_file_path: ""  # injected by launch
```

---

## Toolkit integration checklist

What the Toolkit needs to template per robot / per deployment:

- **Always per robot:** `left_axis`, `right_axis`, `wheel_radius`,
  `wheel_to_wheel`, and the WMX parameter XML (`wmx_param_file_path`).
- **Per deployment config:** `publish_tf` — must be `true` exactly when the
  localization EKF is *not* running (Nova: EKF launches only with an IMU
  configured); otherwise two publishers would fight over `odom → base_link`.
- **Usually defaults:** topic names (already the Nova contract), frames, `rate`,
  `cmd_vel_timeout`, `accel_publish_rate`, `accel_alpha`, `acc_time`/`dec_time`.
- **Not parameterized (by design):** the `wmx/engine/ready` gate topic, the
  `/odom_enc` covariance values, servo-on/alarm-clear handling (engine/general
  nodes own these), and any gear-ratio scaling (WMX XML owns it).

## Build

`wmx_ros2_package` compiles against the WMX3 SDK at the CMake cache path
`WMX3_SDK_PATH` (default `/opt/wmx3`); the same path (with a trailing `/`
appended by CMake) is compiled in and passed to `CreateDevice` at runtime.
Depends on `nova_diff_drive_logic` (plain ament C++ library, no WMX/ROS deps
in its core; unit-tested via 4 gtest suites —
`colcon test --packages-select nova_diff_drive_logic`).

At **runtime** the dynamic linker must be able to find the SDK's shared
libraries (`libimdll.so` etc.): either an `ld.so.conf.d` entry for
`/opt/wmx3/lib` (the SDK installer's default) or
`LD_LIBRARY_PATH=/opt/wmx3/lib` — relevant when running in containers that
only mount the SDK.

Verified on ROS 2 Humble (Ubuntu 22.04, native) and ROS 2 Jazzy
(Ubuntu 24.04, `ros:jazzy-ros-base` container): clean build of all four
packages against the real WMX3 SDK, all unit tests green, node startup smoke
test.
