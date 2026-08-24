# servo_stream_controller Reference

Buffered path for MoveIt Servo commands: setpoints are recorded into a WMX3 API
buffer and replayed by the engine under the real-time OS, instead of being
issued from a ROS callback.

- Package: `wmx_r2_package`
- Executable: `servo_stream_controller`
- Source: [`src/servo_stream_controller.cpp`](../wmx_r2_package/src/servo_stream_controller.cpp)
- Config: `servo_stream_controller:` block in `config/cr3a_manipulator_config.yaml` / `config/cr5a_manipulator_config.yaml`

---

## What it is

`joint_position_controller` (the default) calls `StartLinearIntplPos` directly
from the ROS subscription callback. Every command crosses from a non-real-time
Linux thread straight into the engine, so ROS scheduling jitter lands on motion
timing and a late thread leaves a gap in the stream.

`servo_stream_controller` puts the API buffer in between:

```
Servo topic -> ROS ring (bounded, drop-oldest) -> pump thread -> API buffer -> engine
```

The buffer absorbs ROS jitter, and its watch function adds a safety net that
lives entirely on the RT side.

**The two nodes subscribe to the same topic and must not run together.** The
launch files enforce this with the `use_api_buffer` argument.

### How a setpoint becomes motion

Each Servo setpoint is recorded as **two blocks**:

| block | purpose |
|---|---|
| `StartLinearIntplPos(n axes)` | coordinated move, lands as a `FastBlending` override |
| `USleep(T)` | paces the queue; `T` is trimmed to hold the target depth |

`api_buffer_probe` measured 396 bytes per setpoint (320 motion + 76 sleep), so a
1 MB channel holds ~2650 setpoints — over a minute of stream at Servo's 40 Hz.

`T` is in microseconds and is **not** quantised to the 1 kHz engine cycle;
`USleep` honours sub-millisecond requests to within ~0.3%.

### Pacing loop

The pump adjusts `T` to hold the buffer at `target_queue_depth`:

```
T = nominal_period_us + pacing_kp * 1000 * (target_queue_depth - observed_depth)
T = clamp(T, period_clamp_lo_us, period_clamp_hi_us)
```

The sign is deliberate. The period is what the *engine* spends per setpoint while
the pump feeds at whatever rate ROS delivers, so a depth **below** target calls
for a **longer** period. Depth is read from `remainingBlockCount / 2`.

Queue depth is the latency dial: at 40 Hz, each setpoint of depth is 25 ms of
deliberate latency.

### Safety and hand-off behaviour

| situation | behaviour |
|---|---|
| Watched axis goes servo-off, offline, amp-alarm, or hits a limit | The engine halts the motion channel and runs a pre-recorded `Stop` on every joint axis from `estop_buffer_channel`. No dependency on ROS or Linux still being alive. |
| No new setpoint for `starvation_timeout_ms` | Controlled stop, then `Halt` + `Clear`. The queue is discarded, not played out. |
| `move_group` execution starts (`/moveit2_trajectory/execution_active` true) | Ring cleared, stream ended without a `Stop` — the trajectory controller owns the axes from that point. Setpoints are discarded until execution finishes. |
| Motion channel reports `Stop` with a recorded error | Fault backoff, doubling from 500 ms to a 5 s cap, then retry. |
| A previous run died without freeing its channel | The channel is reclaimed on startup rather than failing with "Queue ID is already used". |

---

## Prerequisites

The engine must be up and the axes ready before this node can stream. Follow the
startup sequence in
[reference_wmx_r2_general_nodes.md](reference_wmx_r2_general_nodes.md): load axis
parameters, set gear ratios, clear alarms, servo on, home.

The node waits for `wmx/engine/ready` before initialising, so it can be started
first — it will sit at `waiting for engine...` until the engine node comes up.

Engine access needs root. All commands below assume this wrapper, which is the
same `sudo --preserve-env` invocation used throughout `doc/`:

```bash
cat > ~/wmxrun <<'EOF'
#!/bin/bash
sudo --preserve-env=PATH --preserve-env=AMENT_PREFIX_PATH \
     --preserve-env=COLCON_PREFIX_PATH --preserve-env=PYTHONPATH \
     --preserve-env=LD_LIBRARY_PATH --preserve-env=ROS_DISTRO \
     --preserve-env=ROS_VERSION --preserve-env=ROS_PYTHON_VERSION \
     --preserve-env=ROS_DOMAIN_ID --preserve-env=RMW_IMPLEMENTATION \
  bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && source $HOME/workspaces/movensys_ws/install/setup.bash && $*"
EOF
chmod +x ~/wmxrun
```

---

## How to run

### 1. With the manipulator launch (normal use)

Pass `use_api_buffer:=true` to swap `joint_position_controller` for this node:

```bash
~/wmxrun ros2 launch wmx_r2_package wmx_r2_cr5a_manipulator.launch.py \
  use_sim_time:=false use_api_buffer:=true
```

Use `wmx_r2_cr3a_manipulator.launch.py` for the CR3A. Parameters come from that
robot's `*_manipulator_config.yaml`.

### 2. Standalone (bench work and tuning)

Run the node by itself against an already-running engine, using the single-axis
bench config that ships with the package:

```
wmx_r2_package/config/bench_servo_stream_config.yaml
```

It carries a section for each node a bench session uses —
`servo_stream_controller`, `joint_state_broadcaster`, `servo_stream_test_source`,
and `api_buffer_probe` — so the same file drives all of them. Each top-level key
must match a node's name, and each node reads only its own section.

Point `ros2 run` at the **source tree** copy so it can be edited and re-run
without a rebuild:

```bash
export BENCH=$HOME/workspaces/movensys_ws/src/wmx-r2/wmx_r2_package/config/bench_servo_stream_config.yaml
ls -l "$BENCH"        # confirm it resolves — see below
```

Check that `ls` every time you open a new terminal. If `BENCH` is already
exported from a shell rc file or an earlier session, your value silently wins
over the one you just typed, and `rcl` then reports
`Error opening YAML file` — a message that does not tell you the path was wrong
rather than the YAML.

The installed copy also exists after a build, at
`install/wmx_r2_package/share/wmx_r2_package/config/bench_servo_stream_config.yaml`,
but editing that one is overwritten by the next `colcon build`.

This is a bring-up and measurement config, not a robot config — the launch files
use `cr3a_manipulator_config.yaml` / `cr5a_manipulator_config.yaml`. Keep tuning
experiments in the bench file and promote settled values into the robot config.

#### Bench session bring-up

> For the full single-axis test procedure — bring-up, hold test, motion test,
> starvation and restart checks, pass criteria — see
> [launch_single_servo_streamer.md](launch_single_servo_streamer.md). The summary
> below is the bring-up order only.

A bench session needs four processes, in this order. Nothing streams until all of
them are up, and the failure at each stage is a quiet wait rather than an error.

**1. Engine and fieldbus** (one terminal):

```bash
~/wmxrun ros2 launch wmx_r2_package wmx_r2_general_nodes.launch.py
```

This starts `wmx_engine_node`, `wmx_core_motion_node`, `wmx_io_node`, and
`wmx_ethercat_node`. It does **not** start `joint_state_broadcaster` — the
manipulator launches add that separately, so a bench session has to run it by
hand.

**2. Axis bring-up.** Load axis parameters, set gear ratios, clear alarms, servo
on, home — the sequence in
[reference_wmx_r2_general_nodes.md](reference_wmx_r2_general_nodes.md). Nothing
below will move an axis until this is done.

**3. Joint feedback** (second terminal). Needs root, since it reads axis status
straight from the engine:

```bash
~/wmxrun ros2 run wmx_r2_package joint_state_broadcaster --ros-args \
  --params-file $BENCH
```

Confirm it before going further — the test source will wait forever otherwise:

```bash
ros2 topic hz /joint_states          # expect ~100 Hz
ros2 topic echo --once /joint_states # expect joint1 in `name`
```

**4. The controller** (third terminal):

```bash
~/wmxrun ros2 run wmx_r2_package servo_stream_controller --ros-args \
  --params-file $BENCH
```

Wait for `Stream started (channel 0 Active)`. If it stays at
`waiting for engine...`, step 1 is not up.

The test source (§3 below) is the fourth process.

If the params file is missing or unreadable, `rcl` fails before the node starts
with `Couldn't parse params file ... Error opening YAML file` — check the path
first, since that message does not distinguish "absent" from "malformed".

Single overrides work too:

```bash
~/wmxrun ros2 run wmx_r2_package servo_stream_controller --ros-args \
  --params-file $BENCH -p target_queue_depth:=3
```

The node echoes every parameter at startup — check that block first when
behaviour does not match expectations.

### 3. Driving it without MoveIt: `servo_stream_test_source`

`servo_stream_test_source` is a synthetic stand-in for MoveIt Servo. It publishes
`trajectory_msgs/JointTrajectory` on Servo's output topic at Servo's rate, so the
streaming path can be characterised without IK, collision checking, smoothing, or
an operator's hand in the loop. The input is a known analytic function, so
commanded-vs-actual measures the streaming path alone.

It needs no root, but it **does** need a live `/joint_states` carrying the joints
it is asked to publish — it centres the waveform on their current positions and
will not start streaming until it sees them. Bring up
`joint_state_broadcaster` first (bench bring-up step 3 above).

```bash
ros2 run wmx_r2_package servo_stream_test_source --ros-args \
  --params-file $BENCH
```

The bench config sets `amplitude_rad: 0.0`. Raise it with an inline override
rather than by editing the file, so the safe default is what you get next time:

```bash
ros2 run wmx_r2_package servo_stream_test_source --ros-args \
  --params-file $BENCH -p amplitude_rad:=0.02
```

| parameter | default | notes |
|---|---|---|
| `joint_name` | `["joint1"]` | Publishes only these joints |
| `joint_states_topic` | `/joint_states` | Used to centre the waveform |
| `output_topic` | `/movensys_manipulator_arm_controller/joint_trajectory` | Must match the controller's `joint_trajectory_topic` |
| `amplitude_rad` | `0.0` | Hard-capped at 0.5 rad so a typo cannot command a large excursion |
| `frequency_hz` | `0.25` | |
| `publish_rate_hz` | `40.0` | Servo's rate |
| `ramp_seconds` | `2.0` | Amplitude ramps in from zero, so starting never steps |
| `waveform` | `sine` | `sine` or `triangle` |

The waveform is centred on the joint's **current** position read from
`/joint_states`, so starting the node never produces a step command.

**Recommended bring-up order** on any new robot or config:

1. `amplitude_rad: 0.0` — a pure hold. The full streaming path runs at rate with
   zero commanded motion. This is the correct first test with the servos on,
   before commanding any movement at all. Queue depth should settle at
   `target_queue_depth`.
2. Small amplitude, e.g. `amplitude_rad: 0.02`, and watch queue depth and
   tracking.
3. Increase only once depth is stable and no warnings are firing.

Ctrl-C on the test source stops the stream, which is the intended way to exercise
the starvation path.

### 4. `api_buffer_probe` — characterise the buffer first

```bash
~/wmxrun ros2 run wmx_r2_package api_buffer_probe --ros-args \
  --params-file $BENCH
```

Run-once node: attaches, measures, prints a report, exits. It answers bytes per
block, real `USleep` resolution, and whether the block counters can stand in for
the `GetCumulativeBlockCount` this SDK does not provide.

It is a **different node** with its own parameters — `joint_axes`,
`api_buffer_channel`, `api_buffer_size_mb`, `sample_count`,
`sleep_sample_count` — which is why the bench config carries an
`api_buffer_probe:` section of its own. Without one, the probe would start
happily and silently fall back to its defaults of all six axes and 5 MB, giving a
report for a configuration you did not ask for.

**It commands no motion** — every recorded target is the axis' current commanded
position, and the motion blocks are cleared before anything is executed. Safe to
run with the servos off, and that is the recommended way.

---

## Parameters

| parameter | default | meaning |
|---|---|---|
| `joint_axes` | `[]` | WMX3 axis indices. **Order matters**: `axis[0]` determines `LinearIntplProfileCalcMode` for the whole interpolation. |
| `joint_name` | `[]` | ROS joint names, positionally paired with `joint_axes` |
| `joint_trajectory_topic` | — | Servo's `command_out_topic` |
| `api_buffer_channel` | `0` | Motion channel |
| `estop_buffer_channel` | `1` | Watch trigger routine; must differ from the motion channel, or the trigger is disabled |
| `api_buffer_size_mb` | `5` | 1 MB ≈ 2650 setpoints ≈ 66 s of stream |
| `target_queue_depth` | `2` | Latency/robustness dial; 1 setpoint = 25 ms at 40 Hz |
| `nominal_period_us` | `25000` | Must equal Servo's publish period |
| `period_clamp_lo_us` | `20000` | Pacing authority, low end |
| `period_clamp_hi_us` | `30000` | Pacing authority, high end |
| `pacing_kp` | `0.15` | Correction in engine cycles per setpoint of depth error (150 µs each) |
| `ros_queue_depth` | `8` | Ring size; drop-oldest above it, since stale teleop is worthless |
| `starvation_timeout_ms` | `75` | Gap without a setpoint before a controlled stop |
| `accel_ratio` | `0.3` | Blend spans ~30% of each segment |
| `stop_on_error` | `true` | Stop rather than skip a rejected setpoint |

---

## Topics

| topic | type | direction | notes |
|---|---|---|---|
| `<joint_trajectory_topic>` | `trajectory_msgs/JointTrajectory` | in | Servo setpoints; only the last point of each message is used |
| `wmx/engine/ready` | `std_msgs/Bool` | in | Transient-local; gates initialisation |
| `/moveit2_trajectory/execution_active` | `std_msgs/Bool` | in | Transient-local; blocks streaming while `move_group` owns the arm |
| `/servo_stream_controller/queue_depth` | `std_msgs/Int32` | out | Setpoints in flight, published at 10 Hz |

### Joint matching

- **Named messages**: matched by joint name, so a subset config
  (`joint_axes: [0]`) can consume Servo's full six-joint output, and the
  publisher's ordering does not have to match ours.
- **Unnamed messages**: positional, assumed already in `joint_axes` order.
- A setpoint that does not command **every** axis this node drives is dropped —
  a partial setpoint would hold some joints and move others, bending the path
  Servo computed.

---

## Checking that it is working

```bash
ros2 topic echo /servo_stream_controller/queue_depth
ros2 topic hz /joint_states
```

A healthy stream:

```
[INFO] Attached WMX3 device 'servo_stream_rec'
[INFO] Attached WMX3 device 'servo_stream_ctl'
[INFO] Recorded e-stop routine into channel 1
[INFO] Watch enabled on 1 axes, trigger routine on channel 1
[INFO] servo_stream_controller is ready
[INFO] Stream started (channel 0 Active)
```

- One `Stream started`, with no fault and no `Stream ended` following it.
- `queue_depth` settling near `target_queue_depth` and staying there. A flicker
  between two adjacent values is normal — depth is `remainingBlockCount / 2`, so
  one block of sampling phase flips the reported integer.
- No repeating warnings.

`queue_depth` goes to 0 whenever the stream is down (starvation, fault,
`move_group` takeover), so it is a usable liveness signal, not only a depth
signal.

### Recording a run for analysis

```bash
ros2 bag record -o stage_test \
  /joint_states \
  /movensys_manipulator_arm_controller/joint_trajectory \
  /servo_stream_controller/queue_depth
```

Commanded-vs-measured cross-correlation on those three topics gives the true
transport latency, which is the check that does not depend on trusting the depth
topic. `stage5_buffered/` in the repo root is an example recording.

---

## Troubleshooting

| message | meaning | what to do |
|---|---|---|
| `waiting for engine...` and nothing more | `wmx/engine/ready` never arrived | Start the general nodes (`wmx_r2_general_nodes.launch.py`); check the engine and EtherCAT state |
| `servo_stream_test_source`: `Waiting for 'joint1' on /joint_states ...` forever | Either nothing publishes `/joint_states`, or it does but without that joint name | `ros2 topic info /joint_states` — `Publisher count: 0` means `joint_state_broadcaster` is not running; a nonzero count means the name does not match, so compare its `joint_name` with the test source's |
| `API buffer backlog: N setpoints in flight (target M)` | Depth exceeded `4 × target_queue_depth` | The engine is consuming slower than the pacing model assumes — see Known issues |
| `Servo stream starved (>75 ms); stopping axes` | No setpoint arrived within the timeout | Check the publisher's rate and the ROS transport; expected when you Ctrl-C the source |
| `ROS queue full (8); dropping oldest setpoint` | The pump cannot keep up with arrivals | Check `nominal_period_us` against the publisher's actual rate |
| `Dropped trajectory: no command for axis N` | The setpoint did not cover every driven axis | `joint_name` does not match the publisher's joint names |
| `Dropped trajectory: N positions for M axes` | Unnamed message shorter than `joint_axes` | Fix the publisher or name the joints |
| `Motion channel stopped (watch fired on axis A, code C)` | The RT watch tripped | Servo-off, amp alarm, limit, or drive offline on axis A |
| `Motion channel stopped (block N failed, code C)` | A recorded block was rejected at execution | Usually a limit or a velocity/accel violation on that segment |
| `Motion channel stopped (no error recorded, channel never left Stop after Execute)` | The channel did not start and nothing errored | Check that the axes are servo-on and homed |
| `CreateApiBuffer failed ... reclaiming` | A previous run left the channel allocated | Informational; the node reclaims it |
| `estop_buffer_channel must differ from api_buffer_channel` | Both set to the same channel | Fix the config — the watch trigger is disabled until you do |

---

## Known issues

Measured on a **single axis** (`joint_axes: [0]`) with `servo_stream_test_source`
at 0.25 Hz, ±50 mrad, 40 Hz. Not yet re-measured on a full six-axis config.

**1. Queue depth settles far above target during motion.**
`StartLinearIntplPos` costs roughly 1.05 ms of engine time to set up real motion,
which the pacing model does not account for — it assumes `USleep` is the only
per-setpoint cost. Since the loop is proportional-only, it can only supply that
correction by holding a standing depth error of about `1050 / 150 ≈ 7` setpoints.
Observed: depth 9–10 instead of 2, i.e. 225–250 ms of latency instead of 50 ms,
with the backlog warning firing continuously.

With `amplitude_rad: 0.0` the blocks are true no-ops, the cost drops below
150 µs, and depth settles at target. So the cost is effectively bimodal — present
whenever there is motion, absent when targets repeat — rather than proportional
to step size.

Workaround for now: expect the extra latency, and read `queue_depth` as
"target + block-cost error" rather than as a fault. A feedforward of the measured
block cost is the fix.

**2. The correction may exceed the loop's authority on six axes.**
The pacing loop's total authority is `nominal_period_us - period_clamp_lo_us` =
5000 µs, reached at a depth error of 33. If the per-setpoint block cost scales
with axis count, six axes could need more correction than that, in which case
depth grows without bound instead of settling. **Untested.** Before running the
buffered path on a six-axis arm, run with the test source at low amplitude and
confirm `queue_depth` settles rather than climbing.

**3. Motion is delivered in surges, not smoothly.**
Position tracking of the commanded fundamental is good (gain 1.015, ~290 ms of
pure delay, 3.1 mrad rms residual on a 50 mrad amplitude), but the joint moves in
bursts: measured peak velocity is ~8× the per-segment `maxVelocity` the node
computes, with roughly 24 velocity sign changes per second. Stiction is ruled out
(following error shows no correlation with direction of travel).

This contradicts the design intent recorded in the source header, which expects
`FastBlending` to make consecutive interpolations blend rather than stop-and-go.
Nothing in this repo sets `LinearIntplProfileCalcMode`, which is inherited from
`axis[0]`'s configuration, so per-segment velocity limits may not be in force at
all. Unresolved — check the calc mode and record `pos_cmd` from `wmx/axis/state`
alongside `actual_pos` to tell whether the surging originates in block execution
or downstream in the servo loop.

Caveat on that measurement: `wmx_core_motion_node` sizes `wmx/axis/state` from
the axis count it reads out of `GetEngineStatus` at attach time, and on a CR3A
launch it has been observed logging `is ready (1 axes, 100 Hz)` — so the topic
may carry only axis 0 even on a six-axis arm. Check that line before relying on
the topic for anything but axis 0.

**4. The CR5A config is untuned.**
Its `servo_stream_controller` values were copied from the CR3A. Re-run
`api_buffer_probe` and re-tune before relying on them.
