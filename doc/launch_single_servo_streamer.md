# Launch and Test the Single-Axis Servo Streamer

Step-by-step procedure for bringing up `servo_stream_controller` on **one axis**
and verifying it, without MoveIt, IK, or a teleop device in the loop.

For what the node is and what every parameter means, see
[reference_servo_stream_controller.md](reference_servo_stream_controller.md).

**What this exercises**

- The full streaming path: command source → ROS ring → pump → API buffer → engine
- The pacing loop holding the buffer at `target_queue_depth`
- The starvation path (controlled stop when commands stop arriving)
- Clean restart after a starvation teardown

**Processes involved** — four, plus monitors. Nothing streams until all four are
up, and each stage fails by *waiting quietly* rather than erroring, so bring them
up in order and check each one.

| # | process | root? | provides |
|---|---|---|---|
| 1 | `wmx_r2_general_nodes.launch.py` | yes | engine, core motion, io, EtherCAT |
| 2 | `joint_state_broadcaster` | yes | `/joint_states` |
| 3 | `servo_stream_controller` | yes | the node under test |
| 4 | `servo_stream_test_source` | no | synthetic Servo command stream |

---

## Safety, before anything else

- **E-stop within reach.** Step 3 energises the axis on its own, before you
  command any motion.
- **Clear travel around the test joint**, with margin: the streaming path
  currently delivers motion in surges reaching several times the commanded
  velocity (see Known issues in the reference doc).
- **Do not run a manipulator launch or `joint_position_controller` at the same
  time.** They subscribe to the same topic as `servo_stream_controller` and would
  double-command the arm.
- Only the axis in `joint_axes` is driven. Everything else stays put.

---

## 0. Environment

Every terminal needs the workspace sourced and both variables set. Put them in
`~/.bashrc` so new terminals inherit them:

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspaces/movensys_ws/install/setup.bash

export ROBOT=cr3a     # or cr5a — must match the actual arm
export BENCH=$HOME/workspaces/movensys_ws/src/wmx-r2/wmx_r2_package/config/bench_servo_stream_config.yaml
```

**Verify both resolve, in every new terminal:**

```bash
echo "$ROBOT"
ls -l "$BENCH"
ls -l "$HOME/workspaces/movensys_ws/install/wmx_r2_package/share/wmx_r2_package/config/${ROBOT}_wmx_parameters.xml"
```

A `BENCH` left over from a shell rc file or an earlier session silently wins over
one you export by hand, and the resulting failure is
`Couldn't parse params file ... Error opening YAML file` — which does not
distinguish a wrong path from malformed YAML. An unset `ROBOT` collapses the
parameter path to `_wmx_parameters.xml` and fails the load. Both are cheap to
check and expensive to debug.

Build if you have not already:

```bash
cd ~/workspaces/movensys_ws
colcon build --packages-select wmx_r2_package
source install/setup.bash
```

Root access uses the same `sudo --preserve-env` invocation as the other launch
docs. If you do not have the wrapper yet:

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

## 1. Engine and fieldbus — terminal 1

```bash
~/wmxrun ros2 launch wmx_r2_package wmx_r2_general_nodes.launch.py
```

Verify:

```bash
ros2 service call /wmx/engine/get_status std_srvs/srv/Trigger "{}"
ros2 node list
# expect: wmx_engine_node, wmx_core_motion_node, wmx_io_node, wmx_ethercat_node
```

This launch does **not** start `joint_state_broadcaster` — the manipulator
launches add that, so a bench session runs it by hand in step 3.

---

## 2. Axis bring-up

Plain service calls, no root needed.

```bash
ros2 service call /wmx/params/load wmx_r2_message/srv/LoadWmxParams \
  "{file_path: '$HOME/workspaces/movensys_ws/install/wmx_r2_package/share/wmx_r2_package/config/${ROBOT}_wmx_parameters.xml'}"
```

**This must return `success=True` before you go on.** On failure the engine keeps
whatever parameters it already had — possibly defaults with the wrong gear ratio
and wrong soft limits, in which case a commanded 0.02 rad can scale into a much
larger physical move. The three calls below will happily succeed on an axis whose
parameters never loaded, so a `success=True` from them is not evidence of a good
state.

Both robots' parameter files exist, so loading the **wrong** one will not error.
Confirm `ROBOT` matches the arm in front of you.

```bash
ros2 service call /wmx/axis/clear_alarm wmx_r2_message/srv/SetAxis "{index: [0], data: [0]}"
ros2 service call /wmx/axis/set_on      wmx_r2_message/srv/SetAxis "{index: [0], data: [1]}"
ros2 service call /wmx/axis/homing      wmx_r2_message/srv/SetAxis "{index: [0], data: [0]}"
```

If you reload parameters later, **re-run these three** — a home position recorded
under the old scaling is not meaningful under new parameters.

**Do not skip homing.** `servo_stream_controller` seeds its first segment from
`posCmd`, while `servo_stream_test_source` centres its waveform on `actualPos`
from `/joint_states`. If those disagree because the axis was never homed, the
first segment is a jump between them, commanded at that segment's velocity limit.
Homing makes them equal.

---

## 3. Joint feedback — terminal 2 ⚠ energises the axis

```bash
~/wmxrun ros2 run wmx_r2_package joint_state_broadcaster --ros-args --params-file $BENCH
```

This node calls `wmx/axis/clear_alarm` and `wmx/axis/set_on` for its own
`joint_axes` during init, so **starting it turns the servos on**. It also does
not retry: if those service calls fail because the engine is not up yet, it logs
`Init failed at ...` and never publishes. Start it only after step 1 is confirmed.

Verify — the test source waits forever otherwise:

```bash
ros2 topic hz /joint_states            # expect ~100 Hz
ros2 topic echo --once /joint_states   # `name` must contain joint1
```

---

## 4. The controller — terminal 3

```bash
~/wmxrun ros2 run wmx_r2_package servo_stream_controller --ros-args --params-file $BENCH
```

Expect the parameter echo, then:

```
Attached WMX3 device 'servo_stream_rec'
Attached WMX3 device 'servo_stream_ctl'
Recorded e-stop routine into channel 1
Watch enabled on 1 axes, trigger routine on channel 1
servo_stream_controller is ready
```

**It stops there, and that is correct.** The stream starts on the first setpoint,
so `Stream started` does not appear until step 6. If it stays at
`waiting for engine...` instead, step 1 is not up.

---

## 5. Depth monitor — terminal 4

```bash
ros2 topic echo /servo_stream_controller/queue_depth
```

Reads `0` until the stream starts. This topic goes to 0 whenever the stream is
down, so it doubles as a liveness signal.

---

## 6. Hold test — terminal 5

The bench config ships `amplitude_rad: 0.0`, which runs the whole path at 40 Hz
with **zero commanded motion**. This is the correct first test with servos on.

```bash
ros2 run wmx_r2_package servo_stream_test_source --ros-args --params-file $BENCH
```

| check | expected |
|---|---|
| test source | `Centred on [...]; streaming.` |
| controller | `Stream started (channel 0 Active)` — once, with no fault and no `Stream ended` |
| `queue_depth` | settles at **2–3** and stays there |
| the axis | does not move |

A flicker between two adjacent depth values is normal: depth is
`remainingBlockCount / 2`, so one block of sampling phase flips the reported
integer.

**Do not proceed until depth settles at target here.** This is the measurement
that says the pacing loop is healthy, because a no-op interpolation block costs
the engine almost nothing and so leaves the loop nothing to fight.

---

## 7. Motion test

Raise amplitude with an inline override rather than by editing the file, so the
safe default survives to next time.

```bash
ros2 run wmx_r2_package servo_stream_test_source --ros-args \
  --params-file $BENCH -p amplitude_rad:=0.02
```

Amplitude ramps in over `ramp_seconds` (2 s), so there is no step at start. Step
up gradually — 0.02, then 0.05 — watching depth and the arm at each level. The
node hard-caps amplitude at 0.5 rad.

**Expected, and not a failure:**

- Depth climbs to **9–10** over ~5 s and stays there
- `API buffer backlog: 9 setpoints in flight (target 2)` repeating at 1 Hz

That is the known unmodelled interpolation-block cost: the loop is
proportional-only, so it can only supply that correction by holding a standing
depth error of about 7 setpoints. It costs 225–250 ms of latency instead of the
intended 50 ms. See Known issues in the reference doc.

**Real failures:**

| symptom | meaning |
|---|---|
| Depth climbs past ~33 and keeps going | The required correction exceeds the loop's clamp authority; it has no equilibrium |
| `ROS queue full (8); dropping oldest setpoint` | The pump cannot keep up with arrivals |
| Any `Motion channel stopped (...)` | A watch trip or a rejected block — read the cause in the message |

---

## 8. Starvation test

Ctrl-C the test source.

| check | expected |
|---|---|
| controller | `Servo stream starved (>75 ms); stopping axes` |
| controller | `Stream ended: stream starved (axes stopped)` |
| `queue_depth` | → **0** within ~100 ms, and holds 0 |

The buffer is `Halt`ed and `Clear`ed rather than played out, so depth drops
straight to 0 with no ramp-down. A gradual decay would mean the queue was
draining on its own, which is not what a controlled stop does.

---

## 9. Restart test

Start the test source again, at `amplitude_rad: 0.0`.

| check | expected |
|---|---|
| controller | a single `Stream started (channel 0 Active)` |
| must **not** appear | `Motion channel stopped`, `holding off`, `Stream ended` |

Repeat two or three times. The leftover `Halt` from step 8 is exactly the
condition that used to make the first status poll after `Execute` read back a
stale `Stop` and tear down the stream it had just started — and because it was a
race, one clean restart is not proof.

---

## 10. Measurement run (optional)

```bash
ros2 bag record -o bench_$(date +%H%M) \
  /joint_states \
  /movensys_manipulator_arm_controller/joint_trajectory \
  /servo_stream_controller/queue_depth
```

Record 60 s or more at a fixed amplitude. Cross-correlating commanded against
measured position gives the true transport latency — the check that does not
depend on trusting the depth topic. Expect roughly 290 ms at depth 9 and 315 ms
at depth 10.

---

## 11. Shutdown

In order: test source → controller → broadcaster → engine launch.

Ctrl-C on the controller runs `endStream`, which stops the axes and frees the
engine-side buffer channels. A `kill -9` leaves the channel allocated instead;
the next run detects and reclaims it
(`CreateApiBuffer failed ... reclaiming`), so it is recoverable, just noisy.

Servo off when you are done:

```bash
ros2 service call /wmx/axis/set_on wmx_r2_message/srv/SetAxis "{index: [0], data: [0]}"
```

---

## Pass criteria

| test | pass |
|---|---|
| Hold (§6) | Depth settles at `target_queue_depth`; axis still; no warnings |
| Motion (§7) | Axis tracks the waveform; depth stable (9–10 today); no faults, no dropped setpoints |
| Starvation (§8) | Starve warning, controlled stop, depth → 0 and stays |
| Restart (§9) | Single clean `Stream started`, no fault, repeatable |

---

## Troubleshooting

| symptom | cause | fix |
|---|---|---|
| `Couldn't parse params file ... Error opening YAML file` | `$BENCH` unset, stale, or pointing at a deleted file | `ls -l "$BENCH"`; the message does not distinguish a missing file from bad YAML |
| `LoadWmxParams` → `success=False, 'File operation failed'` | `$ROBOT` unset, so the path collapsed to `_wmx_parameters.xml` | `echo "$ROBOT"`, then re-run |
| Controller stuck at `waiting for engine...` | `wmx/engine/ready` never arrived | Step 1 is not up |
| Test source repeats `Waiting for 'joint1' on /joint_states ...` | No publisher, or a joint-name mismatch | `ros2 topic info /joint_states` — `Publisher count: 0` means the broadcaster is not running; nonzero means compare its `joint_name` with the test source's |
| Broadcaster logs `Init failed at clear_alarm` / `at set_on` | Engine services were not available at its startup | It does not retry — restart it after step 1 is confirmed |
| Broadcaster crashes immediately | `joint_feedback_rate` is 0 (its default), giving `1000 / 0` in the timer period | Set it; the bench config uses 100 |
| Controller runs but nothing moves | `/moveit2_trajectory/execution_active` is latched true, so setpoints are discarded | Check that no `move_group` execution is in progress |
