# Position streaming through the WMX3 cyclic buffer

Lets a continuously updated joint target — MoveIt Servo, a teleop device — drive
the CR3A, which `StartCSplinePos` cannot do because it needs the whole trajectory
up front and cannot be blended into.

`WmxSystemHardware` exports a `position` command interface per joint and streams
each commanded value into the WMX3 **cyclic buffer**, one command per
`ros2_control` cycle. A stock `JointTrajectoryController` sits on top, which gives
both endpoints MoveIt already expects — on the same names it uses in simulation.

```
MoveIt Servo ─► /movensys_manipulator_arm_controller/joint_trajectory ─┐
move_group   ─► /movensys_manipulator_arm_controller/follow_joint_trajectory ─┤
                                                                              ▼
                                        JointTrajectoryController (stock ros2_control)
                                                     │ interpolates to the 100 Hz loop
                                                     ▼
                                  WmxSystemHardware::write() ─► cyclic buffer (AbsolutePos)
                                                     │ engine interpolates intervalCycles
                                                     ▼
                                              WMX engine ─► EtherCAT ─► servos
```

Because the controller is named `movensys_manipulator_arm_controller`, **no
changes are needed on the `movensys-manipulator` side**: it matches both
`command_out_topic` in `config/<model>/servo.yaml` and `action_ns` in
`config/<model>/moveit_controllers.yaml`.

## Bring-up

```bash
ros2 launch wmx_r2_control wmx_r2_control_cr3a_manipulator.launch.py use_sim_time:=false
```

(wrapped in the `sudo --preserve-env` block from
[launch_dobot_cr3a_manipulator.md](launch_dobot_cr3a_manipulator.md) — real-time
scheduling needs root.)

On activation the hardware logs the timing it resolved:

```
Cyclic buffer streaming 6 axes: engine cycle 0.500 ms, seed push period 10.000 ms
  -> 20 cycles per command, depth setpoint 2 commands, maxAcc 5.000
```

Check `engine cycle` against your engine's actual configuration. It is read from
`CoreMotionStatus::cycleTimeMilliseconds`, never hardcoded, so a surprising value
here means the engine is configured differently than you expect and every
commanded velocity scales with it.

## Validation ladder

Work through these in order; each is a gate. Run steps 1–3 in **SIL** (simulated
engine, no arm attached) before touching hardware.

**1. Hold still.** Load the controller but do not activate it, so nothing writes
to the command interfaces and the hardware holds the position it seeded:

```bash
ros2 launch wmx_r2_control wmx_r2_control_cr3a_manipulator.launch.py \
     activate_arm_controller:=false
```

Leave it for 60 s. Expect **zero** commanded drift and no warnings. This is the
step that catches a bad seed or a sign error in the depth regulator — the two
failure modes that would otherwise show up as motion.

**2. Single-axis ramp.** Activate the controller and send a slow one-joint
trajectory. Watch for `MaxAccError` (see Tuning) and confirm commanded tracks
actual.

**3. Full move_group plan.** Plan and execute with MoveIt through the new
controller. Compare against the same plan run via the old `StartCSplinePos` path
(`stream_position:=false`) — this is the regression gate for replacing a qualified
motion path.

**4. Keyboard Servo teleop.** `keyboard_teleop` from
`movensys_manipulator_moveit_config` — continuous streaming with a deterministic
input. Tune `update_rate` and `cyclic_target_periods` here.

**5. Quest teleop.** Last, because it adds clutch edges and TF behaviour on top of
everything above.

## Tuning

| Knob | Where | Effect |
|------|-------|--------|
| `update_rate` | `config/cr3a_controllers.yaml` | Loop rate, so the size of each buffered command. Raising it shrinks the quantum, letting `cyclic_target_periods` come down — the main lever on teleop latency. |
| `cyclic_target_periods` | `urdf/cr3a.wmx.ros2_control.xacro` | Commands of runway held queued. Each buys tolerance to one late `write()` and costs one push-period of latency. |
| `cyclic_max_acc` | same | Acceleration **trip** in rad/s². Exceeding it quick-stops the axis and clears the buffer. |
| `cyclic_push_period_ms` | same | Seed only; the hardware measures the real period and converges. Keep it at `1000 / update_rate`. |

`cyclic_max_acc` defaults to 5.0, with `joint_limits.yaml` allowing 2.0. If normal
motion trips `MaxAccError`, raise it — it is a safety limit, not a smoother, and
too tight means nuisance stops.

Note the config and URDF are installed copies, so `colcon build` is required
after editing them unless the workspace was built with `--symlink-install`.

## Diagnostics

| Log | Meaning |
|-----|---------|
| `Cyclic push rejected (state=… depth=…)` | Transient: buffer full, or a non-finite command. The axis holds at its last commanded position. Occasional entries under load are survivable; a steady stream is not. |
| `Cyclic buffer faulted (MaxAccError…)` | Latching. Pushing again cannot recover it; the hardware returns an error so the controller manager can deactivate. Fix the cause, then re-activate. |
| `… (ServoOff)` | The servo dropped under a running stream. Also latching. |
| `depth` climbing steadily | The regulator is losing ground — commanded motion is falling behind. Check for a loop period far from `cyclic_push_period_ms`. |
| `depth` pinned at 0 | Underrunning every cycle: the arm dwells then steps. Raise `cyclic_target_periods`. |

## Constraints

- **The axes are exclusive while streaming.** Once the buffer executes, the axes
  are in `DirectControl` and the standalone
  `wmx_r2_package/joint_trajectory_controller` node's `StartCSplinePos` will be
  rejected. The launch file therefore starts one or the other, never both.
- **Position and velocity joints cannot share a `<ros2_control>` block.** Position
  joints need `DirectControl`; velocity joints need the axis idle for `StartVel`.
  `on_init` rejects the mix rather than letting them fight at runtime.
- **Axes must be in Position command mode** (`AxisCommandMode` 0, which
  `cr3a_wmx_parameters.xml` already sets). `on_activate` checks this and fails
  with a clear message, because the alternative is the latching
  `CommandModeMismatchError` at the first command.

## Falling back

No file edits needed:

```bash
ros2 launch wmx_r2_control wmx_r2_control_cr3a_manipulator.launch.py \
     stream_position:=false
```

Joints go back to feedback-only and the standalone
`joint_trajectory_controller` node provides motion, as before. The fully separate
`wmx_r2_package/launch/wmx_r2_cr3a_manipulator.launch.py` stack is also untouched.
