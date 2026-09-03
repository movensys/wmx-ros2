# Jogging

Hold-to-move jog on `/wmx/axis/jog`. The publisher must keep republishing while
the operator holds the control; the axis stops once refreshes stop arriving.

## 1. Prepare (after `launch_wmx_r2_general_nodes.md`)
```
# Clear alarms, servo on, home
ros2 service call /wmx/axis/clear_alarm wmx_r2_message/srv/SetAxis "{index: [0], data: [0]}"
ros2 service call /wmx/axis/set_on    wmx_r2_message/srv/SetAxis "{index: [0], data: [1]}"
ros2 service call /wmx/axis/homing    wmx_r2_message/srv/SetAxis "{index: [0], data: [0]}"

# Jog requires Position mode (0)
ros2 service call /wmx/axis/set_mode  wmx_r2_message/srv/SetAxis "{index: [0], data: [0]}"

# Set Gear Ratio
## Panasonic MADLNO5BE servo driver
ros2 service call /wmx/axis/set_gear_ratio wmx_r2_message/srv/SetAxisGearRatio \
  "{index: [0], numerator: [8388608.0], denominator: [360.0]}"
```

## 2a. Jog with the keyboard
```
ros2 run wmx_r2_package jog_keyboard_node --ros-args \
  -p axis:=0 -p velocity:=1000.0 -p acc:=10000.0 -p dec:=10000.0
```
`a` = negative, `d` = positive, `q` = quit. Run it from a terminal, not a launch file.

A terminal reports characters, not key releases, so the node treats a key as held
while auto-repeat characters keep arriving and stops the axis once they stop. How
quickly a release is noticed therefore equals the keyboard repeat delay, which the
node measures and adapts to on the first press and prints:

```
[INFO] Measured a 150 ms key repeat delay, so a released key now stops the axis within 0.21 s.
```

That delay belongs to the machine you type on, which over ssh is not the WMX
machine, so shorten it there:
```
xset r rate 150 30    # on your own PC (660 ms -> 150 ms)
xset r rate           # restore the default
```
macOS and Windows expose the same setting in their keyboard control panel.

## Parameters

`wmx_core_motion_node`:
```
ros2 param set /wmx_core_motion_node jog_timeout_ms 200.0    # stop this long after refreshes stop
ros2 param set /wmx_core_motion_node jog_run_time_ms 2000.0  # max duration of one jog
ros2 param set /wmx_core_motion_node jog_jerk_ratio 0.75     # profile jerk ratio
```

`jog_keyboard_node`:
```
axis velocity acc dec        # what to command
publish_rate      20.0       # refresh rate while a key is held
hold_grace_s      0.1        # release detection once auto-repeat is flowing
initial_grace_s   0.8        # release detection before it is, replaced by the measurement
grace_margin_s    0.06       # headroom added to the measured repeat delay
```

## Notes
- Velocity sign selects the direction.
- Once `jog_run_time_ms` elapses the axis stays stopped until you release and press again.
- The jog uses the `TimeAccJerkRatio` profile, as WOS does; a run time requires a
  time-based profile. `acc`/`dec` are still accelerations on the wire and are
  converted to ramp times internally.
- If the jog stutters right after a key is pressed, lower the terminal key repeat
  delay (`xset r rate 150 30`) or raise `hold_grace_s`.
