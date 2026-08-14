# WMX R2 General Nodes Reference

## Typical Startup Sequence
```bash
# 1. Verify engine is communicating
ros2 service call /wmx/engine/get_engine_status std_srvs/srv/Trigger "{}"

# 2. Set Gear Ratio 
ros2 service call /wmx/axes/set_gear_ratio wmx_r2_message/srv/SetAxesGearRatio \
  "{indices: [0, 1], numerators: [8388608.0, 8388608.0], denominators: [360.0, 360.0]}"

# 3. Clear any amp alarms
ros2 service call /wmx/axes/clear_amp_alarm wmx_r2_message/srv/SetAxes "{indices: [0,1], data: [0,0]}"

# 4. Enable servos
ros2 service call /wmx/axes/set_servo_on wmx_r2_message/srv/SetAxes "{indices: [0,1], data: [1,1}"

# 5. Home all axes (sets current position as home)
ros2 service call /wmx/axes/start_home wmx_r2_message/srv/SetAxes "{indices: [0,1], data: [0,0]}"
```
---

## Engine Topics (Warning: These command will rotate the axes)
### Send Axis Absolute Position
```
ros2 topic pub --once /wmx/axes/start_pos wmx_r2_message/msg/AxesPose \
    "{indices: [0,1], positions: [45, -90 ], velocities: [10, 20], accelerations: [10, 20], decelerations: [10, 20]}" 
```

### Send Axis Relative Position
```
ros2 topic pub --once /wmx/axes/start_mov wmx_r2_message/msg/AxesPose \
    "{indices: [0, 1], positions: [10, -10], velocities: [10, 20], accelerations: [10, 10], decelerations: [10, 20]}"
```


### Send Axis Velocity
```
ros2 topic pub --once /wmx/axes/start_vel wmx_r2_message/msg/AxesVelocity \
    "{indices: [0, 1], velocities: [10, -10], accelerations: [10, 20], decelerations: [10, 20]}"  
```



### Jog (hold-to-move)
`/wmx/axes/start_jog` is a dead-man command: the publisher must keep republishing
while the operator holds the control. The axis is stopped once refreshes stop
arriving (`jog_timeout_ms`). The sign of `velocities` selects the direction.

Jog requires the axis to be in **Position mode**, on top of the usual startup
sequence above:
```
ros2 service call /wmx/axes/set_axis_command_mode wmx_r2_message/srv/SetAxes \
    "{indices: [0], data: [0]}"
```

#### Jog from the CLI
```
# Positive direction. Ctrl-C acts as the release.
ros2 topic pub -r 20 /wmx/axes/start_jog wmx_r2_message/msg/AxesVelocity \
    "{indices: [0], velocities: [10000], accelerations: [100000], decelerations: [100000]}"

# Negative direction
ros2 topic pub -r 20 /wmx/axes/start_jog wmx_r2_message/msg/AxesVelocity \
    "{indices: [0], velocities: [-10000], accelerations: [100000], decelerations: [100000]}"
```

#### Stop
```
ros2 service call /wmx/axes/stop wmx_r2_message/srv/SetAxes "{indices: [0], data: [0]}"
```

`wmx_core_motion_node`'s jog tuning (`jog_timeout_ms`, `jog_run_time_ms`,
`jog_jerk_ratio`) is read once at startup, so set it in the launch config YAML
rather than with `ros2 param set`. See **Behavior Notes** below.

---

## Engine Services
### Get Status
```
ros2 service call /wmx/engine/get_engine_status std_srvs/srv/Trigger "{}"
```

### Set Communication (start/stop EtherCAT comms)
```
# Start communication
ros2 service call /wmx/engine/set_comm std_srvs/srv/SetBool "{data: true}"

# Stop communication
ros2 service call /wmx/engine/set_comm std_srvs/srv/SetBool "{data: false}"
```

### Set Engine Device (create/close WMX3 device)
The path and device name are fixed in `wmx_engine_node.hpp` (`/opt/wmx3/`, named
`wmx_r2`), so this is a plain on/off.
```
# Create device
ros2 service call /wmx/engine/set_engine std_srvs/srv/SetBool "{data: true}"

# Close device (wmx_lifecycle_manager_node follows: every managed node goes to
# unconfigured within one discovery_period)
ros2 service call /wmx/engine/set_engine std_srvs/srv/SetBool "{data: false}"
```

### Scan Network
Moved to `wmx_ethercat_node` (it owns the EtherCAT master handle) and now takes
the master id. The node must be `active`.
```
ros2 service call /wmx/ecat/scan_network wmx_r2_message/srv/EcatScanNetwork \
  "{master_id: 0}"
```

---

## Lifecycle Node Services (wmx_engine_node)
`wmx_core_motion_node`, `wmx_io_node`, `wmx_ethercat_node` and the robot
controllers are lifecycle nodes. They start `unconfigured` and only attach to the
WMX3 device once `wmx_lifecycle_manager_node` drives them through `configure` and
`activate`.

**The managed nodes follow the engine.** Every `discovery_period` (2 s by
default) the manager asks `wmx/engine/get_engine_status`:

- **Engine `Communicating`** — it scans the ROS graph for nodes offering
  `<node>/change_state` and brings up every one it has not handled yet, so all
  managed nodes end up `active`. A node that joins late or respawns is picked up
  on a later sweep; a node an operator deactivated on purpose is left alone.
- **Engine stopped, or its device closed** — every managed node is deactivated
  **and cleaned up**, back to `unconfigured`, in reverse bring-up order. Cleanup
  matters: the engine closed the device out from under them, so the handles they
  hold are dead and must be re-created. The manager then forgets them, so the
  next time the engine comes back they are configured and activated again.

The down sweep is edge-triggered on the engine going away, so it lands within one
`discovery_period` of the engine stopping — not instantly. Lower
`discovery_period` if you need a tighter window.

This applies to **every** lifecycle node in the manager's namespace, not only the
ones in this package — a lifecycle `joint_state_publisher`, a nav2 node or your
own managed node is configured and activated the same way, after the WMX
device-level nodes it may depend on. List them in `managed_nodes` to fix the
order; leave it empty to take every lifecycle node on the graph.

There are two services, and both answer for **all** nodes: one reads the states,
one drives a transition.

### Read the states
Lists every lifecycle node currently on the graph, with its state. `node_names`
and `states` are index aligned, in bring-up order; `message` is the same list as
one human-readable string.
```
ros2 service call /wmx/lifecycle/get_node_states wmx_r2_message/srv/GetNodeStates "{}"
```

### Drive a transition
`transition` is one of `configure`, `activate`, `deactivate`, `cleanup`,
`shutdown`, `bringup` (configure + activate), `bringdown` (deactivate).

One node, by name:
```
ros2 service call /wmx/lifecycle/set_node_state wmx_r2_message/srv/SetNodeState \
  "{node_name: 'wmx_io_node', transition: 'deactivate'}"
```
Every node found on the graph — leave `node_name` empty:
```
ros2 service call /wmx/lifecycle/set_node_state wmx_r2_message/srv/SetNodeState \
  "{node_name: '', transition: 'bringup'}"
```
Bring-up transitions (`configure`, `activate`, `bringup`) run in `managed_nodes`
order; transitions that take nodes down (`deactivate`, `cleanup`, `shutdown`,
`bringdown`) run in reverse, so the controllers stop before the device-level
nodes whose axes they command. The response reports every node it touched and
the state each ended in; `success` is true only if all of them succeeded.

A plain name is resolved against `wmx_lifecycle_manager_node`'s namespace; an
absolute name (`/robot1/wmx_io_node`) is used as given.

Whatever an operator sets through this service stands: those nodes are marked
handled, so the discovery sweep leaves them alone until they leave the graph.

The standard CLI works too, and bypasses the engine:
```
ros2 lifecycle nodes
ros2 lifecycle get /wmx_io_node
ros2 lifecycle set /wmx_io_node deactivate
```

---

## WMX Parameter Services
Served by `wmx_engine_node`, which owns the device. Set the engine's
`wmx_param_file_path` parameter (the robot launch files do) to import a file
automatically once communication starts.
### Load Parameters from File
```
# Dobot CR3A
ros2 service call /wmx/engine/load_wmx_params wmx_r2_message/srv/LoadWmxParams \
  "{file_path: '/home/$USER/movensys_ws/install/wmx_r2_package/share/wmx_r2_package/config/cr3a_wmx_parameters.xml'}"

# Diffbot
ros2 service call /wmx/engine/load_wmx_params wmx_r2_message/srv/LoadWmxParams \
  "{file_path: '/home/$USER/movensys_ws/install/wmx_r2_package/share/wmx_r2_package/config/diffbot_wmx_parameters.xml'}"
```

### Get Parameters (inspect active axis config)
```
# Single axis
ros2 service call /wmx/engine/get_wmx_params wmx_r2_message/srv/GetWmxParams "{indices: [0]}"

# Multiple axes
ros2 service call /wmx/engine/get_wmx_params wmx_r2_message/srv/GetWmxParams "{indices: [0,1,2,3,4,5]}"
```

---

## Axis Services

### Clear Alarm
```
ros2 service call /wmx/axes/clear_amp_alarm wmx_r2_message/srv/SetAxes "{indices: [0,1], data: [0,0]}"
```

### Set Servo On / Off
```
# Servo On
ros2 service call /wmx/axes/set_servo_on wmx_r2_message/srv/SetAxes "{indices: [0,1], data: [1,1]}"

# Servo Off
ros2 service call /wmx/axes/set_servo_on wmx_r2_message/srv/SetAxes "{indices: [0,1], data: [0,0]}"
```

### Set Command Mode
```
# Position mode (0)
ros2 service call /wmx/axes/set_axis_command_mode wmx_r2_message/srv/SetAxes "{indices: [0,1], data: [0,0]}"

# Velocity mode (1)
ros2 service call /wmx/axes/set_axis_command_mode wmx_r2_message/srv/SetAxes "{indices: [0,1], data: [1,1]}"
```

### Set Polarity
```
# Normal (1)
ros2 service call /wmx/axes/set_axis_polarity wmx_r2_message/srv/SetAxes "{indices: [0,1], data: [1,1]}"

# Reversed (-1)
ros2 service call /wmx/axes/set_axis_polarity wmx_r2_message/srv/SetAxes "{indices: [0,1], data: [-1,-1]}"
```

### Set Gear Ratio
```
ros2 service call /wmx/axes/set_gear_ratio wmx_r2_message/srv/SetAxesGearRatio \
  "{indices: [0,1], numerators: [1.0,1.0], denominators: [1.0,1.0]}"
```



### Homing (sets current position as home)
```
ros2 service call /wmx/axes/start_home wmx_r2_message/srv/SetAxes "{indices: [0,1], data: [0,0]}"
```

### Stop (decelerate to a stop)
```
ros2 service call /wmx/axes/stop wmx_r2_message/srv/SetAxes "{indices: [0,1], data: [0,0]}"
```

---

## IO Services

### Read Input Bit
```
ros2 service call /wmx/io/get_input_bit wmx_r2_message/srv/GetIoBit "{byte: 0, bit: 0}"
```

### Read Output Bit
```
ros2 service call /wmx/io/get_output_bit wmx_r2_message/srv/GetIoBit "{byte: 0, bit: 0}"
```

### Read Input Byte
```
ros2 service call /wmx/io/get_input_byte wmx_r2_message/srv/GetIoByte "{byte: 0}"
```

### Read Output Byte
```
ros2 service call /wmx/io/get_output_byte wmx_r2_message/srv/GetIoByte "{byte: 0}"
```

### Read Input Bytes
```
ros2 service call /wmx/io/get_input_bytes wmx_r2_message/srv/GetIoBytes "{byte: 0, length: 4}"
```

### Read Output Bytes
```
ros2 service call /wmx/io/get_output_bytes wmx_r2_message/srv/GetIoBytes "{byte: 0, length: 4}"
```

### Set Output Bit
```
# Set bit
ros2 service call /wmx/io/set_output_bit wmx_r2_message/srv/SetIoBit "{byte: 0, bit: 0, value: 1}"

# Clear bit
ros2 service call /wmx/io/set_output_bit wmx_r2_message/srv/SetIoBit "{byte: 0, bit: 0, value: 0}"
```

### Set Output Bits
```
# Several scattered bits in one call: byte 0 bit 1 -> 1, byte 2 bit 5 -> 0.
# They are written together in a single SDK call, not one cycle apart.
ros2 service call /wmx/io/set_output_bits wmx_r2_message/srv/SetIoBits \
  "{byte: [0, 2], bit: [1, 5], value: [1, 0]}"
```

### Set Output Byte
```
# Set output byte 2 to 0x0F
ros2 service call /wmx/io/set_output_byte wmx_r2_message/srv/SetIoByte "{byte: 2, value: 15}"
```

### Set Output Bytes
```
# Set output byte 2 to 0x0F, byte 3 to 0x0E
ros2 service call /wmx/io/set_output_bytes wmx_r2_message/srv/SetIoBytes "{byte: 2, data: [15, 14]}"
```

---

## EtherCAT Services

### Get Network State
```
# Master 0
ros2 service call /wmx/ecat/get_network_state wmx_r2_message/srv/EcatGetNetworkState "{master_id: 0}"

# Master 1
ros2 service call /wmx/ecat/get_network_state wmx_r2_message/srv/EcatGetNetworkState "{master_id: 1}"
```

### Register Read
```
# 1 byte from slave 0, register 0x000 (type register)
ros2 service call /wmx/ecat/register_read wmx_r2_message/srv/EcatRegisterRead \
  "{master_id: 0, slave_id: 0, reg_address: 0, length: 1}"

# 4 bytes from slave 0, register 0x010 (vendor ID)
ros2 service call /wmx/ecat/register_read wmx_r2_message/srv/EcatRegisterRead \
  "{master_id: 0, slave_id: 0, reg_address: 16, length: 4}"

# 16 bytes from slave 1, register 0x100 (DL status)
ros2 service call /wmx/ecat/register_read wmx_r2_message/srv/EcatRegisterRead \
  "{master_id: 0, slave_id: 1, reg_address: 256, length: 16}"
```

### Reset Statistics
```
# Resets ref-clock, transmit statistics, and re-scans network
ros2 service call /wmx/ecat/reset_statistics wmx_r2_message/srv/EcatResetStatistics "{master_id: 0}"
```

### Start Hotconnect
```
# Enable dynamic slave discovery (call once after network reaches Op state)
ros2 service call /wmx/ecat/start_hotconnect wmx_r2_message/srv/EcatStartHotconnect "{master_id: 0}"
```

---

## Notes

> Standard ROS2 parameter services (get, set, list, describe) are available on all nodes via
> `ros2 param list|get|set <node_name>`.

**General**
- Use `ros2 service list` to see all available services
- Use `ros2 service type <service_name>` to verify service types
- `indices` and `data` arrays must be the same length

**Axis**
- `data` for `set_on`: `1` = servo on, `0` = servo off
- `data` for `set_mode`: `0` = Position, `1` = Velocity, `2` = Torque
- `data` for `set_polarity`: `1` = normal, `-1` = reversed
- Homing always uses `CurrentPos` type — sets the current encoder position as home (zero)
- `stop` uses `Motion::Stop` (decelerate); `data` is ignored

**Jog**
- Runs `Motion::StartJog`, which requires the axis to be in Position mode (`set_mode` `0`)
- `jog_timeout_ms` (default `200`) — axis stops this long after refreshes stop arriving
- `jog_run_time_ms` (default `2000`) — max duration of a single jog, enforced by the
  engine so the axis still stops if the publishing node dies. Once it elapses the axis
  stays stopped until the operator releases and presses again
- Republishing the same velocities only refreshes the dead-man; it does not re-issue
  `StartJog` (jog-over-jog override is undefined in WMX3)

**WMX Parameters**
- `params/load` requires an absolute path to a valid WMX3 XML file; engine must be ready first
- `params/get` returns a structured dump in `params_dump` per requested axis
- `CommandMode`: `0`=Position, `1`=Velocity, `2`=Torque
- `HomeType`: `0`=CurrentPos, `1`=ZPulse, `2`=HS, `4`=HSZPulse
- `HomeDirection`: `0`=Positive, `1`=Negative

**IO**
- `byte` is the IO byte address; `bit` is the indices within that byte (0–7)
- `set_output_bit` value must be `0` or `1`
- `set_output_bytes` data values are decimal (e.g. `15` = `0x0F`)

**EtherCAT**
- Master state values: `None`=0, `Init`=1, `Preop`=2, `Boot`=4, `Safeop`=8, `Op`=16
- Master mode values: `CyclicMode`=0, `PPMode`=1, `MonitorMode`=2
- `reg_address` is a 12-bit ESC register address (decimal), valid range `0x000`–`0xFFF`
- `reg_address + length` must not exceed `0x1000` (4096 bytes)
- `reset_statistics` calls `ResetRefClockInfo` + `ResetTransmitStatisticsInfo` + `ScanNetwork` in sequence
- `scan_network` moved here from `wmx_engine_node` (`/wmx/engine/scan_network` no longer exists)

**Lifecycle**
- A service on an inactive node answers `success: false` with the current state instead of
  touching the device; topics and timers are stopped while inactive
- `deactivate` stops what the node was driving (jogging axes, wheel commands, trajectories)
  but keeps the device attached; `cleanup` also detaches from the device
- Bring-up order is device-level nodes (`wmx_*`) first, then the controllers that command
  their axes; take-down is the reverse
- `wmx/engine/set_comm false` deactivates the lifecycle nodes first, and
  `wmx/engine/set_engine false` cleans them up, so no node holds a handle to a device
  the engine is about to close
