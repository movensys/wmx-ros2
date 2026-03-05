# ROS2 Service Test Commands

## Axis Services

### Clear Alarm
```bash
ros2 service call /wmx/axis/clear_alarm wmx_ros2_message/srv/SetAxis "{index: [0, 1], data: [1, 1]}"
```

### Homing
```bash
ros2 service call /wmx/axis/homing wmx_ros2_message/srv/SetAxis "{index: [0, 1], data: [1, 1]}"
```

### Set Gear Ratio
```bash
ros2 service call /wmx/axis/set_gear_ratio wmx_ros2_message/srv/SetAxisGearRatio "{index: [0, 1], numerator: [1.0, 1.0], denumerator: [1.0, 1.0]}"
```

### Set Mode
```bash
ros2 service call /wmx/axis/set_mode wmx_ros2_message/srv/SetAxis "{index: [0, 1], data: [1, 1]}"
```

### Set On
```bash
ros2 service call /wmx/axis/set_on wmx_ros2_message/srv/SetAxis "{index: [0, 1], data: [1, 1]}"
```

### Set Polarity
```bash
ros2 service call /wmx/axis/set_polarity wmx_ros2_message/srv/SetAxis "{index: [0, 1], data: [1, 1]}"
```

## Engine Services

### Get Status
```bash
ros2 service call /wmx/engine/get_status std_srvs/srv/Trigger "{}"
```

### Set Communication
```bash
ros2 service call /wmx/engine/set_comm std_srvs/srv/SetBool "{data: true}"
```

### Set Device
```bash
ros2 service call /wmx/engine/set_device wmx_ros2_message/srv/SetEngine "{data: true, path: '/path/to/device', name: 'device_name'}"
```

## Parameter Services - wmx_core_motion_node

### Describe Parameters
```bash
ros2 service call /wmx_core_motion_node/describe_parameters rcl_interfaces/srv/DescribeParameters "{names: ['parameter_name']}"
```

### Get Parameter Types
```bash
ros2 service call /wmx_core_motion_node/get_parameter_types rcl_interfaces/srv/GetParameterTypes "{names: ['parameter_name']}"
```

### Get Parameters
```bash
ros2 service call /wmx_core_motion_node/get_parameters rcl_interfaces/srv/GetParameters "{names: ['parameter_name']}"
```

### List Parameters
```bash
ros2 service call /wmx_core_motion_node/list_parameters rcl_interfaces/srv/ListParameters "{prefixes: [], depth: 0}"
```

### Set Parameters
```bash
ros2 service call /wmx_core_motion_node/set_parameters rcl_interfaces/srv/SetParameters "{parameters: [{name: 'parameter_name', value: {type: 1, bool_value: true}}]}"
```

### Set Parameters Atomically
```bash
ros2 service call /wmx_core_motion_node/set_parameters_atomically rcl_interfaces/srv/SetParametersAtomically "{parameters: [{name: 'parameter_name', value: {type: 1, bool_value: true}}]}"
```

## Parameter Services - wmx_engine_node

### Describe Parameters
```bash
ros2 service call /wmx_engine_node/describe_parameters rcl_interfaces/srv/DescribeParameters "{names: ['parameter_name']}"
```

### Get Parameter Types
```bash
ros2 service call /wmx_engine_node/get_parameter_types rcl_interfaces/srv/GetParameterTypes "{names: ['parameter_name']}"
```

### Get Parameters
```bash
ros2 service call /wmx_engine_node/get_parameters rcl_interfaces/srv/GetParameters "{names: ['parameter_name']}"
```

### List Parameters
```bash
ros2 service call /wmx_engine_node/list_parameters rcl_interfaces/srv/ListParameters "{prefixes: [], depth: 0}"
```

### Set Parameters
```bash
ros2 service call /wmx_engine_node/set_parameters rcl_interfaces/srv/SetParameters "{parameters: [{name: 'parameter_name', value: {type: 1, bool_value: true}}]}"
```

### Set Parameters Atomically
```bash
ros2 service call /wmx_engine_node/set_parameters_atomically rcl_interfaces/srv/SetParametersAtomically "{parameters: [{name: 'parameter_name', value: {type: 1, bool_value: true}}]}"
```

## IO Services

### Get Input Bit
```bash
ros2 service call /wmx/io/get_input_bit wmx_ros2_message/srv/GetIoBit "{byte: 0, bit: 0}"
```

### Get Output Bit
```bash
ros2 service call /wmx/io/get_output_bit wmx_ros2_message/srv/GetIoBit "{byte: 0, bit: 0}"
```

### Get Input Bytes
```bash
ros2 service call /wmx/io/get_input_bytes wmx_ros2_message/srv/GetIoBytes "{byte: 0, length: 4}"
```

### Get Output Bytes
```bash
ros2 service call /wmx/io/get_output_bytes wmx_ros2_message/srv/GetIoBytes "{byte: 0, length: 4}"
```

### Set Output Bit
```bash
# Set bit 0 of output byte 0 to 1
ros2 service call /wmx/io/set_output_bit wmx_ros2_message/srv/SetIoBit "{byte: 0, bit: 0, value: 1}"

# Clear bit 0 of output byte 0
ros2 service call /wmx/io/set_output_bit wmx_ros2_message/srv/SetIoBit "{byte: 0, bit: 0, value: 0}"
```

### Set Output Bytes
```bash
# Set output byte 2 to 0x0F, byte 3 to 0x0E
ros2 service call /wmx/io/set_output_bytes wmx_ros2_message/srv/SetIoBytes "{byte: 2, data: [15, 14]}"
```

## Parameter Services - wmx_io_node

### Describe Parameters
```bash
ros2 service call /wmx_io_node/describe_parameters rcl_interfaces/srv/DescribeParameters "{names: ['parameter_name']}"
```

### Get Parameter Types
```bash
ros2 service call /wmx_io_node/get_parameter_types rcl_interfaces/srv/GetParameterTypes "{names: ['parameter_name']}"
```

### Get Parameters
```bash
ros2 service call /wmx_io_node/get_parameters rcl_interfaces/srv/GetParameters "{names: ['parameter_name']}"
```

### List Parameters
```bash
ros2 service call /wmx_io_node/list_parameters rcl_interfaces/srv/ListParameters "{prefixes: [], depth: 0}"
```

### Set Parameters
```bash
ros2 service call /wmx_io_node/set_parameters rcl_interfaces/srv/SetParameters "{parameters: [{name: 'parameter_name', value: {type: 1, bool_value: true}}]}"
```

### Set Parameters Atomically
```bash
ros2 service call /wmx_io_node/set_parameters_atomically rcl_interfaces/srv/SetParametersAtomically "{parameters: [{name: 'parameter_name', value: {type: 1, bool_value: true}}]}"
```

## EtherCAT Services

### Get Network State (ec-state)
```bash
# Get full master statistics and all slave details for master 0
ros2 service call /wmx/ecat/get_network_state wmx_ros2_message/srv/EcatGetNetworkState "{master_id: 0}"

# master 1
ros2 service call /wmx/ecat/get_network_state wmx_ros2_message/srv/EcatGetNetworkState "{master_id: 1}"
```

### Register Read (ec-reg)
```bash
# Read 1 byte from slave 0, register 0x000 (type register)
ros2 service call /wmx/ecat/register_read wmx_ros2_message/srv/EcatRegisterRead "{master_id: 0, slave_id: 0, reg_address: 0, length: 1}"

# Read 4 bytes from slave 0, register 0x010 (vendor ID area)
ros2 service call /wmx/ecat/register_read wmx_ros2_message/srv/EcatRegisterRead "{master_id: 0, slave_id: 0, reg_address: 16, length: 4}"

# Read 16 bytes from slave 1, register 0x100 (DL status)
ros2 service call /wmx/ecat/register_read wmx_ros2_message/srv/EcatRegisterRead "{master_id: 0, slave_id: 1, reg_address: 256, length: 16}"
```

### Reset Statistics (ec-reset)
```bash
# Reset ref-clock info, transmit statistics, and re-scan network on master 0
ros2 service call /wmx/ecat/reset_statistics wmx_ros2_message/srv/EcatResetStatistics "{master_id: 0}"
```

### Start Hotconnect (ec-hc)
```bash
# Enable hot-connect mode on master 0 (allows slaves to join/leave without restart)
ros2 service call /wmx/ecat/start_hotconnect wmx_ros2_message/srv/EcatStartHotconnect "{master_id: 0}"
```

## Parameter Services - wmx_ethercat_node

### Describe Parameters
```bash
ros2 service call /wmx_ethercat_node/describe_parameters rcl_interfaces/srv/DescribeParameters "{names: ['parameter_name']}"
```

### Get Parameters
```bash
ros2 service call /wmx_ethercat_node/get_parameters rcl_interfaces/srv/GetParameters "{names: ['parameter_name']}"
```

### List Parameters
```bash
ros2 service call /wmx_ethercat_node/list_parameters rcl_interfaces/srv/ListParameters "{prefixes: [], depth: 0}"
```

## WMX Parameter Services

### Load Parameters from File
```bash
# Load CR3A parameters
ros2 service call /wmx/params/load wmx_ros2_message/srv/LoadWmxParams \
  "{file_path: '/home/engine/ros2_ws/install/wmx_ros2_package/share/wmx_ros2_package/config/cr3a_wmx_parameters.xml'}"

# Load Baymax parameters
ros2 service call /wmx/params/load wmx_ros2_message/srv/LoadWmxParams \
  "{file_path: '/home/engine/ros2_ws/install/wmx_ros2_package/share/wmx_ros2_package/config/baymax_wmx_parameters.xml'}"
```

### Get Parameters (display all active params for specified axes)
```bash
# Display parameters for axis 0
ros2 service call /wmx/params/get wmx_ros2_message/srv/GetWmxParams "{index: [0]}"

# Display parameters for axes 0 and 1
ros2 service call /wmx/params/get wmx_ros2_message/srv/GetWmxParams "{index: [0, 1]}"
```

## Notes
- Adjust array values (index, data, numerator, denumerator) based on your actual axis configuration
- For SetDevice service, replace '/path/to/device' and 'device_name' with actual values
- For parameter services, replace 'parameter_name' with actual parameter names from your nodes
- For IO services, `byte` is the IO byte address and `bit` is the bit index within that byte (0–7)
- `set_output_bit` value must be 0 or 1; `set_output_bytes` data values are decimal (e.g. 15 = 0x0F)
- EtherCAT master state values: None=0, Init=1, Preop=2, Boot=4, Safeop=8, Op=16
- EtherCAT master mode values: CyclicMode=0, PPMode=1, MonitorMode=2
- `reg_address` is a 12-bit ESC register address (decimal or hex), valid range 0x000–0xFFF
- `reg_address + length` must not exceed 0x1000 (4096 bytes)
- `reset_statistics` calls ResetRefClockInfo + ResetTransmitStatisticsInfo + ScanNetwork in sequence
- `start_hotconnect` enables dynamic slave discovery; call once after network is Op
- Use `ros2 service type <service_name>` to verify service types
- Use `ros2 service list` to see all available services
- `params/load` requires an absolute path to a valid WMX3 XML parameter file; engine must be ready first
- `params/get` returns a formatted string in `params_dump`; print it with `echo` or check the terminal output directly
- `CommandMode` values: 0=Position, 1=Velocity, 2=Torque
- `HomeType` values: 0=CurrentPos, 1=ZPulse, 2=HS, 4=HSZPulse (see CoreMotionApi.h `HomeType::T`)
- `HomeDirection` values: 0=Positive, 1=Negative

