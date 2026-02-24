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

## Notes
- Adjust array values (index, data, numerator, denumerator) based on your actual axis configuration
- For SetDevice service, replace '/path/to/device' and 'device_name' with actual values
- For parameter services, replace 'parameter_name' with actual parameter names from your nodes
- For IO services, `byte` is the IO byte address and `bit` is the bit index within that byte (0–7)
- `set_output_bit` value must be 0 or 1; `set_output_bytes` data values are decimal (e.g. 15 = 0x0F)
- Use `ros2 service type <service_name>` to verify service types
- Use `ros2 service list` to see all available services

