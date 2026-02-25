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

### Get Master Info List
```bash
# Discover how many EtherCAT masters are available
ros2 service call /wmx/ecat/get_master_info_list wmx_ros2_message/srv/EcatGetMasterInfoList "{}"
```

### Get Master Info
```bash
# Get state and slave counts for master 0
ros2 service call /wmx/ecat/get_master_info wmx_ros2_message/srv/EcatGetMasterInfo "{master_id: 0}"
```

### Change Slave State
```bash
# Put slave 0 into PreOp (state=2)
ros2 service call /wmx/ecat/change_slave_state wmx_ros2_message/srv/EcatChangeSlaveState "{master_id: 0, slave_id: 0, state: 2}"

# Put slave 0 into SafeOp (state=4)
ros2 service call /wmx/ecat/change_slave_state wmx_ros2_message/srv/EcatChangeSlaveState "{master_id: 0, slave_id: 0, state: 4}"

# Put slave 0 into Op (state=8)
ros2 service call /wmx/ecat/change_slave_state wmx_ros2_message/srv/EcatChangeSlaveState "{master_id: 0, slave_id: 0, state: 8}"
```

### SDO Download (write slave parameter)
```bash
# Write 1 byte (value=6) to slave 0, object index 0x6040 (control word), subindex 0
ros2 service call /wmx/ecat/sdo_download wmx_ros2_message/srv/EcatSdoDownload "{master_id: 0, slave_id: 0, index: 24640, subindex: 0, data: [6]}"

# Write 4 bytes to slave 0, object index 0x607A (target position), subindex 0
ros2 service call /wmx/ecat/sdo_download wmx_ros2_message/srv/EcatSdoDownload "{master_id: 0, slave_id: 0, index: 24698, subindex: 0, data: [0, 0, 0, 0]}"
```

### SDO Upload (read slave parameter)
```bash
# Read 2 bytes from slave 0, object index 0x6041 (status word), subindex 0
ros2 service call /wmx/ecat/sdo_upload wmx_ros2_message/srv/EcatSdoUpload "{master_id: 0, slave_id: 0, index: 24641, subindex: 0, length: 2}"

# Read 4 bytes from slave 0, object index 0x6064 (actual position), subindex 0
ros2 service call /wmx/ecat/sdo_upload wmx_ros2_message/srv/EcatSdoUpload "{master_id: 0, slave_id: 0, index: 24676, subindex: 0, length: 4}"
```

### PDO Read (read process data)
```bash
# Read 2 bytes from slave 0 PDO, index 0x6041 (status word), subindex 0
ros2 service call /wmx/ecat/pdo_read wmx_ros2_message/srv/EcatPdoRead "{master_id: 0, slave_id: 0, index: 24641, subindex: 0, length: 2}"

# Read 4 bytes from slave 0 PDO, index 0x6064 (actual position), subindex 0
ros2 service call /wmx/ecat/pdo_read wmx_ros2_message/srv/EcatPdoRead "{master_id: 0, slave_id: 0, index: 24676, subindex: 0, length: 4}"
```

### TxPDO Write (send command to slave)
```bash
# Write control word 0x000F (enable operation) to slave 0, index 0x6040, subindex 0
ros2 service call /wmx/ecat/txpdo_write wmx_ros2_message/srv/EcatTxPdoWrite "{master_id: 0, slave_id: 0, index: 24640, subindex: 0, data: [15, 0]}"

# Write target velocity 0x00000064 (100) to slave 0, index 0x60FF, subindex 0
ros2 service call /wmx/ecat/txpdo_write wmx_ros2_message/srv/EcatTxPdoWrite "{master_id: 0, slave_id: 0, index: 24831, subindex: 0, data: [100, 0, 0, 0]}"
```

### Diagnosis Scan
```bash
# Run diagnosis scan on master 0 — results published to /diagnostics
ros2 service call /wmx/ecat/diagnosis_scan wmx_ros2_message/srv/EcatDiagnosisScan "{master_id: 0}"

# Monitor diagnostics output
ros2 topic echo /diagnostics

# Monitor 1 Hz master health topic
ros2 topic echo /wmx/ecat/master_info
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

## Notes
- Adjust array values (index, data, numerator, denumerator) based on your actual axis configuration
- For SetDevice service, replace '/path/to/device' and 'device_name' with actual values
- For parameter services, replace 'parameter_name' with actual parameter names from your nodes
- For IO services, `byte` is the IO byte address and `bit` is the bit index within that byte (0–7)
- `set_output_bit` value must be 0 or 1; `set_output_bytes` data values are decimal (e.g. 15 = 0x0F)
- For EtherCAT services, `index` is the SDO/PDO object index in decimal (e.g. 0x6040 = 24640)
- `data` bytes are little-endian; multi-byte values should be split accordingly (e.g. 0x000F = [15, 0])
- EtherCAT state values: Init=1, PreOp=2, SafeOp=4, Op=8
- PDO read/write requires `AllowPdoReadWrite:1` set in the ENI file for the target slave
- Use `ros2 service type <service_name>` to verify service types
- Use `ros2 service list` to see all available services

