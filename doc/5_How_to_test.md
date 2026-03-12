## Testing WMX ROS2 Package

### Prerequisites

- WMX3 SDK installed at `/opt/lmx/`
- ROS 2 Humble workspace built:
  ```bash
  source /opt/ros/humble/setup.bash
  colcon build --packages-up-to wmx_ros2_package
  ```
- Integration tests require `sudo` (WMX3 SDK needs root for license and `/dev/cpu_dma_latency`)
- When running with `sudo`, ROS 2 environment variables must be preserved

### Run all tests

```bash
cd ~/wmx_ros2_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
sudo --preserve-env=PATH \
     --preserve-env=AMENT_PREFIX_PATH \
     --preserve-env=COLCON_PREFIX_PATH \
     --preserve-env=PYTHONPATH \
     --preserve-env=LD_LIBRARY_PATH \
     --preserve-env=ROS_DISTRO \
     --preserve-env=ROS_VERSION \
     --preserve-env=ROS_PYTHON_VERSION \
     --preserve-env=ROS_DOMAIN_ID \
     --preserve-env=RMW_IMPLEMENTATION \
  colcon test --packages-select wmx_ros2_message wmx_ros2_package
```

### Run a specific test

```bash
sudo --preserve-env=PATH \
     --preserve-env=AMENT_PREFIX_PATH \
     --preserve-env=COLCON_PREFIX_PATH \
     --preserve-env=PYTHONPATH \
     --preserve-env=LD_LIBRARY_PATH \
     --preserve-env=ROS_DISTRO \
     --preserve-env=ROS_VERSION \
     --preserve-env=ROS_PYTHON_VERSION \
     --preserve-env=ROS_DOMAIN_ID \
     --preserve-env=RMW_IMPLEMENTATION \
  colcon test --packages-select wmx_ros2_package \
    --ctest-args -R test_engine_lifecycle
```

### Run unit tests only (no sudo)

Message interface tests do not require hardware or sudo:

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon test --packages-select wmx_ros2_package \
  --ctest-args -R test_message_interfaces
```

### View test results

```bash
colcon test-result --verbose --test-result-base build/wmx_ros2_package
```

### Test coverage

| Test | Subtests | What it covers |
|------|----------|----------------|
| test_message_interfaces | 28 | All 3 msg types (AxisState, AxisPose, AxisVelocity) and 13 srv types: field existence, types, population, renames (in_pos->motion_complete, denumerator->denominator) |
| test_engine_lifecycle | 3 | Engine publishes ready=True, get_status service responds with valid state, clean shutdown |
| test_core_motion_node | 9 | AxisState publishing with header (stamp, frame_id), array field length consistency, 5 service availability checks, get_params returns data |
| test_io_node | 7 | All 6 IO services available and responding (get/set input/output bit/bytes), clean shutdown |
| test_ethercat_node | 6 | All 4 EtherCAT services available (get_network_state, register_read, reset_statistics, start_hotconnect), master field population, clean shutdown |
| Lint (6 tests) | — | cppcheck, flake8, lint_cmake, pep257, uncrustify, xmllint |

**Total: 11 test targets, 53+ subtests**

### Notes

- Integration tests launch `wmx_engine_node` and the node under test, then shut them down after tests complete
- The WMX3 free license mode limits communication to one hour
- EtherCAT `get_network_state` may return `success=false` without EtherCAT hardware connected — the test verifies the service responds, not the hardware state
- If a build cache from a previous branch causes errors, clean and rebuild:
  ```bash
  rm -rf build/wmx_ros2_message install/wmx_ros2_message
  colcon build --packages-up-to wmx_ros2_package
  ```
