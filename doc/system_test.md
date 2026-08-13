## Testing WMX R2 Package

> **WARNING: Do NOT run tests on a machine connected to real EtherCAT hardware.**
> Integration tests start and stop the WMX3 engine, send service calls, and may trigger axis commands. 
Running them while connected to real hardware (e.g. the CR3A manipulator) could cause unexpected motor movement, servo faults, or equipment damage.
>
> Tests are designed to run with the WMX3 SDK in **simulation mode** (no EtherCAT slaves connected). The SDK creates the device successfully but cannot communicate with real drives, so service calls return safely with error responses. No special simulation configuration is needed — just ensure no EtherCAT hardware is connected.
>
> For full WMX3 simulation with virtual axes (using the simulation platform at `/opt/wmx3/Module.ini`), refer to the WMX3 SDK documentation.

### Prerequisites

- WMX3 SDK installed at `/opt/wmx3/`
- ROS 2 workspace built:
  ```
  source /opt/ros/$ROS_DISTRO/setup.bash
  colcon build --packages-up-to wmx_r2_package
  ```
- Integration tests require `sudo` (WMX3 SDK needs root for license and `/dev/cpu_dma_latency`)
- When running with `sudo`, ROS 2 environment variables must be preserved

### Run all tests
```
cd ~/workspaces/movensys_ws
source /opt/ros/$ROS_DISTRO/setup.bash && source install/setup.bash
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
  colcon test --packages-select wmx_r2_message wmx_r2_package
```

### Run a specific test
```
cd ~/workspaces/movensys_ws
source /opt/ros/$ROS_DISTRO/setup.bash && source install/setup.bash
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
  colcon test --packages-select wmx_r2_package \
    --ctest-args -R test_engine_lifecycle
```

### Run unit tests only (no sudo)
Message interface tests do not require hardware or sudo:
```
source /opt/ros/$ROS_DISTRO/setup.bash && source install/setup.bash
colcon test --packages-select wmx_r2_package \
  --ctest-args -R test_message_interfaces
```

### View test results
```
colcon test-result --verbose --test-result-base build/wmx_r2_package
```

### Test coverage

| Test | Subtests | What it covers |
|------|----------|----------------|
| test_message_interfaces | 31 | All 3 msg types (AxisState, AxisPose, AxisVelocity) and 15 srv types: field existence, types, population, renames (in_pos->motion_complete, denumerator->denominator) |
| test_engine_lifecycle | 3 | get_node_states lists every managed node, get_engine_status service responds with valid state, clean shutdown |
| test_lifecycle_nodes | 6 | Nodes expose get_state/change_state, engine discovers and reports their states, rejects an empty node_name and an unknown transition, deactivate/activate round trip, clean shutdown |
| test_core_motion_node | 9 | AxisState publishing with header (stamp, frame_id), array field length consistency, 5 service availability checks, get_params returns data |
| test_io_node | 7 | All 6 IO services available and responding (get/set input/output bit/bytes), clean shutdown |
| test_ethercat_node | 6 | All 4 EtherCAT services available (get_network_state, register_read, reset_statistics, start_hotconnect), master field population, clean shutdown |
| Lint (6 tests) | — | cppcheck, flake8, lint_cmake, pep257, uncrustify, xmllint |

**Total: 12 test targets, 62+ subtests**

### Notes

- Integration tests launch `wmx_engine_node` and the node under test, then shut them down after tests complete
- The nodes under test are lifecycle nodes: `wmx_engine_node` finds them on the graph and
  configures and activates them, so their services only appear once the engine is
  communicating
- The WMX3 free license mode limits communication to one hour
- EtherCAT `get_network_state` may return `success=false` without EtherCAT hardware connected — the test verifies the service responds, not the hardware state
- If a build cache from a previous branch causes errors, clean and rebuild:
  ```
  rm -rf build/wmx_r2_message install/wmx_r2_message
  colcon build --packages-up-to wmx_r2_package
  ```
