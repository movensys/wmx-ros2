# Aggregated robot arm state, published periodically by wmx_robot_option_node.
# Mirrors the get_status payload of wmx_arm_server (wmx-server-development).
std_msgs/Header header

bool device               # robot is configured and the engine is communicating
bool servo                # all robot joint servos are on
bool in_motion            # robot is currently executing a motion
bool is_pass_motion       # robot is in a blended/pass motion

string motion_state       # textual motion state (idle, cp_in_motion, ptp_in_motion, ...)
string motion_error       # textual motion error (none, collision_detected, ...)
int32 motion_error_axis   # offending joint (1-based) or -1 when not applicable

float64[] cartesian_pose_fb   # tool pose feedback [x, y, z, qx, qy, qz, qw] (m, quaternion)
float64[] joint_angles_fb     # joint feedback [j1..jN] (rad for revolute, m for prismatic)
float64[] cartesian_pose_cmd  # tool pose command  [x, y, z, qx, qy, qz, qw] (m, quaternion)
float64[] joint_angles_cmd    # joint command  [j1..jN] (rad for revolute, m for prismatic)

bool light                # auxiliary light state (placeholder, mirrors wmx_arm_server)
float64 gripper           # gripper state (placeholder, mirrors wmx_arm_server)
