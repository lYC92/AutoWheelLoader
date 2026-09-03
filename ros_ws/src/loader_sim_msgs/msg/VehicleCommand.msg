# All values use SI units. Commands outside documented ranges must be rejected
# or saturated by loader_command_controller, never by the planner implicitly.
std_msgs/Header header

int8 GEAR_REVERSE=-1
int8 GEAR_NEUTRAL=0
int8 GEAR_FORWARD=1
int8 gear

# Signed total traction torque request at the transmission output [N*m].
float64 traction_torque_nm

# Normalized service-brake request [0, 1].
float64 brake_command

# Desired articulation yaw angle [rad], positive left.
float64 target_articulation_angle_rad

# Normalized hydraulic valve requests [-1, 1].
float64 lift_valve_command
float64 tilt_valve_command

bool emergency_stop
