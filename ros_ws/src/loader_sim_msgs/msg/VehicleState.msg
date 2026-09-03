# State values use SI units and share the simulation timestamp in header.
std_msgs/Header header

float64 longitudinal_speed_mps

# Order: front_left, front_right, rear_left, rear_right [rad/s].
float64[4] wheel_speed_radps

sensor_msgs/JointState joint_state

float64 lift_cylinder_position_m
float64 tilt_cylinder_position_m
float64 lift_cylinder_pressure_pa
float64 tilt_cylinder_pressure_pa

float64 bucket_payload_mass_kg
geometry_msgs/Point bucket_payload_center_of_mass_m

# Bit mask owned by the simulator safety specification.
uint32 fault_flags
bool emergency_stop_active
