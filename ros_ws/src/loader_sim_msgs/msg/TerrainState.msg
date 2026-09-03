std_msgs/Header header

string material_type
float64 initial_volume_m3
float64 remaining_volume_m3
float64 excavated_volume_m3
float64 dumped_volume_m3

# (remaining + bucket + dumped - initial) / initial.
float64 relative_volume_conservation_error
float64 relative_mass_conservation_error
uint64 update_sequence
