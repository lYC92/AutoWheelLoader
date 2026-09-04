std_msgs/Header header

string material_type
float64 domain_min_m
float64 cell_size_m
float64 slice_width_m
float64[] height_profile_m
float64 initial_volume_m3
float64 remaining_volume_m3  # Current terrain, including material deposited by unloading.
float64 excavated_volume_m3
float64 dumped_volume_m3  # Cumulative bucket-to-terrain transfer; not a separate reservoir.

# (remaining + bucket - initial) / initial.
float64 relative_volume_conservation_error
float64 relative_mass_conservation_error
uint64 update_sequence
