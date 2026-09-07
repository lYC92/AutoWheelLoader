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

# Version 1 uses the legacy extruded profile above. Version 2 uses the XY grid
# below (row-major: index = row * columns + column), cell_size_m on both axes.
# Grid corner origin is (domain_min_m, origin_y_m), height is above world z=0.
uint32 schema_version
float64 origin_y_m
uint32 rows
uint32 columns
float64[] height_grid_m
