# Radar Track Msg
uint8           track_ID
float32         track_dist_long
float32         track_dist_lat
float32         track_vrel_long
float32         track_vrel_lat
uint8           track_dyn_prop
float32         track_RCS 
float32         track_arel_long
uint8           track_class
float32         track_arel_lat
float32         track_orientation_angle
float32         track_length
float32         track_width
uint8           track_dist_long_rms
uint8           track_vrel_long_rms
uint8           track_dist_lat_rms
uint8           track_vrel_lat_rms
uint8           track_arel_lat_rms
uint8           track_arel_long_rms
uint8           track_orientation_rms
uint8           track_meas_state
uint8           track_prob_ofexist

uint8           nof_objects
uint16          meas_counter

# unique for cluster 
uint8           cluster_nof_near
uint8           cluster_nof_far

uint8           cluster_phd0
uint8           cluster_abvig_state
uint8           cluster_ivalid_state



