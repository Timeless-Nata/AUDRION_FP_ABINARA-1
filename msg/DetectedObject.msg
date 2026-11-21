# One colour blob found in the camera image, localised on the table plane.
string color                       # key from camera_config.yaml colors (e.g. yellow, green, white)
geometry_msgs/Point position       # world frame [m]; blob centre ray intersected with z = object_center_z
float32 pixel_u                    # blob centroid, image pixels (continuous coords, origin top-left)
float32 pixel_v
float32 area_px
float32 bbox_w_px
float32 bbox_h_px
# Cross-check with the HC-SR04: only evaluated when the sensor axis passes within range_gate_m of the object.
bool range_checked
bool range_ok                      # |measured - expected| <= range_tol_m (false if not checked)
float32 range_expected             # distance to the object surface along the sensor axis [m]; NaN if not checked
