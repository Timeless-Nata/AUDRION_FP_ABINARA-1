# One colour blob detected by the perception node.
string color                       # orange | green | white | yellow
float32 u                          # image coordinates of the (top-face) centroid [px]
float32 v
float32 area_px                    # blob area [px]
geometry_msgs/Point position       # object position on the ground-plane assumption (plane z = plane_z of the array)
geometry_msgs/Point ray_origin     # camera centre in the world frame
geometry_msgs/Vector3 ray_direction  # unit ray through the centroid, world frame
