std_msgs/Header header                     # copied from the PC2

uint32 n                                   # seq num of the obstacle in the PC2

geometry_msgs/Point32 position_3D          # at front, upper, left of the 3D bounding box
geometry_msgs/Point32 dimensions_3D        # of the 3D bounding box
geometry_msgs/Point32 centroid_3D          # of the points, NOT of the bounding box      
emulated_srs/ImageCoord32 position_2D      # of the 2D bounding box
emulated_srs/ImageCoord32 dimensions_2D    # of the 2D bounding box
emulated_srs/ImageCoord32 centroid_2D      # of the points

uint32 n_points                            # num of the points

string filename_saved

string type_class                          # by YOLO, or "UNKNOWN"
float32 confidence_class                   # by YOLO

uint32 n_points_within
