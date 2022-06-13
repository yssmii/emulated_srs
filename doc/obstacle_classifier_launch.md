# obstacle_classifier.launch

## Usage

1. Run the sensor package to publish the organized PC2 topics;

        roslaunch realsense2_camera rs_rgbd.launch       # if realsense-ros

2. Run obstacle_classifier.

        roslaunch emulated_srs obstacle_classifier.launch

## ROS args

### _zkey_

* Limit distance of detection. Pixels that are farther away from the sensor than
  this distance are excluded from the obstacle detection.
* default: 1500.0 [mm]

### _min_gap_of_occluding_boundary_

* Minimum distance of occluding contours.
* If the 3D distance between two pixels adjacent on a distance image is
  greater than this value, they are regarded as being contained in separate
  objects.
* default: 100.0 [mm]

### _min_pixels_as_object_

* Minimum obstacle size on a distance image.
* If smaller than this, it is regarded as noise.
* default: 500 [pixels]

### _min_overlap_rate_

* Overlap rate on an image between an obstacle detected in a distance image and
  an object detected/classified in a corresponding RGB image by YOLO.
* If greater than this value, the obstacle is tagged with the YOLO object class.
* If smaller, it is tagged with "unknown".
* default: 0.8

### _use_mask_p_

* If non-zero, the FOV is limited to the mask image.
* default: 0

### _display_images_p_

* If non-zero, detection results are visualized and displayed with OpenCV.
* default: 1

### _publish_images_p_

* If non-zero, detection results are visualized and published.
* default: 0

### _publish_markers_p_

* If non-zero, detection results are published as visualization markers for rviz.
* default: 0

### _save_images_p_

* If non-zero, subscribed PC2s are saved as depth and RGB image files, and
  detection results are saved as overwritten depth images.
* default: 0

### _filename_mask_

* Name of the mask file.
* defaule: "MASK.png"

### _dirname_log_

* Name of the parent directory where the images will be saved, if _save_images_p_
  is non-zero.
* Within the directory, the child directories  D/, I/, and R/ must be made.
* default: "Data/"

### _topic_name_

* topic name of PC2 to subscribe to.
* default: "/camera/depth_registered/points"

### _sensor_name_

* Name of the sensor publishing the PC2
* default: "unknown"

### _experimental_doublecheck_p_

* If non-zero, exec the experimental classification.
* default: 0
