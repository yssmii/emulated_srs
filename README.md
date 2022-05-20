# emulated_srs: A ROS package for obstacle detection

## Description

* A ROS package for obstacle detection emulating the architecture of
  Safety-Related Sensors (SRS) according to [IEC TS 62998-1][1].
* It aims to provide a software platform for evaluating the object detection
  performance of various distance-imaging sensor devices.
* It subscribes to [PointCloud2 (PC2)][2], executes object detection based on
  [Connected-component labeling (CCL)][3] and publishes detection results.
  Optionally, it performs obstacle classification by [YOLOv3][4] or [YOLOv4][5].
* For more details on the object detection algorithm, please see the appendix
  of [the journal article][6].

|![example of object detection results](/doc/IEEESensors2021Fig10.png)|
|:--:|
| [Example of detection results][6] |

[1]: https://webstore.iec.ch/publication/31009
[2]: http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/PointCloud2.html
[3]: https://en.wikipedia.org/wiki/Connected-component_labeling
[4]: https://github.com/AlexeyAB/darknet
[5]: https://pjreddie.com/darknet/install/
[6]: https://doi.org/10.1109/JSEN.2021.3089207

## Requirements

* Ubuntu 16.04, 18.04, 20.04
* ROS Kinetic, Melodic, Noetic
* Sensor package that publishes organized PC2 or depth images that can be
  converted PC2 with the [_depth_image_proc_][7] nodelet.
  e.g., [_freenect_launch_](http://wiki.ros.org/freenect_launch),
  [_realsense-ros_](https://github.com/IntelRealSense/realsense-ros), etc.
* (optional) darknet for [YOLOv3][4] or [YOLOv4][5]

[7]: http://wiki.ros.org/depth_image_proc#depth_image_proc.2Fpoint_cloud_xyzrgb

## ROS Nodes

### obstacle_detector

* A ROS node that performs object detection.
* It subscribes to [organized (image-like) PC2][2] topics and executes obstacle
  detection, and publishes custom topics containing a list of detected obstacles.
* The detected obstacles are labeled as "unknown".

### obstacle_classifier

* A ROS node that performs object detection and classification.
* It subscribes to organized (image-like) PC2 topics and executes obstacle 
  detection and classification, and publishes custom topics containing a list of
  detected and classified obstacles.
* The detected obstacles are labeled according to recognition results by
  YOLO, such as "person", or as "unknown".
* If the CMake option _-DWITH_YOLO=OFF_ or _DWITH_YOLOV4=OFF_ is specified, the
  classification function is not implemented. It is just the same as
  _obstacle_detector_.

### obstacle_measurer

* A ROS node that performs object detection, classification and distance
  measurement. It is still under development.

## Installation

1. (optional) Build YOLO.

   For [YOLOv4][5]:

        git clone https://github.com/AlexeyAB/darknet darknetAB
        cd darknetAB
        export PATH_DARKNET=$(pwd)
        export LD_LIBRARY_PATH=$(pwd):$LD_LIBRARY_PATH
        <edit ./Makefile>           # GPU=1, OPENCV=1, LIBSO=1
        make
        wget https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v3_optimal/yolov4.weights
        <edit ./cfg/yolov4.cfg>     # batch=1, subdivisions=1
        <edit ./cfg/coco.data>      # names = "your PATH_DARKNET"/data/coco.names
        ./darknet detect cfg/yolov4.cfg yolov4.weights data/dog.jpg   # test

    For [YOLOv3][4]:

        git clone https://github.com/pjreddie/darknet.git
        cd darknet
        export PATH_DARKNET=$(pwd)
        <edit ./Makefile>           # GPU=1, OPENCV=1
        make
        wget https://pjreddie.com/media/files/yolov3.weights
        <edit ./cfg/yolov3.cfg>     # batch=1, subdivisions=1
        <edit ./cfg/coco.data>      # names = "your PATH_DARKNET"/data/coco.names
        ./darknet detect cfg/yolov3.cfg yolov3.weights data/dog.jpg   # test

2. Build emulated_srs.

        cd ~/catkin_ws/src
        git clone http://github.com/yssmii/emulated_srs.git
        catkin_init_workspace
        cd ..
        catkin_make                 # -DWITH_YOLO=OFF or -DWITH_YOLOV4=OFF

## Usage

     rosrun emulated_srs obstacle_(detector|classifier|measurer)

## ROS args

* _zkey_: The limit distance of detection. Pixels that are farther away from
    the sensor than this distance are excluded from the obstacle detection.
    (default: 3000.0 [mm])
* _min_gap_of_occluding_boundary_: The minimum distance of occluding contours.
    If the 3D distance between two pixels adjacent on a distance image is
    greater than this value, they are regarded as being contained in separate
    objects. (default: 50.0 [mm])
* _min_pixels_as_object_: Minimum obstacle size on a distance image. If smaller
    than this, it is regarded as noise.（default: 500 [pixels])
* _min_overlap_rate_: The overlap rate on an image between an obstacle detected
    in a distance image and an object detected/classified in a corresponding RGB
    image by YOLO. If greater than this value, the obstacle is tagged with the
    YOLO object class. If smaller, it is tagged with "unknown". (default: 0.8)
* _use_mask_p_: If non-zero, the FOV is limited to the mask image. (default: 0)
* _display_images_p_: If non-zero, detection results are visualized and
    displayed with OpenCV. (default: 1)
* _publish_images_p_: If non-zero, detection results are visualized and
    published. (default: 1)
* _publish_markers_p_: If non-zero, detection results are published as
    visualization markers for rviz. (default: 0)
* _save_images_p_: if non-zero, subscribed PC2s are saved as depth and RGB
    image files, and detection results are saved as overwritten depth images.
    (default: 0)
* _filename_mask_: name of the mask file. (default: "MASK.png")
* _dirname_log_: name of the parent directory where the images will be saved,
    if save_images_p is non-zero. Within the directory, the child directories
    D/, I/, and R/ must be made. (default: "Data/")

## Subscribed ROS topics

### /camera/depth_registered/points ([sensor_msgs/PointCloud2.msg])

* Organized (image-like) point cloud

## Published ROS topics

### obstacle ([emulated_srs/Obstacle.msg])

* Information of the detected obstacle
* Example:

        $ rostopic echo -n 1 /emulated_srs/obstacle
        header:                      # stamp and frame_id copied from the PC2
          seq: 0
          stamp:
            secs: 1626834032
            nsecs:  49269199
          frame_id: "camera_color_optical_frame"
        n: 0                         # seq num of the obstacle in the frame
        position_3D:                 # at front, upper, left of the 3D bounding box
          x: -0.060076452791690826
          y: -0.02606579288840294
          z: 1.2140001058578491
        dimensions_3D:               # of the 3D bounding box
          x: 0.23947076499462128
          y: 0.6017187237739563
          z: 0.28299999237060547
        centroid_3D:                 # of the point cloud, NOT of the bounding box
          x: 0.060652490705251694
          y: 0.197583869099617
          z: 1.2704710960388184
        position_2D:                 # of the 2D bounding box
          x: 301.0
          y: 228.0
        dimensions_2D:               # of the 2D bounding box
          x: 115.0
          y: 251.0
        centroid_2D:                 # of the 2D obstacle region
          x: 60.65249252319336
          y: 197.5838623046875
        n_points: 19902              # num of the points
        n_points_within: 0           # num of them within the correct region
        filename_saved: "20210721_112032_49269199.png"
        type_class: "unknown"
        confidence_class: 0.0
        ---

### depth/image_raw ([sensor_msgs/Image])

* Depth images converted from PointCloud2, where colored obstacle regions are
  overwritten according to the results of obstacle detection and optional
  classification by YOLO

### depth_labeled/image_raw ([sensor_msgs/Image])

* Depth images converted from PointCloud2, where colored obstacle regions and
  text labels are overwritten according to the results of the obstacle
  detection and 
  optional classification by YOLO

### color/image_raw ([sensor_msgs/Image])

* RGB images converted from PointCloud2

### visualization_marker ([visualization_msgs/Maker])

* [Marker](http://wiki.ros.org/rviz/DisplayTypes/Marker) for rviz

### setup_experiment ([emulated_srs/ExpSetup])

* Latched topic for broadcasting the experimental setup

## ROS Services

### set_mask ([emulated_srs/SetMask])

* Set the mask image to specify the detection zone for obstacle detection. See
  [the geometry of _emulated_srs_](/doc/IEEESensors2021Fig12.png).

* emulated_srs/SetMask.srv

      string cmd               # command string: "start", "pause", or "restart"
      sensor_msgs/Image mask   # mask image, ignored on "pause" or "restart"
      ---
      string res               # response string from obstacle detector nodes

## ROS launches

* [obstacle_classifier.launch](/doc/obstacle_classifier_launch.md)
* [sensing_unit.launch](/doc/sensing_unit_launch.md)
* [processing_unit.launch](/doc/processing_unit_launch.md)

## License

* [MIT](https://opensource.org/licenses/mit-license.php)

