# emulated_srs: A ROS package for obstacle detection

## Description

* A ROS package for obstacle detection emulating the architecture of
  Safety-Related Sensors (SRS) according to [IEC/TS
  62998-1](https://webstore.iec.ch/publication/31009).
* It aims to provide a software platform for evaluating a variety of
  distance-imaging sensor devices with a common mature object
  detection algorithm.
* It subscribes to [organized PointCloud2
  (PC2)](https://answers.ros.org/question/234455/pointcloud2-and-pointfield/)
  messages, detects obstacles by image processing based on [the CCL
  algorithm](https://en.wikipedia.org/wiki/Connected-component_labeling)
  and publishes the detection results. Optionally, it performs obstacle
  classification by Deep Learning.

## Requirements

* Ubuntu 18.04
* ROS Melodic
* Sensor package that publishes organized PC2 messages,
  e.g., [freenect_launch](http://wiki.ros.org/freenect_launch),
  [realsense-ros](https://github.com/IntelRealSense/realsense-ros), etc.
* [darknet and YOLOv3](https://pjreddie.com/darknet/) (optional)
  - [CUDA 10.2](https://developer.nvidia.com/cuda-toolkit-archive)
  - [CUDNN 7.6](https://developer.nvidia.com/rdp/cudnn-archive)

## ROS Nodes

### obstacle_classifier

* A ROS node that performs object detection and classification.
* It subscribes to organized PC2 messages and executes the obstacle detection
  and classification, and publishes custom messages containing a list of the
  detected obstacles.
* If the CMake option "-DWITH_YOLO=OFF" is specified, the
  classification function is not implemented. All detected obstacles
  are classified as "unknown".

### obstacle_detector

* A ROS node that performs object detection.
* Equivalent to obstacle_classifier implemented without the classification
  function. All detected obstacles are classified as "unknown".

## Installation

1. Install sensor packages.
2. Build [YOLOv3](https://pjreddie.com/darknet/install/).（optional）

        cd ~/src                    # if build in $HOME/src/darknet
        git clone https://github.com/pjreddie/darknet.git
        cd darknet
        export PATH_DARKNET=$(pwd)
        <edit ./Makefile>           # GPU=1, CUDNN=1, OPENCV=1, OPENMP=1
        make
        wget https://pjreddie.com/media/files/yolov3.weights
        <edit ./cfg/yolov3.cfg>     # batch=1, subdivisions=1
        ./darknet detect cfg/yolov3.cfg yolov3.weights data/dog.jpg   # test

3. Build emulated_srs.

        cd ~/catkin_ws/src
        git clone http://github.com/yssmii/emulated_srs.git
        catkin_init_workspace
        cd ..
        catkin_make                 # add -DWITH_YOLO=OFF, without YOLOv3

4. Update the mask image, if necessary.

        cd ~/catkin_make/devel/lib/emulated_srs
        rm MASK.png
        ./makeMask 320 0 320 480    # mask for the right half of a VGA image

## Usage

1. Run the sensor package to publish the organized PC2 messages;

        roslaunch freenect_launch freenect.launch        # for freenect_launch
        
        roslaunch realsense2_camera rs_rgbd.launch       # for realsense-ros

2. Run obstacle_classifier.

        roslaunch emulated_srs obstacle_classifier.launch

* ROS launch parameters
  - _zkey_: The limit distance of detection. Pixels that are farther away from
    the sensor than this distance are excluded from the obstacle detection. See
    figure 1. (default: 1500.0 [mm])
  - _min_gap_of_occluding_boundary_: The minimum distance of occluding contours.
    If the 3D distance between two pixels adjacent on a distance image is
    greater than this value, they are regarded as being contained in separate
    objects. (default: 100.0 [mm])
  - _min_pixels_as_object_: Minimum obstacle size on a distance image. If
    smaller than this, it is regarded as noise.（default: 500 [pixels])
  - _min_overlap_rate_: The overlap rate on an image between an obstacle
    detected in a distance image and an object detected/classified in a
    corresponding RGB image by YOLO. If greater than this value, the obstacle
    is tagged with the YOLO object class. If smaller, it is tagged with
    "unknown". (default: 0.8)
  - _use_mask_p_: If non-zero, the FOV is limited to the mask image. See
    figure 1. (default: 0)
  - _display_images_p_: If non-zero, detection results are visualized and
    displayed with OpenCV. (default: 1)
  - _publish_images_p_: If non-zero, detection results are visualized and
    published. (default: 0)
  - _publish_markers_p_: If non-zero, detection results are published as
    visualization markers for rviz. (default: 0)
  - _save_images_p_: if non-zero, subscribed PC2s are saved as depth and RGB
     image files, and detection results are saved as overwritten depth images.
     (default: 0)
  - _filename_mask_: name of the mask file. (defaule: "MASK.png")
  - _dirname_log_: name of the parent directory where the images will be saved,
    if save_images_p is non-zero. Within the directory, the child directories
    D/, I/, and R/ must be made. (default: "Data/")
  - _topic_name_: topic name of PC2 to subscribe to. (default: "/camera/depth_registered/points")
  - _sensor_name_: name of the sensor publishing the PC2 (default: "unknown")
  - _experimental_doublecheck_p_: If non-zero, exec the experimental
    classfication. (default: 0)

## Subscribed Topics

### /camera/depth_registered/points ([sensor_msgs/PointCloud2])

* Orginized PC2 messages, which have a width and height of at least two.

## Published Topics

### /emulated_srs/obstacle ([emulated_srs/Obstacle])

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

### /emulated_srs/image_depth_classified ([sensor_msgs/Image])

* Depth images overwritten with the results of the obstacle detection and
  the optional object classification by YOLO

### /emulated_srs/image_yolo ([sensor_msgs/Image])

* RGB images overwritten with the results of the object detection and the
  optional object classification by YOLO

### /emulated_srs/visualization_marker ([visualization_msgs/Maker])

* [Marker](http://wiki.ros.org/rviz/DisplayTypes/Marker) for rviz

### /emulated_srs/setup_experiment ([emulated_srs/ExpSetup])

* Latched topic for broadcasting the experimental setup

## License

* [MIT](https://opensource.org/licenses/mit-license.php)
