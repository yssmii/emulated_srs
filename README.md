# emulated_srs: A ROS package for obstacle detection

## Description

* A ROS package emulating the architecture of Safety-Related Sensors
  (SRS) according to [IEC/TS
  62998-1](https://webstore.iec.ch/publication/31009). It aims to
  provide a software platform for evaluating a variety of
  distance-imaging sensor devices with a common mature object
  detection algorithm.
* It subscribes to [organized PointCloud2
  (PC2)](https://answers.ros.org/question/234455/pointcloud2-and-pointfield/)
  messages, detects obstacles by image processing based on [the CCL
  algorithm](https://en.wikipedia.org/wiki/Connected-component_labeling)
  and publishes the detection results. Optionally, it performs obstacle
  classification by Deep Learing.

## Requirements

* Ubuntu 18.04
* ROS Melodic
* Sensor package that publishes organized PC2 messages, e.g., [freenect_launch](http://wiki.ros.org/freenect_launch), [realsense-ros](https://github.com/IntelRealSense/realsense-ros), etc.
* [darknet and YOLOv3](https://pjreddie.com/darknet/) (optional)
  - [CUDA 10.2](https://developer.nvidia.com/cuda-toolkit-archive)
  - [CUDNN 7.6](https://developer.nvidia.com/rdp/cudnn-archive)

## ROS Nodes

### obstacle_classifier

* A ROS node that performs object detection and classification.
* It subscribes to organized PC2 messages and executes the obstacle detection
  and classification, and publishes custom messages containing a list of the
  detected obstacles.
* If the CMake option "-DWITH_YOLO=OFF" is specified at the installation, the
  classification function is not implemented. All detected obstacles are
  classified as "unknown".

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
        catkin_make                 # add -DWITH_YOLO=OFF, if not installing YOLOv3

4. Update the mask image, if necessary.

        cd ~/catkin_make/devel/lib/emulated_srs
        rm MASK.png
        ./makeMask 320 0 320 480     # if mask the right half of VGA images

## Usage

1. Run the sensor package to publish the organized PC2 messages. 

        roslaunch freenect_launch freenect.launch     # for freenect_launch
        roslaunch realsense2_camera rs_rgbd.launch    # for realsense-ros

2. Run obstacle_classifier.

        roslaunch emulated_srs obstacle_classifier.launch

* ROS launch parameters
  - _zkey_: The limit distance of detection. Pixels that are farther away from
    the sensor than this distance are excluded from the obstacle detection. See
    figure 1. (default: 1500.0 mm)
  - _min_gap_of_occluding_boundary_: The minimum distance of occluding contours.
    If the 3D distance between two pixels adjacent on a distance image is
    greater than this value, they are regarded as being contained in separate
    objects. (default: 100.0 mm)
  - _min_pixels_as_object_: Minimum obstacle size on a distance image. If
    smaller than this, it is regarded as noise.（default: 500 pixels)
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
  - _experimental_doublecheck_p_: If non-zero, exec the experimental
    classfication. (default: 0)

## Published Topics

### /emulated_srs/obstacles

* List of obstacles detected on a subscribed distance image.
* The 3D coordinates of the obstacle are shown in the right-hand system. The
  sensor position is the origin, the z axis is the upward direction, and the x
  axis is the optical axis of the sensor. The units are in merters.
* An example of published topic messages;

        $ rostopic echo -n 1 /emulated_srs/obstacles
        ---
        obstacles: 
          - 
            n: 0                        # numbering
            stamp:                      # time of the detection
              secs: 1569300407
              nsecs: 309263605
            type: "person"              # object class by YOLO
            point:                      # upper-left-front of
              x: 0.51700001955          #     the 3D bounding box
              y: 0.0526745580137
              z: 0.309713751078
            scale:                      # dims of the bounding box
              x: 0.427000075579
              y: 0.487814188004
              z: 0.640797376633
            confidence: 0.999856948853  # confidence by YOLO
          - 
            n: 1
            stamp: 
              secs: 1569300407
              nsecs: 309263605
            type: "unknown"
            point: 
              x: 1.44900012016
              y: 0.799512386322
              z: 0.294261574745
            scale: 
              x: 0.0489998795092
              y: 0.0868123173714
              z: 0.0640079528093
            confidence: 0.0

### /emulated_srs/image_depth_classified

* Distance images overwritten with the results of the obstacle detection and
  the optional object classification by YOLO

### /emulated_srs/image_yolo

* RGB images overwritten with the results of the object detection and the
  optional object classification by YOLO

### /emulated_srs/visualization_marker

* [Marker](http://wiki.ros.org/rviz/DisplayTypes/Marker) for rviz

## License

* [MIT](https://opensource.org/licenses/mit-license.php)
