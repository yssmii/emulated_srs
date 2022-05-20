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

|![geometry](/doc/IEEESensors2021Fig12.png)|
|:--:|
| [Geometry of emulated_srs][6] | |

[1]: https://webstore.iec.ch/publication/31009
[2]: http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/PointCloud2.html
[3]: https://en.wikipedia.org/wiki/Connected-component_labeling
[4]: https://github.com/AlexeyAB/darknet
[5]: https://pjreddie.com/darknet/install/
[6]: https://doi.org/10.1109/JSEN.2021.3089207

## Requirements

* Ubuntu 16.04, 18.04, 20.04
* ROS Kinetic, Melodic, Noetic
* Sensor package that publishes organized PC2 messages or ,
  e.g., [freenect_launch](http://wiki.ros.org/freenect_launch),
  [realsense-ros](https://github.com/IntelRealSense/realsense-ros), etc.
* (optional) darknet for [YOLOv3][4] or [YOLOv4][5]

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

* [obstacle_classifier.launch](/doc/usage_obstacle_classifier_launch.md)
* sensing_unit.launch and processing_unit.launch

## License

* [MIT](https://opensource.org/licenses/mit-license.php)

