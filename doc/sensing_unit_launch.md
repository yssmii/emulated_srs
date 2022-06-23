# sensing_unit.launch

* _sensing_unit.launch_ initiates one of the supported sensor nodes and
    publishes the topics, including PointCloud2, for processing_unit.launch.
* Optionally, it records the topics into ROSBAG files.
* This emulates the sensing unit of
    [the SRS architecture](SRSArchitecture.png) according to Fig.1 in IEC
    TS 62998-1.

## Usage

    roslaunch emulated_srs sensing_unit.launch [sensor_name:=<sensor_name>]

## ROS args

### _sensor_name_

* LABEL corresponding to the sensor to be initiated, one of the following:
    D435|D455|L515|LIPSDL|Xtion2|Structure|Kinect|Kinect2|Azure|Astra|Vzense|ZED|ZED2|ZED2i

### _exp_distance_

* Distance from the sensor optical window to the test piece.
* If a negative value, an error message is printed for attention
* default: -1000.0 [mm]

### _record_p_

* If non-zero, the following topcs are saved into ROSBAG files, prefixed with 
  "<dirname_data>/<sensor_name>" (default: 0)

### _dirname_data_

* Name of the directory where the ROSBAG files will be saved

### _fps_

* FPS for publishing PointCloud2 topics
* default: 4

## Supported sensor nodes

| LABEL     | ROS package                                                       |
| --------- | ----------------------------------------------------------------- |
| D435      | https://github.com/IntelRealSense/realsense-ros                   |
| D455      | https://github.com/IntelRealSense/realsense-ros                   |
| L515      | https://github.com/IntelRealSense/realsense-ros                   |
| LIPSDL    | https://github.com/lips-hci/openni2_camera                        |
| Xtion2    | https://github.com/ros-drivers/openni2_camera ([SDK][1] required) |
| Structure | https://github.com/ros-drivers/openni2_camera                     |
| Kinect    | http://wiki.ros.org/freenect_launch                               |
| Kinect2   | https://github.com/code-iai/iai_kinect2                           |
| Azure     | https://github.com/microsoft/Azure_Kinect_ROS_Driver              |
| Astra     | https://github.com/orbbec/ros_astra_camera                        |
| Vzense    | https://github.com/Vzense/Vzense_ROS_Plugin_Linux                 |
| ZED       | https://github.com/stereolabs/zed-ros-wrapper                     |
| ZED2      | https://github.com/stereolabs/zed-ros-wrapper                     |
| ZED2i     | https://github.com/stereolabs/zed-ros-wrapper                     |

[1]: https://dlcdnets.asus.com/pub/ASUS/Multimedia/Xtion_2/ASUS-Linux-x64-OpenNI2.2.tar.gz