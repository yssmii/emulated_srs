# Tools for emulated_srs

## mask_maker.py

* GUI tool for specifying a mask region for object detection by user's mouse
  operations.
* It is implemented as a ROS node, which continuously subscribes to distance
  image topics and displays them in a window, and specifies mask regions with
  the mouse operations.
* According to the specified rectangle, a mask image is generated and sent as
  the custom service, _emulated_srs/SetMask.srv_, to the obstacle detection
  node, that is, _obstacle_detector_, _obstacle_classifier_, or
  _obstacle_measurer_.

### Usage

    rosrun emulated_srs mask_maker.py

### Mouse operations

#### Left button press and move

* mask region specification

#### Left button release

* end of mask region specification

### Key bindings

#### _s_

* Sends the specified mask image to the obstacle detection node and
  initiates the masking process

#### _p_

* Pauses the masking process at the obstacle detection node

#### _r_

* Restarts the masking process at the obstacle detection node

## transmittance_monitor.py

* Monitor tool that subscribes the topics, _/transmittance_, published from
  _transmittance_publisher.py_ and displays their spatial transmittances values

### Usage

    rosrun emulated_srs transmittance_monitor.py

## makeMask

* Command line tool to generate a mask image, like [this
  example](/doc/MASK_example.png), for specifying the detection zone in
  _obstacle_detector_. See
  [the geometry of _emulated_srs_](/doc/IEEESensors2021Fig12.png).
* This tool is obsolete. Use the ROS node, _mask_maker.py_, instead.

### Example

    cd ~/catkin_make/devel/lib/emulated_srs
    ./makeMask -n MASK_example.png 270 200 100 140
    ./makeMask -h
      Usage: testMask [options] x y width height
       x y:       left shoulder point
       width:     width
       height:    height
       -g wxh:    width and height of baseimage (640x480 default)
       -n fname:  mask image filename ("MASK.png" default)
       -h:        print this
