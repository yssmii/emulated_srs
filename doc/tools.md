# Tools for emulated_srs

## makeMask

* _makeMask_ is a command line tool to generate a mask image, like [this
  example](/doc/MASK_example.png), for specifying the detection zone in
  obstacle_detector_. See
  [the geometry of _emulated_srs_](/doc/IEEESensors2021Fig12.png).
* This tool is obsolete. Use the ROS node, _mask_maker.py_, instead.

### Usage

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
