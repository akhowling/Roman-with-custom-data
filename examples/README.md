# Examples

## `roman_ros2` with D455 RealSense stereo camera and Kimera-VIO

#### Demo setup

Some additional installation steps must be taken to set up the D455 and Kimera-VIO. 
As prerequisites, follow the [RealSense instructions](https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file#installation-on-ubuntu) for installing the RealSense SDK and RealSense ROS2 Wrapper.
Additionally, install `docker`, and the following packages:

```
sudo apt install tmux tmuxp ros-${ROS_DISTRO}-rmw-zenoh-cpp
```

Next, clone and install the `kimera-vio-ros2-realsense` repo:

```
git clone https://github.com/mbpeterson70/kimera-vio-ros2-realsense.git src/kimera-vio-ros2-realsense
./src/kimera-vio-ros2-realsense/build.bash
```

This will create a docker image for running Kimera-VIO in ROS1, meanwhile bridging camera data from your local machine in ROS2 to the docker in ROS1. 

#### Running the demo

The goal of this demo is to give ROMAN to video sequences and let ROMAN align maps created from the two videos to perform global localization across the two sessions.

After following the installation instructions above, start the demo with

```
./src/roman_ros2/examples/d455_two_session/run.bash
```

You will be prompted to enter a directory to store the output of the demo.
You will then be prompted to press enter when you are ready to start the first video.
Record a video  (duration isn't super important, but ROMAN performs better with more complete maps) and press Enter when finished.
Move the camera to the start of the second session and press enter to beging recording the second session.
Press enter again to finish the second session.
Finally, a video will be generated and opened automatically showing the object matches found by ROMAN. 

Optionally, the `run.bash` script can be run with command line arguments:

```
./src/roman_ros2/examples/d455_two_session/run.bash <output directory name> <sessions to run>
```

where `<sessions to run>` can be `1`, `2`, `12`, or `none`. This defaults to `12` to create maps for sessions 1 and 2. 
This functionality enables re-running if a single sessions needs to be re-run but you would like to use one of the previously recorded sessions.

## `roman_ros2` on bagged data

An example tmux file is included for running ROMAN mapping using D455 RGB-D images.

To run this example, first install `tmuxp` (`sudo apt install tmuxp`) and set the following environment variables:

```
export ROBOT=<robot name>
export CAMERA=<camera name>
export BAG=<path to bag file>
export ROMAN_WS=<path to ROS2 workspace where roman_ros2 is installed>
export ACTIVATE_ROMAN_ENV=<command to activate python environment where roman is installed>
```

Inside of this directory, run the following command to launch ROMAN:

```
tmuxp load ./roman_ros2/tmux/example.yaml
```