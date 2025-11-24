# Examples

Examples are provided for running ROMAN online with a D455 or Zed2i, or for running ROMAN on a bag.

The online examples are set up so that the user:
1. Creates an initial map in a first session
2. Ends the first session and sets up for a second run (perhaps starting from a new location)
3. Creates a second map and uses ROMAN to relocalize in the previous map

After running the two sessions, a video is created showing the best set of associations found by ROMAN.
Example output can be visualized in [this YouTube video](https://www.youtube.com/watch?v=y51NDoPpBy8&t=5s).

The following sections give further instructions for setting up and running the examples.

## `roman_ros2` with D455 RealSense stereo camera and Kimera-VIO

#### Demo setup

Some additional installation steps must be taken to set up the D455 and Kimera-VIO. 
As prerequisites, follow the [RealSense instructions](https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file#installation-on-ubuntu) for installing the RealSense SDK and RealSense ROS2 Wrapper.
Additionally, install `docker`, and the following packages:

```
sudo apt install tmux tmuxp ros-${ROS_DISTRO}-rmw-zenoh-cpp zsh
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

An example tmux file is included for running ROMAN mapping using D455 RGB-D images. This example allows you to first create an initial map (and find loop closures while performing initial mapping). If you have a second bag in the same area, you can perform loop closures to the prior map as well.

To run the session, run [this file](./d455_bag/run_single_session.sh):

`./run_single_session.sh <bag path> <output directory (optional)>`

Rviz will pop up for visualizing the mapping. Visualizations of the loop closures will he saved to the output directory. Once the bag has finished, press enter in the bottom tmux pane and that will save the ROMAN map into the output directory and close the tmux.

To run a second session with loop closures to a prior map, run [this file](./d455_bag/run_with_prior_map.sh):

`./run_with_prior_map.sh <bag path> <prior map path> <output directory (optional)>`.