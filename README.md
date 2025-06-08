# ROMAN ROS2

<img src="./media/opposite_view_loop_closure.jpg" alt="Opposite view loop closure" width="500"/>

Welcome to roman_ros2, a ROS2 wrapper for [ROMAN](https://acl.mit.edu/ROMAN/) (<ins>R</ins>obust <ins>O</ins>bject <ins>M</ins>ap <ins>A</ins>lignment A<ins>n</ins>ywhere).
ROMAN is a view-invariant global localization method that maps open-set objects and uses the geometry, shape, and semantics of objects to find the transformation between a current pose and previously created object map.
This enables loop closure between robots even when a scene is observed from *opposite views.*

Demo videos, the paper, and more can be found at the [ROMAN project website](https://acl.mit.edu/ROMAN-project/). 
Checkout the main branch for the ROS1 wrapper.

## Citation

If you find ROMAN useful in your work, please cite our [paper](https://www.roboticsproceedings.org/rss21/p029.pdf):

M.B. Peterson, Y.X. Jia, Y. Tian, A. Thomas, and J.P. How, "ROMAN: Open-Set Object Map Alignment for Robust View-Invariant Global Localization,"
*Robotics: Science and Systems*, 2025.

```
@article{peterson2025roman,
  title={ROMAN: Open-Set Object Map Alignment for Robust View-Invariant Global Localization},
  author={Peterson, Mason B and Jia, Yi Xuan and Tian, Yulun and Thomas, Annika and How, Jonathan P},
  booktitle={Robotics: Science and Systems (RSS)},
  pdf={https://www.roboticsproceedings.org/rss21/p029.pdf},
  year={2025}
}
```

# Install

In the root directory of your ROS workspace run:

```
git clone -b ros2 git@github.com:mit-acl/roman_ros.git src/roman_ros2
vcs import src < src/roman_ros2/install/packages.yaml
colcon build
```

Then, activate the python environment you would like to use with ROMAN and run:

```
./src/roman_ros2/install/install_roman.bash
```

to install the ROMAN python package and download required model weights.

For running `roman_ros2`, you will need to set the `ROMAN_WEIGHTS` environment variable. 
You may want to run the following to add this environment variable to your `.zshrc` file:

```
echo export ROMAN_WEIGHTS=$(realpath ./src/roman_ros2/weights) >> ~/.zshrc
```

or `.bashrc` file:

```
echo export ROMAN_WEIGHTS=$(realpath ./src/roman_ros2/weights) >> ~/.bashrc
```

Finally, the examples will source your python environment by calling `$ROMAN_ENV_ACTIVATE`. 
Before running the examples run (or put the following in your `~/.zshrc` or `~/.bashrc`):

```
export ROMAN_ENV_ACTIVATE=<environment activation command>
```

# Example use with D455 RealSense stereo camera and Kimera-VIO

Some additional installation steps must be taken to set up the D455 and Kimera-VIO. 
As prerequisites, follow the [RealSense instructions](https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file#installation-on-ubuntu) for installing the RealSense SDK and RealSense ROS2 Wrapper.
Additionally, install `docker`. 

Next, clone and install the `kimera-vio-ros2-realsense` repo:

```
git clone https://github.com/mbpeterson70/kimera-vio-ros2-realsense.git src/kimera-vio-ros2-realsense
./src/kimera-vio-ros2-realsense/build.bash
```

# Example use with bagged data

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

---

This research is supported by Ford Motor Company, DSTA, ONR, and
ARL DCIST under Cooperative Agreement Number W911NF-17-2-0181.