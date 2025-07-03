# ROMAN ROS2

<img src="./media/opposite_view_loop_closure.jpg" alt="Opposite view loop closure" width="500"/>

Welcome to roman_ros2, a ROS2 wrapper for [ROMAN](https://github.com/mit-acl/ROMAN) (<ins>R</ins>obust <ins>O</ins>bject <ins>M</ins>ap <ins>A</ins>lignment A<ins>n</ins>ywhere).
ROMAN is a view-invariant global localization method that maps open-set objects and uses the geometry, shape, and semantics of objects to find the transformation between a current pose and previously created object map.
This enables loop closure between robots even when a scene is observed from *opposite views.*

Demo videos, the paper, and more can be found at the [ROMAN project website](https://acl.mit.edu/roman/). 
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

# Examples

A few example demos can be run following the [examples instructions](./examples/README.md).
The provided examples include running ROMAN on a pre-recorded bag, using ROMAN for single robot loop closures, and the main `roman_ros2` demo that creates ROMAN maps across two camera sessions and then aligns the maps from the camera sessions without any initial pose information.

---

This research is supported by Ford Motor Company, DSTA, ONR, and
ARL DCIST under Cooperative Agreement Number W911NF-17-2-0181.
