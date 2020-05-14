# low_speed_AV
This is a ROS package including a path following algorithm(MPC) and its visualization. It comes from 2019-2020 MEng Capstone Project: Heavy Load Low Speed AV.

## Requirements
- Ubuntu
- ROS melodic or kinetic
- Julia 0.4v
- python

## How to run
1. Clone this repo.
```
git clone https://github.com/MPC-Berkeley/low_speed_AV.git
```
2. Create work space.
```
catkin_make
```
3. Run the package.
```
roslaunch path_follower LaunchFile.launch
```
4. Run localization bag.
```
rosbag play localization.bag
```

## Contact
- Muireann Spain: muireann@berkeley.edu
- Wei Hu: wei-hu@berkeley.edu
