## MPC path follower
This is a path following algorithm with visualization under ROS archetecture.

The Conversion_Node.py recieves data from the ROS topic /localization(in localization.bag), do coordinate conversion and calculation.
pathfollower_jl_ROS3.jl recieves data from Conversion_Node.py, use MPC to calculate the optimal input. 
PlotMap_MPCandLocalization.py recieves data from Conversion_Node.py and pathfollower_jl_ROS3.jl, visualize the algorithm.


# Requirements
Ubuntu
ROS melodic or kinetic
Julia 0.4v
python

# How to use
1. Clone this repo.
`git clone https://github.com/MPC-Berkeley/low_speed_AV.git`
2. Create work space.
`catkin_make`
3. Run the package.
`roslaunch path_follower LaunchFile.launch`
4. Run localization bag.
`rosbag play localization.bag`



# Contact
Muireann Spain
Wei Hu              azurehw6227@gmail.com
