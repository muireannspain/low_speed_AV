This is a path following algorithm with visualization under ROS archetecture.

The Conversion_Node.py recieves data from the ROS topic /localization(in localization.bag), do coordinate conversion and calculation.
pathfollower_jl_ROS3.jl recieves data from Conversion_Node.py, use MPC to calculate the optimal input. 
PlotMap_MPCandLocalization.py recieves data from Conversion_Node.py and pathfollower_jl_ROS3.jl, visualize the algorithm.


Building path_follower
Environment: Ubuntu
Make sure you have ROS melodic or kinetic and Julia 0.4v installed
1. Type 'roslaunch path_follower LaunchFile.launch'
2. Type 'rosbag play localization.bag'
