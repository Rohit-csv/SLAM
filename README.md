
RUN THESE COMMANDS

Go to checkpoint 1 folder then then in slam2
Run the following commands-

This will extract data from topics

ros2 run hello node1 & ros2 run hello node2 & ros2 run hello v

To visulaize the motion model marker(green colour)-
ros2 run hello node

To visulize the ground truth actual node (red colour)-
ros2 run hello ground_truth
 
Then visualize in rviz2
Also run the rosbag
ros2 bag play rosbag_motion_update/
