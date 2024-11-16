# assignment1_rt
The package "assignment1_rt" consists in 2 nodes that implement the following:
- UI (node1):
    ✓ Spawn a new turtle in the environment: turtle2
    ✓ Implement a simple textual interface to retrieve the user command. The user should be able to select the robot they want to control (turtle1 or turtle2), and the velocity of the robot.
    ✓ The command should be sent for 1 second, and then the robot should stop, and the user should be able again to insert the command. 

- Distance (node2):
    ✓ A node that checks the relative distance between turtle1 and turtle2 and:
        - publish on a topic the distance (you can use a std_msgs/Float32 for
        that)
        - stops the moving turtle if the two turtles are “too close” (you may
        set a threshold to monitor that)
        - stops the moving turtle if the position is too close to the boundaries
        (.e.g, x or y > 10.0, x or y < 1.0)


## How to run it:
1) Build the package and generate executables, by running in your ROS wokspace:
```bash
catkin_make
```

2) Start the ROS Master
'''bash
roscore
```

3) Launch the Turtlesim Node
```bash
rosrun turtlesim turtlesim_node
```

4) Launch the ui_node
```bash
rosrun assignment1_rt ui_node 
```

5) Launch the distance_node
```bash
rosrun assignment1_rt distance_node
```