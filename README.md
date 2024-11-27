# assignment1_rt
This assignment involves creating two ROS nodes to control and monitor two turtles in the turtlesim environment. The nodes work together to manage turtle movement and ensure they operate within safe constraints.
## UI Node
This node allows the user to control the movement of two turtles in the Turtlesim environment.
- Spawn a second turtle named turtle2 at an arbitrary location
```c++
ros::ServiceClient spawn_client = nh.serviceClient<turtlesim::Spawn>("/spawn");
turtlesim::Spawn srv;
srv.request.x = 1.0;
srv.request.y = 5.0;
srv.request.theta = 0.0;
srv.request.name = "turtle2";
spawn_client.waitForExistence();
spawn_client.call(srv);
```
- Let the user choose which turtle to control (turtle1 or turtle2) and quit if the input is not "1" or "2"
```c++
std::cout << "Select the turtle to control (1 for turtle1, 2 for turtle2): ";
std::cin >> turtle_choice;
if (turtle_choice != 1 && turtle_choice != 2){
   ROS_WARN("Invalid turtle choice. Quitting...");
   break;
}
```
- Accept user inputs for linear and angular velocities to command the selected turtle's movement, including also a check to discard the input if it's not valid
```c++
std::cout << "Enter linear velocity: ";
std::cin >> linear_velocity;
if (std::cin.fail()) {  // Check if the input failed
   std::cin.clear();   // Clear the error flag
   std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Discard invalid input
   ROS_WARN("Invalid input. Please enter a numeric value for linear velocity.");
   continue; // Go to next iteration
}
```
- Create two publishers for sending velocity commands to turtle1 and turtle2
```c++
ros::Publisher pub_turtle1 = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
ros::Publisher pub_turtle2 = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 1);
```
```c++
// Create the velocity command
geometry_msgs::Twist velocity_command;
velocity_command.linear.x = linear_velocity;
velocity_command.angular.z = angular_velocity;
// Publish the command to the selected turtle
if (turtle_choice == 1) {
   pub_turtle1.publish(velocity_command);
   ROS_INFO("Sent velocity to turtle1: linear=%f, angular=%f", linear_velocity, angular_velocity);
} // ... same for turtle2
```
- Automatically stop the turtle after 1 second and wait for further commands.
```c++
ros::Duration(1.0).sleep();
// Stop the turtle by sending zero velocity
velocity_command.linear.x = 0.0;
velocity_command.angular.z = 0.0;
if (turtle_choice == 1) {
    pub_turtle1.publish(velocity_command);
} else if (turtle_choice == 2) {
    pub_turtle2.publish(velocity_command);
}
```
- Moreover there is a Publisher "pub_moving_turtle" that publishes the id of the moving turtle, allowing the "distance node" to perform its actions based also on that information.
```c++
    ros::Publisher pub_moving_turtle = nh.advertise<std_msgs::Int32>("moving_turtle_id", 10);
    std_msgs::Int32 moving_turtle_msg;
 // ... after having picked the turtle and selected velocity:
    pub_moving_turtle.publish(moving_turtle_msg);
```

## Distance Node
This node monitors the relative distance between the two turtles (turtle1 and turtle2) in the environment and performs specific actions based on proximity and boundary conditions.
- Store the latest positions of turtle1 and turtle2 as updated by their respective subscribers.
```c++ 
turtlesim::Pose turtle1_pose;
turtlesim::Pose turtle2_pose;
```

```c++
ros::Subscriber sub_turtle1 = nh.subscribe("turtle1/pose", 10, turtle1Callback);
ros::Subscriber sub_turtle2 = nh.subscribe("turtle2/pose", 10, turtle2Callback);
```

- Distance calculation computed using the Euclidean formula
```c++
float dx = turtle2_pose.x - turtle1_pose.x;
float dy = turtle2_pose.y - turtle1_pose.y;
float distance = std::sqrt(dx * dx + dy * dy);
```
- Publish on a topic the distance (std_msgs/Float32)
```c++
std_msgs::Float32 distance_msg;
distance_msg.data = distance;
pub_distance.publish(distance_msg);
ROS_INFO("Distance between turtle1 and turtle2: %.2f", distance);
```
- Boundary check: these conditions determine if either turtle has moved outside the defined boundaries (1.0 <= x, y <= 10.0).
```c++
bool out_of_bounds1 = turtle1_pose.x < BOUNDARY_MIN || turtle1_pose.x > BOUNDARY_MAX ||
                      turtle1_pose.y < BOUNDARY_MIN || turtle1_pose.y > BOUNDARY_MAX;
// ... same for turtle2
```
#### Stopping the Turtles
Moreover the distance node needs:
1) Subscriber to get which turtle is moving:
```c++
    ros::Subscriber sub_moving_turtle = nh.subscribe("moving_turtle_id", 10, movingTurtleCallback);
```
2) and subscribers to the turtles' velocities:
```c++
// Subscribers to turtle1 and turtle2 velocities
    ros::Subscriber sub_turtle1_vel = nh.subscribe("turtle1/cmd_vel", 10, turtle1VelocityCallback);
    ros::Subscriber sub_turtle2_vel = nh.subscribe("turtle2/cmd_vel", 10, turtle2VelocityCallback);
```
The moving turtle is stopped if:
- the distance between the turtles is below the threshold (0.5)
- its position is out of the allowable boundary.
``` c++ 
if (moving_turtle == 1 && (out_of_bounds1 || distance < DISTANCE_THRESHOLD)) 
```

If one of the conditions is met:
1) First the velocity of the moving turtle is reversed:
```c++  
      reverse_cmd.linear.x = -turtle1_velocity.linear.x; // Reverse linear velocity
      reverse_cmd.angular.z = 0.0;                      // Stop angular rotation
      pub_turtle1_stop.publish(reverse_cmd);
      ROS_WARN("Turtle1 out of bounds! Reversing velocity.");
      ros::Duration(1.0).sleep();
```
2) and then the stopping command is sent:
```c++
     reverse_cmd.linear.x = 0.0;
     pub_turtle1_stop.publish(reverse_cmd);
     ROS_WARN("Turtles are too close or out of bounds! Stopped turtle1.");
```

## Usage
1) Build the package and generate executables, by running in your ROS wokspace:
```bash
catkin_make
```

2) Start the ROS Master
```
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