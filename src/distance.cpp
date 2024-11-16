#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include <cmath>

// Global variables to store turtle poses
turtlesim::Pose turtle1_pose;
turtlesim::Pose turtle2_pose;

const float DISTANCE_THRESHOLD = 0.5;
const float BOUNDARY_MIN = 1.0;
const float BOUNDARY_MAX = 10.0;

int moving_turtle = 0; // 0: No turtle is moving, 1: turtle1 is moving, 2: turtle2 is moving.

// Callback for turtle1's pose
void turtle1Callback(const turtlesim::Pose::ConstPtr &msg)
{
    turtle1_pose = *msg;
}

// Callback for turtle2's pose
void turtle2Callback(const turtlesim::Pose::ConstPtr &msg)
{
    turtle2_pose = *msg;
}
void movingTurtleCallback(const std_msgs::Int32::ConstPtr &msg)
{
    moving_turtle = msg->data; // Update the ID of the moving turtle
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "distance_node");
    ros::NodeHandle nh;

    // Subscribers to turtle1 and turtle2 poses
    ros::Subscriber sub_turtle1 = nh.subscribe("turtle1/pose", 10, turtle1Callback);
    ros::Subscriber sub_turtle2 = nh.subscribe("turtle2/pose", 10, turtle2Callback);

    // Publisher for the distance
    ros::Publisher pub_distance = nh.advertise<std_msgs::Float32>("turtles_distance", 10);

    // Publishers for stopping turtles
    ros::Publisher pub_turtle1_stop = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
    ros::Publisher pub_turtle2_stop = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 1);

    ros::Subscriber sub_moving_turtle = nh.subscribe("moving_turtle_id", 10, movingTurtleCallback);

    ros::Rate rate(10);

    while (ros::ok())
    {
        ros::spinOnce();

        float dx = turtle2_pose.x - turtle1_pose.x;
        float dy = turtle2_pose.y - turtle1_pose.y;
        float distance = std::sqrt(dx * dx + dy * dy);

        // Publish the distance
        std_msgs::Float32 distance_msg;
        distance_msg.data = distance;
        pub_distance.publish(distance_msg);

        ROS_INFO("Distance between turtle1 and turtle2: %.2f", distance);

        bool out_of_bounds1 = turtle1_pose.x < BOUNDARY_MIN || turtle1_pose.x > BOUNDARY_MAX ||
                              turtle1_pose.y < BOUNDARY_MIN || turtle1_pose.y > BOUNDARY_MAX;

        bool out_of_bounds2 = turtle2_pose.x < BOUNDARY_MIN || turtle2_pose.x > BOUNDARY_MAX ||
                              turtle2_pose.y < BOUNDARY_MIN || turtle2_pose.y > BOUNDARY_MAX;

        // Check if the turtles are "too close"
        if (distance < DISTANCE_THRESHOLD || out_of_bounds1 || out_of_bounds2)
        {

            geometry_msgs::Twist stop_cmd;
            stop_cmd.linear.x = 0.0;
            stop_cmd.angular.z = 0.0;
            // Stop only the moving turtle
            if (moving_turtle == 1)
            {
                pub_turtle1_stop.publish(stop_cmd);
                ROS_WARN("Turtles are too close or out of bounds! Stopped turtle1.");
            }
            else if (moving_turtle == 2)
            {
                pub_turtle2_stop.publish(stop_cmd);
                ROS_WARN("Turtles are too close or out of bounds! Stopped turtle2.");
            }
        }

        rate.sleep();
    }

    return 0;
}
