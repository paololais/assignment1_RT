#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"
#include <string>
#include <iostream>
#include <thread>
#include <chrono>

int main(int argc, char** argv) {
    ros::init(argc, argv, "ui_node");
    ros::NodeHandle nh;

    // Spawn turtle2
    ros::ServiceClient spawn_client =  nh.serviceClient<turtlesim::Spawn>("/spawn");
    turtlesim::Spawn srv;
    if (spawn_client.call(srv)) {
        ROS_INFO("Spawned turtle2 successfully.");
    } else {
        ROS_WARN("Failed to spawn turtle2.");
    }

    // Publishers for controlling turtles
    ros::Publisher pub_turtle1 = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
    ros::Publisher pub_turtle2 = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

    while (ros::ok()) {
        int turtle_choice;
        float linear_velocity, angular_velocity;
        
        // Get user input
        std::cout << "Select the turtle to control (1 for turtle1, 2 for turtle2): ";
        std::cin >> turtle_choice;
        if (turtle_choice != 1 && turtle_choice != 2){
            ROS_WARN("Invalid turtle choice. Quitting...");
            break;
        }
        std::cout << "Enter linear velocity: ";
        std::cin >> linear_velocity;
        if (std::cin.fail()) {  // Check if the input failed
            std::cin.clear();   // Clear the error flag
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Discard invalid input
            ROS_WARN("Invalid input. Please enter a numeric value for linear velocity.");
            continue;
        }
        
        std::cout << "Enter angular velocity: ";
        std::cin >> angular_velocity;
        if (std::cin.fail()) {  // Check if the input failed
            std::cin.clear();   // Clear the error flag
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Discard invalid input
            ROS_WARN("Invalid input. Please enter a numeric value for angular velocity.");
            continue;
        }

        // Create the velocity command
        geometry_msgs::Twist velocity_command;
        velocity_command.linear.x = linear_velocity;
        velocity_command.angular.z = angular_velocity;

        // Publish the command to the selected turtle
        if (turtle_choice == 1) {
            pub_turtle1.publish(velocity_command);
            ROS_INFO("Sent velocity to turtle1: linear=%f, angular=%f", linear_velocity, angular_velocity);
        } else if (turtle_choice == 2) {
            pub_turtle2.publish(velocity_command);
            ROS_INFO("Sent velocity to turtle2: linear=%f, angular=%f", linear_velocity, angular_velocity);
        }

        // Let the turtle move for 1 second
        ros::Duration(1.0).sleep();

        // Stop the turtle by sending zero velocity
        velocity_command.linear.x = 0.0;
        velocity_command.angular.z = 0.0;
        if (turtle_choice == 1) {
            pub_turtle1.publish(velocity_command);
        } else if (turtle_choice == 2) {
            pub_turtle2.publish(velocity_command);
        }

        // Give the user another chance to input commands
        ROS_INFO("Robot stopped. Ready for the next command.");
    }

    return 0;
}
