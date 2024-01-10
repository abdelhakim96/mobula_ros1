#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <string>
#include <cstdio> // Include for file deletion

std::string file_name = "recorded_data.txt";
std::ofstream print_results;

geometry_msgs::Vector3 ref_position;
geometry_msgs::Vector3 ref_velocity;
std_msgs::Float64 ref_yaw;
nav_msgs::Odometry rov_odometry;

void ref_position_cb(const geometry_msgs::Vector3::ConstPtr& msg) {
    ref_position = *msg;
}

void ref_velocity_cb(const geometry_msgs::Vector3::ConstPtr& msg) {
    ref_velocity = *msg;
}

void ref_yaw_cb(const std_msgs::Float64::ConstPtr& msg) {
    ref_yaw = *msg;
}

void rov_odometry_cb(const nav_msgs::Odometry::ConstPtr& msg) {
    rov_odometry = *msg;
}

void write_data_to_file(const ros::TimerEvent& event) {
    print_results.open(file_name.c_str(), std::ios_base::app);
    print_results << ref_position.x << "," << ref_position.y << "," << ref_position.z << "," << ref_yaw.data << ",";
    print_results << rov_odometry.pose.pose.position.x << "," << rov_odometry.pose.pose.position.y << "," << rov_odometry.pose.pose.position.z << ",";
    // Assuming yaw is stored in the quaternion
    print_results << rov_odometry.pose.pose.orientation.z << std::endl;  // Extract yaw from the quaternion if available
    print_results.close();
}

void delete_existing_file() {
    if (std::remove(file_name.c_str()) != 0) {
        ROS_INFO("Error deleting the file.");
    } else {
        ROS_INFO("File deleted successfully.");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "data_recorder");
    ros::NodeHandle nh;

    delete_existing_file(); // Delete existing file before starting data recording

    ros::Subscriber ref_pos_sub = nh.subscribe<geometry_msgs::Vector3>("ref_trajectory/position", 1, ref_position_cb);
    ros::Subscriber ref_vel_sub = nh.subscribe<geometry_msgs::Vector3>("ref_trajectory/velocity", 1, ref_velocity_cb);
    ros::Subscriber ref_yaw_sub = nh.subscribe<std_msgs::Float64>("ref_trajectory/yaw", 1, ref_yaw_cb);
    ros::Subscriber rov_odometry_sub = nh.subscribe<nav_msgs::Odometry>("/mobula/rov/odometry", 1, rov_odometry_cb);

    // Set the timer to periodically write data to file (adjust the rate as needed)
    ros::Timer timer = nh.createTimer(ros::Duration(0.1), write_data_to_file);  // Change duration to desired rate

    ros::spin();

    return 0;
}
