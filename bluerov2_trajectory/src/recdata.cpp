#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Wrench.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <string>
#include <chrono>
#include <std_msgs/Bool.h>

std::string file_prefix = "/home/hakim/catkin_ws/src/mobula_ros1/bluerov2_trajectory/test_recordings/recorded_data_brov_";
std::string file_extension = ".txt";
std::ofstream print_results;

geometry_msgs::Vector3 ref_position;
geometry_msgs::Vector3 ref_velocity;
std_msgs::Float64 ref_yaw;
nav_msgs::Odometry rov_odometry;
std_msgs::Float64 W1_opt_y;
std_msgs::Float64 W2_opt_y;
std_msgs::Float64 W3_opt_y;
std_msgs::Float64MultiArray mu_y;
std_msgs::Float64MultiArray mu_x;
geometry_msgs::Wrench disturbance;

bool is_mu_y_received = false;
bool traj_on = false;  // Flag to indicate if recording should start

void ref_position_cb(const geometry_msgs::Vector3::ConstPtr& msg) {
    ref_position = *msg;
}

void ref_velocity_cb(const geometry_msgs::Vector3::ConstPtr& msg) {
    ref_velocity = *msg;
}

void disturbance_cb(const geometry_msgs::Wrench::ConstPtr& msg) {
    disturbance = *msg;
}

void ref_yaw_cb(const std_msgs::Float64::ConstPtr& msg) {
    ref_yaw = *msg;
}

void rov_odometry_cb(const nav_msgs::Odometry::ConstPtr& msg) {
    rov_odometry = *msg;
}

void W1_cb(const std_msgs::Float64::ConstPtr& msg) {
    W1_opt_y = *msg;
}

void W2_cb(const std_msgs::Float64::ConstPtr& msg) {
    W2_opt_y = *msg;
}

void W3_cb(const std_msgs::Float64::ConstPtr& msg) {
    W3_opt_y = *msg;
}

void mu_y_cb(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    mu_y = *msg;
    is_mu_y_received = !mu_y.data.empty();
}

void mu_x_cb(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    mu_x = *msg;
}

void traj_on_cb(const std_msgs::Bool::ConstPtr& msg) {
    traj_on = msg->data;
}

void write_data_to_file(const ros::TimerEvent& event) {
    if (!traj_on || !is_mu_y_received || mu_y.data.empty())
        return;

    if (!print_results.is_open()) {
        print_results.open(file_prefix + std::to_string(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count()) + file_extension);
        if (!print_results.is_open()) {
            ROS_ERROR("Failed to open the file for writing.");
            return;
        }
    }

    double position_error_x = rov_odometry.pose.pose.position.x - ref_position.x;
    double position_error_y = rov_odometry.pose.pose.position.y - ref_position.y;
    double position_error_z = rov_odometry.pose.pose.position.z - ref_position.z;

    double velocity_error_x = rov_odometry.twist.twist.linear.x - ref_velocity.x;
    double velocity_error_y = rov_odometry.twist.twist.linear.y - ref_velocity.y;
    double velocity_error_z = rov_odometry.twist.twist.linear.z - ref_velocity.z;

    print_results << ref_position.x << "," << ref_position.y << "," << ref_position.z << ",";
    print_results << rov_odometry.pose.pose.position.x << "," << rov_odometry.pose.pose.position.y << "," << rov_odometry.pose.pose.position.z << ",";
    print_results << position_error_x << "," << position_error_y << "," << position_error_z << ",";

    print_results << ref_velocity.x << "," << ref_velocity.y << "," << ref_velocity.z << ",";
    print_results << rov_odometry.twist.twist.linear.x << "," << rov_odometry.twist.twist.linear.y << "," << rov_odometry.twist.twist.linear.z << ",";
    print_results << velocity_error_x << "," << velocity_error_y << "," << velocity_error_z << ",";

    print_results << ref_yaw.data << ",";
    print_results << rov_odometry.pose.pose.orientation.z << ",";

    print_results << W1_opt_y.data << "," << W2_opt_y.data << "," << W3_opt_y.data << ",";

    print_results << disturbance.force.x << "," << disturbance.force.y << "," << disturbance.force.z << ",";

    for (size_t i = 0; i < mu_y.data.size(); ++i) {
        print_results << mu_y.data[i] << ",";
    }

    for (size_t i = 0; i < mu_x.data.size(); ++i) {
        print_results << mu_x.data[i] << ",";
    }

    print_results << std::endl;
}

void stop_recording(const ros::TimerEvent& event) {
    if (print_results.is_open()) {
        print_results.close();
    }
    ros::shutdown();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "data_recorder");
    ros::NodeHandle nh;

    ros::Subscriber ref_pos_sub = nh.subscribe<geometry_msgs::Vector3>("ref_trajectory/position", 1, ref_position_cb);
    ros::Subscriber ref_vel_sub = nh.subscribe<geometry_msgs::Vector3>("ref_trajectory/velocity", 1, ref_velocity_cb);
    ros::Subscriber ref_yaw_sub = nh.subscribe<std_msgs::Float64>("ref_trajectory/yaw", 1, ref_yaw_cb);
    ros::Subscriber rov_odometry_sub = nh.subscribe<nav_msgs::Odometry>("/mobula/rov/odometry", 1, rov_odometry_cb);
    ros::Subscriber W1_sub = nh.subscribe<std_msgs::Float64>("/W1_opt_y", 1, W1_cb);
    ros::Subscriber W2_sub = nh.subscribe<std_msgs::Float64>("/W2_opt_y", 1, W2_cb);
    ros::Subscriber W3_sub = nh.subscribe<std_msgs::Float64>("/W3_opt_y", 1, W3_cb);
    ros::Subscriber mu_y_sub = nh.subscribe<std_msgs::Float64MultiArray>("/gp_disturb_reg/mu/y", 1, mu_y_cb);
    ros::Subscriber mu_x_sub = nh.subscribe<std_msgs::Float64MultiArray>("/gp_disturb_reg/mu/x", 1, mu_x_cb);
    ros::Subscriber disturbance_sub = nh.subscribe<geometry_msgs::Wrench>("/mobula/rov/disturbance", 1, disturbance_cb);
    ros::Subscriber traj_on_sub = nh.subscribe<std_msgs::Bool>("/trajectory_on", 1, traj_on_cb);

    ros::Timer data_timer = nh.createTimer(ros::Duration(0.1), write_data_to_file);
    ros::Timer stop_timer = nh.createTimer(ros::Duration(150.0), stop_recording);

    ros::spin();

    return 0;
}
