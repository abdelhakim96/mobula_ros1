#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Wrench.h>
#include <deque>
 
std::vector<double> angles = {0.0, 0.0, 0.0};
std::vector<double> angles_d = {0.0, 0.0, 0.0};
std::vector<double> acceleration = {0.0, 0.0, 0.0};
std::vector<double> velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
std::vector<double> position = {0.0, 0.0, 0.0};
std::vector<double> control = {0.0, 0.0, 0.0, 0.0};
 
double Fx_dist;
double Fy_dist;
double Fz_dist;
 
// BlueROV2 Model Parameters
const double f_s = 1.0;    //scaling factor;
const double m = 11.4;    // BlueROV2 mass (kg)  
const double g = 9.82;  // gravitational field strength (m/s^2)
const double F_bouy = 114.8; // Buoyancy force (N)
 
const double X_ud = -2.6 ; // Added mass in x direction (kg)
const double Y_vd = -18.5 ; // Added mass in y direction (kg)
const double Z_wd = -13.3 ; // Added mass in z direction (kg)
const double N_rd = -0.28 ; // Added mass for rotation about z direction (kg)
 
const double I_xx = 0.21 ; // Moment of inertia (kg.m^2)
const double I_yy = 0.245 ; // Moment of inertia (kg.m^2)
const double I_zz = 0.245 ; // Moment of inertia (kg.m^2)
 
const double X_u = -0.09 ; // Linear damping coefficient in x direction (N.s/m)
const double Y_v  = -0.26 ; // Linear damping coefficient  in y direction (N.s/m)
const double Z_w = -0.19; // Linear damping coefficient  in z direction (N.s/m)
const double N_r = -4.64 ;  // Linear damping coefficient for rotation about z direction (N.s/rad)
 
const double X_uc = -34.96 ; // quadratic damping coefficient in x direction (N.s^2/m^2)
const double Y_vc = -103.25 ; // quadratic damping coefficient  in y direction (N.s^2/m^2)
const double Z_wc = -74.23 ; // quadratic damping coefficient  in z direction (N.s^2/m^2)
const double N_rc = - 0.43 ; // quadratic damping coefficient for rotation about z direction (N.s^2/rad^2)
 
double wind_x = 0.0;
int count = 0;
 
// Define the size of the buffer for the moving mean filter
const size_t buffer_size = 5;
 
// Define a deque to store the last few acceleration measurements
std::deque<std::vector<double>> acceleration_buffer;
 
class FeatureGeneratorNode {
public:
  FeatureGeneratorNode() {
    // Initialize the ROS node
    ros::NodeHandle nh;
 
    // Subscribe to the odometry topic
    odom_sub = nh.subscribe<nav_msgs::Odometry>("/mobula/rov/odometry", 1, &FeatureGeneratorNode::odomCallback, this);
 
    acc_sub = nh.subscribe<geometry_msgs::Vector3Stamped>("/mobula/rov/acceleration", 1, &FeatureGeneratorNode::accCallback, this);
 
    nmpc_cmd_wrench_sub = nh.subscribe<geometry_msgs::Wrench>("/mobula/rov/wrench", 1, &FeatureGeneratorNode::nmpcCmdWrenchCallback, this);
 
    wind_sub = nh.subscribe<geometry_msgs::Wrench>("/mobula/rov/disturbance", 1, &FeatureGeneratorNode::windCallback, this);
 
    // Advertise the gp_features topic
    gp_x_features_pub = nh.advertise<std_msgs::Float64MultiArray>("/dev/gp/feature_x_filtered", 1);
    gp_y_features_pub = nh.advertise<std_msgs::Float64MultiArray>("/dev/gp/feature_y_filtered", 1);
    gp_z_features_pub = nh.advertise<std_msgs::Float64MultiArray>("/dev/gp/feature_z_filtered", 1);
 
    // Initialize last_trajectory_data with zeros
    last_trajectory_data.resize(3, 0.0);
  }
 
  // Callback function for odometry data
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Access the velocity data from the received message
    position = {msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};
    velocity = {msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z,
                msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z};
  }
 
  // Callback function for NMPC wrench command
  void nmpcCmdWrenchCallback(const geometry_msgs::Wrench::ConstPtr& msg) {
    // Process the received Wrench message
    control = {msg->force.x, msg->force.y, msg->force.z, msg->torque.z};
  }
 
  // Callback function for acceleration data
  void accCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
    // Access the trajectory data from the received message
    acceleration = {msg->vector.x, msg->vector.y, msg->vector.z};
 
    // Calculate moving mean of acceleration
    acceleration_buffer.push_back(acceleration);
    if (acceleration_buffer.size() > buffer_size) {
        acceleration_buffer.pop_front();
    }
 
    std::vector<double> mean_acceleration(3, 0.0);
    for (const auto& acc : acceleration_buffer) {
        for (size_t i = 0; i < 3; ++i) {
            mean_acceleration[i] += acc[i];
        }
    }
    for (auto& acc_value : mean_acceleration) {
        acc_value /= acceleration_buffer.size();
    }
 
    // Update acceleration with the moving mean
    acceleration = mean_acceleration;
  }
 
  // Callback function for wind disturbance
  void windCallback(const geometry_msgs::Wrench::ConstPtr& msg) {
    wind_x = msg->force.x;
    count++;
  }
 
public:
  ros::Subscriber odom_sub;
  ros::Subscriber acc_sub;
  ros::Subscriber nmpc_cmd_wrench_sub;
  ros::Subscriber wind_sub;
  ros::Publisher gp_x_features_pub;
  ros::Publisher gp_y_features_pub;
  ros::Publisher gp_z_features_pub;
  std::vector<double> last_trajectory_data;
};
 
int main(int argc, char **argv) {
  // Initialize the ROS node
  ros::init(argc, argv, "feature_generator_node");
 
  // Create an instance of the feature generator node
  FeatureGeneratorNode featureGeneratorNode;
 
  // Calculate gp_features using odometry and acceleration data
  std_msgs::Float64MultiArray gp_x_features_msg;
  std_msgs::Float64MultiArray gp_y_features_msg;
  std_msgs::Float64MultiArray gp_z_features_msg;
 
  ros::Rate rate(1 / 0.01);
 
  while(ros::ok()) {
    // Model calculations
    // Calculate Fx_dist, Fy_dist, Fz_dist using the filtered acceleration data
    //std::cout << "yaw rate: " << velocity[5]<<std::endl;
    Fx_dist = (m - X_ud) * acceleration[0] - (control[0] +
     (m * velocity[1] -Y_vd * velocity[1]) * velocity[5] +
      (X_u + X_uc * sqrt(velocity[0] * velocity[0])) * velocity[0]);
 
    Fy_dist = (m - Y_vd) * acceleration[1] -
    (control[1] -
     (m * velocity[0] - X_ud * velocity[0]) * velocity[5] +
     (Y_v + Y_vc * sqrt(velocity[1] * velocity[1])) * velocity[1]);
 
    Fz_dist = (m - Z_wd) * acceleration[2] -
             (control[2] + (control[3] + Z_wc * sqrt(velocity[2] * velocity[2])) *
              velocity[2] + (m * g - F_bouy));
 
    // Clear previous data
    gp_x_features_msg.data.clear();
    gp_y_features_msg.data.clear();
    gp_z_features_msg.data.clear();
 
    // Populate message data
    gp_x_features_msg.data.push_back(Fx_dist);
    gp_x_features_msg.data.push_back(Fx_dist);
    //gp_x_features_msg.data.push_back(position[0]);
    gp_x_features_msg.data.push_back(velocity[0]);
    gp_x_features_msg.data.push_back(control[0]);
 
    gp_y_features_msg.data.push_back(Fy_dist);
    gp_y_features_msg.data.push_back(Fy_dist);
    //gp_y_features_msg.data.push_back(position[1]);
    gp_y_features_msg.data.push_back(velocity[1]);
    gp_y_features_msg.data.push_back(control[1]);
 
    gp_z_features_msg.data.push_back(Fz_dist);
    gp_z_features_msg.data.push_back(Fz_dist);
    //gp_z_features_msg.data.push_back(position[2]);
    gp_z_features_msg.data.push_back(velocity[2]);
    gp_z_features_msg.data.push_back(control[2]);
 
    // Publish the gp_features
    featureGeneratorNode.gp_x_features_pub.publish(gp_x_features_msg);
    featureGeneratorNode.gp_y_features_pub.publish(gp_y_features_msg);
    featureGeneratorNode.gp_z_features_pub.publish(gp_z_features_msg);
 
    // Spin to handle incoming messages
    ros::spinOnce();
    rate.sleep();
  }
 
  return 0;
}