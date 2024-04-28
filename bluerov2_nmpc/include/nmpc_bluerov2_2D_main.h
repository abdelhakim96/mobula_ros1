#ifndef _NMPC_PC_MAIN_H
#define _NMPC_PC_MAIN_H

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h>
#include "tf2/LinearMath/Matrix3x3.h>

#include "std_msgs/msg/Bool.h>
#include "std_msgs/msg/Float64.h>
#include "std_msgs/msg/Float64MultiArray.h>
#include "geometry_msgs/msg/Wrench.h>
#include "geometry_msgs/msg/PoseStamped.h"
#include "geometry_msgs/msg/TwistStamped.h"
#include <geometry_msgs/msg/Vector3Stamped.h>
#include "nav_msgs/msg/Odometry.h> 
#include "mavros_msgs/msg/State.h"
#include "mavros_msgs/msg/Thrust.h>

#include "nmpc_bluerov2_2D.h>


// Subscribers
rclcpp::Subscriber state_sub;
rclcpp::Subscriber ref_position_sub;
rclcpp::Subscriber ref_velocity_sub;
rclcpp::Subscriber ref_yaw_sub;
rclcpp::Subscriber ref_point_sub;
rclcpp::Subscriber pos_sub;
rclcpp::Subscriber vel_sub;
rclcpp::Subscriber dist_Fx_predInit_sub;
rclcpp::Subscriber dist_Fy_predInit_sub;
rclcpp::Subscriber dist_Fz_predInit_sub;
rclcpp::Subscriber dist_Fx_data_sub;
rclcpp::Subscriber dist_Fy_data_sub;
rclcpp::Subscriber dist_Fz_data_sub;
rclcpp::Subscriber orientation_sub;


// Publishers
rclcpp::Publisher att_throttle_pub;
rclcpp::Publisher attitude_pub;
rclcpp::Publisher nmpc_cmd_wrench_pub;
rclcpp::Publisher nmpc_cmd_Fz_pub;
rclcpp::Publisher nmpc_cmd_exeTime_pub;
rclcpp::Publisher nmpc_cmd_kkt_pub;
rclcpp::Publisher nmpc_cmd_obj_pub;
rclcpp::Publisher nmpc_ctrl_pub;
rclcpp::Publisher s_sdot_pub;
rclcpp::Publisher nmpc_pred_traj_pub;


// State variables
nmpc_struct_ nmpc_struct;
online_data_struct_ online_data;

std::string mocap_topic_part, dist_Fx_predInit_topic, dist_Fy_predInit_topic, dist_Fz_predInit_topic,
    dist_Fx_data_topic, dist_Fy_data_topic, dist_Fz_data_topic;
bool online_ref_yaw;
bool control_stop;
bool use_dist_estimates;

double m_in, g_in;
Eigen::VectorXd Uref_in(NMPC_NU);
Eigen::VectorXd W_in(NMPC_NY);

int print_flag_offboard = 1, print_flag_arm = 1, print_flag_altctl = 1, print_flag_traj_finished = 0;

Eigen::Vector3d ref_position, ref_velocity;
double ref_yaw_rad;
int ref_traj_type;
std::vector<double> ref_trajectory;
std::vector<double> angles;
std::vector<double> angles_d;

double t, t_cc_loop;

tf2::Quaternion current_att_quat;
tf2::Matrix3x3 current_att_mat;
std::vector<double> pos_ref;
std::vector<double> current_pos_att;
std::vector<double> current_vel_rate;
std::vector<double> current_vel_body;
std::vector<double> current_states;
std::vector<double> current_s_sdot;

struct _dist_struct
{
    bool predInit;
    int print_predInit = 1;
    std::vector<double> data;
    std::vector<double> data_zeros;
} dist_Fx, dist_Fy, dist_Fz;

#endif
