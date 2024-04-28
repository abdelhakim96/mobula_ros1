/**
 * @file   nmpc_bluerov2_2-D_main.cpp
 * @author Mohit Mehindratta / Hakim Amer
 * @date   Jan 2023
 *
 */

#include <nmpc_bluerov2_2D_main.h>

#include <Eigen/Core>
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;
using namespace rclcpp;
double sampleTime = 0.02;

mavros_msgs::msg::State current_state_msg;


void NMPCBlueROV2Node::state_cb(const mavros_msgs::msg::State::ConstSharedPtr& msg)
{
    current_state_msg = *msg;
}
void NMPCBlueROV2Node::ref_position_cb(const geometry_msgs::msg::Vector3::ConstSharedPtr& msg)
{
    ref_position << msg->x, msg->y, msg->z;
}
void NMPCBlueROV2Node::ref_velocity_cb(const geometry_msgs::msg::Vector3::ConstSharedPtr& msg)
{
    ref_velocity << msg->x, msg->y, msg->z;
}
void NMPCBlueROV2Node::ref_yaw_cb(const std_msgs::msg::Float64::ConstSharedPtr& msg)
{
    ref_yaw_rad = msg->data;
}

void NMPCBlueROV2Node::pos_cb(const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
{
    double roll = 0, pitch = 0, yaw = 0;
    current_att_quat = {
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w};
    current_vel_rate = {msg->twist.twist.linear.x,
                        msg->twist.twist.linear.y,
                        msg->twist.twist.linear.z,
                        msg->twist.twist.angular.x,
                        msg->twist.twist.angular.y,
                        msg->twist.twist.angular.z};


    //current_att_mat.setRotation(current_att_quat);
    //current_att_mat.getRPY(roll, pitch, yaw);
    //current_att_mat.getRPY(pitch, roll, yaw);
    current_pos_att = {msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, roll, pitch, yaw};
}



void NMPCBlueROV2Node::vel_cb(const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg)
{
    current_vel_body = {msg->twist.linear.x,
                        msg->twist.linear.y,
                        msg->twist.linear.z,
                        msg->twist.angular.x,
                        msg->twist.angular.y,
                        msg->twist.angular.z};
}


void NMPCBlueROV2Node::orientation_cb(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr& msg)
{   
    
    angles = {msg->vector.x*(M_PI/180),
                   msg->vector.y*(M_PI/180),
                   msg->vector.z*(M_PI/180)};   
    angles_d ={msg->vector.x,
                   msg->vector.y,
                   msg->vector.z}; 
}


// Disturbance estimator Call back functions X, Y,Z

void NMPCBlueROV2Node::dist_Fx_predInit_cb(const std_msgs::msg::Bool::ConstSharedPtr& msg)
{
    dist_Fx.predInit = msg->data;
    if (nmpc_struct.verbose && dist_Fx.predInit && dist_Fx.print_predInit == 1)
    {
        std::cout << "Prediction initialized for Fx estimates! \n";
        dist_Fx.print_predInit = 0;
    }
}
void NMPCBlueROV2Node::dist_Fy_predInit_cb(const std_msgs::msg::Bool::ConstSharedPtr& msg)
{
    dist_Fy.predInit = msg->data;
    if (nmpc_struct.verbose && dist_Fy.predInit && dist_Fy.print_predInit == 1)
    {
        std::cout << "Prediction initialized for Fy estimates! \n";
        dist_Fy.print_predInit = 0;
    }
}
void NMPCBlueROV2Node::dist_Fz_predInit_cb(const std_msgs::msg::Bool::ConstSharedPtr& msg)
{
    dist_Fz.predInit = msg->data;
    if (nmpc_struct.verbose && dist_Fz.predInit && dist_Fz.print_predInit == 1)
    {
        std::cout << "Prediction initialized for Fz estimates! \n";
        dist_Fz.print_predInit = 0;
    }
}

void NMPCBlueROV2Node::dist_Fx_data_cb(const std_msgs::msg::Float64MultiArray::ConstSharedPtr& msg)
{
    if (use_dist_estimates && dist_Fx.predInit)
    {
        dist_Fx.data.clear();
        dist_Fx.data.insert(dist_Fx.data.end(), msg->data.begin(), msg->data.end());
    }
    else
        dist_Fx.data = dist_Fx.data_zeros;
}
void NMPCBlueROV2Node::dist_Fy_data_cb(const std_msgs::msg::Float64MultiArray::ConstSharedPtr& msg)
{
    if (use_dist_estimates && dist_Fy.predInit)
    {
        dist_Fy.data.clear();
        dist_Fy.data.insert(dist_Fy.data.end(), msg->data.begin(), msg->data.end());
    }
    else
        dist_Fy.data = dist_Fy.data_zeros;
}
void NMPCBlueROV2Node::dist_Fz_data_cb(const std_msgs::msg::Float64MultiArray::ConstSharedPtr& msg)
{
    if (use_dist_estimates && dist_Fz.predInit)
    {
        dist_Fz.data.clear();
        dist_Fz.data.insert(dist_Fz.data.end(), msg->data.begin(), msg->data.end());
    }
    else
        dist_Fz.data = dist_Fz.data_zeros;
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NMPCBlueROV2Node>());
  rclcpp::shutdown();
  return 0;
}
__
NMPCBlueROV2Node::NMPCBlueROV2Node() : Node("nmpc_bluerov2_node")
{
    this->declare_parameter("mocap_topic_part", rclcpp::PARAMETER_STRING);
    mocap_topic_part = this->get_parameter("mocap_topic_part");

    this->declare_parameter("dist_Fx_predInit_topic", rclcpp::PARAMETER_);  dist_Fx_predInit_topic = this->get_parameter("dist_Fx_predInit_topic");
    this->declare_parameter("dist_Fy_predInit_topic", rclcpp::PARAMETER_);  dist_Fy_predInit_topic = this->get_parameter("dist_Fy_predInit_topic");
    this->declare_parameter("dist_Fz_predInit_topic", rclcpp::PARAMETER_);  dist_Fz_predInit_topic = this->get_parameter("dist_Fz_predInit_topic");
    this->declare_parameter("dist_Fx_data_topic", rclcpp::PARAMETER_);  dist_Fx_data_topic = this->get_parameter("dist_Fx_data_topic");
    this->declare_parameter("dist_Fy_data_topic", rclcpp::PARAMETER_);  dist_Fy_data_topic = this->get_parameter("dist_Fy_data_topic");
    this->declare_parameter("dist_Fz_data_topic", rclcpp::PARAMETER_);  dist_Fz_data_topic = this->get_parameter("dist_Fz_data_topic");


    // ----------
    // Subscribers
    // ----------

    state_sub = this->create_subscription<mavros_msgs::msg::State>("mavros/state",, 1, state_cb);
    ref_position_sub = this->create_subscription<geometry_msgs::msg::Vector3>("ref_trajectory/position", 1, ref_position_cb);
    ref_velocity_sub = this->create_subscription<geometry_msgs::msg::Vector3>("ref_trajectory/velocity", 1, ref_velocity_cb);
    ref_yaw_sub = this->create_subscription<std_msgs::msg::Float64>("ref_trajectory/yaw", 1, ref_yaw_cb);
    pos_sub = this->create_subscription<nav_msgs::msg::Odometry>("/mobula/rov/odometry", 1, pos_cb);
    dist_Fx_predInit_sub = this->create_subscription<std_msgs::msg::Bool>(dist_Fx_predInit_topic, 1, dist_Fx_predInit_cb);
    dist_Fy_predInit_sub = this->create_subscription<std_msgs::msg::Bool>(dist_Fy_predInit_topic, 1, dist_Fy_predInit_cb);
    dist_Fz_predInit_sub = this->create_subscription<std_msgs::msg::Bool>(dist_Fz_predInit_topic, 1, dist_Fz_predInit_cb);
    dist_Fx_data_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(dist_Fx_data_topic, 1, dist_Fx_data_cb);
    dist_Fy_data_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(dist_Fy_data_topic, 1, dist_Fy_data_cb);
    dist_Fz_data_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(dist_Fz_data_topic, 1, dist_Fz_data_cb);
    orientation_sub = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("/mobula/rov/orientation", 1, orientation_cb);

    // ----------
    // Publishers
    // ----------
    nmpc_cmd_wrench_pub = this->create_publisher<geometry_msgs::msg::Wrench>("/mobula/rov/wrench", 1);
    nmpc_cmd_exeTime_pub = this->create_publisher<std_msgs::msg::Float64>("outer_nmpc_cmd/exeTime", 1);
    nmpc_cmd_kkt_pub = this->create_publisher<std_msgs::msg::Float64>("outer_nmpc_cmd/kkt", 1);
    nmpc_cmd_obj_pub = this->create_publisher<std_msgs::msg::Float64>("outer_nmpc_cmd/obj", 1);

    nmpc_pred_traj_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("nmpc_predicted_trajectory", 1); 
    
    s_sdot_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("outer_nmpc_cmd/s_sdot", 1);

    nmpc_struct.U_ref.resize(NMPC_NU);
    nmpc_struct.W.resize(NMPC_NY);

    // Roslaunch parameters
    this->declare_parameter("verbose", rclcpp::PARAMETER_);  nmpc_struct.verbose = this->get_parameter("verbose");
    this->declare_parameter("yaw_control", rclcpp::PARAMETER_);  nmpc_struct.yaw_control = this->get_parameter("yaw_control");
    this->declare_parameter("online_ref_yaw", rclcpp::PARAMETER_);  online_ref_yaw = this->get_parameter("online_ref_yaw");
    this->declare_parameter("use_dist_estimates", rclcpp::PARAMETER_);  use_dist_estimates = this->get_parameter("use_dist_estimates");


    this->declare_parameter("W_Wn_factor", rclcpp::PARAMETER_);  nmpc_struct.W_Wn_factor = this->get_parameter("W_Wn_factor");
    int u_idx = 0;
    this->declare_parameter("F_x_ref", rclcpp::PARAMETER_);  nmpc_struct.U_ref(u_idx++) = this->get_parameter("F_x_ref");
    this->declare_parameter("F_y_ref", rclcpp::PARAMETER_);  nmpc_struct.U_ref(u_idx++) = this->get_parameter("F_y_ref");
    this->declare_parameter("F_z_ref", rclcpp::PARAMETER_);  nmpc_struct.U_ref(u_idx++) = this->get_parameter("F_z_ref");
    this->declare_parameter("Mz_ref", rclcpp::PARAMETER_);  nmpc_struct.U_ref(u_idx++) = this->get_parameter("Mz_ref");

    assert(u_idx == NMPC_NU);

    int w_idx = 0;
    this->declare_parameter("W_x", rclcpp::PARAMETER_);  nmpc_struct.W(w_idx++) = this->get_parameter("W_x");
    this->declare_parameter("W_y", rclcpp::PARAMETER_);  nmpc_struct.W(w_idx++) = this->get_parameter("W_y");
    this->declare_parameter("W_z", rclcpp::PARAMETER_);  nmpc_struct.W(w_idx++) = this->get_parameter("W_z");
    this->declare_parameter("W_u", rclcpp::PARAMETER_);  nmpc_struct.W(w_idx++) = this->get_parameter("W_u");
    this->declare_parameter("W_v", rclcpp::PARAMETER_);  nmpc_struct.W(w_idx++) = this->get_parameter("W_v");
    this->declare_parameter("W_w", rclcpp::PARAMETER_);  nmpc_struct.W(w_idx++) = this->get_parameter("W_w");
    this->declare_parameter("W_psi", rclcpp::PARAMETER_);  nmpc_struct.W(w_idx++) = this->get_parameter("W_psi");
    this->declare_parameter("W_r", rclcpp::PARAMETER_);  nmpc_struct.W(w_idx++) = this->get_parameter("W_r");
    this->declare_parameter("W_Fx", rclcpp::PARAMETER_);  nmpc_struct.W(w_idx++) = this->get_parameter("W_Fx");
    this->declare_parameter("W_Fy", rclcpp::PARAMETER_);  nmpc_struct.W(w_idx++) = this->get_parameter("W_Fy");
    this->declare_parameter("W_Fz", rclcpp::PARAMETER_);  nmpc_struct.W(w_idx++) = this->get_parameter("W_Fz");
    this->declare_parameter("W_Mz", rclcpp::PARAMETER_);  nmpc_struct.W(w_idx++) = this->get_parameter("W_Mz");
    assert(w_idx == NMPC_NY);

    nmpc_struct.sample_time = sampleTime;

    NMPC_PC* nmpc_pc = new NMPC_PC(nmpc_struct);
    rclcpp::Rate rate(1 / sampleTime);

    current_pos_att.resize(6);
    current_vel_rate.resize(6);
    current_vel_body.resize(6);
    dist_Fx.data.resize(NMPC_N + 1);
    dist_Fy.data.resize(NMPC_N + 1);
    dist_Fz.data.resize(NMPC_N + 1);
    dist_Fx.data_zeros.resize(NMPC_N + 1, 0.0);
    dist_Fy.data_zeros.resize(NMPC_N + 1, 0.0);
    dist_Fz.data_zeros.resize(NMPC_N + 1, 0.0);

    ref_traj_type = 0;
    ref_position << 0, 0, 0;
    ref_velocity << 0, 0, 0;

     angles = { 0,0,0};
    
    control_stop = false;

    for (int i = 0; i < (int)(1 / sampleTime); ++i)
    {
        rclcpp::spinOnce();
        rate.sleep();
    }

    while (rclcpp::ok() && !control_stop)
    {
        t = rclcpp::Time::now().toSec();

        if (current_state_msg.mode != "OFFBOARD" && print_flag_offboard == 1)
        {
            ROS_INFO("OFFBOARD mode is not enabled!");
            print_flag_offboard = 0;
        }
        if (!current_state_msg.armed && print_flag_arm == 1)
        {
            ROS_INFO("Vehicle is not armed!");
            print_flag_arm = 2;
        }
        else if (current_state_msg.armed && print_flag_arm == 2)
        {
            ROS_INFO("Vehicle is armed!");
            print_flag_arm = 0;
        }

        if (current_state_msg.mode == "ALTCTL")
        {
            pos_ref = current_pos_att;
            if (print_flag_altctl == 1)
            {
                ROS_INFO("ALTCTL mode is enabled!");
                print_flag_altctl = 0;
            }
        }

        if (!nmpc_pc->return_control_init_value())
        {
            nmpc_pc->nmpc_init(pos_ref, nmpc_pc->nmpc_struct);
            if (nmpc_struct.verbose && nmpc_pc->return_control_init_value())
            {
                std::cout << "***********************************\n";
                std::cout << "NMPC: initialized correctly\n";
                std::cout << "***********************************\n";
            }
        }

        while (rclcpp::ok() && !control_stop)
        {

            t_cc_loop = rclcpp::Time::now().toSec() - t;
            if (std::fmod(std::abs(t_cc_loop - (int)(t_cc_loop)), (double)(sampleTime)) == 0)
                std::cout << "loop time for outer NMPC: " << t_cc_loop << " (sec)"
                          << "\n";
          //  if (M_PI<angles.at(2) < 2 * M_PI)
           // {
           //   angles.at(2) = angles.at(2) - 2 * M_PI;


            //}
            // Setting up state-feedback [x,y,z,u,v,w,psi,r]
          //  current_states = {current_pos_att.at(0)-6376979,
           //                   current_pos_att.at(1)-1673713.7,
              current_states = {current_pos_att.at(0),
                              current_pos_att.at(1),
                              current_pos_att.at(2),
                              current_vel_rate.at(0),
                              current_vel_rate.at(1),
                              current_vel_rate.at(2),
                              angles.at(2),
                              current_vel_rate.at(5)
                              };


                    ref_trajectory = {ref_position[0],  //x
                                      ref_position[1],  //y
                                      ref_position[2],   //z
                                      ref_velocity[0],   //u
                                      ref_velocity[1],   //v
                                      ref_velocity[2],   //w
                                      ref_yaw_rad,
                                      0.0
                             };                   




            std::cout << "current_states = ";
            for (int idx = 0; idx < current_states.size(); idx++)
            {
                std::cout << current_states[idx] << ",";
            }
            std::cout << "\n";

            std::cout << "ref_trajectory = ";
            for (int idx = 0; idx < ref_trajectory.size(); idx++)
            {
                std::cout << ref_trajectory[idx] << ",";
            }
            std::cout << "\n";

            
            std::cout << "ref  yaw = "<< ref_yaw_rad << std::endl ;
            std::cout << "current yaw = "<< angles.at(2) << std::endl ;





           double F_d_max = 50;

          //  online_data.distFx = std::min(dist_Fx.data, F_d_max);
          //  online_data.distFy = std::min(dist_Fy.data, F_d_max);
          //  online_data.distFz = std::min(dist_Fz.data, F_d_max);
          //cap disturbance to F_dmax
                for (size_t i = 0; i < dist_Fx.data.size(); ++i) {
                    dist_Fx.data[i] = std::min(std::max(dist_Fx.data[i], -F_d_max), F_d_max);
                    dist_Fy.data[i] = std::min(std::max(dist_Fy.data[i], -F_d_max), F_d_max);
                    dist_Fz.data[i] = std::min(std::max(dist_Fz.data[i], -F_d_max), F_d_max);
                }


                
           online_data.distFx = dist_Fx.data;
           online_data.distFy = dist_Fy.data;
           online_data.distFz = dist_Fx.data_zeros;


       //   ROS_ERROR_STREAM("online_data = " << online_data.distFx[0] << " (sec)");
       //   ROS_ERROR_STREAM("online_data = " << online_data.distFy[0] << " (sec)");
        //  ROS_ERROR_STREAM("online_data = " << online_data.distFz[0] << " (sec)");
          std::cout << "\033[1;31m" << "online_data = " << online_data.distFx[0] << " (sec)" << "\033[0m" << std::endl;
          std::cout << "\033[1;31m" << "online_data = " << online_data.distFy[0] << " (sec)" << "\033[0m" << std::endl;
          std::cout << "\033[1;31m" << "online_data = " << online_data.distFz[0] << " (sec)" << "\033[0m" << std::endl;


            nmpc_pc->nmpc_core(nmpc_struct,
                               nmpc_pc->nmpc_struct,
                               nmpc_pc->nmpc_cmd_struct,
                               ref_trajectory,
                               online_data,
                               current_states);

            if (nmpc_pc->acado_feedbackStep_fb != 0)
                control_stop = true;

            if (std::isnan(nmpc_pc->nmpc_struct.u[0]) == true || std::isnan(nmpc_pc->nmpc_struct.u[1]) == true ||
                std::isnan(nmpc_pc->nmpc_struct.u[2]) == true || std::isnan(nmpc_pc->nmpc_struct.u[3]) == true)
            {
                ROS_ERROR_STREAM("Controller ERROR at time = " << rclcpp::Time::now().toSec() - t << " (sec)");
                control_stop = true;
                exit(0);
            }

            std::cout << "predicted dist x "<< online_data.distFx[0]<<endl;
           /// std::cout << "thrust input x1"<< nmpc_pc->nmpc_struct.x[1+9]<<endl;

            nmpc_pc->publish_pred_tarjectory(nmpc_pc->nmpc_struct);
            nmpc_pc->publish_wrench(nmpc_pc->nmpc_cmd_struct);

            print_flag_offboard = 1;
            print_flag_arm = 1;
            print_flag_altctl = 1;

            rclcpp::spinOnce();
            rate.sleep();
        }
        



        
        nmpc_pc->publish_wrench(nmpc_pc->nmpc_cmd_struct);
        



        rclcpp::spinOnce();
        rate.sleep();
    }


    return 0;
}
