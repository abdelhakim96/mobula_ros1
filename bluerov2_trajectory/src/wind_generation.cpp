/**
 * @file   wind_generation.cpp
 * @author Hakim Amer / Mohit Mehindratta
 * @date   Jan 2024

 */

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Wrench.h>
#include <Eigen/Dense>
#include <random>
#include <dynamic_reconfigure/server.h>
#include <bluerov2_trajectory/set_wind_generationConfig.h>
#include "geometry_msgs/Twist.h"

using namespace std;
using namespace Eigen;

double sampleTime = 0.001;
bool wind_start, noise_on;
Vector3d wind_component;
int wind_type, noise_type;
float max_wind_force, mean_wind_force, wind_time_period, noise_stddev, noise_time_period;
float ws_x;
float ws_y;
geometry_msgs::Twist wind_speed;

void dynamicReconfigureCallback(bluerov2_trajectory::set_wind_generationConfig& config, uint32_t level)
{
    wind_start = config.wind_start;
    wind_component = { config.wind_component_x, config.wind_component_y, config.wind_component_z };

    std::cout << config.wind_component_x;
    std::cout << config.mean_wind_force;
    std::cout << config.max_wind_force;
    wind_type = config.wind_type;
    max_wind_force = config.max_wind_force;
    mean_wind_force = config.mean_wind_force;
    wind_time_period = config.wind_time_period;
    noise_on = config.noise_on;
    noise_type = config.noise_type;
    noise_stddev = config.noise_stddev;
    noise_time_period = config.noise_time_period;
}










std_msgs::Bool trajectory_start_flag;

void trajectory_start_cb(const std_msgs::Bool::ConstPtr& msg)
{
    trajectory_start_flag = *msg;
}

void windmodel_cb(const geometry_msgs::Twist::ConstPtr& vel)
{
    wind_speed = *vel;
    ws_x = wind_speed.linear.x;
    ws_y = wind_speed.linear.y;
}

geometry_msgs::Wrench prepare_wind_msg(Eigen::Vector3d& wind)
{
    geometry_msgs::Wrench _wind_msg;

    // Set the force component of the Wrench message
    _wind_msg.force.x = wind(0);
    _wind_msg.force.y = wind(1);
    _wind_msg.force.z = wind(2);

    // Set the torque components to zero (assuming no rotational influence)
    _wind_msg.torque.x = 0.0;
    _wind_msg.torque.y = 0.0;
    _wind_msg.torque.z = 0.0;

    return _wind_msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wind_generation");
    ros::NodeHandle nh;

    dynamic_reconfigure::Server<bluerov2_trajectory::set_wind_generationConfig> server;
    dynamic_reconfigure::Server<bluerov2_trajectory::set_wind_generationConfig>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    std::string trajectory_on_sub_topic, wind_pub_topic;

    nh.param("trajectory_on_sub_topic", trajectory_on_sub_topic, std::string("/trajectory_on"));
    nh.param("wind_pub_topic", wind_pub_topic, std::string("/mobula/rov/disturbance"));

    ros::Subscriber trajectory_start_sub = nh.subscribe<std_msgs::Bool>(trajectory_on_sub_topic, 1, trajectory_start_cb);
    ros::Subscriber windmodel_sub = nh.subscribe<geometry_msgs::Twist>("/new_vel", 1, windmodel_cb);
    ros::Publisher wind_pub = nh.advertise<geometry_msgs::Wrench>(wind_pub_topic, 1, true);

    ros::Rate rate(1 / sampleTime);

    Vector3d wind_magnitude;
    geometry_msgs::Wrench wind_msg;

    double t, t_last, t_loop, t_last_noise, t_loop_noise;

    int print_flag_traj_start = 1, print_flag_wind_start = 1,
        print_flag_const = 1, print_flag_sinus = 1, print_flag_sinus_comb1 = 1,
        print_flag_sinus_comb2 = 1, print_flag_sinus_comb3 = 1,
        print_flag_Gauss_noise = 1, print_flag_sine_noise = 1, print_flag_comb_noise = 1, print_flag_windfarm = 1;

    std::default_random_engine rand_seed;
    rand_seed.seed(std::time(0));

    for (int i = 0; i < (int)1 / sampleTime; i++)
    {
        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok())
    {
        if (wind_start && trajectory_start_flag.data)
        {
            t = ros::Time::now().toSec();
            t_loop = t - t_last;
            t_loop_noise = t - t_last_noise;

            switch (wind_type)
            {
            case 0: // Constant wind force
                if (print_flag_const == 1)
                {
                    ROS_INFO("--------Constant wind force selected!--------");
                    print_flag_traj_start = 1;
                    print_flag_wind_start = 1;
                    print_flag_const = 0;
                    print_flag_sinus = 1;
                    print_flag_sinus_comb1 = 1;
                    print_flag_sinus_comb2 = 1;
                    print_flag_sinus_comb3 = 1;
                }
                wind_magnitude = wind_component.array() * max_wind_force;
                break;

            case 1: // Sinusoidal wind force
                if (print_flag_sinus == 1)
                {
                    t_last = ros::Time::now().toSec();
                    t_loop = t - t_last;

                    ROS_INFO("--------Sinusoidal wind force selected!--------");
                    print_flag_traj_start = 1;
                    print_flag_wind_start = 1;
                    print_flag_const = 1;
                    print_flag_sinus = 0;
                    print_flag_sinus_comb1 = 1;
                    print_flag_sinus_comb2 = 1;
                    print_flag_sinus_comb3 = 1;
                }
                wind_magnitude = wind_component.array() * mean_wind_force +
                    wind_component.array() *  max_wind_force * sin((2 * M_PI / wind_time_period) * t_loop);
                break;

            case 2: // Sinusoidal combination wind force type 1
                if (print_flag_sinus_comb1 == 1)
                {
                    t_last = ros::Time::now().toSec();
                    t_loop = t - t_last;

                    ROS_INFO("--------Sinusoidal combination (type 1) wind force selected!--------");
                    print_flag_traj_start = 1;
                    print_flag_wind_start = 1;
                    print_flag_const = 1;
                    print_flag_sinus = 1;
                    print_flag_sinus_comb1 = 0;
                    print_flag_sinus_comb2 = 1;
                    print_flag_sinus_comb3 = 1;
                }
                wind_magnitude = wind_component.array() * mean_wind_force +
                    wind_component.array() * max_wind_force *
                    (std::pow(sin((2 * M_PI / wind_time_period) * t_loop), 2) +
                        sin((2 * M_PI / wind_time_period) * t_loop));
                break;

            case 3: // Sinusoidal combination wind force type 2
                if (print_flag_sinus_comb2 == 1)
                {
                    t_last = ros::Time::now().toSec();
                    t_loop = t - t_last;

                    ROS_INFO("--------Sinusoidal combination (type 2) wind force selected!--------");
                    print_flag_traj_start = 1;
                    print_flag_wind_start = 1;
                    print_flag_const = 1;
                    print_flag_sinus = 1;
                    print_flag_sinus_comb1 = 1;
                    print_flag_sinus_comb2 = 0;
                    print_flag_sinus_comb3 = 1;
                }
                wind_magnitude = wind_component.array() * mean_wind_force +
                    wind_component.array() * max_wind_force *
                    (0.5 * std::pow(sin((4 * M_PI / wind_time_period) * t_loop), 4) +
                        std::pow(cos((2 * M_PI / wind_time_period) * (t_loop)), 3) +
                        std::pow(sin((2 * M_PI / wind_time_period) * t_loop), 2) +
                        sin((2 * M_PI / wind_time_period) * t_loop));
                break;
                //hakim edit

                /*
            case 4: // Wind Farm 
      if (print_flag_sinus_comb3 == 1)
    {
        t_last = ros::Time::now().toSec();
        t_loop = t - t_last;

        ROS_INFO("--------Wind Farm wind force selected!--------");
        print_flag_traj_start = 1;
        print_flag_wind_start = 1;
        print_flag_const = 1;
        print_flag_sinus = 1;
        print_flag_sinus_comb1 = 1;
        print_flag_sinus_comb2 = 1;
        print_flag_sinus_comb3 = 0;
        print_flag_windfarm = 0;
    }

    // Set wind magnitude based on input wind speed
    //wind_magnitude = { ws_x, ws_y, 0 };

    // Calculate time elapsed since the last update
    t_loop = ros::Time::now().toSec() - t_last;

    // Define the duration of each phase of the wind pattern
    double phase_duration = wind_time_period / 4.0;
    std::normal_distribution<double> disturb(0, noise_stddev/5);

    // Calculate wind force based on the current phase of the pattern
    if (t_loop < phase_duration) {
        // Linearly increasing phase
        wind_magnitude = wind_component.array() * (disturb(rand_seed) + 2*t_loop / phase_duration +  max_wind_force * sin(12 * M_PI * (t_loop - phase_duration) / phase_duration));
    } else if (t_loop < 2 * phase_duration) {
        // Sine wave phase
        wind_magnitude = wind_component.array() * (disturb(rand_seed) +mean_wind_force)  ;
    } else if (t_loop < 3 * phase_duration ) {
        // Step function phase
       //wind_magnitude = wind_component.array() * ( mean_wind_force + 5 * max_wind_force * sin(12 * M_PI * (t_loop - phase_duration) / phase_duration));
                            wind_magnitude = wind_component.array() * mean_wind_force +
                    wind_component.array() * max_wind_force * 5*
                    (0.5 * std::pow(sin((4 * M_PI *10 / wind_time_period) * t_loop), 4) +
                        std::pow(cos((2 * M_PI*10 / wind_time_period) * (t_loop)), 3) +
                        std::pow(sin((2 * M_PI*10 / wind_time_period) * t_loop), 2) +
                        sin((2 * M_PI*10 / wind_time_period) * t_loop));

    } else if (t_loop < 4 * phase_duration) {
        // Linearly decreasing phase
        wind_magnitude = wind_component.array() * (disturb(rand_seed) + mean_wind_force - mean_wind_force * ((t_loop - 3 * phase_duration) / phase_duration)+  0.5 * max_wind_force * sin(22 * M_PI * (t_loop - phase_duration) / phase_duration));
    } else {
        // Restart the pattern
        t_last = ros::Time::now().toSec();
    }
    break;
   */
                
case 4: // Wind Farm (replacing with Test case)
    if (print_flag_sinus_comb3 == 1)
    {
        t_last = ros::Time::now().toSec();
        t_loop = t - t_last;

        ROS_INFO("--------Test case: Sine wave for the first 30 seconds, then switch to sine wave comb2--------");
        print_flag_traj_start = 1;
        print_flag_wind_start = 1;
        print_flag_const = 1;
        print_flag_sinus = 1;
        print_flag_sinus_comb1 = 1;
        print_flag_sinus_comb2 = 1;
        print_flag_sinus_comb3 = 0; // Update the print flag
    }
    if (t_loop < 30) {
        // Sine wave for the first 30 seconds
        wind_magnitude = wind_component.array() * (mean_wind_force + 2 * max_wind_force * sin((2 * M_PI / wind_time_period) * t_loop));
    } else {

       if (t_loop < 60) {
        std::normal_distribution<double> disturb(0, 0.1);
        // Switch to sine wave comb2 for the rest of the time
        wind_magnitude = wind_component.array() * (mean_wind_force) * 5 +disturb(rand_seed)+
            wind_component.array() * max_wind_force * 5 *
            (0.5 * std::pow(sin((2 * M_PI / wind_time_period) * t_loop), 4) +
                std::pow(cos((1 * M_PI / wind_time_period) * (t_loop)), 3) +
                std::pow(sin((1 * M_PI / wind_time_period) * t_loop), 2) +
                sin((2 * M_PI / wind_time_period) * t_loop));
    }
     
    else {

        // Apply a square wave of amplitude -2 and frequency of 10 seconds
        double square_wave_amplitude = -2.0;
        double square_wave_frequency = 1.0 / 20.0; // Frequency is the inverse of the time period
         wind_magnitude = wind_component.array() * (mean_wind_force) + 1.0 + 
            wind_component.array() * max_wind_force * 5  * (std::sin(2 * M_PI * square_wave_frequency * t_loop) > 0 ? 1 : -1);
      
    }
    }
    break;



     //       default:
       //         break;
            }

            if (noise_on)
            {
                switch (noise_type)
                {
                case 0:
                {
                    if (print_flag_Gauss_noise == 1)
                    {
                        ROS_INFO("--------Gaussian noise added!--------");
                        print_flag_traj_start = 1;
                        print_flag_wind_start = 1;
                        print_flag_Gauss_noise = 0;
                        print_flag_sine_noise = 1;
                        print_flag_comb_noise = 1;
                    }
                    std::normal_distribution<double> disturb(0, noise_stddev);
                    wind_magnitude.array() += wind_component.array() * disturb(rand_seed);
                    break;
                }

                case 1:
                {
                    if (print_flag_sine_noise == 1)
                    {
                        t_last_noise = ros::Time::now().toSec();
                        t_loop_noise = t - t_last_noise;

                        ROS_INFO("--------Sinusoidal noise added!--------");
                        print_flag_traj_start = 1;
                        print_flag_wind_start = 1;
                        print_flag_Gauss_noise = 1;
                        print_flag_sine_noise = 0;
                        print_flag_comb_noise = 1;
                    }
                    wind_magnitude.array() += wind_component.array() * (noise_stddev * sin((2 * M_PI / noise_time_period) * t_loop_noise));
                    break;
                }

                case 2:
                {
                    if (print_flag_comb_noise == 1)
                    {
                        t_last_noise = ros::Time::now().toSec();
                        t_loop_noise = t - t_last_noise;

                        ROS_INFO("--------Combination of Gaussian and sinusoidal noise added!--------");
                        print_flag_traj_start = 1;
                        print_flag_wind_start = 1;
                        print_flag_Gauss_noise = 1;
                        print_flag_sine_noise = 1;
                        print_flag_comb_noise = 0;
                    }
                    std::normal_distribution<double> disturb(0, noise_stddev);
                    wind_magnitude.array() += wind_component.array() * (
                        disturb(rand_seed) +
                        noise_stddev * sin((2 * M_PI / noise_time_period) * t_loop_noise));
                    break;
                }

                default:
                    break;
                }
            }
            else
            {
                if (print_flag_Gauss_noise == 0 || print_flag_sine_noise == 0 || print_flag_sine_noise == 0)
                {
                    print_flag_traj_start = 1;
                    print_flag_wind_start = 1;
                    print_flag_Gauss_noise = 1;
                    print_flag_sine_noise = 1;
                    print_flag_comb_noise = 1;
                }
            }
        }
        else
        {
            if (!trajectory_start_flag.data && print_flag_traj_start == 1)
            {
                ROS_INFO("Waiting for trajectory start switch to begin!");
                print_flag_traj_start = 0;
            }
            else if (trajectory_start_flag.data && print_flag_traj_start == 0)
            {
                std::cout << "traj_start --> 1\n";
                print_flag_traj_start = 1;
            }

            if (!wind_start && print_flag_wind_start == 1)
            {
                ROS_INFO("Waiting for wind start switch to begin!");
                print_flag_wind_start = 0;
            }
            else if (wind_start && print_flag_wind_start == 0)
            {
                std::cout << "wind_start --> 1\n";
                print_flag_wind_start = 1;
            }
            wind_magnitude.setZero();
            print_flag_const = 1;
            print_flag_sinus = 1;
            print_flag_sinus_comb1 = 1;
            print_flag_sinus_comb2 = 1;
            print_flag_sinus_comb3 = 1;
            print_flag_Gauss_noise = 1;
            print_flag_sine_noise = 1;
            print_flag_comb_noise = 1;
        }

        wind_msg = prepare_wind_msg(wind_magnitude);
        wind_pub.publish(wind_msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}