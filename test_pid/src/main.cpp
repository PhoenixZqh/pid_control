#include "pid.h"
#include <geometry_msgs/TwistStamped.h>
#include <iostream>
#include <ros/ros.h>
#include <turtlesim/Pose.h>

float           dt = 0.02;
turtlesim::Pose cur_pos;
turtlesim::Pose target_pos;

/**
 * @brief
 * kp: 响应更快，意味着更大的振荡
 * ki: 更快的消除稳态误差
 * kd: 增加系统稳定性，减小振荡，水桶模型防止超出1m的水
 */
PID pid(0.005, 0.001, 0.0001);

ros::Publisher vel_pub;

void timerCallback(const ros::TimerEvent&)
{

    float x_diff     = target_pos.x - cur_pos.x;
    float target_vel = x_diff / dt;

    float y_diff       = target_pos.y - cur_pos.y;
    float y_target_vel = y_diff / dt;

    ROS_INFO_THROTTLE(1.0, "\033[1;32m cur_x: %f \033[0m, \033[1;31m target_x: %f \033[0m", cur_pos.x, target_pos.x);
    ROS_INFO_THROTTLE(1.0, "\033[1;32m cur_y: %f \033[0m, \033[1;31m target_y: %f \033[0m", cur_pos.y, target_pos.y);
    // ROS_INFO_THROTTLE(1.0, "cur_vel: %f, target_vel: %f ", cur_pos.linear_velocity, target_vel);

    float x_vel = pid.computeVal(cur_pos.linear_velocity, target_vel, dt);
    float y_vel = pid.computeVal(cur_pos.linear_velocity, y_target_vel, dt);

    ROS_INFO_THROTTLE(1.0, "PID x_vel: %f, y_vel: %f", x_vel, y_vel);
    ROS_INFO_THROTTLE(1.0, "======================================");

    geometry_msgs::Twist gui_vel;
    gui_vel.linear.x  = x_vel;
    gui_vel.linear.y  = y_vel;
    gui_vel.linear.z  = 0.0;
    gui_vel.angular.x = 0.0;
    gui_vel.angular.y = 0.0;
    gui_vel.angular.z = 0.0;
    vel_pub.publish(gui_vel);
}

void guiPosCB(const turtlesim::Pose::ConstPtr& msg)
{
    cur_pos = *msg;
}

void targetPosCB(const turtlesim::Pose::ConstPtr& msg)
{
    target_pos = *msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "time_interval_node");
    ros::NodeHandle nh;

    ros::Timer timer = nh.createTimer(ros::Duration(dt), timerCallback);

    ros::Subscriber cur_sub    = nh.subscribe("/turtle1/pose", 10, guiPosCB);
    ros::Subscriber target_sub = nh.subscribe("/target/pose", 10, targetPosCB);
    vel_pub                    = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    ros::spin();
    return 0;
}
