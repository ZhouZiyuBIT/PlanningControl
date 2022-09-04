#include "Px4Bridge.hpp"

#include<iostream>
#include<string>

#include<ros/ros.h>
#include<sensor_msgs/Imu.h>
#include<nav_msgs/Odometry.h>
#include"quadrotor_sim/control_u.h"

Px4Bridge quad;

ros::Publisher _imu_pub;
ros::Publisher _state_pub;
ros::Subscriber _slam_odom_sub;
ros::Subscriber _control_u_sub;

void rcv_imu_callback(float w[3], float a[3])
{
    sensor_msgs::Imu s_imu;
    s_imu.header.stamp = ros::Time::now();
    s_imu.angular_velocity.x = w[0];
    s_imu.angular_velocity.y = w[1];
    s_imu.angular_velocity.z = w[2];
    s_imu.linear_acceleration.x = a[0];
    s_imu.linear_acceleration.y = a[1];
    s_imu.linear_acceleration.z = a[2];

    _imu_pub.publish(s_imu);
}

void rcv_state_callback(float state[10])
{
    nav_msgs::Odometry odom;
    odom.pose.pose.position.x = state[0];
    odom.pose.pose.position.y = -state[1];
    odom.pose.pose.position.z = -state[2];
    odom.twist.twist.linear.x = state[3];
    odom.twist.twist.linear.y = -state[4];
    odom.twist.twist.linear.z = -state[5];
    odom.pose.pose.orientation.w = state[6];
    odom.pose.pose.orientation.x = state[7];
    odom.pose.pose.orientation.y = -state[8];
    odom.pose.pose.orientation.z = -state[9];
    _state_pub.publish(odom);
}

void rcv_slam_odom_cb(nav_msgs::Odometry msg)
{
    quad.send_odom_data(msg.pose.pose.position.x, -msg.pose.pose.position.y, -msg.pose.pose.position.z,
                        -msg.pose.pose.orientation.x, msg.pose.pose.orientation.w, -msg.pose.pose.orientation.z, msg.pose.pose.orientation.y);
}

void rcv_control_u_cb(quadrotor_sim::control_u msg)
{
    quad.send_control_u(-msg.az/20, msg.wx, msg.wy, msg.wz);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Px4Bridge");
    ros::NodeHandle n("~");

    _imu_pub = n.advertise<sensor_msgs::Imu>("/px4/imu", 1);
    _state_pub = n.advertise<nav_msgs::Odometry>("/px4/state", 1);
    _slam_odom_sub = n.subscribe("/slam/odom", 1, rcv_slam_odom_cb);
    _control_u_sub = n.subscribe("/px4/control_u", 1, rcv_control_u_cb);

    quad.registe_rcv_state_callback(rcv_state_callback);
    quad.registe_rcv_sensor_imu_callback(rcv_imu_callback);

    quad.set_thread_rt(90);
    if(quad.setup_port("/dev/ttyACM0") == -1)
    {
        exit(-1);
    }

    // quad.setup_optitrack("192.168.1.200");
    quad.add_fordwarding("127.0.0.1", 8976, "127.0.0.1", 14550);
    quad.core_start();

    ros::spin();
    // ros::Rate loop_rate(100);
    // while(ros::ok())
    // {
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

    return 0;
}
