#include"quadrotor_sim.hpp"

#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include"quadrotor_sim/control_u.h"

#include<string>
#include<vector>
#include<iostream>

ros::Subscriber _control_u_sub;
ros::Publisher _odom_pub;

quadrotor_sim::control_u ctrl_u;

bool rcv_control = false;
void rcv_control_u_cb(const quadrotor_sim::control_u &msg)
{
    rcv_control = true;
    ctrl_u = msg;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "quadrotor_sim");
    ros::NodeHandle n("~");

    float pos[3];
    n.param("init_x", pos[0],  0.0f);
    n.param("init_y", pos[1],  0.0f);
    n.param("init_z", pos[2],  0.0f);
    pos[1] = -pos[1];
    pos[2] = -pos[2];

    _control_u_sub = n.subscribe("control_u", 1, rcv_control_u_cb);
    _odom_pub = n.advertise<nav_msgs::Odometry>("odometry", 1);

    if(argc < 2)
    {
        std::cout<< "Argument Error" << std::endl;
        return -1;
    }
    std::string cfg_n(argv[1]);
    QuadrotorSimulation quadrotor(cfg_n);
    quadrotor.set_position(pos);

    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        ros::spinOnce();

        float az = ctrl_u.az;
        float w[3] = {ctrl_u.wx, ctrl_u.wy, ctrl_u.wz};
        if(rcv_control)
            quadrotor.control(az, w);
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "world";

        std::vector<float> pos, quat, vel, w_B;
        pos = quadrotor.get_position();
        quat = quadrotor.get_quaternion();
        vel = quadrotor.get_velocity();
        w_B = quadrotor.get_w_B();
        odom.pose.pose.position.x = pos[0];
        odom.pose.pose.position.y = -pos[1];
        odom.pose.pose.position.z = -pos[2];
        odom.pose.pose.orientation.w = quat[0];
        odom.pose.pose.orientation.x = quat[1];
        odom.pose.pose.orientation.y = -quat[2];
        odom.pose.pose.orientation.z = -quat[3];
        odom.twist.twist.linear.x = vel[0];
        odom.twist.twist.linear.y = -vel[1];
        odom.twist.twist.linear.z = -vel[2];
        odom.twist.twist.angular.x = w_B[0];
        odom.twist.twist.angular.y = -w_B[1];
        odom.twist.twist.angular.z = -w_B[2];
        
        _odom_pub.publish(odom);
        
        loop_rate.sleep();
    }

    return 0;
}
