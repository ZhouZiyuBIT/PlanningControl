#include "Px4Bridge.hpp"

#include<iostream>
#include<string>

#include<ros/ros.h>
#include<sensor_msgs/Imu.h>
#include<nav_msgs/Odometry.h>
#include"quadrotor_sim/control_u.h"
#include"px4_ctrl/track_trj.h"

#include"mpc.hpp"

// export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/ziyuzhou/catkin_ws/src/px4_ctrl/third_party/libcasadi-linux-gcc5-v3.5.5/lib
QuadrotorMPC *_mpc;

Px4Bridge quad;

void state_print(float px, float py, float pz,
                 float qw, float qx, float qy, float qz,
                 float vx, float vy, float vz)
{
    std::cout << "pxyz:" << px << ", " << py << ", " << pz << std::endl;
}

void sensor_imu_print(float w[3], float a[3])
{
    std::cout << "pxyz:" << w[0] << ", " << w[1] << ", " << w[2] << ", "
              << a[0] << ", " << a[1] << ", " << a[2] << std::endl;
}

ros::Publisher _sensor_imu_pub;
ros::Publisher _quad_control_u_pub;
ros::Subscriber _track_trj_sub;
ros::Subscriber _odom_sub;

px4_ctrl::track_trj _track_trj;
bool track_trj_received = false;
void rcv_track_trj_cb(const px4_ctrl::track_trj &msg)
{
    _track_trj = msg;
    track_trj_received = true;
}

void sensor_imu_callback(float w[3], float a[3])
{
    sensor_msgs::Imu s_imu;
    s_imu.header.stamp = ros::Time::now();
    s_imu.angular_velocity.x = w[0];
    s_imu.angular_velocity.y = w[1];
    s_imu.angular_velocity.z = w[2];
    s_imu.linear_acceleration.x = a[0];
    s_imu.linear_acceleration.y = a[1];
    s_imu.linear_acceleration.z = a[2];

    _sensor_imu_pub.publish(s_imu);
}

// MPC
void mpc_init()
{
    double _quad_x0[10] = {0, 0, 0, 1, 0, 0, 0, 0, 0, 0};
    double _quad_u0[4] = {0.5, 0, 0, 0};
    double _x_min[10] = {-1000, -1000, -1000, -1000, -1000, -1000, -1000, -1000, -1000, -1000};
    double _x_max[10] = { 1000,  1000,  1000,  1000,  1000,  1000,  1000,  1000,  1000,  1000};
    double _u_min[4] = {0, -6, -6, -6};
    double _u_max[4] = {1,  6,  6,  6};
    double _g_min[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    double _g_max[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    std::vector<double> nlp_x0, nlp_lbx, nlp_ubx, nlp_lbg, nlp_ubg;

    std::vector<double> quad_x0(_quad_x0, _quad_x0+sizeof(_quad_x0)/sizeof(double));
    std::vector<double> quad_u0(_quad_u0, _quad_u0+sizeof(_quad_u0)/sizeof(double));

    std::vector<double> x_min(_x_min, _x_min+sizeof(_x_min)/sizeof(double));
    std::vector<double> x_max(_x_max, _x_max+sizeof(_x_max)/sizeof(double));
    std::vector<double> u_min(_u_min, _u_min+sizeof(_u_min)/sizeof(double));
    std::vector<double> u_max(_u_max, _u_max+sizeof(_u_max)/sizeof(double));
    std::vector<double> g_min(_g_min, _g_min+sizeof(_g_min)/sizeof(double));
    std::vector<double> g_max(_g_max, _g_max+sizeof(_g_max)/sizeof(double));

    nlp_x0.insert(nlp_x0.end(), quad_x0.begin(), quad_x0.end());
    nlp_lbx.insert(nlp_lbx.end(), x_min.begin(), x_min.end());
    nlp_ubx.insert(nlp_ubx.end(), x_max.begin(), x_max.end());
    nlp_lbg.insert(nlp_lbg.end(), g_min.begin(), g_min.end());
    nlp_ubg.insert(nlp_ubg.end(), g_max.begin(), g_max.end());
    for(int i=0; i<10; i++)
    {
        nlp_x0.insert(nlp_x0.end(), quad_u0.begin(), quad_u0.end());
        nlp_x0.insert(nlp_x0.end(), quad_x0.begin(), quad_x0.end());
        nlp_lbx.insert(nlp_lbx.end(), u_min.begin(), u_min.end());
        nlp_ubx.insert(nlp_ubx.end(), u_max.begin(), u_max.end());
        nlp_lbx.insert(nlp_lbx.end(), x_min.begin(), x_min.end());
        nlp_ubx.insert(nlp_ubx.end(), x_max.begin(), x_max.end());
        nlp_lbg.insert(nlp_lbg.end(), g_min.begin(), g_min.end());
        nlp_ubg.insert(nlp_ubg.end(), g_max.begin(), g_max.end());
    }

    _mpc->set_x0(nlp_x0);
    _mpc->set_lbx_ubx(nlp_lbx, nlp_ubx);
    _mpc->set_lbg_ubg(nlp_lbg, nlp_ubg);
}

void track_trj_mpc(float q_state[10])
{
    std::vector<double> ref_path(11*13, 0);
    if(track_trj_received)
    {
        ref_path[0] = q_state[0];ref_path[1] = q_state[1];ref_path[2] = q_state[2];
        ref_path[3] = q_state[3];ref_path[4] = q_state[4];ref_path[5] = q_state[5];ref_path[6] = q_state[6];
        ref_path[7] = q_state[7];ref_path[8] = q_state[8];ref_path[9] = q_state[9];
        ref_path[10] = 0;ref_path[11] = 0;ref_path[12] = 0;

        for(int i=0; i<10; i++)
        {
            ref_path[13*(i+1)+0] = _track_trj.pos_pts[i].x;
            ref_path[13*(i+1)+1] = -_track_trj.pos_pts[i].y;
            ref_path[13*(i+1)+2] = -_track_trj.pos_pts[i].z;
            ref_path[13*(i+1)+3] = 1;
            ref_path[13*(i+1)+4] = 0;
            ref_path[13*(i+1)+5] = 0;
            ref_path[13*(i+1)+6] = 0;
            ref_path[13*(i+1)+7] = 0;
            ref_path[13*(i+1)+8] = 0;
            ref_path[13*(i+1)+9] = 0;
            ref_path[13*(i+1)+10] = 1;
            ref_path[13*(i+1)+11] = 0;
            ref_path[13*(i+1)+12] = 0;
        }
        std::cout << "******************************************************************" << ref_path[0] << std::endl;
        std::vector<double> u0 = _mpc->solve(ref_path);
        // quad.send_control_u(u0[0], u0[1], u0[2], u0[3]);

        quadrotor_sim::control_u control_u_msg;
        control_u_msg.f = u0[0];
        control_u_msg.wx = u0[1];
        control_u_msg.wy = u0[2];
        control_u_msg.wz = u0[3];
        _quad_control_u_pub.publish(control_u_msg);
    }
}

void rcv_odom_cb(const nav_msgs::Odometry &odom)
{
    float q_s[10] = {odom.pose.pose.position.x, -odom.pose.pose.position.y, -odom.pose.pose.position.z,
                      odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, -odom.pose.pose.orientation.y, -odom.pose.pose.orientation.z,
                      odom.twist.twist.linear.x, -odom.twist.twist.linear.y, -odom.twist.twist.linear.z};
    track_trj_mpc(q_s);   
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Px4Bridge");
    ros::NodeHandle n("~");

    if(argc < 2)
    {
        std::cout<< "argument error" << std::endl;
        return -1;
    }
    std::string mpc_path(argv[1]);
    _mpc = new QuadrotorMPC(mpc_path, 1, 0.1);

    _sensor_imu_pub = n.advertise<sensor_msgs::Imu>("/px4/imu", 1);
    _quad_control_u_pub = n.advertise<quadrotor_sim::control_u>("control_u", 1);
    _track_trj_sub = n.subscribe("track_trj", 1, rcv_track_trj_cb);
    _odom_sub = n.subscribe("sim_odom", 1, rcv_odom_cb);

    // quad.registe_rcv_state_callback(track_trj_mpc);
    // quad.registe_rcv_sensor_imu_callback(sensor_imu_print);
    quad.registe_rcv_sensor_imu_callback(sensor_imu_callback);

    quad.set_thread_rt(90);
    if(quad.setup_port("/dev/ttyACM0") == -1)
    {
        exit(-1);
    }

    // quad.setup_optitrack("192.168.1.200");
    quad.add_fordwarding("127.0.0.1", 8976, "127.0.0.1", 14550);
    quad.core_start();

    mpc_init();

    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
