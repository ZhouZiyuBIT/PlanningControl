
#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include"quadrotor_sim/control_u.h"
#include"px4_ctrl/track_traj.h"

#include"mpc.hpp"

// export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/zhouziyu/PlanningControl/src/px4_ctrl/third_party/libcasadi-linux-gcc5-v3.5.5/lib
QuadrotorMPC *_mpc;

ros::Publisher _control_u_pub;

ros::Subscriber _track_traj_sub;
px4_ctrl::track_traj _track_traj_msg;

ros::Subscriber _state_sub;
nav_msgs::Odometry _state_msg;

int step_num = 5;

bool track_traj_received = false;
void rcv_track_traj_cb(const px4_ctrl::track_traj &msg)
{
    _track_traj_msg = msg;
    track_traj_received = true;
}

void rcv_state_cb(nav_msgs::Odometry msg)
{
    float q_state[10] = {msg.pose.pose.position.x, -msg.pose.pose.position.y, -msg.pose.pose.position.z,
                         msg.twist.twist.linear.x, -msg.twist.twist.linear.y, -msg.twist.twist.linear.z,
                         msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, -msg.pose.pose.orientation.y, -msg.pose.pose.orientation.z,};
    std::vector<double> ref_path(10+(7+3)*step_num, 0);
    if(track_traj_received)
    {
        ref_path[0] = q_state[0];ref_path[1] = q_state[1];ref_path[2] = q_state[2];
        ref_path[3] = q_state[3];ref_path[4] = q_state[4];ref_path[5] = q_state[5];
        ref_path[6] = q_state[6];ref_path[7] = q_state[7];ref_path[8] = q_state[8];ref_path[9] = q_state[9];

        for(int i=0; i<step_num; i++)
        {
            ref_path[10*(i+1)+0] = _track_traj_msg.pos_pts[i].x;
            ref_path[10*(i+1)+1] = -_track_traj_msg.pos_pts[i].y;
            ref_path[10*(i+1)+2] = -_track_traj_msg.pos_pts[i].z;
            ref_path[10*(i+1)+3] = 0;
            ref_path[10*(i+1)+4] = 0;
            ref_path[10*(i+1)+5] = 0;
            ref_path[10*(i+1)+6] = -_track_traj_msg.yaw_pts[i];

            ref_path[10*(i+1)+7] = 10;
            ref_path[10*(i+1)+8] = 0;
            if(_track_traj_msg.yaw_pts[i] > 500)
            {
                ref_path[10*(i+1)+9] = 0;
            }
            else
            {
                ref_path[10*(i+1)+9] = 1;
            }
        }
        std::vector<double> u0 = _mpc->solve(ref_path);

        quadrotor_sim::control_u control_u_msg;
        control_u_msg.az = u0[0];
        control_u_msg.wx = u0[1];
        control_u_msg.wy = u0[2];
        control_u_msg.wz = u0[3];
        _control_u_pub.publish(control_u_msg);
    }
}


// MPC
void mpc_init()
{
    double _quad_x0[10] = {0, 0, 0, 0, 0, 0, 1, 0, 0, 0};
    double _quad_u0[4] = {-9.8, 0, 0, 0};
    double _x_min[10] = {-1000, -1000, -1000, -1000, -1000, -1000, -1000, -1000, -1000, -1000};
    double _x_max[10] = { 1000,  1000,  1000,  1000,  1000,  1000,  1000,  1000,  1000,  1000};
    double _u_min[4] = {-20, -6, -6, -6};
    double _u_max[4] = {0,  6,  6,  6};
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
    for(int i=0; i<step_num; i++)
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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "px4_mpc");
    ros::NodeHandle n("~");

    if(argc < 2)
    {
        std::cout<< "argument error" << std::endl;
        return -1;
    }
    std::string mpclib_path(argv[1]);
    _mpc = new QuadrotorMPC(mpclib_path, 1, 0.1);

    _control_u_pub = n.advertise<quadrotor_sim::control_u>("/px4/control_u", 1);
    _track_traj_sub = n.subscribe("/planning/track_traj", 1, rcv_track_traj_cb);
    _state_sub = n.subscribe("/px4/state", 1, rcv_state_cb);

    mpc_init();

    ros::spin();
    // ros::Rate loop_rate(100);
    // while(ros::ok())
    // {
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

    return 0;
}

