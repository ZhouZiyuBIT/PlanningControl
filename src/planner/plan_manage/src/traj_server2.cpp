#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "traj_utils/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include "px4_ctrl/track_traj.h"
#include <ros/ros.h>

ros::Publisher pos_cmd_pub;
ros::Publisher track_traj_pub;

px4_ctrl::track_traj track_traj_msg;
quadrotor_msgs::PositionCommand cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

using ego_planner::UniformBspline;

bool receive_traj_ = false;
vector<UniformBspline> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;

double time_forward_;

void bsplineCallback(traj_utils::BsplineConstPtr msg)
{
  // parse pos traj

  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());

  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  // parse yaw traj

  // Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  // for (int i = 0; i < msg->yaw_pts.size(); ++i) {
  //   yaw_pts(i, 0) = msg->yaw_pts[i];
  // }

  //UniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());

  traj_duration_ = traj_[0].getTimeSum();

  receive_traj_ = true;
}

void cmdCallback(const ros::TimerEvent &e)
{
  /* no publishing before receive traj_ */
  if (!receive_traj_)
    return;

  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - start_time_).toSec();

  Eigen::Vector3d pos, end, vel;
  double yaw=0;
  static double yaw_last = 0, yaw_drift=0;
  
  track_traj_msg.pos_pts.clear();
  track_traj_msg.yaw_pts.clear();

  for(int i=0; i<10; i++)
  {
    double t_f = t_cur + 0.1*i;
    if (t_f < traj_duration_ && t_f >= 0.0)
    {
      pos = traj_[0].evaluateDeBoorT(t_f);
      if(t_f < traj_duration_-1.0)
      {
        end = traj_[0].evaluateDeBoorT(traj_duration_-0.9);
      // vel = traj_[1].evaluateDeBoorT(t_f);
      // vel[2] = 0;
      vel = end-pos;
      vel[2] = 0;
      if(vel.norm() > 0.1)
      {
        double yaw_tem = atan2(vel[1], vel[0]);
        if(yaw_tem+yaw_drift-yaw_last > 3.2)
        {
          yaw_drift -= 3.14159265*2;
        }
        else if(yaw_tem+yaw_drift-yaw_last < -3.2)
        {
          yaw_drift += 3.14159265*2;
        }
        yaw = yaw_tem + yaw_drift;
        yaw_last = yaw;
      }
      else
      {
        yaw = 1000;
      }
      }
      else
      {
        yaw = 1000;
      }

    }
    else if (t_f >= traj_duration_)
    {
      /* hover when finish traj_ */
      pos = traj_[0].evaluateDeBoorT(traj_duration_);
      yaw = 1000;
    }
    else
    {
      cout << "[Traj server]: invalid time." << endl;
    }
    geometry_msgs::Point pt;
    pt.x = pos(0);
    pt.y = pos(1);
    pt.z = pos(2);
    track_traj_msg.pos_pts.push_back(pt);
    track_traj_msg.yaw_pts.push_back(yaw);
  }
  track_traj_pub.publish(track_traj_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  // ros::NodeHandle node;
  ros::NodeHandle nh("~");

  ros::Subscriber bspline_sub = nh.subscribe("planning/bspline", 10, bsplineCallback);

  pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
  track_traj_pub = nh.advertise<px4_ctrl::track_traj>("/planning/track_traj", 1);

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);
  nh.param("traj_server/time_forward", time_forward_, -1.0);

  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  ros::spin();

  return 0;
}