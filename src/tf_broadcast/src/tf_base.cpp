/* 
    sub real /cmd_vel
    pub virtual /odom
    mtx write in 20240304
*/

#include <math.h>

#include "ros/ros.h"
#include "vector"
#include "nav_msgs/Odometry.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf_broadcast/tf_base.h"

TfBase::TfBase() : cur_v_linear_(0.0), cur_v_theta_(0.0), tar_v_linear_(0.0), tar_v_theta_(0.0),
                  real_x_(0.0), real_y_(0.0), real_th_(0.0), odom_x_(0.0), odom_y_(0.0), odom_th_(0.0),
                  tf_listener_(tf_buffer_) {
  ros::NodeHandle private_node("~");
  private_node.param("odom_frame", odom_frame_, std::string("odom"));
  private_node.param("base_link_frame", base_link_frame_, std::string("base_link"));

  private_node.param("max_a_linear", max_a_linear_, 999.0);
  private_node.param("max_a_theta", max_a_theta_, 999.0);

  private_node.param("max_v_linear", max_v_linear_, 1.0);
  private_node.param("max_v_theta", max_v_theta_, 1.57);

  private_node.param("rate", rate_, 30);

  private_node.param("is_tf_broadcast", is_tf_broadcast_, true);

  last_time_ = ros::Time::now();
}

void TfBase::cmdReceived(const geometry_msgs::Twist::ConstPtr &cmd) {
  cmd_mutex_.lock();
  tar_v_linear_ = cmd->linear.x;
  tar_v_theta_ = cmd->angular.z;
  cmd_mutex_.unlock();
}

void TfBase::updateOdometry() {
  double dt = (current_time_ - last_time_).toSec();
  double ds, dth, ns, nth;
  if (tar_v_linear_ > cur_v_linear_) {
    cur_v_linear_ = std::min(std::min(tar_v_linear_, max_v_linear_), cur_v_linear_ + max_a_linear_ * dt);
  } else if (tar_v_linear_ < cur_v_linear_) {
    cur_v_linear_ = std::max(std::max(-max_v_linear_, tar_v_linear_), cur_v_linear_ - max_a_linear_ * dt);
  } else {
    cur_v_linear_ = tar_v_linear_;
  }
  ds = cur_v_linear_ * dt;
  ns = ds == 0 ? 0 : cur_v_linear_ * dt;

  if (tar_v_theta_ > cur_v_theta_) {
    cur_v_theta_ = std::min(std::min(tar_v_theta_, max_v_theta_), cur_v_theta_ + max_a_theta_ * dt);
  } else if (tar_v_theta_ < cur_v_theta_) {
    cur_v_theta_ = std::max(std::max(-max_v_theta_, tar_v_theta_), cur_v_theta_ - max_a_theta_ * dt);
  } else {
    cur_v_theta_ = tar_v_theta_;
  }
  dth = cur_v_theta_ * dt;
  nth = dth == 0 ? 0 : cur_v_theta_ * dt;

  real_th_ += dth;
  real_x_ += ds * cos(real_th_);
  real_y_ += ds * sin(real_th_);

  odom_th_ += nth;
  odom_x_ += ns * cos(odom_th_);
  odom_y_ += ns * sin(odom_th_);
}

void TfBase::pubOdomCallback(const ros::TimerEvent &event) {
  current_time_ = ros::Time::now();
  updateOdometry();

  // tf : odom --> base_link
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time_;
  odom_trans.header.frame_id = odom_frame_;
  odom_trans.child_frame_id = base_link_frame_;
  odom_trans.transform.translation.x = odom_x_;
  odom_trans.transform.translation.y = odom_y_;
  odom_trans.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, odom_th_);
  odom_trans.transform.rotation = tf2::toMsg(q);
  if (is_tf_broadcast_) {
    tf_broadcaster_.sendTransform(odom_trans);
  }

  // publish topic
  nav_msgs::Odometry odom_msg;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_link_frame_;
  odom_msg.header.stamp = current_time_;
  odom_msg.pose.pose.position.x = odom_x_;
  odom_msg.pose.pose.position.y = odom_y_;
  odom_msg.pose.pose.position.z = 0;
  odom_msg.pose.pose.orientation.x = odom_trans.transform.rotation.x;
  odom_msg.pose.pose.orientation.y = odom_trans.transform.rotation.y;
  odom_msg.pose.pose.orientation.z = odom_trans.transform.rotation.z;
  odom_msg.pose.pose.orientation.w = odom_trans.transform.rotation.w;
  odom_msg.twist.twist.linear.x = cur_v_linear_;
  odom_msg.twist.twist.angular.z = cur_v_theta_;
  odom_pub_.publish(odom_msg);
  // update time
  last_time_ = current_time_;
}

void TfBase::run() {
  cmd_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &TfBase::cmdReceived, this);
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 10);
  pub_odom_timer_ = nh_.createTimer(ros::Duration(1.0 / rate_), &TfBase::pubOdomCallback, this);
  ros::spin();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "simulation_base_node");
  TfBase tf_base;
  tf_base.run();
  return 0;
}
