#ifndef TF_BASE_H_
#define TF_BASE_H_

#include "ros/ros.h"

#include "geometry_msgs/Twist.h"

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"


class TfBase {
public:
  TfBase();
  ~TfBase(){}

  void run();
private:
  void cmdReceived(const geometry_msgs::Twist::ConstPtr& cmd);
  void pubOdomCallback(const ros::TimerEvent& event);

  void updateOdometry();

  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;

  ros::NodeHandle nh_;
  ros::Subscriber cmd_sub_;
  ros::Publisher odom_pub_;
  ros::Timer pub_odom_timer_;

  boost::mutex cmd_mutex_;

  ros::Time last_time_, current_time_, last_cb_;

  std::string map_frame_, odom_frame_, base_link_frame_;

  int rate_;
  double max_a_linear_, max_a_theta_;
  double max_v_linear_, max_v_theta_;
  double cur_v_linear_, cur_v_theta_;
  double tar_v_linear_, tar_v_theta_;
  double real_x_, real_y_, real_th_;
  double odom_x_, odom_y_, odom_th_;
  bool   is_tf_broadcast_;
};


#endif
