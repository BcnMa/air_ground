/* 
    pub real /odom
    mtx write in 20240426
*/
#include <map>
#include <vector>
#include "ros/ros.h"
#include <math.h>

#include "vector"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "tf/tf.h"
#include "geometry_msgs/Polygon.h"
#include "nav_msgs/OccupancyGrid.h"

#include "custom_msg/VehicleCtrlCmd.h"
#include "custom_msg/VehicleState.h"

#define LR          0.5   // rear suspension deistance
#define LF          0.5   // front suspension deistance
#define L           1.0   // the wheelbase of x1

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define PURPLE  "\033[35m"

// 以下注释仅在NUC上可运行
// #include "driverless_common/VehicleCtrlCmd.h"
// #include "driverless_common/VehicleState.h"

class PubOdom
{
public:
  PubOdom()
  {
    nh_.param("odom_frame", odom_frame_, std::string("odom"));
    nh_.param("base_link_frame", base_link_frame_, std::string("base_link"));  
    last_time_ = ros::Time::now();     
  }
  ~PubOdom(){}

  void stateReceived(const driverless_common::VehicleState& state)
  // void stateReceived(const custom_msg::VehicleState& state)
  {
    state_mutex_.lock();
    vehicle_speed_ = state.speed;
    roadwheel_angle_ = state.roadwheel_angle;
    state_mutex_.unlock();
  }

  void pubOdomCallback(const ros::TimerEvent &event)
  {
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
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(odom_th_);

    tf_broadcaster_.sendTransform(odom_trans);

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

  void updateOdometry()
  {
    double dt = (current_time_ - last_time_).toSec();

    state_mutex_.lock();
    // 根据我的变量，写一个简单的里程计
    double v = vehicle_speed_ / 3.6; // km/h -> m/s
    double w = roadwheel_angle_ * M_PI / 180.0; // degree -> rad

    double dth = tan(w) * v;
    double delta_x = (v * cos(w)) * dt;
    double delta_y = (v * sin(w)) * dt;
    cur_v_linear_ = v;
    cur_v_theta_ = dth/dt;
    odom_x_ += delta_x;
    odom_y_ += delta_y;
    odom_th_ += delta_th;
  }



  void run()
  {
    // state_sub_ = nh_.subscribe<custom_msg::VehicleState>("vehicleStateSet", 10, &PubOdom::stateReceived, this);
    state_sub_ = nh_.subscribe<driverless_common::VehicleState>("vehicleStateSet", 10, &PubOdom::stateReceived, this);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1000);
    pub_odom_timer_ = nh_.createTimer(ros::Duration(1.0/rate_), &PubOdom::pubOdomCallback, this);
    ros::spin();
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber state_sub_;
  ros::Publisher odom_pub_;
  ros::Timer pub_odom_timer_;
  ros::Time last_time_, current_time_, last_cb_;
  int rate_ = 100;

  // 变量前轮转角和速度
  double vehicle_speed_ = 0.0;      // km/h
  double roadwheel_angle_ = 0.0;    // degree
  double odom_x_ = 0.0;
  double odom_y_ = 0.0;
  double odom_th_= 0.0;
  double cur_v_linear_ = 0.0;
  double cur_v_theta_ = 0.0;
  boost::mutex state_mutex_;

  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;

  std::string odom_frame_, base_link_frame_;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pub_odom");
  PubOdom pub_odom;
  pub_odom.run();
  return 0;
}
