#ifndef CMDVEL2ACKERMANN_H_
#define CMDVEL2ACKERMANN_H_

#include <map>
#include <vector>
#include <cmath>
#include "ros/ros.h"

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
#include "driverless_common/VehicleCtrlCmd.h"
#include "driverless_common/VehicleState.h"

class Cmdvel2Ackermann
{
public:
  Cmdvel2Ackermann();
  ~Cmdvel2Ackermann(){}

  void run();
private:
  void cmdvelCallback(const geometry_msgs::Twist::ConstPtr& cmd);
  void timerCallback(const ros::TimerEvent& event);
  ros::NodeHandle nh_;
  ros::Subscriber cmdvel_sub_;
  ros::Publisher vehicle_cmd_pub_;
  ros::Timer timer_;
  boost::mutex cmdvel_mutex_;

  ros::Time last_time_, current_time_; 
  float cmd_vel_x_, cmd_vel_y_, cmd_vel_th_; 
  float v_, v_kmh_, thf_, thf_degree_, r_; // thf_ is front wheel degree, r_ is truning radius
};




#endif
