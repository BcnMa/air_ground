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


class Cmdvel2Ackermann
{
public:
  Cmdvel2Ackermann();
  ~Cmdvel2Ackermann(){}

  void run();
private:
  void cmdvelCallback(const geometry_msgs::Twist::ConstPtr& cmd);

  ros::NodeHandle nh_;
  ros::Subscriber cmdvel_sub_;
  ros::Publisher vehicle_cmd_pub_;

  boost::mutex cmdvel_mutex_;

  ros::Time last_time_, current_time_; 
  float vx_, vy_, thz_; 
  float v_, v_kmh_, thf_, thf_degree_, r_; // thf_ is front wheel degree, r_ is truning radius
};




#endif
