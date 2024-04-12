#include "ros/ros.h"
#include "serial/cmd_vel2ackermann.h"
Cmdvel2Ackermann::Cmdvel2Ackermann(){
  ros::NodeHandle private_node("~");
}

void Cmdvel2Ackermann::cmdvelCallback(const geometry_msgs::Twist::ConstPtr& cmd){
  current_time_ = ros::Time::now();
  cmdvel_mutex_.lock();
  custom_msg::VehicleCtrlCmd vehicle_cmd;

  // 登记cmd_vel
  vx_ = cmd->linear.x;
  vy_ = cmd->linear.y;
  thz_ = cmd->angular.z;

  // solve the speed and front wheel truning degree
  v_ = sqrt(vx_ * vx_ + vy_ * vy_);
  r_ = v_ / thz_;
  v_kmh_ = v_ * 18 / 5;
  thf_ = atan((LR / r_) * (L / (L - LR)));

  vehicle_cmd.drive_mode = 0;
  vehicle_cmd.driverless = true;
  vehicle_cmd.gear = 1;
  vehicle_cmd.speed = 0.0;     // v_kmh_;  

  vehicle_cmd.brake_mode = 0;  
  vehicle_cmd.brake_pedel_aperture = 0; 
  vehicle_cmd.deceleration = 0; 
  vehicle_cmd.acceleration = 0;

  thf_degree_ = thf_ / M_PI * 180.0;

  // judge the roadwheel angle
  if (-30.0 < thf_degree_ && thf_ < 30.0){
  vehicle_cmd.roadwheel_angle = thf_degree_;
}  
  else{
  vehicle_cmd.roadwheel_angle = 0.0;
}
  ROS_WARN("pub angle is: %.2f, real angle is : %.2f", vehicle_cmd.roadwheel_angle, thf_degree_);
  vehicle_cmd.emergency_brake = false;
  vehicle_cmd.hand_brake = false;
  vehicle_cmd.turnlight_r = false;
  vehicle_cmd.turnlight_l = false;
  vehicle_cmd.low_beam = false;
  vehicle_cmd.high_beam = false;
  vehicle_cmd.horn = false;

  vehicle_cmd_pub_.publish(vehicle_cmd);

  // update time
  last_time_ = current_time_;
  cmdvel_mutex_.unlock();

}


void Cmdvel2Ackermann::run(){
  cmdvel_sub_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &Cmdvel2Ackermann::cmdvelCallback, this);
  vehicle_cmd_pub_ = nh_.advertise<custom_msg::VehicleCtrlCmd>("/vehicleCmdSet",1);
  ros::spin();
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "cmd_vel2ackermann");
  Cmdvel2Ackermann cmdvel_ackermann;
  cmdvel_ackermann.run();
  return 0;
}
