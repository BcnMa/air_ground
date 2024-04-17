#include "ros/ros.h"
#include "serial/cmd_vel2ackermann.h"
Cmdvel2Ackermann::Cmdvel2Ackermann()
{
  ros::NodeHandle private_node("~");
}

void Cmdvel2Ackermann::cmdvelCallback(const geometry_msgs::Twist::ConstPtr& cmd)
{
  current_time_ = ros::Time::now();
  cmdvel_mutex_.lock();

  cmd_vel_x_ = cmd->linear.x;
  cmd_vel_y_ = cmd->linear.y;
  cmd_vel_th_ = cmd->angular.z;

  // 考虑到低速低转速，解算这里先不用
  // v_ = sqrt(cmd_vel_x_ * cmd_vel_x_ + cmd_vel_y_ * cmd_vel_y_);
  // r_ = v_ / cmd_vel_th_;
  // v_kmh_ = v_ * 18 / 5;
  // thf_ = atan((LR / r_) * (L / (L - LR)));
  // thf_degree_ = thf_ / M_PI * 180.0; 
  last_time_ = current_time_;
  cmdvel_mutex_.unlock();
}

/*
 * 这里发布车辆控制指令，有几个值需要注意：
 * 1. 速度speed，单位km/h
 * 2. 前轮转角gear，单位度
 * 3. 驱动模式gear，0为初始化，1为前进档，2为中位档，3为后退档，11为停车
*/
void Cmdvel2Ackermann::timerCallback(const ros::TimerEvent& event)
{
  cmdvel_mutex_.lock();
  //custom_msg::VehicleCtrlCmd vehicle_cmd;

  driverless_common::VehicleCtrlCmd vehicle_cmd;
  if (cmd_vel_x_ > 0.1)
  {
    vehicle_cmd.speed = cmd_vel_x_ * 18 / 5; 
    if (vehicle_cmd.speed > 5) vehicle_cmd.speed = 5; 
    vehicle_cmd.gear = 1; 
  }
  else if (cmd_vel_x_ < -0.1)
  {
    vehicle_cmd.speed = cmd_vel_x_ * 18 / 5;
    if (vehicle_cmd.speed < -5) vehicle_cmd.speed = -5;  
    vehicle_cmd.gear = 3; 
  }
  else
  {
    vehicle_cmd.speed = 0.0;
    vehicle_cmd.gear = 2;
  }

  if (cmd_vel_th_ > M_PI/100)
  {
    vehicle_cmd.roadwheel_angle = cmd_vel_th_ * 180 / M_PI;
    if (vehicle_cmd.roadwheel_angle > 20) vehicle_cmd.roadwheel_angle = 20;
  }
  else if (cmd_vel_th_ < -M_PI/100)
  {
    vehicle_cmd.roadwheel_angle = cmd_vel_th_ * 180 / M_PI;
    if (vehicle_cmd.roadwheel_angle < -20) vehicle_cmd.roadwheel_angle = -20;
  }
  else
  {
    vehicle_cmd.roadwheel_angle = 0.0;
  }
  
  if (vehicle_cmd.speed == 0.0) vehicle_cmd.roadwheel_angle = 0.0;

  vehicle_cmd.drive_mode = 0;
  vehicle_cmd.driverless = true;

  vehicle_cmd.brake_mode = 0;  
  vehicle_cmd.brake_pedel_aperture = 0; 
  vehicle_cmd.deceleration = 0; 
  vehicle_cmd.acceleration = 0;
  vehicle_cmd.emergency_brake = false;
  vehicle_cmd.hand_brake = false;
  vehicle_cmd.turnlight_r = false;
  vehicle_cmd.turnlight_l = false;
  vehicle_cmd.low_beam = false;
  vehicle_cmd.high_beam = false;
  vehicle_cmd.horn = false;

  std::cout << "Now X1 is in gear mod: " << RED << static_cast<unsigned int>(vehicle_cmd.gear) << RESET 
  << ", Speed: " << YELLOW << vehicle_cmd.speed << RESET 
  << ", Roadwheel angle: " << GREEN << vehicle_cmd.roadwheel_angle << RESET << std::endl;
  vehicle_cmd_pub_.publish(vehicle_cmd);
  cmdvel_mutex_.unlock();
}

void Cmdvel2Ackermann::run()
{
  cmdvel_sub_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &Cmdvel2Ackermann::cmdvelCallback, this);
  //vehicle_cmd_pub_ = nh_.advertise<custom_msg::VehicleCtrlCmd>("/vehicleCmdSet",1);
  timer_ = nh_.createTimer(ros::Duration(0.01), &Cmdvel2Ackermann::timerCallback, this);
  vehicle_cmd_pub_ = nh_.advertise<driverless_common::VehicleCtrlCmd>("/vehicleCmdSet",1);
  ros::spin();
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "cmd_vel2ackermann");
  Cmdvel2Ackermann cmdvel_ackermann;
  cmdvel_ackermann.run();
  return 0;
}
