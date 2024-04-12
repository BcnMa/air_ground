/* 
    sub data from perception
    pub data from drone serial
    mtx write in 20240410
*/

#include <math.h>
#include "serial/serial.h"
#include "vector"
#include "ros/ros.h"
#include <std_msgs/Header.h>
#include <custom_msg/DronePerception.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/MarkerArray.h>

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define PURPLE  "\033[35m"

class pubDronePerception
{
public:
    pubDronePerception( )  
    {
        // 设定初始化位姿
        vehicle_position_.x = 0.0;
        vehicle_position_.y = 0.0;
        vehicle_position_.z = 0.0;
    
        vehicle_attitude_.x = 0.0;
        vehicle_attitude_.y = 0.0;
        vehicle_attitude_.z = 0.0;
        vehicle_attitude_.w = 1.0;
    }
    
    ~pubDronePerception()
    {
        
    }
    
    void pubDronePerceptionMsg()
    {
        custom_msg::DronePerception drone_perception;
        
        // 消息头
        std_msgs::Header header;
        header.seq = dataPublishNum++;
        header.stamp = ros::Time::now();
        header.frame_id = "drone";
        drone_perception.header = header;
        
        // 无人机视角下无人车的位姿
        drone_perception.vehicle_position = vehicle_position_;
        drone_perception.vehicle_attitude = vehicle_attitude_;
        
        // 视觉获取的障碍物信息
        visualization_msgs::MarkerArray empty_marker_array;
        drone_perception.obstacle_markers = empty_marker_array;
        for (int i = 0; i < 10; ++i) 
        {
            visualization_msgs::Marker marker;
            marker.pose.position.x = i;
            marker.pose.position.y = i;
            marker.pose.position.z = i;
            marker.id = i; 
            drone_perception.obstacle_markers.markers.push_back(marker);
        }
        // 发布
        drone_perception_pub_.publish(drone_perception);
        // Debug

        std::cout << "----------Perception----------" << std::endl;
        std::cout << "Header: " << YELLOW << std::endl << header << RESET << std::endl;
        std::cout << "Vehicle Position: " << GREEN << std::endl << drone_perception.vehicle_position << RESET << std::endl;
        std::cout << "Vehicle Attitude: " << BLUE << std::endl << drone_perception.vehicle_attitude << RESET << std::endl;
        std::cout << "Obstacle Markers: " << PURPLE << std::endl;
        if (drone_perception.obstacle_markers == empty_marker_array) 
            std::cout << "No Obstacle!" << RESET << std::endl;        
        else
            std::cout << "Number of Obstacle Markers: " << drone_perception.obstacle_markers.markers.size() << RESET << std::endl;
        std::cout << std::endl;
    }
    
    void run()
    {
        drone_perception_pub_ = nh_.advertise<custom_msg::DronePerception>("/drone_perception",10);
        ros::Rate loop_rate(10);
        while (ros::ok()) 
        {
            ROS_INFO("Node is running...");
            pubDronePerceptionMsg();
            loop_rate.sleep();
            ros::spinOnce();
        }
    }

private:
    int dataPublishNum = 0;
    ros::NodeHandle nh_;
    ros::Subscriber cmdvel_sub_;
    ros::Publisher drone_perception_pub_;
    geometry_msgs::Point vehicle_position_;  
    geometry_msgs::Quaternion vehicle_attitude_;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pub_perception");
  pubDronePerception pubPerception;
  pubPerception.run();
  return 0;
}
