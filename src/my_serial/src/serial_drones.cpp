/* 
    sub data from UGV
    pub data from UAV
    mtx write in 20240407
*/

#include <math.h>
#include "serial/serial.h"
#include "vector"
#include "ros/ros.h"
#include <custom_msg/DronePerception.h>

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define PURPLE  "\033[35m"

class DroneSerial
{
public:
    DroneSerial(const std::string &port_path, const int &baud_rate)  
    {
        drone_serial_.setPort(port_path);
        drone_serial_.setBaudrate(baud_rate);
        time_ =serial::Timeout::simpleTimeout(2000);//超时等待
        drone_serial_.setTimeout(time_);
        drone_serial_.open();
        if(drone_serial_.isOpen()) ROS_INFO("Serial port is : %s, baud rate is : %d, open success!", port_path.c_str(), baud_rate);
        else ROS_ERROR("Serial port is %s:, open error!", port_path.c_str());
    }
    
    ~DroneSerial()
    {
        
    }
    
    void sendSerialData(const custom_msg::DronePerception &data)
    {
        visualization_msgs::MarkerArray empty_marker_array;
        custom_msg::DronePerception my_data_msg;
        my_data_msg.header = data.header;
        my_data_msg.vehicle_position = data.vehicle_position;
        my_data_msg.vehicle_attitude = data.vehicle_attitude;
        my_data_msg.obstacle_markers = data.obstacle_markers;

        uint32_t serial_size = ros::serialization::serializationLength(my_data_msg);
        boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
        ros::serialization::OStream stream(buffer.get(), serial_size);
        ros::serialization::serialize(stream, my_data_msg);
        
        buffer[serial_size - 1] = '\r';
        buffer[serial_size] = '\n';
        drone_serial_.write(buffer.get(), serial_size + 1);

        std::cout << "----------Drone send new message----------" << std::endl;
        std::cout << "Header: " << YELLOW << std::endl << data.header << RESET << std::endl;
        std::cout << "Vehicle Position: " << GREEN << std::endl << data.vehicle_position << RESET << std::endl;
        std::cout << "Vehicle Attitude: " << BLUE << std::endl << data.vehicle_attitude << RESET << std::endl;
        std::cout << "Obstacle Markers: " << PURPLE << std::endl;
        if (data.obstacle_markers == empty_marker_array) 
            std::cout << "No Obstacle!" << RESET << std::endl;        
        else
            std::cout << "Number of Obstacle Markers: " << data.obstacle_markers.markers.size() << RESET << std::endl;
        std::cout << std::endl;
    }
    
    void run()
    {
        ros::Rate loop_rate(10);
        while (ros::ok()) 
        {
            loop_rate.sleep();
            ros::spinOnce();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber perception_sub_ = nh_.subscribe("/drone_perception", 1000, &DroneSerial::sendSerialData, this);
    serial::Serial drone_serial_;
    serial::Timeout time_;
    int dataPublishNum = 0;
};

int main(int argc, char **argv)
{
  std::string port = "/dev/ttyUSB1";
  int baudRate = 115200;
  ros::init(argc, argv, "drone_serial");
  DroneSerial drone_serial(port, baudRate);
  drone_serial.run();
  return 0;
}
