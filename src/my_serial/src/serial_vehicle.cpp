/* 
    sub data from UAV
    pub data from UGV
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

class VehicleSerial
{
public:
    VehicleSerial(const std::string &port_path, const int &baud_rate)  
    {
        vehicle_serial_.setPort(port_path);
        vehicle_serial_.setBaudrate(baud_rate);
        time_ =serial::Timeout::simpleTimeout(2000);//超时等待
        vehicle_serial_.setTimeout(time_);
        vehicle_serial_.open();
        if(vehicle_serial_.isOpen()) ROS_INFO("Serial port is : %s, baud rate is : %d, open success!", port_path.c_str(), baud_rate);
        else ROS_ERROR("Serial port is %s:, open error!", port_path.c_str());
    }
    
    ~VehicleSerial()
    {
        
    }
    
    void sendSerialData()
    {
        
    }

    /*
    * 读取串口数据
    */ 
    void readSerialData()
    {
        if (vehicle_serial_.available())
        {
            visualization_msgs::MarkerArray empty_marker_array;
            std::string data = vehicle_serial_.read(vehicle_serial_.available());
            std::cout << "----------Vehicle received new message----------" << std::endl;
            std::cout << ("Received data: %s", data.c_str()) << std::endl;

            size_t data_length = data.length();
            if (data_length > 0)
            {
                data.erase(std::remove(data.begin(), data.end(), '\r'), data.end());
                std::vector<uint8_t> buffer(data.begin(), data.end());
                custom_msg::DronePerception received_msg;
                uint32_t serial_size = buffer.size();
                ros::serialization::IStream stream(buffer.data(), serial_size);
                ros::serialization::deserialize(stream, received_msg);

                // std::cout << "----------Drone send new message----------" << std::endl;
                std::cout << "Header: " << YELLOW << std::endl << received_msg.header << RESET << std::endl;
                std::cout << "Vehicle Position: " << GREEN << std::endl << received_msg.vehicle_position << RESET << std::endl;
                std::cout << "Vehicle Attitude: " << BLUE << std::endl << received_msg.vehicle_attitude << RESET << std::endl;
                std::cout << "Obstacle Markers: " << PURPLE << std::endl;
                if (received_msg.obstacle_markers == empty_marker_array) 
                    std::cout << "No Obstacle!" << RESET << std::endl;        
                else
                    std::cout << "Number of Obstacle Markers: " << received_msg.obstacle_markers.markers.size() << RESET << std::endl;
                std::cout << std::endl;

                buffer.clear();
            }
        }
    }

    
    void run()
    {
        ros::Rate loop_rate(100);
        while (ros::ok()) 
        {
            readSerialData();
            loop_rate.sleep();
            ros::spinOnce();
        }
    }

private:
    ros::NodeHandle nh_;
    serial::Serial vehicle_serial_;
    serial::Timeout time_;
    int dataPublishNum = 0;
};

int main(int argc, char **argv)
{
    std::string port = "/dev/ttyUSB0";
    int baud_rate = 115200;
    ros::init(argc, argv, "vehicle_serial");
    VehicleSerial vehicle_serial(port, baud_rate);
    vehicle_serial.run();
    return 0;
}
