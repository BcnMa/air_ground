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
    bool readSerialData()
    {
        if (vehicle_serial_.available())
        {
            std::string data = vehicle_serial_.read(vehicle_serial_.available());
            std::cout << "----------Vehicle received new message----------" << std::endl;
            std::cout << "Received Message: " << data << std::endl;

            data_buffer_ += data;  

            size_t start_pos = data_buffer_.find("start");
            size_t end_pos = data_buffer_.find("end");

            if (start_pos != std::string::npos && end_pos != std::string::npos && start_pos < end_pos)
            {
                std::string message = data_buffer_.substr(start_pos, end_pos + 3 - start_pos);  

                std::istringstream iss(message);
                std::string header;
                int num_obstacle_markers;

                iss >> header;  

                if (header != "start")
                {
                    std::cout << RED << "Header is not correct!" << RESET << std::endl;
                    return false;
                }

                iss >> num_obstacle_markers;

                custom_msg::DronePerception received_msg;
                received_msg.header.stamp = ros::Time::now();
                iss >> received_msg.header.seq;
                iss >> received_msg.header.frame_id;

                iss >> received_msg.vehicle_position.x;
                iss >> received_msg.vehicle_position.y;
                iss >> received_msg.vehicle_position.z;

                iss >> received_msg.vehicle_attitude.x;
                iss >> received_msg.vehicle_attitude.y;
                iss >> received_msg.vehicle_attitude.z;
                iss >> received_msg.vehicle_attitude.w;

                std::cout << "Header: " << YELLOW << std::endl << received_msg.header << RESET << std::endl;
                std::cout << "Vehicle Position: " << GREEN << std::endl << received_msg.vehicle_position << RESET << std::endl;
                std::cout << "Vehicle Attitude: " << BLUE << std::endl << received_msg.vehicle_attitude << RESET << std::endl;

                if (num_obstacle_markers == 0)
                    std::cout << "No Obstacle!" << std::endl;
                else
                    std::cout << "Number of Obstacle Markers: " << PURPLE << num_obstacle_markers << std::endl;

                std::cout << std::endl;

                // 从缓冲区中移除已处理的消息
                data_buffer_ = data_buffer_.substr(end_pos + 3);
                return true;
            }
            else
            {
                std::cout << RED << "Data is not complete " << RESET << std::endl;
                return false;
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
    std::string data_buffer_;  
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
