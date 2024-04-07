/* 
    sub data from UGV
    pub data from UAV
    mtx write in 20240407
*/

#include <math.h>
#include "serial/serial.h"
#include "vector"
#include "ros/ros.h"

class Drone2Vehicle
{
public:
    Drone2Vehicle(const std::string &port_path, const int &baud_rate)  
    {
        _droneSerial.setPort(port_path);
        _droneSerial.setBaudrate(baud_rate);
        _time =serial::Timeout::simpleTimeout(2000);//超时等待
        _droneSerial.setTimeout(_time);
        _droneSerial.open();
        if(_droneSerial.isOpen()) ROS_INFO("Serial port is : %s, baud rate is : %d, open success!", port_path.c_str(), baud_rate);
        else ROS_ERROR("Serial port is %s:, open error!", port_path.c_str());
    }
    
    ~Drone2Vehicle()
    {
        
    }
    
    void run()
    {
        while()
    }

private:
    serial::Serial _droneSerial;
    serial::Timeout _time;
};

int main(int argc, char **argv)
{
  std::string port = "/dev/ttyUSB0";
  int baud_rate = 115200;
  ros::init(argc, argv, "drone2vehicle");
  Drone2Vehicle drone_vehicle(port, baud_rate);
  //Drone2Vehicle drone_vehicle;
  return 0;
}
