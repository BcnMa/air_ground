#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <can2serial/can2serial.h>

#define LOOP_TEST 0

class CanMsgTransmitter{
public:
    CanMsgTransmitter(){
        can2serial_ = new Can2serial();
    }
    bool init(){
        ros::NodeHandle nh, nh_private("~");
        std::string port = nh_private.param<std::string>("usb_port", "/dev/ttyUSB0");
        int baudrate = nh_private.param<int>("baud_rate", 500);

        if(can2serial_->configure_port(port)){
            can2serial_->configBaudrate(baudrate);
            can2serial_->StartReading();  
            can2serial_->registerCallback(&CanMsgTransmitter::receiveCanMsgCallback, this, 100);
        }else{
            return false;
        }
        sub_ = nh.subscribe("/send_can_msg", 100, &CanMsgTransmitter::sendCanMsgCallback, this);
        pub_ = nh.advertise<can_msgs::Frame>("/receive_can_msg", 100);
    }

    void receiveCanMsgCallback(const CanMsg::Ptr& can_msg){
        can_msgs::Frame frame;
        frame.header.stamp = ros::Time::now();
        frame.id = can_msg->ID;
        for(int i=0; i<can_msg->len; ++i){
            frame.data[i] = can_msg->data[i];
        }
        frame.dlc = can_msg->len;
        switch(can_msg->type){
            case Can2serial::EXT_DATA_FRAME:
                frame.is_extended = true;
                break;
            case Can2serial::STD_REMOTE_FRAME:
                frame.is_rtr = true;
                break;
            case Can2serial::STD_DATA_FRAME:
                break;
            default:
                ;
        }
        pub_.publish(frame);

        #if LOOP_TEST //回环测试, 收到的消息再发送出去
        CanMsg_t w_can_msg = *can_msg;
        w_can_msg.ID += 50; //发送前修改为新ID
        can2serial_->sendCanMsg(w_can_msg);
        #endif 
    }

    void sendCanMsgCallback(const can_msgs::Frame::ConstPtr& msg){
        CanMsg_t w_can_msg;
        for(int i=0; i<msg->dlc; ++i){
            w_can_msg.data[i] = msg->data[i];
        }
        w_can_msg.ID = msg->id;
        w_can_msg.len = msg->dlc;
        if(msg->is_rtr){
            w_can_msg.type = Can2serial::STD_REMOTE_FRAME;
        }else if(msg->is_error){

        }else if(msg->is_extended){
            w_can_msg.type = Can2serial::EXT_DATA_FRAME;
        }else{
             w_can_msg.type = Can2serial::STD_DATA_FRAME;
        }
        can2serial_->sendCanMsg(w_can_msg);
    }

private:
    ros::Subscriber sub_;
    ros::Publisher pub_;
    Can2serial *can2serial_;
};


int main(int argc, char*argv[]){
    ros::init(argc, argv, "canmsg_transmitter_node");
    CanMsgTransmitter app;
    if(app.init()){
        ros::spin();
    }

	return 0;
}
