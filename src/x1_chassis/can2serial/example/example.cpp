#include <can2serial/can2serial.h>
#include <ros/ros.h>
#include <chrono>

class Example{
private:
	void canMsgCallback(const CanMsg_t::Ptr& msg){

		auto timeNow= std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
		uint64_t time = timeNow.count(); //时间戳

		Can2serial::showCanMsg(*msg, std::to_string(time/1000.0));
		
		//usleep(5000);
	}

	std::string port_;
    int baudrate_;
	Can2serial *can2serial_;

public:
    Example(const std::string& port, int baudrate=500){
		port_ = port;
        baudrate_ = baudrate;
	}

	bool init(){
		can2serial_ = new Can2serial(); 
		                 
		if(!can2serial_->configure_port(port_)){         //配置串口
			return false;
		}

		can2serial_->StartReading();                 //使能数据读取，包括can数据以及can分析仪应答数据
																								 //使能后才可接收到应答数据

		can2serial_->clearCanFilter();               //清除can滤波器
        can2serial_->configBaudrate(baudrate_);            //配置can通讯波特率

		//can2serial_->configBaudrate(13, 0, 5);     //配置can通讯波特率(方式2, 涉及芯片频率,不建议使用)

		//can2serial_->setCanFilter_alone(0x01,0x4E0); // 设置单独滤波器
		//can2serial_->setCanFilter(0x02,0x4E0,0x7ff); // 设置范围滤波器 id, mask
		//can2serial_->setCanFilter(0x03,0x500,0x7f0);
		
		//can2serial_->inquireFilter(0x01); // 获取序号为1的can滤波器
		//can2serial_->inquireFilter(0x02);

		int baudrate = can2serial_->inquireBaudrate();
		if(baudrate > 0){
			std::cout << "baudrate: " << baudrate << std::endl;
		}else{
			std::cout << "inquireBaudrate failed!" << std::endl;
		}

		return true;
	}

	void runMode1(){
		// 方式1: 绑定回调函数获取can消息
		can2serial_->registerCallback(&Example::canMsgCallback, this, 10000);

		while(1){
			// do something
			usleep(500000);
		}
	}

	void runMode2(){
		 //方式2: while循环取消息

		 int id = 0x01;
		 CanMsg_t can_msg_r, can_msg_w;
		 std::shared_ptr<CanMsg_t> can_msg_ptr;
		 while(true){
		   bool ok;

			 // 读取方式1
		   ok = can2serial_->getCanMsg(can_msg_r);
			 if(ok){
					Can2serial::showCanMsg(can_msg_r, "read mode1");
			 }
			 usleep(1000);
			 // 读取方式2
		   ok = can2serial_->getData(can_msg_ptr);
			 if(ok){
					Can2serial::showCanMsg(*can_msg_ptr, "read mode2");
			 }

			
			 // 发送CAN消息
			 can_msg_w.resetData();
		   can_msg_w.ID = id++ ;
			 can_msg_w.len = 8;
			 for(int i = 0; i<can_msg_w.len; i++){
					can_msg_w.data[i] = i;
			 }
		   can2serial_->sendCanMsg(can_msg_w); 
		   usleep(1000);

		 }
	}
};


int main(int argc, char*argv[])
{
	std::string port="/dev/ttyUSB0";
    int baundrate = 500;
	if(argc > 1){
		port = argv[1];
    }
    if(argc > 2){
        baundrate = atoi(argv[2]);
    }

    std::cout << "set port: " << port << ", baundrate: " << baundrate << std::endl;

    Example example(port, baundrate);
	if(example.init()){
		example.runMode1();
		// example.runMode2();
	}

	return 0;
}
