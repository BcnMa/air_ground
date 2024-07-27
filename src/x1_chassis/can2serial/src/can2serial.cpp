#include <can2serial/can2serial.h>
#include <cstring>

/*
2020.11.22 添加数据读取线程锁，读取线程退出后锁释放，当外部请求停止读取数据时，等待读取线程完全退出再删除相关对象，然后才能析构
修改动机: 当can分析仪意外停止工作时，用户程序期望释放历史can2serial 并重新实例can2serial，若没有上述机制，释放历史对象时将出现访问非法内存的错误(析构函数已经删除对象，而读取线程并未完全退出)，
这对于只需单次构造can2serial的程序无关紧要，因为在出现错误时客户程序也已经停止，但对于需要多次构造、析构的场景，上述的修改时十分必要！

2020.11.24 写串口之前先加写锁，获得锁后判断serial非空方可发送，因为当串口重配置时加了虽加了写锁，但若没有配置成功，锁释放后被写串口获得锁权限，此时将访问空指针而报错！
*/

static void printBuffer(const unsigned char *buf,int len)
{
	for(size_t i=0;i<len;i++){
		printf("%x	 ",buf[i]);
	}
	printf("\n");
}

Can2serial::Can2serial(uint8_t version) :
	inquire_filter_response_ptr_((inquireFilterResponsePkg_t *)data_buffer_),
	serial_is_bad_(false),
	newBaudrateResponse_(false),
	version_(version)
{
	serial_port_=NULL;
	reading_status_=false;
	buffer_index_ = 0;
	bytes_remaining_ = 0;
	baudRateCofigStatus_ =BaudrateCofig_None;
	filterClearStatus_ = false;
}

Can2serial::~Can2serial(){
	this->StopReading();
	serial_port_->close();
	if(serial_port_!=NULL){
		delete serial_port_;
		serial_port_ = NULL;
	}
}

bool Can2serial::configure_port(std::string port,int baud_rate){
	reading_status_ = false;	 //停止接收
	std::unique_lock<std::mutex> lock_recv(recv_thread_mutex_);	//等待接收线程退出并获得锁
	boost::mutex::scoped_lock lock_send(send_mutex_);	 //锁定发送，防止正在发送时删除指针

	try{
		if(serial_port_ != NULL)
			delete serial_port_;
		serial_port_ = new serial::Serial(port,baud_rate,serial::Timeout::simpleTimeout(10)); 

		if (!serial_port_->isOpen()){
			std::stringstream output;
			output << "Serial port: " << port << " failed to open." << std::endl;
			delete serial_port_;
			serial_port_ = NULL;
			return false;
		} 
		else{
				std::stringstream output;
				output << "Serial port: " << port << " opened successfully." << std::endl;
				std::cout << output.str() <<std::endl;
		}
		serial_port_->flush();
	} 
	catch (serial::IOException &e){
		std::stringstream output;
		output << "Error	" << port << ": " << e.what();
		std::cout << output.str() <<std::endl;
		serial_is_bad_ = true;
		return false;
	}
	serial_is_bad_ = false;

	return true;
}

void Can2serial::StartReading(){
	if (reading_status_)
		return;
		
	serial_port_->flushInput();
	// create thread to read from sensor
	reading_status_=true;
	read_thread_ptr_ = boost::shared_ptr<boost::thread >(new boost::thread(boost::bind(&Can2serial::ReadSerialPort, this)));
}

void Can2serial::StopReading(){
	reading_status_=false;
	recv_thread_mutex_.lock();//等待读取线程退出，此处才能获得锁
	
	recv_thread_mutex_.unlock();
}
 
void Can2serial::ReadSerialPort(){
	size_t len;
	recv_thread_mutex_.lock();
	// continuously read data from serial port
	while (reading_status_) 
	{
		usleep(1000);
		try{
			// read data
			if(serial_port_)
				len = serial_port_->read(buffer_, MAX_NOUT_SIZE);
			else
				len = 0;
		}
		catch (serial::SerialException &e){
			std::stringstream output;
			output << "Error read from serial: " << e.what();
			std::cout << output.str() <<std::endl;
			reading_status_ = false;
			serial_is_bad_ = true;
			throw(e);
			break ;
		}
		if(len==0) continue;
		
		// add data to the buffer to be parsed
		
		//printf("read length :%d\r\n",len);
		BufferIncomingData(buffer_, len);
		
	//	std::cout << std::dec<< "length:" <<len <<std::endl;
		/*for(size_t i=0;i<len;i++)
			printf("%x\t",buffer[i]);
		printf("\n\n");
		*/
	}
	
	recv_thread_mutex_.unlock();
}

			
void Can2serial::BufferIncomingData(unsigned char *message, unsigned int length)
{
	// add incoming data to buffer
	for (unsigned int ii=0; ii<length; ii++) 
	{// make sure bufIndex is not larger than buffer
		if(buffer_index_>=MAX_PKG_BUF_LEN)
		{
			buffer_index_ = 0;
				printf("Overflowed receive buffer. Buffer cleared.");
		}
		//std::cout << "bytes_remaining_...:" <<bytes_remaining_	<<std::endl;
		switch(buffer_index_)
		{
			case 0: //nothing
				if(message[ii]==HeaderByte0)
				{
					data_buffer_[buffer_index_++]=message[ii];
				}
				bytes_remaining_ = 0;
				break;
			case 1:
				if(message[ii]==HeaderByte1)
				{
					data_buffer_[buffer_index_++]=message[ii];
					bytes_remaining_ =2; //2 bytes pkgLen
				}
				else
				{
					buffer_index_=0;
					bytes_remaining_ =0;
				}
				break;
			case 2:
			case 3:
				data_buffer_[buffer_index_++]=message[ii];
				bytes_remaining_ --;
				if(bytes_remaining_==0)
				{
					package_len_ = (data_buffer_[buffer_index_-2] << 8)+data_buffer_[buffer_index_-1] ;
					bytes_remaining_ = package_len_;
					if(bytes_remaining_ > 16 || bytes_remaining_<2)
					{
						buffer_index_ = 0;
						break;
					}
					//printf("\t%x\t%x\n",data_buffer_[buffer_index_-2],data_buffer_[buffer_index_-1]);
				}
				break;
			default:
				data_buffer_[buffer_index_++]=message[ii];
				bytes_remaining_--;
				if(bytes_remaining_==0)
				{
				 //std::cout << "package_len_...:" <<package_len_ << std::endl;
					parse(data_buffer_,package_len_+4);
					buffer_index_ = 0;
				}
				break;
		}
	}	// end for
}

void Can2serial::parse(uint8_t * message,uint16_t length){
#if USE_CAN2SERIAL_DEBUG
	printf("recv: ");
	for(int i=0; i<length; ++i){
		printf("%02x	", message[i]);
	}
	std::cout << std::endl;
#endif

	if(generateCheckNum(message,length)!= message[length-1])
	{
		//printf("check failed !!!\r\n");
		return;
	}
	
	uint8_t cmd = data_buffer_[4];
	//printf("cmd:%x\n",cmd);
	switch(cmd){
		case CanMsgCmd:{
			std::shared_ptr<CanMsg_t> can_msg(new CanMsg_t);
				//if(0==(data_buffer_[12] >>5)) break;							 //data[1] esr_radar object status
																			//just used in esr_radar to filter invalid objects
			can_msg->type = data_buffer_[5];
			can_msg->ID = (data_buffer_[6]<<24)+(data_buffer_[7]<<16)+(data_buffer_[8]<<8)+(data_buffer_[9]);
			can_msg->len = data_buffer_[10];
				
				//for(size_t i=0;i<canMsg_.len;i++)
				//	canMsg_.data[i] = data_buffer_[11+i];
				
			memcpy(can_msg->data, data_buffer_+11, can_msg->len);
			putData(can_msg);

			//std::cout<< "putData:" << std::hex << can_msg->ID << std::endl;;
				break;
		}
		case FilterClearResponseCmd:
			filterClearStatus_ = !data_buffer_[5];
			
			break;
		
		case FilterSetResponseCmd:
			filterSetStatus_[data_buffer_[6]] = !data_buffer_[5];
			break;
			
		case BaudrateInquire0x13ResponseCmd:
			currentBaudrate_ = data_buffer_[6]*5;
			newBaudrateResponse_ = true;
			break;

		case BaudrateInquire0x15ResponseCmd:{
			uint32_t tpclk;
			if(version_ == 1) tpclk = 36; //mhz
			else if(version_ == 0) tpclk = 48; //mhz

			uint8_t bs1 = data_buffer_[7];
			uint8_t bs2 = data_buffer_[8];
			uint16_t brp = (data_buffer_[9] << 8) + data_buffer_[10];
			float tq = (brp + 1) / (tpclk*1000000.0);

			currentBaudrate_ = 1e-3 / (tq * (bs1 + bs2 + 3));
			newBaudrateResponse_ = true;
		#if USE_CAN2SERIAL_DEBUG
			std::cout << "currentBaudrate_: " << currentBaudrate_ << "kbps" << std::endl;
			std::cout << int(bs1) << " " << int(bs2) << " " << int(brp) << std::endl;
		#endif
			break;
		}
			
		case InquireFilterResponse:
			if((ntohl(inquire_filter_response_ptr_->filterID)>>21)!=0)
				printf("filter_num:%x\t filterID:%x\t filterMask:%x\r\n",
					inquire_filter_response_ptr_->filterNum,ntohl(inquire_filter_response_ptr_->filterID)>>21,
					ntohl(inquire_filter_response_ptr_->filterMask)>>21);
			break;
			
		//case FilterInquireResponseCmd;
			
		default:
			break;
	}
}

uint8_t Can2serial::generateCheckNum(const uint8_t* ptr,size_t len){
	uint8_t sum=0;

	for(int i=2; i<len-1 ; ++i){
		sum += ptr[i];
	}
	return sum;
}


bool Can2serial::sendCanMsg(const CanMsg_t &can_msg){
	//串口错误/未分配内存/内存别外部删除，直接返回
	if(serial_is_bad_ || serial_port_ == NULL)
		return false;
		
	uint16_t bufLen = 12 + can_msg.len;
	
	uint8_t *sendBuf = new uint8_t[bufLen];
	
	sendBuf[0] = 0x66;
	sendBuf[1] = 0xCC;
	
	uint16_t pkgLen = bufLen - 4;
	
	sendBuf[2] = pkgLen >>8;
	sendBuf[3] = pkgLen;
	
	sendBuf[4] = 0x30;
	sendBuf[5] = can_msg.type;
	sendBuf[6] = can_msg.ID >>24;
	sendBuf[7] = can_msg.ID >>16;
	sendBuf[8] = can_msg.ID >>8;
	sendBuf[9] = can_msg.ID;
	
	sendBuf[10] = can_msg.len;
	
	for( size_t i=0; i<can_msg.len; ++i)
		sendBuf[11+i] = can_msg.data[i];
		
	sendBuf[bufLen-1] = generateCheckNum(sendBuf,bufLen);
	
	//printBuffer(sendBuf,bufLen);

	try{
		boost::mutex::scoped_lock lock(send_mutex_);
		if(serial_port_) //防止指内存被外部释放而导致错误
			serial_port_->write(sendBuf,bufLen);
		else{
			delete [] sendBuf;
			return false;
		}
	}
	catch (serial::SerialException &e){
		std::stringstream output;
		output << "Error write to serial: " << e.what();
			std::cout << output.str() <<std::endl;
		delete [] sendBuf;
		serial_is_bad_ = true;
		throw(e);
		return false;
	}
	delete [] sendBuf;
	return true;
}

void Can2serial::sendCmd(uint8_t cmdId, const uint8_t *buf,uint8_t count){
	uint8_t send_pkg_len = count + 2;//cmd + checknum + count

	uint8_t * sendBuf = new uint8_t[send_pkg_len +4]; //add head bytes

	sendBuf[0] = 0x66;
	sendBuf[1] = 0xCC;
	sendBuf[2] = (uint8_t)(send_pkg_len >> 8) ;
	sendBuf[3] = (uint8_t)(send_pkg_len);
	sendBuf[4] = cmdId;
	memcpy(&sendBuf[5],buf,count);
	sendBuf[send_pkg_len+3] = generateCheckNum(sendBuf,send_pkg_len+4);
	if(serial_port_)
		serial_port_->write(sendBuf,send_pkg_len+4);
	
#if USE_CAN2SERIAL_DEBUG
	printf("send: ");
	for(int i =0;i<send_pkg_len+4;i++)
		printf("%02x	",sendBuf[i]);
	printf("\n");
#endif

	delete [] sendBuf;
}

bool Can2serial::configBaudrate(int baudrate){
	uint8_t buf[2];
	buf[0] = 0x01;//port1
	buf[1] = (baudrate/5)&0xff;
	sendCmd(0x12,buf,2);
	usleep(100000);
	sendCmd(0x12,buf,2);
	
	bool status=false;
	
	if(baudRateCofigStatus_==BaudRateCofig_Ok){
		status = true;
		baudRateCofigStatus_ = BaudrateCofig_None;
	}
	return status;
}

bool Can2serial::configBaudrate(uint8_t bs1, uint8_t bs2, uint16_t brp){
	uint8_t buf[5];
	buf[0] = 0x01;//port1
	buf[1] = bs1;
	buf[2] = bs2;
	buf[3] = (brp >> 8);
	buf[4] = brp;

	sendCmd(0x14,buf,5);
	
	usleep(100000);
	
	sendCmd(0x14,buf,5);
	
	bool status=false;
	
	if(baudRateCofigStatus_==BaudRateCofig_Ok)
	{
		status = true;
		baudRateCofigStatus_ = BaudrateCofig_None;
	}
	return status;
}

bool Can2serial::setCanFilter(uint8_t filterNum,int filterID,int filterMask,uint8_t filterMode){

	uint8_t buf[11];
	buf[0] = 0x01; //port
	buf[1] = filterNum;
	filterID <<= 21;
	filterMask <<=21;
	if(filterMode==0x00)
	{
		buf[2] = (filterID >> 24)&0xff;
		buf[3] = (filterID >>16)&0xff;
		buf[4] = 0x00;
		buf[5] = 0x00;
		
		buf[6] = (filterMask >> 24)&0xff;
		buf[7] = (filterMask >> 16)&0xff;
		buf[8] = 0x00;
		buf[9] = 0x00;
		
	}
	buf[10] = filterMode;
	sendCmd(0x18,buf,11);
	
	usleep(100000);
	
	bool status = false;
	if(filterSetStatus_[filterNum] == true)
	{
		status =true;
		filterSetStatus_[filterNum] = false;
	}
	
	return status;
}

bool Can2serial::setCanFilter_alone(uint8_t filterNum,int filterID,uint8_t filterMode){

	uint8_t buf[11];
	buf[0] = 0x01; //port
	buf[1] = filterNum;
	filterID <<= 21;

	buf[2] = (filterID >> 24)&0xff;
	buf[3] = (filterID >>16)&0xff;
	buf[4] = 0x00;
	buf[5] = 0x00;
		
	buf[6] = 0xff;
	buf[7] = 0xff;
	buf[8] = 0xff;
	buf[9] = 0xff;
		

	buf[10] = filterMode;
	sendCmd(0x18,buf,11);
	
	usleep(100000);
	
	sendCmd(0x18,buf,11);
	
	if(filterSetStatus_[filterNum]==true)
		return true;
	else
		return false;
}

bool Can2serial::clearCanFilter(uint8_t filterNum){
	uint8_t buf[2];
	buf[0] = 0x01;//port
	buf[1] = filterNum;
	sendCmd(0x19,buf,2);
	
	usleep(100000);
	
	sendCmd(0x19,buf,2);
	
	if(filterClearStatus_==true)
	{
		filterClearStatus_ =false;
		return true;
	}
	else
		return false;
	
}


int Can2serial::inquireBaudrate(uint8_t port)
{
	uint8_t cmd_type;
	if(version_ == 0) cmd_type = 0x13;
	else if(version_ == 1) cmd_type = 0x15;


	uint8_t buf[1];
	if(port==1)
		buf[0] = 0x01;
	else if(port==2)
		buf[0] = 0x02;
	else
		return -2;
	
	while(5){
		sendCmd(cmd_type,buf,1);
        usleep(200000);//200ms
		if(newBaudrateResponse_){
			newBaudrateResponse_ = false;
			return currentBaudrate_;
		}
	}

	return -1;
}

bool Can2serial::getCanMsg(CanMsg_t &msg){
	CanMsg_t::Ptr msg_ptr;
	bool ok = getData(msg_ptr);
	if(ok){
		msg = *msg_ptr;
	}
	return ok;
}


void Can2serial::inquireFilter(uint8_t filterNum ,uint8_t port){
	uint8_t buf[2];
	if(port==1)
		buf[0] = 0x01;
	else if(port==2)
		buf[0] = 0x02;
	else
		return ;
	buf[1] = filterNum;
		
	sendCmd(0x1D,buf,2);
	
	usleep(10000);
}

void Can2serial::showCanMsg(const CanMsg_t& msg, const std::string& prefix){
	if(!prefix.empty())
		printf("%s\t", prefix.c_str());
		
	printf("ID:0x%02x ",msg.ID);
	for(size_t i=0; i<msg.len; ++i)
		printf("%02x	",msg.data[i]);
	printf("\n");
}




