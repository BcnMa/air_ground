#ifndef CAN_2_SERIAL_H__
#define CAN_2_SERIAL_H__

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/circular_buffer.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>

#include "serial/serial.h"
#include <vector>
#include <iostream>
#include <string>
#include <mutex>
#include <arpa/inet.h>

#define USE_CAN2SERIAL_DEBUG 0

#define MAX_NOUT_SIZE  2000  //read from serial per time
#define MAX_MSG_BUF_SIZE 200  //complete can msg max capacity  ring storage area
#define MAX_PKG_BUF_LEN 50   // can2serial max pkg len  >20 to avoid data overflow

#ifdef _MSC_VER // using MSVC
	#define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#else
	#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif

typedef struct CanMsg
{
	using Ptr = std::shared_ptr<CanMsg>;
	using ConstPtr = std::shared_ptr<const CanMsg>;
	CanMsg(){
		len = 8;
		type = 0x03; //stdFrame
	}
	void resetData(){
		memset(data, 0, len);
	}
  uint32_t ID;
	uint8_t len;
	uint8_t data[8];
	uint8_t type; 
}CanMsg_t;

PACK(
struct stdCanMsgPkg_t {
	uint8_t header1; //66
	uint8_t header2; //cc
	uint16_t pkg_len;
	uint8_t pkg_cmd; //B1 stdCanMsg
	uint8_t type; //frame type
	uint32_t frame_id;
	uint8_t frame_len;
	uint8_t data[8];
	uint8_t checknum;
});

PACK(
struct inquireFilterResponsePkg_t {
	uint8_t header1; //66
	uint8_t header2; //cc
	uint16_t pkg_len;
	uint8_t pkg_cmd; //9D inquire Filter Response 
	uint8_t result;
	uint8_t portNum;
	uint8_t filterNum;
	uint32_t filterID;
	uint32_t filterMask;
	uint8_t filterMode;
	uint8_t checknum;
});


/* @brief 数据消费者类
 * 生产者将数据通过putData函数添加进数据队列
 * 消费者线程调用回调函数
 */
template<class DataType>
class DataConsumer{
  typedef void callback_t(const DataType&);
public:
  DataConsumer(size_t queue_size=1000){
		mUseCallback = false;
    mBuffer.set_capacity(queue_size);
    // mTempBuffer.set_capacity(queue_size);
    mBufferMaxSize = queue_size;
  }
  ~DataConsumer(){
    running = false;
  }

private:
  void consumerThread(){
    /* 消费者线程, 当数据队列为空时, 线程挂起, 当新数据到达时线程被唤醒
     * 通过swap方法将队列消息全部取出
     * 然后调用回调函数处理数据
     */
    while(running){
      std::unique_lock<std::mutex> lck(mMutex);
      if(mBuffer.empty()){
        mCv.wait(lck);
      }

      mBuffer.swap(mTempBuffer);
      lck.unlock();

      for(const DataType& data : mTempBuffer){
        mCallback(data);
      }
      mTempBuffer.clear();
    }
  }

  void _registerCallback(boost::function<callback_t> fp, size_t queue_size){
    /* 内置注册回调函数方法
     * 重新设置队列大小, 保存用户回调函数指针, 启动消费线程
     */
		if(mUseCallback){
			std::cerr << "Repeated binding of callback function!" << std::endl;
			return;
		}
    std::unique_lock<std::mutex> lck(mMutex);
    mBuffer.clear();
    mTempBuffer.clear();
    mBuffer.set_capacity(queue_size);
    mTempBuffer.set_capacity(queue_size);
    mBufferMaxSize = queue_size;

    running = true;
    mCallback = fp;
		mUseCallback = true;

    std::thread t(&DataConsumer::consumerThread, this);
    t.detach();
  }

public:
  template<class classT>
  void registerCallback(void(classT::*fp)(DataType), classT* obj, size_t queue_size){
    _registerCallback(boost::bind(fp, obj, _1), queue_size);
  }

  template<class classT>
  void registerCallback(void(classT::*fp)(const DataType&), classT* obj, size_t queue_size){
    _registerCallback(boost::bind(fp, obj, _1), queue_size);
  }

  void registerCallback(void (*fp)(DataType), size_t queue_size){
    _registerCallback(boost::bind(fp, _1), queue_size);
  }

  void registerCallback(void (*fp)(const DataType&), size_t queue_size){
    _registerCallback(boost::bind(fp, _1), queue_size);
  }

  bool getData(DataType& data){
		if(mUseCallback){
			std::cerr << "The callback function has been bound, and the manual acquisition function 'getData' is disabled" << std::endl;
			return false;
		}

    std::unique_lock<std::mutex> lck(mMutex);
    if(mBuffer.empty()){
      return false;
    }

    data = mBuffer.front();
    mBuffer.pop_front();
    return true;
  }

  void putData(const DataType& val){
    /* 生产者调用该方法将数据添加至消息队列
     */
    std::unique_lock<std::mutex> lck(mMutex);
		// std::cout << "mBuffer.size():" << mBuffer.size() << std::endl;
    mBuffer.push_back(val);
    lck.unlock();
    mCv.notify_one();
  }

  void stop(){
    running = false;
  }

  size_t size()const{
    return mBuffer.size();
  }

private:
  std::mutex mMutex;
  std::condition_variable mCv;
  boost::circular_buffer<DataType> mBuffer;
  boost::circular_buffer<DataType> mTempBuffer;
  size_t mBufferMaxSize;
  bool running;

	bool mUseCallback;
  boost::function<callback_t> mCallback;
};


class Can2serial : public DataConsumer<CanMsg_t::Ptr>
{
public:
	Can2serial(uint8_t version=1);  //0 旧版本, 1新版本
	~Can2serial();
	
	bool configure_port(std::string port,int baud_rate=460800);

	void StartReading() ;

	void StopReading() ;

	bool sendCanMsg(const CanMsg_t &can_msg);
	
	bool configBaudrate(int baudrate);

	bool configBaudrate(uint8_t bs1, uint8_t bs2, uint16_t brp);

	bool setCanFilter(uint8_t filterNum,int filterID,int filterMask,uint8_t filterMode=0x00);

	bool setCanFilter_alone(uint8_t filterNum,int filterID,uint8_t filterMode=0x00);

	bool clearCanFilter(uint8_t filterNum=0xff);

	int inquireBaudrate(uint8_t port =0x01);

	bool getCanMsg(CanMsg_t &msg);
	// bool getCanMsg(CanMsg_t::ConstPtr msg_ptr);
	
	void inquireFilter(uint8_t filterNum ,uint8_t port=0x01);
	
	bool isRunning() {return reading_status_ && !serial_is_bad_;}
	
	enum
	{
		STD_DATA_FRAME = 0x03,
		STD_REMOTE_FRAME =  0x01,
		EXT_REMOTE_FRAME = 0x00,
		EXT_DATA_FRAME = 0x02 
	};

public:
  static void showCanMsg(const CanMsg_t& msg, const std::string& prefix="");
	
private:
	void ReadSerialPort() ;

	void BufferIncomingData(unsigned char *message, unsigned int length);

	void parse(uint8_t * message,uint16_t length);

	uint8_t generateCheckNum(const uint8_t* ptr,size_t len);

	void sendCmd(uint8_t cmdId,const uint8_t *buf,uint8_t count);

	uint8_t version_;
	unsigned char buffer_[MAX_NOUT_SIZE];
	serial::Serial *serial_port_;
	boost::shared_ptr<boost::thread> read_thread_ptr_;
	bool reading_status_;
	bool serial_is_bad_;
	
	uint8_t data_buffer_[MAX_PKG_BUF_LEN]; //最大包长为20
	size_t buffer_index_;
	
	uint16_t package_len_;
	size_t bytes_remaining_;
	
	boost::mutex wr_mutex_;
	boost::mutex send_mutex_;
	
	std::mutex recv_thread_mutex_;
	
	const inquireFilterResponsePkg_t * const inquire_filter_response_ptr_;
	
	
	enum{HeaderByte0=0x66,HeaderByte1=0xCC};
	enum
	{
		CanMsgCmd = 0xB1,
		BaudrateInquire0x13ResponseCmd = 0x93,
		BaudrateInquire0x15ResponseCmd = 0x95,

		FilterSetResponseCmd = 0x98,
		FilterClearResponseCmd = 0x99,
		InquireFilterResponse = 0x9D
	};
	
	enum BaudRateCofigStatus_t
	{
		BaudRateCofig_Ok=0x00,
		BaudrateCofig_Err1=0x01,
		BaudrateCofig_Err2=0x02,
		BaudrateCofig_Err3=0x03,
		BaudrateCofig_Err4=0x04,
		BaudrateCofig_None =0x05
	};
	
	BaudRateCofigStatus_t baudRateCofigStatus_;
	int currentBaudrate_;
	bool newBaudrateResponse_;
	
	bool filterSetStatus_[14];
  bool filterClearStatus_;
};

#endif


