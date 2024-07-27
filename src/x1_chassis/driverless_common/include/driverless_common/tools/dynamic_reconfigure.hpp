#ifndef LCM_PARAMETER_CONFIGURER_H
#define LCM_PARAMETER_CONFIGURER_H
#include <lcm/lcm-cpp.hpp>
#include <thread>
#include <mutex>
#include <functional>
#include <unistd.h>
#include <iostream>
#include <assert.h>
#include <sstream>
#include <driverless_common/exlcm/DynamicReconfigureMsg.hpp>

namespace dcom {
class DynamicReconfigure{
    /* 服务器: 被配置方
     * 服务器启动后,通过公有通道(reportChannel_)定时上报当前配置信息,
     * 一方面, 可视化客户端可通过该信息显示出需要配置的服务器(消息中包含服务器名称配置标签以及需要配置的参数)
     * 另一方面, 服务器反馈的数据可使得客户端知晓是否配置成功(当收到配置消息后, 立刻进行反馈)
     * 服务器订阅私有通道(server_msg.config_channel)以获得配置信息(client_msg)
     *
     * 客户端: 配置方
     * 客户端启动后, 订阅公有通道(reportChannel_)获得客户端的请求配置消息(server_msg)
     * 当需要修改参数时,将修改后的消息, 使用私有通道(server_msg.config_channel)发布配置信息(client_msg)
     */
public:

    // 所支持的参数数据类型
    enum ParamType{
        Double,
        Int,
        Bool,
        String,
        Auto,  // 自动类型,根据模板参数进行自动设置
    };

    typedef void (Func)(const exlcm::DynamicReconfigureMsg*);
    DynamicReconfigure():
        reportChannel_("dynamic_reconfigure"),
        lcm_(nullptr),
        runFlag_(false),
        callbackBind_(false),
        lcmSubscriber_(nullptr)
    {

    }

    virtual ~DynamicReconfigure(){
        this->stop();
    }

    template<typename classT>
    void registerCallback(void (classT::*fun)(const exlcm::DynamicReconfigureMsg*), classT* obj){
        callback_ = std::bind(fun, obj, std::placeholders::_1);
        callbackBind_ = true;
    }

    void registerCallback(Func fun){
        callback_ = fun;
        callbackBind_ = true;
    }

    virtual bool start(){
        if(!callbackBind_){
            std::cerr << "DynamicReconfigureServer: Plase call registerCallback before start!" << std::endl;
            return false;
        }
        lcm_ = new lcm::LCM;
        if(!lcm_->good()){
            std::cerr << "lcm not good!" << std::endl;
            delete lcm_;
            return false;
        }
        runFlag_ = true;
        //std::cout << "DynamicReconfigure::start Ready to listen..." << std::endl;
        std::thread t(&DynamicReconfigure::lcmListenThread, this);
        t.detach();

        return true;
    }

    virtual void stop(){
        std::cout << "stoping dynamic_reconfigure channel: " << reportChannel_ << std::endl;
        if(!runFlag_){
            return;
        }

        runFlag_ = false;

        if(lcmSubscriber_ && lcm_){
            lcm_->unsubscribe(lcmSubscriber_);
        }

        std::cout << "stoping... try lock. "  << std::endl;
        // 尝试加锁, 只有监听线程结束, 此处才会拿到锁, 以安全退出
        std::lock_guard<std::mutex> lck(threadMutex_);

        if(lcm_){
            delete lcm_;
            lcm_ = nullptr;
        }

        std::cout << "stoped dynamic_reconfigure channel: " << reportChannel_ << std::endl;
    }

    void lcmListenThread(){
        // 监听线程加锁, 调用close时以关闭服务, 需确保拿到锁,即当前线程已退出
        std::lock_guard<std::mutex> lck(threadMutex_);

        //while(runFlag_ && lcm_->handle() == 0);

        while(runFlag_ && lcm_->handleTimeout(100) >= 0){
        };

    }

protected:
    bool runFlag_;
    lcm::LCM *lcm_;
    std::string reportChannel_;
    exlcm::DynamicReconfigureMsg configMsg_;
    lcm::Subscription* lcmSubscriber_;
    std::mutex threadMutex_;
    bool callbackBind_;
    std::function<void (const exlcm::DynamicReconfigureMsg*)> callback_;
};


class DynamicReconfigureServer : public DynamicReconfigure{
public:
    DynamicReconfigureServer(const std::string& server_name, const std::string& config_msg_label,
                             int param_cnt):
        DynamicReconfigure()
    {
        server_ = server_name;
        label_ = config_msg_label;
        configChannel_ = server_name + config_msg_label;
        paramCount_ = param_cnt;

        configMsg_.config_channel = configChannel_;
        configMsg_.server = server_;
        configMsg_.label = label_;
        configMsg_.param_cnt = paramCount_;
        configMsg_.param_names.resize(paramCount_);
        configMsg_.param_types.resize(paramCount_);
        configMsg_.params.resize(paramCount_);
    }

    template<typename T>
    void addParam(int index, const std::string& param_name, T default_value, ParamType param_type=Auto){
        if(index > paramCount_-1){
            std::cerr << "DynamicReconfigureServer::addParam, Index out of bounds";
            assert(index <= paramCount_-1);
        }

        if(param_type == Auto){
            if(typeid(default_value) == typeid(int))
                param_type = Int;
            else if(typeid(default_value) == typeid(bool))
                param_type = Bool;
            else if(typeid(default_value) == typeid(double) || typeid(default_value) == typeid(float))
                param_type = Double;
            else if(typeid(default_value) == typeid(std::string) || typeid(default_value) == typeid(const char*))
                param_type = String;
            else{
                std::cerr << "DynamicReconfigureServer::addParam Invalid data type! \n("
                          << param_name << ") error type: " <<  typeid(default_value).name() << std::endl;
                exit(0);
            }
        }

//        std::cout << "type: " << typeid(default_value).name() << "\t" << default_value << std::endl;

        configMsg_.param_names[index] = param_name;
        configMsg_.param_types[index] = param_type;

        std::stringstream ss;
        ss << default_value;
        configMsg_.params[index] = ss.str();

//        char param_buf[50];
//        if(param_type == Double){
//            sprintf(param_buf, "%f", default_value);
//            configMsg_.params[index] = std::string(param_buf);
//        }else if(param_type == Int || param_type == Bool){
//            sprintf(param_buf, "%d", default_value);
//            configMsg_.params[index] = std::string(param_buf);;
//        }else if(param_type == String){
//            configMsg_.params[index] = param_name;
//        }else{
//            std::cerr << "DynamicReconfigureServer::addParam Invalid data type: " << int(param_type) << std::endl;
//            configMsg_.param_types[index] = String;
//            configMsg_.params[index] = "Invalid data type!";
//        }
    }

    bool start(){
        if(!DynamicReconfigure::start()){
            return false;
        }
        lcmSubscriber_ = lcm_->subscribe(configChannel_, &DynamicReconfigureServer::lcmMsgCallback, this);

        std::thread t(&DynamicReconfigureServer::reportThread, this);
        t.detach();
        return true;
    }

    double getDouble(const exlcm::DynamicReconfigureMsg* msg, int index){
        assert(index < msg->param_cnt);
        int type = msg->param_types[index];
        assert(type == Double);
        return atof(msg->params[index].c_str());
    }

    std::string getString(const exlcm::DynamicReconfigureMsg* msg, int index){
        assert(index < msg->param_cnt);
        int type = msg->param_types[index];
        assert(type == String);
        return msg->params[index].c_str();
    }

    int getInt(const exlcm::DynamicReconfigureMsg* msg, int index){
        assert(index < msg->param_cnt);
        int type = msg->param_types[index];
        assert(type == Int);
        return atoi(msg->params[index].c_str());
    }

    bool getBool(const exlcm::DynamicReconfigureMsg* msg, int index){
        assert(index < msg->param_cnt);
        int type = msg->param_types[index];
        assert(type == Bool);
        return atoi(msg->params[index].c_str());
    }

private:
    void lcmMsgCallback(const lcm::ReceiveBuffer* rbuf,
                        const std::string& channel,
                        const exlcm::DynamicReconfigureMsg* msg){
        callback_(msg);
        assert(msg->param_cnt == configMsg_.param_cnt);
        for(int i=0; i<msg->param_cnt; ++i){
            if(msg->param_types[i] == Double){
                std::string value = std::to_string(getDouble(msg, i));
                configMsg_.params[i] = value.substr(0, value.find_last_not_of('0')+1);
            }else if(msg->param_types[i] == Int){
                configMsg_.params[i] = std::to_string(getInt(msg, i));
            }else if(msg->param_types[i] == Bool){
                configMsg_.params[i] = std::to_string(getBool(msg, i));
            }else{
                configMsg_.params[i] = getString(msg, i);
            }
            // std::cout << int(msg->param_types[i]) << ":" << configMsg_.params[i] << std::endl;
        }
        this->reportConfig();
    }

    /* @brief 上报当前配置信息
     *  一方面用于通知客户端当前服务器需要配置
     *  另一方面将已当前配置进行反馈
     */
    void reportConfig(){
        lcm_->publish(reportChannel_, &configMsg_);
//        std::cout << "\nreport msg:\n";
//        for(int i=0; i<configMsg_.param_cnt; ++i){
//            std::cout << int(configMsg_.param_types[i]) << ":" << configMsg_.params[i] << std::endl;
//        }
    }

    /* brief 定时上报
     */
    void reportThread(){
        while(runFlag_){
            reportConfig();
            sleep(1);
        }
    }
private:
    std::string server_;
    std::string configChannel_;
    std::string label_;
    int paramCount_;

};

class DynamicReconfigureClient : public DynamicReconfigure{
public:
    DynamicReconfigureClient():
        DynamicReconfigure(){

    }

    bool start(){
        if(!DynamicReconfigure::start()){
            return false;
        }
        lcmSubscriber_ = lcm_->subscribe(reportChannel_, &DynamicReconfigureClient::lcmMsgCallback, this);
//        std::cout << "DynamicReconfigureClient::start " << reportChannel_ << std::endl;
        return true;
    }

    void lcmMsgCallback(const lcm::ReceiveBuffer* rbuf,
                        const std::string& channel,
                        const exlcm::DynamicReconfigureMsg* msg){
        callback_(msg);
    }

    void configure(const exlcm::DynamicReconfigureMsg* msg){
        lcm_->publish(msg->config_channel, msg);
    }
};

}

#endif // LCM_PARAMETER_CONFIGURER_H

