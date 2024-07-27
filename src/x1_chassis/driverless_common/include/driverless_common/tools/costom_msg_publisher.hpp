#ifndef __COSTOM_MSG_PUBLISHER_H_
#define __COSTOM_MSG_PUBLISHER_H_
#include <string>
#include <assert.h>
#include <iostream>

namespace dcom {
/* 自定义消息发布器基类
 */
class CostomMsgPublisher{
public:
    CostomMsgPublisher(const std::string& channal): channel_(channal){
        initialed_ = false;
    }

    virtual bool init(const std::string& sender, const std::string label, int data_cnt){
        sender_ = sender;
        label_ = label;
        data_cnt_ = data_cnt;
        enable_ = true;
        return true;
    }

    virtual bool setData(int index, const std::string& name, const double value, const std::string& unit="") = 0;
    virtual bool publish() = 0;

    void setEnabled(bool enable){
        enable_ = enable;
    }

protected:
    std::string label_;
    std::string sender_;
    int data_cnt_;
    const std::string channel_;
    bool initialed_;
    bool enable_;
};

}// end namespace dcom

#endif
