#ifndef LCM_COSTOM_MSG_PUBLISHER_H
#define LCM_COSTOM_MSG_PUBLISHER_H
#include <lcm/lcm-cpp.hpp>
#include "costom_msg_publisher.hpp"
#include <driverless_common/exlcm/CustomMsg.hpp>

namespace dcom {

/* @brief LCM 自定义数据发布器
 * LcmCostomMsgPublisher lcm_state_publisher;
 * lcm_cm_publisher.init("sender", "state", 3);
 * lcm_cm_publisher.setData(0, "speed", 8.0, "m/s");
 * lcm_cm_publisher.setData(1, "steerangle", 10.2, "deg");
 * lcm_cm_publisher.setData(3, "torque", 200.0, "N");
 * lcm_cm_publisher.publish();
 */

class LcmCostomMsgPublisher: public CostomMsgPublisher{
public:
    LcmCostomMsgPublisher(const std::string& channal="to_analyzer", const std::string& lcm_url=""):
        CostomMsgPublisher(channal),
        lcm_(lcm_url)
    {
    }

    virtual bool init(const std::string& sender, const std::string label, int data_cnt){
        bool ok = CostomMsgPublisher::init(sender, label, data_cnt);
        custom_msg_.sender = sender;
        custom_msg_.label = label;
        custom_msg_.data_cnt = data_cnt;

        // 初始化列表,并置所有data_names为"" 表示为无效数据
        custom_msg_.datas.assign(data_cnt, 0.0);
        custom_msg_.data_names.assign(data_cnt, "");
        custom_msg_.data_units.assign(data_cnt, "");

        initialed_ = ok && lcm_.good();
        return initialed_;
    }

    /* @brief 设置数据
     * @param index 数据索引
     * @param name 数据名称
     * @param value 数据值
     *
     * 索引不得越界(<data_cnt), 且索引与数据名称一一对应, 一旦确定不得更改!
     *
     * setData(0, "speed", 8.0);
     * setData(1, "steerangle", 10.2);
     * setData(3, "torque", 200.0);
     */
    virtual bool setData(int index, const std::string& name, const double value, const std::string& unit=""){
        if(!enable_){
            return false;
        }
        if(!initialed_){
            //std::cout << "Please ensure that the init function is executed and initialized successfully!" << std::endl;
            return false;
        }
        assert(index < data_cnt_);
        std::string& data_name = custom_msg_.data_names[index];
        std::string& data_unit = custom_msg_.data_units[index];

        if(data_name.empty()){
            data_name = name;
        }
        if(data_unit.empty()){
        	data_unit = unit;
        }
        
        assert(data_name == name);
        custom_msg_.datas[index] = value;

        return true;
    }

    virtual bool publish(){
        if(enable_ && initialed_){
            lcm_.publish(channel_, &custom_msg_);
            return true;
        }

        return false;
    }

    void print(){
        std::cout << "========== target channel " << channel_ << " ============" << std::endl;
        for(int i=0; i<data_cnt_; ++i){
            std::cout << custom_msg_.data_names[i] << ": " << custom_msg_.datas[i] << " "
                      << custom_msg_.data_units[i] << std::endl;
        }
        std::cout << std::endl;
    }

private:
    lcm::LCM lcm_;
    exlcm::CustomMsg custom_msg_;
};



}// end namespace dcom

#endif // LCM_COSTOM_MSG_PUBLISHER_H
