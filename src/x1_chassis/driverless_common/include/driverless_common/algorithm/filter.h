#ifndef DRIVERLEES_COMMON_FILTER_H
#define DRIVERLEES_COMMON_FILTER_H

#include <algorithm>
#include <numeric>
#include <boost/circular_buffer.hpp>

namespace dcom {

/* @brief 均值滤波器
 */
class MeanFilter
{
public:
    MeanFilter(int mean_cnt):
        buffer_(mean_cnt)
    {
    }
    ~MeanFilter(){
    }

    void reset(){
        buffer_.clear();
    }

    double filter(double input){
        buffer_.push_back(input);
        return std::accumulate(buffer_.begin(), buffer_.end(), 0.0)/ buffer_.size();
    }
private:
    boost::circular_buffer<double> buffer_;
};

/* @brief 梯度限制滤波器
 */
class GradientLimitFilter{
private:
  float last_value;

  // 增量上下限
  float max_delta;
  float min_delta;
  float increment_coeff;  // 增量系数
  float decrement_coeff;  // 减量系数

  // 死区上下限
  float max_deadband;
  float min_deadband;

  GradientLimitFilter(float _max_delta, float _min_delta,
                      float _max_deadband, float _min_deadband){
    increment_coeff = 1.0f;
    decrement_coeff = 1.0f;

    max_delta = _max_delta;
    min_delta = _min_delta;

    max_deadband = _max_deadband;
    min_deadband = _min_deadband;

    last_value = 0.0;
  }

  float filter(float data){
    if(data > max_deadband){ // 目标数据在死区上限以上
      // 若上一时刻数据小于死区上限，则将其置为上限，防止输出数据落入死区
      if(last_value < max_deadband){
        last_value = max_deadband;
      }
    }else if(data < min_deadband){ // 目标数据在死区下限以下
      // 若上一时刻数据大于死区下限，则将其置为下限，防止输出数据落入死区
      if(last_value > min_deadband){
        last_value = min_deadband;
      }
    }else{// 目标数据在死区内，直接输出
      last_value = data;
      return data;
    }

    float delta = data - last_value;
    float output = 0.0f;

    if(delta > max_delta*increment_coeff){
      output = last_value + max_delta;
    }else if(delta < min_delta*decrement_coeff){
      output = last_value + min_delta;
    }else{
      output = data;
    }
    last_value = output;

    return output;
  }

  void reset(){
      last_value = 0.0;
  }
};


} //end namespace dcom


#endif // DRIVERLEES_COMMON_FILTER_H
