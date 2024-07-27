#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_
#include <cmath>
#include <iostream>
namespace dcom {

struct PidConf{
    float kp, ki, kd;                //pid参数
    float max_err;                   //最大误差
    bool limit_max_err;              //限制最大误差

    double max_output, min_output;    //控制量最大小值
    double max_interval;              //最大时间间隔
    double integral_saturation_low_;  //积分饱和最小值
    double integral_saturation_high_; //积分饱和最大值

    PidConf(){
        max_err = 0.0f;
        limit_max_err = false;
    }

    void print(const std::string& prefix=""){
      if(!prefix.empty()){
        std::cout << prefix << ": " << std::endl;
      }
      std::cout << "kp: " << kp << "  ki: " << ki << "  kd: " << kd << std::endl;
      std::cout << "limit_max_err: " << (limit_max_err?1:0) << "  " << "max_err: " << max_err << std::endl;
    }
};

/* @brief 位置型pid控制器
 * output = U(k) = kp*e(k) + ki*∑e(k)*dt + kd*(e(k)-e(k-1))/dt
 */
class PositionPidController{
public:
    PositionPidController(){}
    void setPid(const PidConf& pid_conf, bool _reset=true);
    PidConf getPid(){return pid_conf_;}

    // 声明为虚函数, 当被子类重载后,基类指针能正确调用
    virtual double control(double expect_val, double current_val, double dt);
	
	void print(){
		std::cout << "pid: " << pid_conf_.kp << "  " << pid_conf_.ki << "  " << pid_conf_.kd << std::endl;
	}
	
    void reset();

protected:
    double getLastOutput(){return last_ouput_;}

private:
    PidConf pid_conf_;
    bool first_hit_ = true;
    double last_err_ = 0;
    double integral_ = 0;
    int integrator_saturation_status_ = 0; //积分饱和状态
    double last_ouput_ = 0;
};

class IncrementPidController: public PositionPidController{
public:
    virtual double control(double expect_val, double current_val, double dt) override;

};


} // end dcom

#endif
