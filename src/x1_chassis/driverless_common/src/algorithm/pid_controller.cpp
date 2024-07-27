#include <driverless_common/algorithm/pid_controller.h>

namespace dcom {

void PositionPidController::setPid(const PidConf& pid_conf, bool _reset){
    pid_conf_ = pid_conf;
    if(_reset){
        reset();
    }
}


void PositionPidController::reset(){
    last_err_ = 0;
    last_ouput_ = 0;
    integral_ = 0;
    first_hit_ = true;
    integrator_saturation_status_ = 0;
}

double PositionPidController::control(double expect_val, double current_val, double dt){
    if(dt <= 0 || dt > pid_conf_.max_interval){
        return last_ouput_;
    }

    double err = expect_val - current_val;
    if(pid_conf_.limit_max_err && err > pid_conf_.max_err){
        err = pid_conf_.max_err;
    }

    integral_ += pid_conf_.ki * err * dt;
    if(integral_ > pid_conf_.integral_saturation_high_){
        integral_ = pid_conf_.integral_saturation_high_;
        integrator_saturation_status_ = 1;
    }else if(integral_ < pid_conf_.integral_saturation_low_){
        integral_ = pid_conf_.integral_saturation_low_;
        integrator_saturation_status_ = -1;
    }else{
        integrator_saturation_status_ = 0;
    }

    double diff = 0;
    if(first_hit_){
        first_hit_ = false;
    }else{
        diff = (err - last_err_) / dt;
    }
    double output = pid_conf_.kp * err + integral_ + pid_conf_.kd * diff;
    if(output > pid_conf_.max_output){
        output = pid_conf_.max_output;
    }else if(output < pid_conf_.min_output){
        output = pid_conf_.min_output;
    }
    last_err_ = err;
    last_ouput_ = output;
    return output;
}

double IncrementPidController::control(double expect_val, double current_val, double dt){
    return PositionPidController::control(expect_val, current_val, dt) - getLastOutput();
}


} //end dcom
