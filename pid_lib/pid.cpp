#include "pid.h"

PID::PID(float kp, float ki, float kd) : kp_(kp), ki_(ki), kd_(kd), last_error_(0.f) {}

float PID::computeVal(const float& cur_status, const float& target_status, const float& delta_t)
{
    float val_p, val_i, val_d, error, res_val;

    error = target_status - cur_status;

    val_p = error;

    val_i += error * delta_t;
    val_d = (error - last_error_) / delta_t;

    last_error_ = error;

    //比例控制 + 积分控制 + 微分控制
    res_val = kp_ * val_p + ki_ * val_i + kd_ * val_d;
    return res_val;
}
