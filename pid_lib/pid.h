#ifndef _PID_H_
#define _PID_H_

#include <cmath>
#include <iostream>

class PID
{
private:
    float kp_;
    float ki_;
    float kd_;
    float last_error_;

public:
    PID(/* args */) = default;
    PID(float kp, float ki, float kd);
    ~PID() = default;

    float computeVal(const float& cur_status, const float& target_status, const float& delta_t);
};

#endif