// PID.cpp
#include "PID.h"
#include <algorithm>

PID::PID(float kp, float ki, float kd, float output_min, float output_max)
    : kp_(kp), ki_(ki), kd_(kd), setpoint_(0.0f), integral_(0.0f), prev_error_(0.0f),
      output_min_(output_min), output_max_(output_max), first_update_(true) {}

float PID::update(float input, float dt) {
    float error = setpoint_ - input;

    if (first_update_) {
        prev_error_ = error;
        first_update_ = false;
    }

    integral_ += error * dt;
    float derivative = (error - prev_error_) / dt;

    float output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    output = clamp(output, output_min_, output_max_);

    prev_error_ = error;
    return output;
}

void PID::reset() {
    integral_ = 0.0f;
    prev_error_ = 0.0f;
    first_update_ = true;
}

void PID::setTunings(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PID::setOutputLimits(float min_output, float max_output) {
    output_min_ = min_output;
    output_max_ = max_output;
}

void PID::setSetpoint(float sp) {
    setpoint_ = sp;
}