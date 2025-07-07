#pragma once

#include "MixingMatrix.h"


class PID {
public:
    PID(float kp, float ki, float kd, float output_min = -1e6f, float output_max = 1e6f);

    float update(float input, float dt);
    void reset();

    void setTunings(float kp, float ki, float kd);
    void setOutputLimits(float min_output, float max_output);
    void setSetpoint(float sp);

private:
    float kp_, ki_, kd_;
    float setpoint_;

    float integral_;
    float prev_error_;
    float output_min_;
    float output_max_;
    bool first_update_;
};
