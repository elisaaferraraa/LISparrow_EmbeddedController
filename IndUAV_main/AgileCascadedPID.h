#pragma once

#undef F  // Fix Eigen vs Arduino macro clash
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "MixingMatrix.h"
#include "PID.h"

class AgileCascadedPID {
public:
    AgileCascadedPID(float dt);

    void setAttitudeReference(float roll_rad_ref, float pitch_rad_ref, float roll_rad, float vel_u );
    void setThrustReference(float vel_ref, float vel_u);

    void update(
        const Eigen::Vector3f& position,
        const Eigen::Quaternionf& orientation,
        const Eigen::Vector3f& omega,
        const Eigen::Vector3f& velocity, // vel_u, vel_v, vel_w
        float u_sw_sym,
        float u_ele,
        float u_rud
    );

    Eigen::VectorXf getActuatorOutputs() const;

private:
    PID pid_qx_, pid_qy_, pid_qz_;
    PID pid_wx_, pid_wy_, pid_wz_;
    PID pid_thrust_;

    float thrust_ref_;
    float dt_;

    Eigen::Quaternionf q_ref_;
    Eigen::VectorXf actuator_outputs_; // 5-element vector

    MixingMatrix mixing_;
};
