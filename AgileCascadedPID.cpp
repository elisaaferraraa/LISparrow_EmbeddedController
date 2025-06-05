// AgileCascadedPID.cpp
#include "AgileCascadedPID.h"
#include <iostream>
#include <cmath>

float yaw_rad_ = 0.0f; // Initialize yaw angle referecnce
float yaw_dot = 0.0f; // Initialize yaw rate
float vel_ref = 0.0f; // Initialize velocity command


AgileCascadedPID::AgileCascadedPID(float dt)
    : pid_qx_(0.53, 0.0, 0.022, -5, 5), //Simu: (1.5, 0.0, 0.001, -5, 5),
      pid_qy_(1.19, 0.03, 0.0, -5, 5),//simu:(1.2, 0.03, 0.002, -5, 5),
      pid_qz_(0.74, 0.0, 0.0, -5, 5),//simu(1.0, 0.01, 0.01, -5, 5),
      pid_wx_(1.07, 0.005, 0.008, -4, 4),//simu(0.9, 0.08, 0.005, -4, 4),
      pid_wy_(0.4, 0.03, 0.001, -4, 4),//simu(0.2, 0.08, 0.001, -4, 4),
      pid_wz_(0.55, 0.0, 0.0, -4, 4),//simu(0.3, 0.01, 0.002, -4, 4),
      pid_thrust_(1.2, 0.0001, 0.0, -0.9f, 1.0f),//simu(1.2, 0.1, 0.01, -0.9f, 1.0f), // PID for thrust control
      thrust_ref_(0.0f),
      dt_(dt),
      actuator_outputs_(5)
{
    actuator_outputs_.setZero();
    q_ref_ = Eigen::Quaternionf::Identity();
}

void AgileCascadedPID::setAttitudeReference(float roll_rad_ref, float pitch_rad_ref, float roll_rad, float vel_u ) {
    // If vel_u is zero set it to a small number to avoid division by zero
    if (std::abs(vel_u) < 1e-3f) {
        vel_u = 1e-3f; // Avoid division by zero
    }
    // Integrate yaw for banked turn
    yaw_dot = 9.81f * std::tan(-roll_rad_ref) / vel_u;  // body forward velocity
    yaw_rad_ += yaw_dot * dt_;  // integrate yaw over controller timestep
    
    // TODO delete line below
    // yaw_rad_ = 0.0f; // Reset yaw for testing

    Eigen::Quaternionf q_roll(Eigen::AngleAxisf(roll_rad_ref, Eigen::Vector3f::UnitX()));
    Eigen::Quaternionf q_pitch(Eigen::AngleAxisf(pitch_rad_ref, Eigen::Vector3f::UnitY()));
    Eigen::Quaternionf q_yaw(Eigen::AngleAxisf(yaw_rad_, Eigen::Vector3f::UnitZ()));
    q_ref_ = q_yaw * q_pitch * q_roll;
}

void AgileCascadedPID::setThrustReference(float vel_ref, float vel_u){ //float thrust) {
    // thrust_ref_ = thrust;
    thrust_ref_ = pid_thrust_.update(-(vel_ref-vel_u), dt_); // Update thrust reference based on velocity command
}

void AgileCascadedPID::update(
    const Eigen::Vector3f& position,
    const Eigen::Quaternionf& orientation,
    const Eigen::Vector3f& omega,
    const Eigen::Vector3f& velocity, // vel_u, vel_v, vel_w body frame
    float u_sw_sym,
    float u_ele,
    float u_rud
) {
    Eigen::Quaternionf q_meas = orientation;
    Eigen::Quaternionf q_err = q_meas.conjugate() * q_ref_;

    if (q_err.w() < 0.0f) q_err.coeffs() *= -1.0f; // Ensure shortest rotation

    Eigen::Vector3f pid_input = 2.0f * q_err.vec();

    Eigen::Vector3f w_ref;
    w_ref[0] = pid_qx_.update(-pid_input[0], dt_);
    w_ref[1] = pid_qy_.update(-pid_input[1], dt_);
    w_ref[2] = pid_qz_.update(-pid_input[2], dt_);

    Eigen::Vector3f w_err = w_ref - omega;
    Eigen::Matrix3f Kf = Eigen::Matrix3f::Identity();
    Kf(0,0) = 1.0f;
    Kf(1,1) = 3.0f;
    Kf(2,2) = 1.5f;

    Eigen::Vector3f feedforward = Kf * w_ref;
    feedforward = feedforward.cwiseMax(-2.0f).cwiseMin(2.0f);

    Eigen::Vector3f w_dot_ref;
    w_dot_ref[0] = pid_wx_.update(-w_err[0], dt_);
    w_dot_ref[1] = pid_wy_.update(-w_err[1], dt_);
    w_dot_ref[2] = pid_wz_.update(-w_err[2], dt_);
    w_dot_ref += feedforward;

    //print w_dot_ref
    // std::cout << "w_dot_ref: " << w_dot_ref.transpose() << std::endl;

    float vel_u = velocity[0];
    float vel_v = velocity[1];
    float vel_w = velocity[2];


    Eigen::Matrix3f sensitivity = mixing_.computeOmegaDotSensitivity(
        vel_u, vel_v, vel_w, u_sw_sym, u_rud, u_ele, q_meas, true
    );

    if (!sensitivity.allFinite()) {
    std::cerr << "[ERROR] sensitivity matrix contains NaNs or Infs!" << std::endl;
    }

    //print sensitivity matrix for debugging
    // std::cout << "Sensitivity Matrix: \n" << sensitivity << std::endl;

    Eigen::Vector3f delta_u = sensitivity * w_dot_ref;

    actuator_outputs_[0] = -0.9f; //clamp(thrust_ref_, -0.9f, 1.0f);
    actuator_outputs_[1] = clamp(u_sw_sym + delta_u[0],-1.0f, 0.5f); // left sweep)
    actuator_outputs_[2] = clamp(u_sw_sym - delta_u[0], -1.0f, 0.5f); // right sweep
    actuator_outputs_[3] = clamp(delta_u[1], -1.0f, 1.0f);            // elevator
    actuator_outputs_[4] = clamp(delta_u[2], -1.0f, 1.0f);            // rudder
}

Eigen::VectorXf AgileCascadedPID::getActuatorOutputs() const {
    return actuator_outputs_;
}
