#pragma once

#include <Eigen/Dense>

template <typename T>
constexpr const T clamp(const T& v, const T& lo, const T& hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}


class MixingMatrix {
public:
    MixingMatrix();

    // Compute omega_dot sensitivity matrix
    Eigen::Matrix3f computeOmegaDotSensitivity(
        float vel_u, float vel_v, float vel_w,
        float u_sw_sym_cmd,
        float u_rud_prev,
        float u_ele_prev,
        Eigen::Quaternionf q_meas,
        bool wind = false

    );

private:
    // Thresholds and constants
    float alpha_thr;
    float vel_thr;
    float vel_trim;

    // Effectiveness coefficients
    float k_eff_rud;
    float k_eff_ele;

    // Geometry parameters
    float S_vert, d_t_z, d_t_x;
    float C_area, b_area, S_root;
    float a_mom, b_mom, c_mom, b_semi_root;
    float S_hor_tail;

    // Actuator ranges
    float theta_sw_min, theta_sw_max;
    float ele_min, ele_max;
    float rud_min, rud_max;

    float theta_sw_min_offset;
    float u_sw_max, u_sw_min;
    float u_ele_max, u_ele_min;
    float u_rud_max, u_rud_min;

    // Transformation matrices
    Eigen::Matrix3f I;
    Eigen::Matrix3f K_act_to_ang;
};
