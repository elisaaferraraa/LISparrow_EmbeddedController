// MixingMatrix.cpp
#include "MixingMatrix.h"
#include <cmath>
#include <algorithm>
#include <iostream>

MixingMatrix::MixingMatrix() {
    alpha_thr = 2 * M_PI / 180;
    vel_thr = 2.0;
    vel_trim = 4.0;

    k_eff_rud = 0.7f; 
    k_eff_ele = 0.7f;

    S_vert = 0.01256f;
    d_t_z = 0.075f;
    d_t_x = 0.380f;

    C_area = -142.97f/(1000000);
    b_area = 16290.0f/(1000000);
    S_root = 0.0289f;

    a_mom = -0.0201f/1000;
    b_mom = 0.0904f/1000;
    c_mom = 165.15f/1000;
    b_semi_root = 0.1685f;

    S_hor_tail = 0.021495f;

    theta_sw_min = -5.0f;
    theta_sw_max = 75.0f;
    ele_min = -26.0f;
    ele_max = 26.0f;
    rud_min = -23.0f;
    rud_max = 23.0f;

    theta_sw_min_offset = 0.5f;

    u_sw_max = 1.0f - theta_sw_min_offset;
    u_sw_min = -1.0f;
    u_ele_max = 1.0f;
    u_ele_min = -1.0f;
    u_rud_max = 1.0f;
    u_rud_min = -1.0f;

    K_act_to_ang = Eigen::Matrix3f::Identity();
    K_act_to_ang(0,0) = (theta_sw_max - theta_sw_min)/(u_sw_max - u_sw_min);
    K_act_to_ang(1,1) = (M_PI/180.0f) * (ele_max - ele_min)/(u_ele_max - u_ele_min);
    K_act_to_ang(2,2) = (M_PI/180.0f) * (rud_max - rud_min)/(u_rud_max - u_rud_min);

    I << 0.000917668, 0.0, 0.00003933,
         0.0, 0.003543007 , 0.0,
         0.00003933, 0.0, 0.004389834; // Inertia matrix
}

Eigen::Matrix3f MixingMatrix::computeOmegaDotSensitivity(
    float vel_u, float vel_v, float vel_w,
    float u_sw_sym_cmd,
    float u_rud_prev,
    float u_ele_prev,
    Eigen::Quaternionf q_meas,
    bool wind

) {

    if (wind) {
        // Wind compensation logic         
        Eigen::Vector3f windspeed_i(5.0f, 0.0f, 0.0f);
        //convert it to body frame
        Eigen::Matrix3f R = q_meas.toRotationMatrix();
        Eigen::Vector3f windspeed_b = R * windspeed_i;
        vel_u += windspeed_b.x();
        vel_v += windspeed_b.y();
        vel_w += windspeed_b.z();

        // print windspeed_b for debugging and vel_body
        // std::cout << "Wind speed in body frame: " << windspeed_b.transpose() << std::endl;
        // std::cout << "Velocity in body frame: " << vel_u << ", " << vel_v << ", " << vel_w << std::endl;

    }

    float alpha = atan2(-vel_w, vel_u);
    alpha = clamp(alpha, -float(M_PI)/2, float(M_PI)/2);

    float beta = atan2(vel_v, vel_u);
    float beta_eff = beta + k_eff_rud * u_rud_prev * K_act_to_ang(2,2);
    beta_eff = clamp(beta_eff, -float(M_PI)/2, float(M_PI)/2);

    float alpha_eff = alpha + k_eff_ele * u_ele_prev * K_act_to_ang(1,1);


    float theta_sw_sym_deg = (K_act_to_ang(0,0) * (u_sw_sym_cmd - u_sw_min)) + theta_sw_min;

    float v_sq = clamp(vel_u*vel_u + vel_v*vel_v + vel_w*vel_w, 1.0f, 100.0f);
    float v_scale = (vel_trim * vel_trim) / v_sq;

    float d_c_s_vert_tail = 2.0f * (cos(2*beta_eff)*cos(beta) + sin(2*beta_eff)*sin(beta));
    float d_c_s_hor_tail = 2.0f * (cos(2*alpha_eff)*cos(alpha) + sin(2*alpha_eff)*sin(alpha));
    float c_l_wing = sin(2*alpha) * pow(cos(beta), 2);
    float c_d_wing = 2 * pow(sin(alpha), 2) * pow(cos(beta), 2);

    float S = C_area * theta_sw_sym_deg + b_area + S_root;
    float dS = C_area;
    float d_mom = (a_mom * theta_sw_sym_deg * theta_sw_sym_deg + b_mom * theta_sw_sym_deg + c_mom + b_semi_root)/2.0f;
    float d_d_mom = (a_mom * theta_sw_sym_deg + b_mom/2);

    float A = 2.0f;
    if (round(u_sw_sym_cmd*10) == round(u_sw_max*10) || round(u_sw_sym_cmd*10) == round(u_sw_min*10)) {
        A = 1.0f;
    }

    Eigen::Matrix3f mix_mat = Eigen::Matrix3f::Zero();
    Eigen::Matrix3f inv_mix_mat = Eigen::Matrix3f::Zero();

    if (fabs(alpha) >= alpha_thr && fabs(alpha) <= float(M_PI)/2 && v_sq >= vel_thr * vel_thr) {
        Eigen::Matrix3f M = Eigen::Matrix3f::Zero();


        M(0,0) = A * (dS * d_mom + d_d_mom * S) * (c_l_wing * cos(alpha) + c_d_wing * sin(alpha));
        M(0,2) = k_eff_rud * S_vert * d_c_s_vert_tail * d_t_z;
        M(2,0) = A * (dS * d_mom + d_d_mom * S) * (c_d_wing * cos(alpha) - c_l_wing * sin(alpha));
        M(2,2) = k_eff_rud * S_vert * d_c_s_vert_tail * d_t_x;

        M(1,1) = k_eff_ele * S_hor_tail * d_c_s_hor_tail * d_t_x;

        mix_mat = v_scale * (M * K_act_to_ang).inverse() * I;
        inv_mix_mat = mix_mat.inverse();
    } else {
        float ele_scale = (k_eff_ele * S_hor_tail * d_c_s_hor_tail * d_t_x) * K_act_to_ang(1,1);
        float rud_scale = (k_eff_rud * S_vert * d_c_s_vert_tail * d_t_x) * K_act_to_ang(2,2);

        mix_mat(1,1) = v_scale * (1.0f / ele_scale) * I(1,1);
        mix_mat(2,2) = v_scale * (1.0f / rud_scale) * I(2,2);

        inv_mix_mat(1,1) = 1.0f / mix_mat(1,1);
        inv_mix_mat(2,2) = 1.0f / mix_mat(2,2);
    }

    Eigen::Matrix3f omega_dot_sensitivity = inv_mix_mat * (0.5f * 1.225f * vel_trim * vel_trim);
    return mix_mat;
}
