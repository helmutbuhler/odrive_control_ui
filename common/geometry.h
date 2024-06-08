#pragma once
#include "common.h"
#include <vector>

const int num_ik_opt_vars = 1+4+3+3;

const float min_leg_angle = (1-0.97f) * (tau / leg_gear_reduction) + leg_angle_1; // see cd.common_leg_pos_max
const float max_leg_angle = (1-(-0.65f)) * (tau / leg_gear_reduction) + leg_angle_1; // see cd.common_leg_pos_min

const float min_hip_angle = hip_angle_0;
const float max_hip_angle = hip_angle_0+1.847749f; // see cd.hip_sensor_angle_range

const float slider_b_length_0  = 83.0f;
const float slider_b_length_40 = 93.36f;
const float slider_c_length_0 = 63.5f;
const float slider_c_length_40 = 53.35f;

extern float test_delta, test_tresh;
extern float test_x, test_y, test_z;

struct RobotJointStateSide
{
	float arm_a, arm_b, arm_c;
	float hip_angle, leg_angle;
	float wheel_angle; // only for visualization
};

struct RobotJointState
{
	float x_angle, side_angle; // These angles are determined by the hip and leg angles and are here only for visualization.
	float x_delta;
	RobotJointStateSide l, r;
	float theta_for_ground_collision;
};

struct RobotIKTarget
{
	float t_correction_angle;
	float side_angle;
	float x_delta;
	float common_hip_angle, common_leg_angle;
	float l_arm_target_x, l_arm_target_y, l_arm_target_z;
	float r_arm_target_x = 180, r_arm_target_y = 80, r_arm_target_z = -140;
	bool enable_arm_mode;
	float theta_for_ground_collision;
};

void hip_wheel_vector_from_angles(float x_angle, float hip_angle, float leg_angle,
		float& x, float& z);
float leg_height_from_angles(float x_angle, float hip_angle, float leg_angle,
		float side_angle, float side_factor);
float leg_height_from_angles_falling(float x_angle, float hip_angle,
		float side_angle, float side_factor);
float leg_delta_from_angles(float x_angle, float hip_angle, float leg_angle);

float calculate_squat_angle_from_angles(float hip_angle, float leg_angle, float* length_out = nullptr, float* debug_render_points_out = nullptr);
float calculate_squat_angle(float hip_pos, float leg_pos, float* length_out = nullptr);

void get_imu_sensor_position(float x_angle,
		float hip_angle_l, float leg_angle_l, float foot_pos_l,
		float hip_angle_r, float leg_angle_r, float foot_pos_r,
		float& x, float& z);

void get_com_world_position(float x_angle, float theta,
		float hip_angle_l, float leg_angle_l, float foot_pos_l,
		float hip_angle_r, float leg_angle_r, float foot_pos_r,
		float* com_x_out, float* com_z_out,
		float* coi_x_out, float* coi_z_out,
		float* coi_theta_acc_out);

float calculate_ik(RobotJointState& s, const RobotIKTarget& target, int iterations,
		std::vector<float>* error_history, float* error_parts);

RobotJointState get_robot_joint_state(const MonitorData& md, bool from_sensor);
RobotIKTarget get_robot_ik_target(const MonitorData& md);

void draw_robot_3d(const RobotJointState& s, const RobotIKTarget* target, bool render);

void get_slider_lengths_from_arm_angles(float l_arm_b, float l_arm_c, float r_arm_b, float r_arm_c,
		float* l_slider_b, float* l_slider_c, float* r_slider_b, float* r_slider_c);
void switch_arm_mode_smooth(MonitorData& md);

