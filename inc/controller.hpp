#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <Eigen/Eigen>

class Controller
{
public:
	Controller();
	void Update(Eigen::Quaterniond &target_quaternion, Eigen::Vector3d &target_thrust);
	
	void set_current_transform(geometry_msgs::msg::Transform new_transform);
	void set_target_transform(geometry_msgs::msg::Transform new_target);

private:
	geometry_msgs::msg::Transform current_transform_;
	geometry_msgs::msg::Transform target_transform_;

	double xy_p_ = 0.5;
	double angle_limit_ = 0.785;
	// double xy_i_;
	// double xy_d_;
	double z_p_ = 0.5;
	double z_i_ = 0.05;
	double z_intergrator_ = 0;
	double z_i_max_ = 3;
	double yaw_p_ = 0.5;

	
};