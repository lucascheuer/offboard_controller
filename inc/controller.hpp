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
	class PID
	{
	public:
		PID(double P, double I, double D, double I_limit=NAN);
		double Update(double error);
		void Reset();
	private:
		double kp_;
		double ki_;
		double kd_;
		double i_limit_;
		double integrator_ = 0;
		double last_error_ = 0;
	};
	geometry_msgs::msg::Transform current_transform_;
	geometry_msgs::msg::Transform target_transform_;

	PID x_position_;//(0.3, 0, 0, 0);
	PID y_position_;//(0.3, 0, 0, 0);
	PID z_position_;//(0.3, 0, 0, 0);

	double angle_limit_ = 0.785;
	double low_thrust_limit_ = 0.1;
	double high_thrust_limit_ = 1.0;
	// double xy_i_;
	// double xy_d_;
	double z_ff_ = 0.7;

};