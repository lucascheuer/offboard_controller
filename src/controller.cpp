#include "controller.hpp"

using namespace Eigen;

Controller::Controller():
x_position_(0.01, 0, 0, 0),
y_position_(0.01, 0, 0, 0),
z_position_(0.02, 0.00, 0.1, 0)
{
	
}
// only correct position for now because we can't independently control angle and position
void Controller::Update(Eigen::Quaterniond &target_quaternion, Vector3d &target_thrust)
{
	double error_x = target_transform_.translation.x - current_transform_.translation.x;
	double error_y = target_transform_.translation.y - current_transform_.translation.y;
	double error_z = target_transform_.translation.z - current_transform_.translation.z;

	double corr_x =	x_position_.Update(error_x);
	double corr_y =	y_position_.Update(error_y);
	double corr_z = z_ff_ + z_position_.Update(error_z);
	
	if (corr_x > angle_limit_)
	{
		corr_x = angle_limit_;
	}
	if (corr_x < -angle_limit_)
	{
		corr_x = -angle_limit_;
	}

	if (corr_y > angle_limit_)
	{
		corr_y = angle_limit_;
	}
	if (corr_y < -angle_limit_)
	{
		corr_y = -angle_limit_;
	}

	corr_z = corr_z / cos(corr_y) / cos(corr_x);

	if (corr_z > high_thrust_limit_)
	{
		corr_z = high_thrust_limit_;
	}
	if (corr_z < low_thrust_limit_)
	{
		corr_z = low_thrust_limit_;
	}
	
	target_quaternion = AngleAxisd(-corr_y, Vector3d::UnitX()) * AngleAxisd(corr_x, Vector3d::UnitY()) * AngleAxisd(0, Vector3d::UnitZ());
	target_thrust(0) = 0;
	target_thrust(1) = 0;
	target_thrust(2) = corr_z;
}


void Controller::set_current_transform(geometry_msgs::msg::Transform new_transform)
{
	current_transform_ = new_transform;
}

void Controller::set_target_transform(geometry_msgs::msg::Transform new_target)
{
	target_transform_ = new_target;
}

Controller::PID::PID(double P, double I, double D, double I_limit):
kp_(P), ki_(I), kd_(D), i_limit_(I_limit), last_error_(0)
{

}

double Controller::PID::Update(double error)
{
	integrator_ += error;
	if (i_limit_ != NAN)
	{
		if (integrator_ > i_limit_)
		{
			integrator_ = i_limit_;
		} else if (integrator_ < -i_limit_)
		{
			integrator_ = -i_limit_;
		}
	}
	double correction = kp_ * error + ki_ * integrator_ + kd_ * (error - last_error_);
	last_error_ = error;
	return correction;
}