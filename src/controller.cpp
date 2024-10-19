#include "controller.hpp"

using namespace Eigen;

Controller::Controller()
{
	
}
// only correct position for now because we can't independently control angle and position
void Controller::Update(Quaterniond &target_quaternion, Vector3d &target_thrust)
{
	double corr_x =		xy_p_ * (current_transform_.translation.x - target_transform_.translation.x);
	double corr_y =		xy_p_ * (current_transform_.translation.y - target_transform_.translation.y);

	double error_z = current_transform_.translation.z - target_transform_.translation.z;
	z_intergrator_ += error_z;
	if (z_intergrator_ > z_i_max_)
	{
		z_intergrator_ = z_i_max_;
	}
	if (z_intergrator_ < -z_i_max_)
	{
		z_intergrator_ = -z_i_max_;
	}
	double corr_z =	z_p_ * error_z + z_i_ * z_intergrator_;
	
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

	target_quaternion = AngleAxisd(corr_y, Vector3d::UnitX()) * AngleAxisd(-corr_x, Vector3d::UnitY()) * AngleAxisd(0, Vector3d::UnitZ());
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