
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <Eigen/Eigen>
#include <px4_ros_com/frame_transforms.h>

#include "offboard_controller.hpp"
// #include <tf2/LinearMath/Quaternion.h>

using namespace px4_msgs::msg;
using std::placeholders::_1;


// /fmu/in/offboard_control_mode
// /fmu/in/vehicle_command do_set_actuator

OffboardController::OffboardController() : Node("offboard_controller")
{
	this->set_parameter(rclcpp::Parameter("use_sim_time", true));
	rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
	odom_sub_ = this->create_subscription<VehicleOdometry>("/px4_1/fmu/out/vehicle_odometry", qos, std::bind(&OffboardController::ControlCallback, this, _1));

	offboard_control_mode_pub_ = this->create_publisher<OffboardControlMode>("/px4_1/fmu/in/offboard_control_mode", 10);
	trajectory_setpoint_pub_ = this->create_publisher<TrajectorySetpoint>("/px4_1/fmu/in/trajectory_setpoint", 10);
	vehicle_command_pub_ = this->create_publisher<VehicleCommand>("/px4_1/fmu/in/vehicle_command", 10);
	tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

}
	
void OffboardController::ControlCallback(const VehicleOdometry::SharedPtr msg)
{
	double current_time = this->get_clock()->now().seconds();
	// start_time_ = this->get_clock()->now();
	// RCLCPP_INFO(this->get_logger(), "%fs", (current_time - last_time_) * 1e-9);
	if (current_time - last_command_publish_time_ > 0.5)
	{
		last_command_publish_time_ = current_time;
		PublishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
		PublishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	}
	PublishOffboardControlMode();
	PublishTrajectorySetpoint();
	geometry_msgs::msg::TransformStamped t;

	// Read message content and assign it to
	// corresponding tf variables
	
	t.header.stamp = this->now();
	t.header.frame_id = "map";
	t.child_frame_id = "x500-Depth";

	Eigen::Vector3d t_px4(msg->position[0], msg->position[1], msg->position[2]);
	Eigen::Vector3d t_ros = px4_ros_com::frame_transforms::ned_to_enu_local_frame(t_px4);
	t.transform.translation.x = t_ros(0);
	t.transform.translation.y = t_ros(1);
	t.transform.translation.z = t_ros(2);
	
	Eigen::Quaterniond q_px4(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
	Eigen::Quaterniond q_ros = px4_ros_com::frame_transforms::px4_to_ros_orientation(q_px4);

	t.transform.rotation.w = q_ros.w();
	t.transform.rotation.x = q_ros.x();
	t.transform.rotation.y = q_ros.y();
	t.transform.rotation.z = q_ros.z();


	tf_broadcaster_->sendTransform(t);
	t.header.stamp = this->now();
	t.transform.rotation.w = 1.0;
	t.transform.rotation.x = 0.0;
	t.transform.rotation.y = 0.0;
	t.transform.rotation.z = 0.0;
	t.child_frame_id = "no-yaw-x500-Depth";
	tf_broadcaster_->sendTransform(t);
	// camera broadcast
	// <pose> 0 0 .242 0 1.5707 0</pose>
	// [ 0, 1, 0, 0.0000013 ]
	t.header.stamp = this->get_clock()->now();
	t.header.frame_id = "x500-Depth";
	t.child_frame_id = "camera";
	t.transform.translation.x = 0;
	t.transform.translation.y = 0;
	t.transform.translation.z = -1;

	t.transform.rotation.w = 0.0;
	t.transform.rotation.x = -0.707106;
	t.transform.rotation.y = 0.707106;
	t.transform.rotation.z = 0.0;

	tf_broadcaster_->sendTransform(t);

}

void OffboardController::PublishVehicleCommand(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 0;
	msg.target_component = 0;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_pub_->publish(msg);
}

void OffboardController::PublishOffboardControlMode()
{
	OffboardControlMode msg{};
	msg.position = false;
	msg.velocity = true;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_pub_->publish(msg);
}

void OffboardController::PublishTrajectorySetpoint()
{
	TrajectorySetpoint msg{};
	double time_now = this->get_clock()->now().seconds();
	double x_vel = 0;
	double y_vel = 0;
	double z_vel = 0;
	double target_x = 0;
	double target_y = 0;
	double target_z = 0;
	std::string fromFrameRel = "tag_0";
	std::string toFrameRel = "no-yaw-x500-Depth";
	geometry_msgs::msg::TransformStamped t;
	try {
		t = tf_buffer_->lookupTransform(
				toFrameRel, fromFrameRel,
				tf2::TimePointZero
			);
		target_x = t.transform.translation.x;
		target_y = t.transform.translation.y;
		target_z = t.transform.translation.z;
		// RCLCPP_INFO(this->get_logger(), "tx: %5.2f\tty: %5.2f\ttz: %5.2f", t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
	} catch (const tf2::TransformException & ex) {
		RCLCPP_INFO(
		this->get_logger(), "Could not transform %s to %s: %s",
		toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
		target_x = 0;
		target_y = 0;
		target_z = 0;
	}
	
	double p_gain = 0.5;
	double max_speed = 0.5;
	Eigen::Vector3d position_error(target_x, target_y, target_z + 6);
	x_vel = position_error(0) * p_gain;
	y_vel = position_error(1) * p_gain;
	z_vel = position_error(2) * p_gain;
	if (x_vel > max_speed)
	{
		x_vel = max_speed;
	}else if (x_vel < -max_speed)
	{
		x_vel = -max_speed;
	}

	if (y_vel > max_speed)
	{
		y_vel = max_speed;
	}else if (y_vel < -max_speed)
	{
		y_vel = -max_speed;
	}

	if (z_vel > max_speed)
	{
		z_vel = max_speed;
	}else if (z_vel < -max_speed)
	{
		z_vel = -max_speed;
	}
	RCLCPP_INFO(this->get_logger(),"x_e: %3.2f\ty_e: %3.2f\tz_e: %3.2f", target_x, target_y, target_z);
	Eigen::Vector3d t_d_ros(x_vel, y_vel, z_vel);

	Eigen::Vector3d t_d_px4 = px4_ros_com::frame_transforms::enu_to_ned_local_frame(t_d_ros);
	x_vel = t_d_px4(0);
	y_vel = t_d_px4(1);
	z_vel = t_d_px4(2);
	
	msg.position = {NAN, NAN, NAN};
	msg.velocity = {float(x_vel), float(y_vel), float(z_vel)};
	msg.acceleration = {0.0, 0.0, 0.0};
	msg.yaw = time_now * 0.1;
	msg.timestamp = time_now * 1000000;
	trajectory_setpoint_pub_->publish(msg);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardController>());

	rclcpp::shutdown();
	return 0;
}