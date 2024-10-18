#pragma once
#include <rclcpp/rclcpp.hpp>
// #include <tf2_msgs>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/buffer.h"
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>

using namespace px4_msgs::msg;

class OffboardController : public rclcpp::Node
{
public:
	OffboardController();
private:

	// callbacks on these
	rclcpp::Subscription<VehicleOdometry>::SharedPtr odom_sub_;
	rclcpp::Subscription<VehicleAttitude>::SharedPtr attitude_sub_;

	// things to do general control of px4
	rclcpp::Subscription<VehicleCommandAck>::SharedPtr command_ack_sub_;
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;

	// Control types for px4
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
	rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_pub_;

	// tf pub and subs
	std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    uint64_t last_command_publish_time_ = 0;
	
    rclcpp::Time start_time_;

	void ControlCallback(const VehicleOdometry::SharedPtr msg);

	void TransformToTree(const geometry_msgs::msg::TransformStamped transform);

	void PublishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void PublishOffboardControlMode();
	void PublishTrajectorySetpoint();
};