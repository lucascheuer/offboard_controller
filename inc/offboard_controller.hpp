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

#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <Eigen/Eigen>

#include "controller.hpp"

using namespace px4_msgs::msg;

class OffboardController : public rclcpp::Node
{
public:
	OffboardController();
private:

	enum class ControllerState
	{
		kSearching,
		kTracking
	};

	// callbacks on these
	rclcpp::Subscription<VehicleOdometry>::SharedPtr odom_sub_;
	rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr april_tag_sub_;
	rclcpp::Subscription<VehicleAttitude>::SharedPtr attitude_sub_;

	// things to do general control of px4
	rclcpp::Subscription<VehicleCommandAck>::SharedPtr command_ack_sub_;
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;
	rclcpp::Publisher<VehicleOdometry>::SharedPtr visual_odometry_pub_;

	// Control types for px4
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
	rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_pub_;

	// tf pub and subs
	std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    double last_command_publish_time_ = 0;
	int consecutive_detections_ = 0;

	ControllerState control_state_;
	Controller controller_;

	geometry_msgs::msg::TransformStamped tag_to_aircraft_;
	geometry_msgs::msg::TransformStamped target_to_aircraft_;
	Eigen::Quaterniond vehicle_orientation_;

	void OdomCallback(const VehicleOdometry::SharedPtr msg);
	void TagCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);

	void TransformToTree(const geometry_msgs::msg::TransformStamped transform);

	void PublishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void PublishOffboardControlMode();
	void PublishTrajectorySetpoint();
	void PublishAttitudeSetpoint(Eigen::Quaterniond &target_quaternion_px4, Eigen::Vector3d &target_thrust_px4);
	// void PublishAttitudeSetpoint(Eigen::Quaterniond &target_quaternion_px4, Eigen::Vector3d &target_thrust_px4);


	bool TargetCheck();
	void PublishModeCommands();
	void PublishStaticTransforms();
	void PublishVehicleTransforms(Eigen::Quaterniond &vehicle_orientation);
	
};