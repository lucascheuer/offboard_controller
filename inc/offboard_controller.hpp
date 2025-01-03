#pragma once
#include <rclcpp/rclcpp.hpp>
// #include <tf2_msgs>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/buffer.h"
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <std_msgs/msg/bool.hpp>

#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <Eigen/Eigen>

#include "controller.hpp"
#include "state.hpp"
#include "min_snap_traj.hpp"

using namespace px4_msgs::msg;

class OffboardController : public rclcpp::Node
{
public:
	OffboardController();
private:

	// callbacks on these
	rclcpp::Subscription<VehicleOdometry>::SharedPtr odom_sub_;
	rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr april_tag_sub_;
	rclcpp::Subscription<VehicleAttitude>::SharedPtr attitude_sub_;

	rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr waypoint_array_sub_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr execute_sub_;

	// things to do general control of px4
	rclcpp::Subscription<VehicleCommandAck>::SharedPtr command_ack_sub_;
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;
	rclcpp::Publisher<VehicleOdometry>::SharedPtr visual_odometry_pub_;

	// Control types for px4
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
	rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_pub_;

	rclcpp::TimerBase::SharedPtr timer_{nullptr};

	// tf pub and subs
	std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    double last_command_publish_time_ = 0;
	int consecutive_detections_ = 0;

	State current_state_;
	State target_;
	MinSnapTraj traj_;

	double traj_start_time_;
	double traj_end_time_;
	bool traj_executing_ = false;
	bool previous_traj_state_;
	double last_traj_update_ = 0.0;
	Eigen::Quaterniond vehicle_orientation_;
	Eigen::Vector3d vehicle_transform_;

	void DeclareParameters();
	void OdomCallback(const VehicleOdometry::SharedPtr msg);
	void TagCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);
	void WaypointUpdateCallback(const geometry_msgs::msg::PoseArray poses);
	void ExecuteCallback(const std_msgs::msg::Bool execute);
	void UpdateMinSnapTrajEndPoints();

	void TransformToTree(const geometry_msgs::msg::TransformStamped transform);

	void PublishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void PublishOffboardControlMode();
	void PublishTrajectorySetpoint(double time);
	void PublishTrajectorySetpoint();
	// void PublishAttitudeSetpoint(Eigen::Quaterniond &target_quaternion_px4, Eigen::Vector3d &target_thrust_px4);
	// void PublishAttitudeSetpoint(Eigen::Quaterniond &target_quaternion_px4, Eigen::Vector3d &target_thrust_px4);

	void PublishVehicleOdometry();

	bool TargetCheck();
	bool GetGroundTruth(geometry_msgs::msg::TransformStamped &vehicle_transform);
	void PublishModeCommands();
	void PublishStaticTransforms();
	void PublishVehicleTransforms(Eigen::Vector3d &vehicle_translation, Eigen::Quaterniond &vehicle_orientation);

	// visualizations
	void PublishNavPath();
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr waypoint_publisher_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_vis_publisher_;

	
};