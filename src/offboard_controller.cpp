
// #include <Eigen/src/Geometry/Quaternion.h>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <px4_ros_com/frame_transforms.h>

#include "offboard_controller.hpp"
// #include <tf2/LinearMath/Quaternion.h>

using namespace px4_msgs::msg;
using namespace px4_ros_com::frame_transforms;
using namespace std::chrono_literals;
using std::placeholders::_1;


// /fmu/in/offboard_control_mode
// /fmu/in/vehicle_command do_set_actuator

OffboardController::OffboardController() : Node("offboard_controller")
{
	// node setup
	this->set_parameter(rclcpp::Parameter("use_sim_time", true));
	rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

	control_state_ = ControllerState::kSearching;

	// subs
	odom_sub_ = this->create_subscription<VehicleOdometry>("/px4_1/fmu/out/vehicle_odometry", qos, std::bind(&OffboardController::OdomCallback, this, _1));
	// attitude_sub_ = this->create_subscription<VehicleAttitude>("/px4_1/fmu/out/vehicle_attitude", qos, std::bind(&OffboardController::AttitudeCallback, this, _1));
	// april_tag_sub_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>("/apriltag/detections", qos, std::bind(&OffboardController::TagCallback, this, _1));
	// pubs
	offboard_control_mode_pub_ = this->create_publisher<OffboardControlMode>("/px4_1/fmu/in/offboard_control_mode", 10);
	vehicle_command_pub_ = this->create_publisher<VehicleCommand>("/px4_1/fmu/in/vehicle_command", 10);
	trajectory_setpoint_pub_ = this->create_publisher<TrajectorySetpoint>("/px4_1/fmu/in/trajectory_setpoint", 10);
	visual_odometry_pub_ = this->create_publisher<VehicleOdometry>("/px4_1/fmu/in/vehicle_visual_odometry", 10);
	attitude_setpoint_pub_ = this->create_publisher<VehicleAttitudeSetpoint>("/px4_1/fmu/in/vehicle_attitude_setpoint", 10);

	// tf stuff
	tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	timer_ = this->create_wall_timer(10ms, std::bind(&OffboardController::PublishVehicleOdometry, this));


}


void OffboardController::OdomCallback(const VehicleOdometry::SharedPtr msg)
{
	Eigen::Quaterniond vehicle_orientation(msg->q[0], msg->q[2], msg->q[1], -msg->q[3]);
	Eigen::Vector3d vehicle_translation(msg->position[1], msg->position[0], -msg->position[2]);
	vehicle_orientation = vehicle_orientation * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ());
	vehicle_transform_ = vehicle_translation;
	vehicle_orientation_ = vehicle_orientation;
	PublishVehicleTransforms(vehicle_translation, vehicle_orientation);
	// if (control_state_ == ControllerState::kTracking)
	// {
	// 	PublishModeCommands();
	// 	PublishTrajectorySetpoint();
	// }
}


void OffboardController::PublishTrajectorySetpoint()
{
	TrajectorySetpoint msg{};
	msg.position = {1, -0.5, -2.0};
	msg.velocity = {0.0, 0.0, 0.0};
	msg.acceleration = {0.0, 0.0, 0.0};
	msg.yaw = this->get_clock()->now().seconds() *0.1;
	msg.timestamp = this->get_clock()->now().seconds() * 1000000;
	trajectory_setpoint_pub_->publish(msg);
}

// void OffboardController::PublishAttitudeSetpoint(Eigen::Quaterniond &target_quaternion_px4, Eigen::Vector3d &target_thrust_px4)
// {
// 	VehicleAttitudeSetpoint setpoint;
// 	setpoint.q_d = {float(target_quaternion_px4.w()), float(target_quaternion_px4.x()), float(target_quaternion_px4.y()), float(target_quaternion_px4.z())};
// 	// setpoint.q_d[1] = 0;//target_quaternion_px4.x();
// 	// setpoint.q_d[2] = 0;//target_quaternion_px4.y();
// 	// setpoint.q_d[3] = 0;//target_quaternion_px4.z();
// 	// setpoint.roll_body = 0.0;//target_rpy(0);
// 	// setpoint.pitch_body = 0;//target_rpy(1);
// 	// setpoint.yaw_body = 0;//target_rpy(2);
// 	RCLCPP_INFO(this->get_logger(), "thrust_z: %f", target_thrust_px4(2));
// 	setpoint.thrust_body = {float(target_thrust_px4(0)), float(target_thrust_px4(1)), float(target_thrust_px4(2))};

// 	// setpoint.thrust_body[0] = target_thrust_px4(0);
// 	// setpoint.thrust_body[1] = target_thrust_px4(1);
// 	// setpoint.thrust_body[2] = target_thrust_px4(2);
// 	setpoint.yaw_sp_move_rate = 0;
// 	setpoint.reset_integral = false;
// 	setpoint.fw_control_yaw_wheel = false;
// 	setpoint.timestamp = this->get_clock()->now().seconds() * 1000000;
// 	attitude_setpoint_pub_->publish(setpoint);
// }
void OffboardController::PublishVehicleOdometry()
{
	geometry_msgs::msg::TransformStamped current_transform;
	if (GetGroundTruth(current_transform))
	{
		VehicleOdometry odom{};
		odom.timestamp = this->get_clock()->now().seconds() * 1000000;
		odom.timestamp_sample = current_transform.header.stamp.sec * 1000000;
		odom.pose_frame = odom.POSE_FRAME_NED;
		Eigen::Vector3d t_ros(current_transform.transform.translation.x, current_transform.transform.translation.y, current_transform.transform.translation.z);
		Eigen::Vector3d t_px4(t_ros(0), -t_ros(1), -t_ros(2));
		// RCLCPP_INFO(this->get_logger(), "Sent xyz: %3.2f,%3.2f\t%3.2f,%3.2f\t%3.2f,%3.2f", t_px4(0), vehicle_transform_(0), t_px4(1), vehicle_transform_(1), t_px4(2),vehicle_transform_(2));

		odom.position = {
			float(t_px4(0)),
			float(t_px4(1)),
			float(t_px4(2))
		};
		Eigen::Quaterniond q_ros(current_transform.transform.rotation.w, current_transform.transform.rotation.x, current_transform.transform.rotation.y, current_transform.transform.rotation.z);
		Eigen::Quaterniond q_px4(q_ros.w(), -q_ros.x(), -q_ros.y(), -q_ros.z());
		odom.q = {
			float(q_px4.w()),
			float(q_px4.x()),
			float(q_px4.y()),
			float(q_px4.z())
		};
		odom.velocity_frame = odom.VELOCITY_FRAME_UNKNOWN;
		odom.velocity = {NAN, NAN, NAN};
		odom.angular_velocity = {NAN, NAN, NAN};
		odom.position_variance = {NAN, NAN, NAN};
		odom.orientation_variance = {NAN, NAN, NAN};
		odom.velocity_variance = {NAN, NAN, NAN};
		visual_odometry_pub_->publish(odom);
		PublishModeCommands();
		// PublishTrajectorySetpoint();
	}
}

bool OffboardController::GetGroundTruth(geometry_msgs::msg::TransformStamped &vehicle_transform)
{
	std::string target_frame = "map";
	std::string source_frame = "x500_mono_cam_1";
	control_state_ = ControllerState::kSearching;
	try {
		vehicle_transform = tf_buffer_->lookupTransform(
				target_frame, source_frame,
				tf2::TimePointZero
			);
		control_state_ = ControllerState::kTracking;
		return true;
	} catch (const tf2::TransformException & ex) {
		control_state_ = ControllerState::kSearching;
		return false;
	}
}


// bool OffboardController::TargetCheck()
// {
// 	std::string target_frame = "map";
// 	std::string source_frame = "tag_0";
// 	control_state_ = ControllerState::kSearching;
// 	try {
// 		geometry_msgs::msg::TransformStamped map_to_tag_ = tf_buffer_->lookupTransform(
// 				target_frame, source_frame,
// 				tf2::TimePointZero
// 			);
// 		geometry_msgs::msg::TransformStamped t;
// 		t.header.stamp = this->get_clock()->now();
// 		t.header.frame_id = "map";
// 		t.child_frame_id = "tag_location";
// 		t.transform.translation.x = map_to_tag_.transform.translation.x;
// 		t.transform.translation.y = map_to_tag_.transform.translation.y;
// 		t.transform.translation.z = map_to_tag_.transform.translation.z;
// 		Eigen::Quaterniond rotation2(map_to_tag_.transform.rotation.w, map_to_tag_.transform.rotation.x, map_to_tag_.transform.rotation.y, map_to_tag_.transform.rotation.z);
// // 		Eigen::Quaterniond rotation(1.0, -map_to_tag_.transform.rotation.x, -map_to_tag_.transform.rotation.y, 0.0);
// // 		rotation.w() = sqrt(1 - sqrt(map_to_tag_.transform.rotation.x * map_to_tag_.transform.rotation.x + map_to_tag_.transform.rotation.y * map_to_tag_.transform.rotation.y));
// // //
// // 		rotation.normalize();
// 		Eigen::Vector3d euler = rotation2.toRotationMatrix().eulerAngles(2, 1, 0);
// 		double z_rot = euler(0);

// 		if (abs(euler(1)) > M_PI_2&& abs(euler(2)) > M_PI_2)
// 		{
// 			z_rot -= M_PI;
// 		}
// 		RCLCPP_INFO(this->get_logger(), "euler zyx z_rot: %3.2f, %3.2f, %3.2f, %3.2f", euler(0), euler(1), euler(2), z_rot);

// 		Eigen::Quaterniond rotation(Eigen::AngleAxisd(z_rot, Eigen::Vector3d::UnitZ()));
// 		t.transform.rotation.w = rotation.w();
// 		t.transform.rotation.x = rotation.x();
// 		t.transform.rotation.y = rotation.y();
// 		t.transform.rotation.z = rotation.z();
// 		tf_broadcaster_->sendTransform(t);

// 		target_frame = "tag_0";
// 		source_frame = "x500-Depth";
// 		tag_to_aircraft_ = tf_buffer_->lookupTransform(
// 				target_frame, source_frame,
// 				tf2::TimePointZero
// 			);
// 		target_frame = "tag_location";
// 		source_frame = "x500-Depth";
// 		target_to_aircraft_ = tf_buffer_->lookupTransform(
// 				target_frame, source_frame,
// 				tf2::TimePointZero
// 			);
// 		return true;
// 	} catch (const tf2::TransformException & ex) {
// 		control_state_ = ControllerState::kSearching;
// 		return false;
// 	}
// }

void OffboardController::PublishModeCommands()
{
	double current_time = this->get_clock()->now().seconds();
	if (current_time - last_command_publish_time_ > 0.5)
	{
		last_command_publish_time_ = current_time;
		// PublishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6); // offboard mode
		PublishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 3); // position mode
		PublishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	}
	// PublishOffboardControlMode();
}

void OffboardController::PublishStaticTransforms()
{
	geometry_msgs::msg::TransformStamped t;
	t.header.stamp = this->get_clock()->now();
	t.header.frame_id = "x500-Depth";
	t.child_frame_id = "camera";
	t.transform.translation.x = 0;
	t.transform.translation.y = 0;
	t.transform.translation.z = 0.5;
	t.transform.rotation.w = 0.0;
	t.transform.rotation.x = -0.7071068;
	t.transform.rotation.y = 0.7071068;
	t.transform.rotation.z = 0.0;

	tf_broadcaster_->sendTransform(t);
}

void OffboardController::PublishVehicleTransforms(Eigen::Vector3d &vehicle_translation, Eigen::Quaterniond &vehicle_orientation)
{
	geometry_msgs::msg::TransformStamped t;
	t.header.stamp = this->get_clock()->now();
	t.header.frame_id = "apriltag_move";
	t.child_frame_id = "x500-Depth";
	t.transform.translation.x = vehicle_translation(0);
	t.transform.translation.y = vehicle_translation(1);
	t.transform.translation.z = vehicle_translation(2);

	t.transform.rotation.w = vehicle_orientation.w();
	t.transform.rotation.x = vehicle_orientation.x();
	t.transform.rotation.y = vehicle_orientation.y();
	t.transform.rotation.z = vehicle_orientation.z();

	tf_broadcaster_->sendTransform(t);

	t.header.stamp = this->get_clock()->now();
	t.header.frame_id = "map";
	t.child_frame_id = "x500-Depth-loc";
	t.transform.translation.x = vehicle_translation(0);
	t.transform.translation.y = vehicle_translation(1);
	t.transform.translation.z = vehicle_translation(2);

	t.transform.rotation.w = 1;
	t.transform.rotation.x = vehicle_orientation.x();
	t.transform.rotation.y = 0;
	t.transform.rotation.z = 0;
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
	// if (control_state_ == ControllerState::kSearching)
	// {
		msg.position = true;
		msg.attitude = false;

	// } else
	// {
	// 	msg.position = false;
	// 	msg.attitude = true;
	// }
	msg.velocity = false;
	msg.acceleration = false;
	msg.body_rate = false;
	msg.actuator = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_pub_->publish(msg);
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
