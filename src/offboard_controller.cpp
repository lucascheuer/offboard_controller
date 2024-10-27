
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <px4_ros_com/frame_transforms.h>

#include "offboard_controller.hpp"
// #include <tf2/LinearMath/Quaternion.h>

using namespace px4_msgs::msg;
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
	april_tag_sub_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>("/apriltag/detections", qos, std::bind(&OffboardController::TagCallback, this, _1));
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

}

void OffboardController::AttitudeCallback(const VehicleAttitude::SharedPtr msg)
{
	PublishStaticTransforms();
	TargetCheck();

	geometry_msgs::msg::TransformStamped t;
	t.header.stamp = this->get_clock()->now();
	t.header.frame_id = "map";
	t.child_frame_id = "x500-Depth";
	t.transform.translation.x = 0;
	t.transform.translation.y = 0;
	t.transform.translation.z = 0;

	Eigen::Quaterniond q_px4(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
	Eigen::Quaterniond q_ros = px4_ros_com::frame_transforms::px4_to_ros_orientation(q_px4);

	t.transform.rotation.w = 1.0;//q_ros.w();
	t.transform.rotation.x = 0;//q_ros.x();
	t.transform.rotation.y = 0;//q_ros.y();
	t.transform.rotation.z = 0;//q_ros.z();

	tf_broadcaster_->sendTransform(t);

	if (control_state_ == ControllerState::kTracking)
	{
		PublishModeCommands();
		controller_.set_current_transform(t.transform);
		t = target_;
		t.transform.translation.z = t.transform.translation.z +5;
		controller_.set_target_transform(t.transform);

		Eigen::Quaterniond target_orientation;
		Eigen::Vector3d target_thrust;

		controller_.Update(target_orientation, target_thrust);
		
		t.child_frame_id = "target-orientation";
		t.transform.translation.x = 0;
		t.transform.translation.y = 0;
		t.transform.translation.z = 0;
		t.transform.rotation.w = target_orientation.w();
		t.transform.rotation.x = target_orientation.x();
		t.transform.rotation.y = target_orientation.y();
		t.transform.rotation.z = target_orientation.z();

		tf_broadcaster_->sendTransform(t);
		// RCLCPP_INFO(this->get_logger(), "thrust: %f", target_thrust(2));

		target_orientation = px4_ros_com::frame_transforms::ros_to_px4_orientation(target_orientation);
		target_thrust = px4_ros_com::frame_transforms::enu_to_ned_local_frame(target_thrust);
		PublishAttitudeSetpoint(target_orientation, target_thrust);
		// PublishTrajectorySetpoint();
		
	}
}

void OffboardController::OdomCallback(const VehicleOdometry::SharedPtr msg)
{
	Eigen::Quaterniond vehicle_orientation(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
	vehicle_orientation_ = vehicle_orientation;
	PublishVehicleTransforms(vehicle_orientation);
	// if (control_state_ == ControllerState::kTracking)
	// {
		PublishModeCommands();
		PublishTrajectorySetpoint();
	// }
}

void OffboardController::TagCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
{
	if (msg->detections.size() > 0)
	{
		PublishStaticTransforms();
		if (TargetCheck())
		{	
			if (consecutive_detections_ < 10 )
			{
				consecutive_detections_++;
			} else
			{
				control_state_ = ControllerState::kTracking;
			}
		
			Eigen::Vector3d t_ros(
				target_.transform.translation.x,
				target_.transform.translation.y,
				target_.transform.translation.z
			);
			Eigen::Vector3d t_px4 = px4_ros_com::frame_transforms::enu_to_ned_local_frame(t_ros);
			
			Eigen::Quaterniond q_ros(
				target_.transform.rotation.w,
				target_.transform.rotation.x,
				target_.transform.rotation.y,
				target_.transform.rotation.z
			);
			Eigen::Quaterniond q_px4 = px4_ros_com::frame_transforms::ros_to_px4_orientation(q_ros);
			VehicleOdometry odom{};
			odom.timestamp = this->get_clock()->now().seconds() * 1000000;
			odom.timestamp_sample = target_.header.stamp.sec * 1000000;
			odom.pose_frame = odom.POSE_FRAME_FRD;
			odom.position = {
				float(t_px4.x()),
				float(t_px4.y()),
				float(t_px4.z())
			};
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
			auto tag_euler = q_ros.toRotationMatrix().eulerAngles(0, 1, 2);
			auto aircraft_euler = vehicle_orientation_.toRotationMatrix().eulerAngles(0, 1, 2);
			RCLCPP_INFO(this->get_logger(), "tag x: %5.2f, y: %5.2f, z: %5.2f", tag_euler(0), tag_euler(1), tag_euler(2));
			RCLCPP_INFO(this->get_logger(), "air x: %5.2f, y: %5.2f, z: %5.2f", aircraft_euler(0), aircraft_euler(1), aircraft_euler(2));
		} else
		{
			if (consecutive_detections_ > 0)
			{
				consecutive_detections_--;
			}
			if (consecutive_detections_ == 0)
			{
				control_state_ = ControllerState::kSearching;
			}
		}
		
	}
}

void OffboardController::PublishTrajectorySetpoint()
{
	TrajectorySetpoint msg{};
	msg.position = {0, 0, -3};
	msg.velocity = {0.0, 0.0, 0.0};
	msg.acceleration = {0.0, 0.0, 0.0};
	msg.yaw = this->get_clock()->now().seconds() *0.1;
	msg.timestamp = this->get_clock()->now().seconds() * 1000000;
	trajectory_setpoint_pub_->publish(msg);
}

void OffboardController::PublishAttitudeSetpoint(Eigen::Quaterniond &target_quaternion_px4, Eigen::Vector3d &target_thrust_px4)
{
	VehicleAttitudeSetpoint setpoint;
	setpoint.q_d = {float(target_quaternion_px4.w()), float(target_quaternion_px4.x()), float(target_quaternion_px4.y()), float(target_quaternion_px4.z())};
	// setpoint.q_d[1] = 0;//target_quaternion_px4.x();
	// setpoint.q_d[2] = 0;//target_quaternion_px4.y();
	// setpoint.q_d[3] = 0;//target_quaternion_px4.z();
	// setpoint.roll_body = 0.0;//target_rpy(0);
	// setpoint.pitch_body = 0;//target_rpy(1);
	// setpoint.yaw_body = 0;//target_rpy(2);
	RCLCPP_INFO(this->get_logger(), "thrust_z: %f", target_thrust_px4(2));
	setpoint.thrust_body = {float(target_thrust_px4(0)), float(target_thrust_px4(1)), float(target_thrust_px4(2))};

	// setpoint.thrust_body[0] = target_thrust_px4(0);
	// setpoint.thrust_body[1] = target_thrust_px4(1);
	// setpoint.thrust_body[2] = target_thrust_px4(2);
	setpoint.yaw_sp_move_rate = 0;
	setpoint.reset_integral = false;
	setpoint.fw_control_yaw_wheel = false;
	setpoint.timestamp = this->get_clock()->now().seconds() * 1000000;
	attitude_setpoint_pub_->publish(setpoint);
}

bool OffboardController::TargetCheck()
{
	std::string target_frame = "map";
	std::string source_frame = "tag_0";
	control_state_ = ControllerState::kSearching;
	try {
		target_ = tf_buffer_->lookupTransform(
				target_frame, source_frame,
				tf2::TimePointZero
			);
		return true;
	} catch (const tf2::TransformException & ex) {
		control_state_ = ControllerState::kSearching;
		return false;
	}
}

void OffboardController::PublishModeCommands()
{
	double current_time = this->get_clock()->now().seconds();
	if (current_time - last_command_publish_time_ > 0.5)
	{
		last_command_publish_time_ = current_time;
		PublishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
		PublishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	}
	PublishOffboardControlMode();
}

void OffboardController::PublishStaticTransforms()
{
	geometry_msgs::msg::TransformStamped t;
	t.header.stamp = this->get_clock()->now();
	t.header.frame_id = "x500-Depth";
	t.child_frame_id = "camera";
	t.transform.translation.x = 0;
	t.transform.translation.y = 0;
	t.transform.translation.z = -0.01;

	t.transform.rotation.w = 0.0;
	t.transform.rotation.x = -0.707106;
	t.transform.rotation.y = 0.707106;
	t.transform.rotation.z = 0.0;

	tf_broadcaster_->sendTransform(t);
}

void OffboardController::PublishVehicleTransforms(Eigen::Quaterniond &vehicle_orientation)
{
	geometry_msgs::msg::TransformStamped t;
	t.header.stamp = this->get_clock()->now();
	t.header.frame_id = "map";
	t.child_frame_id = "x500-Depth";
	t.transform.translation.x = 0;
	t.transform.translation.y = 0;
	t.transform.translation.z = 0;

	t.transform.rotation.w = vehicle_orientation.w();
	t.transform.rotation.x = vehicle_orientation.x();
	t.transform.rotation.y = vehicle_orientation.y();
	t.transform.rotation.z = vehicle_orientation.z();

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