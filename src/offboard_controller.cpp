
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
	DeclareParameters();
	set_parameter(rclcpp::Parameter("use_sim_time", true));
	rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

	// subs
	odom_sub_ = this->create_subscription<VehicleOdometry>("/px4_1/fmu/out/vehicle_odometry", qos, std::bind(&OffboardController::OdomCallback, this, _1));
	waypoint_array_sub_ = create_subscription<geometry_msgs::msg::PoseArray>("/min_snap/path", qos, std::bind(&OffboardController::WaypointUpdateCallback, this, _1));
	execute_sub_ = create_subscription<std_msgs::msg::Bool>("/min_snap/execute", qos, std::bind(&OffboardController::ExecuteCallback, this, _1));
	// attitude_sub_ = this->create_subscription<VehicleAttitude>("/px4_1/fmu/out/vehicle_attitude", qos, std::bind(&OffboardController::AttitudeCallback, this, _1));
	// april_tag_sub_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>("/apriltag/detections", qos, std::bind(&OffboardController::TagCallback, this, _1));
	// pubs
	offboard_control_mode_pub_ = this->create_publisher<OffboardControlMode>("/px4_1/fmu/in/offboard_control_mode", 10);
	vehicle_command_pub_ = this->create_publisher<VehicleCommand>("/px4_1/fmu/in/vehicle_command", 10);
	trajectory_setpoint_pub_ = this->create_publisher<TrajectorySetpoint>("/px4_1/fmu/in/trajectory_setpoint", 10);
	visual_odometry_pub_ = this->create_publisher<VehicleOdometry>("/px4_1/fmu/in/vehicle_visual_odometry", 10);
	attitude_setpoint_pub_ = this->create_publisher<VehicleAttitudeSetpoint>("/px4_1/fmu/in/vehicle_attitude_setpoint", 10);
	waypoint_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/min_snap/waypoints_visualization", 10);
	path_vis_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/min_snap/path_visualization", 10);
	// tf stuff
	tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	timer_ = this->create_wall_timer(10ms, std::bind(&OffboardController::PublishVehicleOdometry, this));

}

void OffboardController::DeclareParameters()
{
	declare_parameter<bool>("min_snap", false);
	declare_parameter<double>("speed", 1.0);
	declare_parameter<double>("cooldown", 0.5);
}
void OffboardController::OdomCallback(const VehicleOdometry::SharedPtr msg)
{
	Eigen::Quaterniond vehicle_orientation(msg->q[0], msg->q[2], msg->q[1], -msg->q[3]);
	Eigen::Vector3d vehicle_translation(msg->position[1], msg->position[0], -msg->position[2]);
	vehicle_orientation = vehicle_orientation * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ());
	vehicle_transform_ = vehicle_translation;
	vehicle_orientation_ = vehicle_orientation;
	PublishVehicleTransforms(vehicle_translation, vehicle_orientation);
	double cooldown;
	get_parameter("cooldown", cooldown);
	if(get_clock()->now().seconds() - last_traj_update_ > cooldown && !traj_executing_)
	{
		UpdateMinSnapTrajEndPoints();
	}
	current_state_.x = msg->position[1];
	current_state_.y = msg->position[0];
	current_state_.z = -msg->position[2];
	current_state_.vx = msg->velocity[1];
	current_state_.vy = msg->velocity[0];
	current_state_.vz = -msg->velocity[2];
	current_state_.ax = 0;
	current_state_.ay = 0;
	current_state_.az = 0;


	Eigen::Vector3d euler = vehicle_orientation.toRotationMatrix().eulerAngles(2, 1, 0);
	double yaw = euler(0);
	if (abs(euler(1)) > M_PI_2 && abs(euler(2)) > M_PI_2)
	{
		yaw -= M_PI;
	}
	current_state_.yaw = yaw; // wrong, needs to be 
	current_state_.vyaw = msg->angular_velocity[2]; // wrong frame? maybe?
}

void OffboardController::WaypointUpdateCallback(const geometry_msgs::msg::PoseArray poses)
{
	if (!traj_executing_)
	{
		traj_.ClearWaypoints();
		for (int waypoint = 0; waypoint < int(poses.poses.size()); ++waypoint)
		{
			// Eigen::Quaterniond eig_quat(poses.poses[waypoint].orientation.w, poses.poses[waypoint].orientation.x, poses.poses[waypoint].orientation.y, poses.poses[waypoint].orientation.z);
			// auto euler = eig_quat.toRotationMatrix().eulerAngles(0, 1, 2);
			MinSnapTraj::Waypoint new_waypoint(Eigen::Vector3d(poses.poses[waypoint].position.x, poses.poses[waypoint].position.y, poses.poses[waypoint].position.z), 0.0);
			traj_.AddWaypoint(new_waypoint);
		}
		MinSnapTraj::Waypoint last_waypoint(Eigen::Vector3d(poses.poses[0].position.x, poses.poses[0].position.y, poses.poses[0].position.z), 0);
		traj_.AddWaypoint(last_waypoint);
	}
}

void OffboardController::ExecuteCallback(const std_msgs::msg::Bool execute)
{
	set_parameter(rclcpp::Parameter("min_snap", true));
}

void OffboardController::UpdateMinSnapTrajEndPoints()
{	
	double speed;
	get_parameter("speed", speed);
	bool solved = traj_.Solve(speed);
	if (solved && traj_.solved())
	{
		last_traj_update_ = get_clock()->now().seconds();

	}
	if (!solved )
	{
		RCLCPP_INFO(this->get_logger(), "trajectory failed to solve");
	}
	
}


void OffboardController::PublishTrajectorySetpoint(double time)
{
	State state;
	traj_.Evaluate(time, state);
	// double  x_error = state.x  - current_state_.x;
	// double vx_error = state.vx - current_state_.vx;
	// double ax_error = state.ax - current_state_.ax;
	// RCLCPP_INFO(this->get_logger(), "des x: %3.2f, x: %3.2f, des vx: %3.2f, vx: %3.2f, des ax: %3.2f, ax: %3.2f", state.x, current_state_.x, state.vx, current_state_.vx, state.ax, current_state_.ax);
	TrajectorySetpoint msg{};
	msg.position = {float(state.x), float(-state.y), float(-state.z)};
	msg.velocity = {float(state.vx), float(-state.vy), float(-state.vz)};
	msg.acceleration = {float(state.ax), float(-state.ay), float(-state.az)};
	msg.yaw = state.yaw;
	msg.yawspeed = state.vyaw;
	msg.timestamp = this->get_clock()->now().seconds() * 1000000;
	trajectory_setpoint_pub_->publish(msg);
}

void OffboardController::PublishTrajectorySetpoint()
{
	// node setup
	MinSnapTraj::Waypoint *waypoint = traj_.GetWaypoint(0);
	if(waypoint != nullptr)
	{
		TrajectorySetpoint msg{};
		msg.position = {float(waypoint->pos[0]), float(-waypoint->pos[1]), float(-waypoint->pos[2])};
		msg.velocity = {0.0, 0.0, 0.0};
		msg.acceleration = {0.0, 0.0, 0.0};
		msg.yaw = float(0.0);
		msg.timestamp = this->get_clock()->now().seconds() * 1000000;
		trajectory_setpoint_pub_->publish(msg);
	} else
	{
		RCLCPP_INFO(this->get_logger(), "Waypoint nullptr");
	}
	
	
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
		bool min_snap;
		get_parameter("min_snap", min_snap);
		PublishModeCommands();
		if (min_snap)
		{
			
			if (!traj_.solved())
			{
				double speed;
				get_parameter("speed", speed);
				bool solved = traj_.Solve(speed);
				if (!solved)
				{
					RCLCPP_INFO(this->get_logger(), "trajectory failed to solve");
					min_snap = false;
				}
				
			}
			if (!traj_executing_)
			{
				traj_executing_ = true;
				traj_start_time_ = this->get_clock()->now().seconds();
			}
			double time_now = this->get_clock()->now().seconds();
			if (time_now - traj_start_time_ > traj_.EndTime())
			{
				min_snap = false;
			}
			if (min_snap)
			{
				if (previous_traj_state_ != min_snap)
				{
					RCLCPP_INFO(this->get_logger(), "publishing min snap points");
				}
				PublishTrajectorySetpoint(this->get_clock()->now().seconds() - traj_start_time_);
			} else
			{
				RCLCPP_INFO(this->get_logger(), "Clearing min snap parameter");
				set_parameter(rclcpp::Parameter("min_snap", false));
				traj_executing_ = false;
			}
		} else
		{
			if (previous_traj_state_ != min_snap)
			{
				RCLCPP_INFO(this->get_logger(), "publishing normal points");
			}
			PublishTrajectorySetpoint();
		}
		previous_traj_state_ = min_snap;
		PublishNavPath();
	}
}

bool OffboardController::GetGroundTruth(geometry_msgs::msg::TransformStamped &vehicle_transform)
{
	std::string target_frame = "map";
	std::string source_frame = "x500_mono_cam_1";
	try {
		vehicle_transform = tf_buffer_->lookupTransform(
				target_frame, source_frame,
				tf2::TimePointZero
			);
		return true;
	} catch (const tf2::TransformException & ex) {
		return false;
	}
}


void OffboardController::PublishModeCommands()
{
	double current_time = this->get_clock()->now().seconds();
	if (current_time - last_command_publish_time_ > 0.5)
	{
		last_command_publish_time_ = current_time;
		PublishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6); // offboard mode
		// PublishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 3); // position mode
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

	// t.header.stamp = this->get_clock()->now();
	// t.header.frame_id = "map";
	// t.child_frame_id = "x500-Depth-loc";
	// t.transform.translation.x = vehicle_translation(0);
	// t.transform.translation.y = vehicle_translation(1);
	// t.transform.translation.z = vehicle_translation(2);

	// t.transform.rotation.w = 1;
	// t.transform.rotation.x = vehicle_orientation.x();
	// t.transform.rotation.y = 0;
	// t.transform.rotation.z = 0;
	// tf_broadcaster_->sendTransform(t);

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
	msg.position = true;
	msg.attitude = false;
	msg.velocity = true;
	msg.acceleration = true;
	msg.body_rate = false;
	msg.actuator = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_pub_->publish(msg);
}

void OffboardController::PublishNavPath()
{
	// waypoints
	visualization_msgs::msg::Marker waypoints;
    waypoints.header.frame_id = "map";
    waypoints.header.stamp = this->get_clock()->now();
    waypoints.ns = "current";
    // waypoints.id = 0;
    // waypoints.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    waypoints.action = visualization_msgs::msg::Marker::ADD;
    waypoints.pose.orientation.x = 0.0;
    waypoints.pose.orientation.y = 0.0;
    waypoints.pose.orientation.z = 0.0;
    waypoints.pose.orientation.w = 1.0;
    // waypoints.scale.x = 0.1;
    // waypoints.scale.y = 0.1;
    // waypoints.scale.z = 0.1;
    // waypoints.color.a = 1.0; // Don't forget to set the alpha!
    // waypoints.color.r = 1.0;
    // waypoints.color.g = 0.0;
    // waypoints.color.b = 0.0;

	// for (int waypoint_num = 0; waypoint_num < traj_.GetWaypointCount(); ++waypoint_num)
	// {
	// 	MinSnapTraj::Waypoint waypoint;
	// 	traj_.GetWaypoint(waypoint_num, waypoint);
	// 	geometry_msgs::msg::Point new_waypoint;
	// 	new_waypoint.x = waypoint.pos[0];
	// 	new_waypoint.y = waypoint.pos[1];
	// 	new_waypoint.z = waypoint.pos[2];
	// 	waypoints.points.push_back(new_waypoint);
	// }
	// waypoint_publisher_->publish(waypoints);
	// path
	if (traj_.solved())
	{
		waypoints.header.stamp = this->get_clock()->now();
		waypoints.type = visualization_msgs::msg::Marker::LINE_LIST;
		waypoints.id = 1;
		waypoints.points.clear();
		waypoints.scale.x = 0.01;
		waypoints.scale.y = 0.01;
		waypoints.scale.z = 0.01;
		waypoints.color.a = 1.0; // Don't forget to set the alpha!
		waypoints.color.r = 0.0;
		waypoints.color.g = 0.0;
		waypoints.color.b = 1.0;
		double time_step = traj_.EndTime() / 100;
		State state;
		geometry_msgs::msg::Point new_path;
		traj_.Evaluate(0.0, state);
		for (double time = time_step; time < traj_.EndTime(); time += time_step)
		{
			new_path.x = state.x;
			new_path.y = state.y;
			new_path.z = state.z;
			waypoints.points.push_back(new_path);
			traj_.Evaluate(time, state);
			new_path.x = state.x;
			new_path.y = state.y;
			new_path.z = state.z;
			waypoints.points.push_back(new_path);
		}
		path_vis_publisher_->publish(waypoints);
	}
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
