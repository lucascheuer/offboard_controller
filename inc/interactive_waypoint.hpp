#pragma once
#include <chrono>
#include <memory>
#include <string>

#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/bool.hpp>

class InteractiveWaypointManager : public rclcpp::Node
{
public:
	InteractiveWaypointManager();
	void MakeInteractiveWaypoint();
	void AddWaypoint();
	inline void	applyChanges()
	{
		server_->applyChanges();
	}
private:
	void InteractionCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &interaction);
	void UpdateCallback();
	void RemoveWaypoint(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &interaction);
	void ExecutePath();
	void SendWaypoints();

	std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
	interactive_markers::MenuHandler menu_handler_;
	rclcpp::TimerBase::SharedPtr interaction_framerate_timer_;

	rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoint_array_pub_;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr execute_pub_;
};