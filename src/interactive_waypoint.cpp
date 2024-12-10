#include "interactive_waypoint.hpp"
using std::placeholders::_1;
InteractiveWaypointManager::InteractiveWaypointManager() : Node("interactive_waypoint_manager"), menu_handler_()
{
	server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
		"interactive_waypoint_manager",
		get_node_base_interface(),
		get_node_clock_interface(),
		get_node_logging_interface(),
		get_node_topics_interface(),
		get_node_services_interface()
	);

	waypoint_array_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/min_snap/path", 10);
	// menu_handler_.insert("Execute Path", std::bind(&InteractiveWaypointManager::ExecutePath, this));
	menu_handler_.insert("Add new waypoint", std::bind(&InteractiveWaypointManager::AddWaypoint, this));
	menu_handler_.insert("Remove This Waypoint", std::bind(&InteractiveWaypointManager::RemoveWaypoint, this, _1));
	// interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler_.insert("Delete");
	interaction_framerate_timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&InteractiveWaypointManager::UpdateCallback, this));
}

void InteractiveWaypointManager::UpdateCallback()
{
	SendWaypoints();
	server_->applyChanges();
}

void InteractiveWaypointManager::InteractionCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &interaction)
{

	switch (interaction->event_type)
	{
		case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:
			SendWaypoints();
			break;
		default:
			break;
	}
	server_->applyChanges();
	
}

void InteractiveWaypointManager::SendWaypoints()
{
	geometry_msgs::msg::PoseArray waypoints;
	
	waypoints.header.frame_id = "map";
	visualization_msgs::msg::InteractiveMarker marker;
	
	int waypoint_count = int(server_->size());
	for (int current_waypoint_number = 0; current_waypoint_number < waypoint_count; ++current_waypoint_number)
	{
		if (server_->get("Waypoint " + std::to_string(current_waypoint_number), marker))
		{
			waypoints.poses.push_back(marker.pose);
		} else
		{
			break;
		}
	}
	waypoints.header.stamp = this->get_clock()->now();
	waypoint_array_pub_->publish(waypoints);
}

void InteractiveWaypointManager::AddWaypoint()
{
	RCLCPP_INFO(get_logger(), "adding a new waypoint");
	int waypoint_number = int(server_->size());
	visualization_msgs::msg::InteractiveMarker marker;
	marker.header.frame_id = "map";
	marker.pose.position.x = 0.0;
	marker.pose.position.y = 0.0;
	marker.pose.position.z = 5.0;
	marker.scale = 0.5;
	marker.name = "Waypoint " + std::to_string(waypoint_number);
	marker.description = "Waypoint " + std::to_string(waypoint_number);

	// make the middle sphere
	visualization_msgs::msg::Marker sphere_visual;
	sphere_visual.type = visualization_msgs::msg::Marker::SPHERE;
	sphere_visual.scale.x = 0.25;
	sphere_visual.scale.y = 0.25;
	sphere_visual.scale.z = 0.25;
	sphere_visual.color.r = 0.0;
	sphere_visual.color.g = 0.0;
	sphere_visual.color.b = 1.0;
	sphere_visual.color.a = 1.0;
	visualization_msgs::msg::InteractiveMarkerControl sphere_control;
	sphere_control.always_visible = true;
	sphere_control.markers.push_back(sphere_visual);
	marker.controls.push_back(sphere_control);
	marker.controls[0].interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D;

	// add the 3 dof movement
	visualization_msgs::msg::InteractiveMarkerControl arrow_control;
	tf2::Quaternion arrow_orientation(1.0, 0.0, 0.0, 1.0);
	arrow_orientation.normalize();
	arrow_control.orientation = tf2::toMsg(arrow_orientation);
	arrow_control.name = "move_x";
	arrow_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
	marker.controls.push_back(arrow_control);

	arrow_orientation = tf2::Quaternion(0.0, 1.0, 0.0, 1.0);
	arrow_orientation.normalize();
	arrow_control.orientation = tf2::toMsg(arrow_orientation);
	arrow_control.name = "move_z";
	arrow_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
	marker.controls.push_back(arrow_control);

	arrow_orientation = tf2::Quaternion(0.0, 0.0, 1.0, 1.0);
	arrow_orientation.normalize();
	arrow_control.orientation = tf2::toMsg(arrow_orientation);
	arrow_control.name = "move_y";
	arrow_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
	marker.controls.push_back(arrow_control);
	server_->insert(marker);
	server_->setCallback(marker.name, std::bind(&InteractiveWaypointManager::InteractionCallback, this, _1));
	menu_handler_.apply(*server_, marker.name);
}

void InteractiveWaypointManager::RemoveWaypoint(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &interaction)
{
	int waypoint_count = int(server_->size());
	if (waypoint_count <= 1)
	{
		return;
	}
	visualization_msgs::msg::InteractiveMarker marker;
	int waypoint_number = std::stoi(interaction->marker_name.substr(8));
	
	server_->erase(interaction->marker_name);
	for (int current_waypoint_number = waypoint_number + 1; current_waypoint_number < waypoint_count; ++current_waypoint_number)
	{
		if (server_->get("Waypoint " + std::to_string(current_waypoint_number), marker))
		{
			marker.name = "Waypoint " + std::to_string(current_waypoint_number - 1);
			marker.description = "Waypoint " + std::to_string(current_waypoint_number - 1);
			if (server_->erase("Waypoint " + std::to_string(current_waypoint_number)))
			{
			}
			server_->insert(marker);
			server_->setCallback(marker.name, std::bind(&InteractiveWaypointManager::InteractionCallback, this, _1));
			menu_handler_.apply(*server_, marker.name);
		} else
		{
			break;
		}
		RCLCPP_INFO(get_logger(), "");
	}
	server_->applyChanges();
}


int main(int argc, char ** argv)
{
	// setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	// rclcpp::init(argc, argv);
	// rclcpp::spin(std::make_shared<InteractiveWaypointManager>());
	// rclcpp::shutdown();
	// return 0;
	rclcpp::init(argc, argv);

	auto interactive_waypoints = std::make_shared<InteractiveWaypointManager>();
	interactive_waypoints->AddWaypoint();
	interactive_waypoints->applyChanges();

	rclcpp::executors::SingleThreadedExecutor executor;
	executor.add_node(interactive_waypoints);
	RCLCPP_INFO(interactive_waypoints->get_logger(), "Ready");
	executor.spin();
	rclcpp::shutdown();
}

