
#include "manual_pointcloud_alignment/interactive_marker_pointcloud_alignment.h"

InteractiveMarkerPointcloudAlignment::InteractiveMarkerPointcloudAlignment()
  : interactive_marker_server_("cloud_moving_marker")
{
	transform_display_server_    = nh_general_.advertiseService("manual_pointcloud_alignment/display_transform", &InteractiveMarkerPointcloudAlignment::displayTransformCallback, this);
	transform_acceptance_server_ = nh_general_.advertiseService("manual_pointcloud_alignment/accept_transform", &InteractiveMarkerPointcloudAlignment::acceptTransformCallback, this);

	increment_transforms_ = false;

	buildInteractiveMarker();

	while(ros::ok())
	{
		ros::spin();
		ros::Duration(0.05).sleep();
	}
}


void InteractiveMarkerPointcloudAlignment::buildInteractiveMarker()
{
	// Interactive Marker 
	interactive_marker_.header.frame_id = "map";
	interactive_marker_.header.stamp 	= ros::Time::now();
	interactive_marker_.name 			= "cloud_moving_interactive_marker";
	interactive_marker_.description  	= "6 DOF Control to Manually Align Pointclouds";
	interactive_marker_.scale 			= 0.5;
	
	// Visual Marker (Spherical Center of Arrows)
	visual_marker_.type = visualization_msgs::Marker::CUBE;
	visual_marker_.scale.x = 0.1;
	visual_marker_.scale.y = 0.1;
	visual_marker_.scale.z = 0.1;
	visual_marker_.color.r = 1.0;
	visual_marker_.color.g = 0.0;
	visual_marker_.color.b = 1.0;
	visual_marker_.color.a = 1.0;
	visual_controller_.always_visible = true;
	visual_controller_.markers.push_back(visual_marker_);
	interactive_marker_.controls.push_back(visual_controller_);

	// Interactive Marker Control
	//   Control in X
	marker_controller_.orientation.w = 1;
    marker_controller_.orientation.x = 1;
    marker_controller_.orientation.y = 0;
    marker_controller_.orientation.z = 0;
    marker_controller_.name = "rotate_x";
    marker_controller_.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    interactive_marker_.controls.push_back(marker_controller_);
    marker_controller_.name = "move_x";
    marker_controller_.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    interactive_marker_.controls.push_back(marker_controller_);
    //   Control in Y
    marker_controller_.orientation.w = 1;
    marker_controller_.orientation.x = 0;
    marker_controller_.orientation.y = 1;
    marker_controller_.orientation.z = 0;
    marker_controller_.name = "rotate_z";
    marker_controller_.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    interactive_marker_.controls.push_back(marker_controller_);
    marker_controller_.name = "move_z";
    marker_controller_.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    interactive_marker_.controls.push_back(marker_controller_);
    //   Control in Z
    marker_controller_.orientation.w = 1;
    marker_controller_.orientation.x = 0;
    marker_controller_.orientation.y = 0;
    marker_controller_.orientation.z = 1;
    marker_controller_.name = "rotate_y";
    marker_controller_.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    interactive_marker_.controls.push_back(marker_controller_);
    marker_controller_.name = "move_y";
    marker_controller_.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    //marker_controller_.markers.push_back(visual_marker_);
	interactive_marker_.controls.push_back(marker_controller_);

	interactive_marker_server_.insert(interactive_marker_, boost::bind(&InteractiveMarkerPointcloudAlignment::interactiveMarkerFeedback, this, _1));
	interactive_marker_server_.applyChanges();
	ROS_INFO_STREAM("[InteractiveMarkerPointcloudAlignment] Finished creating interactive marker object and uploaded it to server.");
}


void InteractiveMarkerPointcloudAlignment::interactiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
	ROS_DEBUG_STREAM("[InteractiveMarkerPointcloudAlignment] Received new marker feedback - pose: " << feedback->pose);
}


// This function allows users to input desired cloud alignment offsets using InteractiveMarkers
bool InteractiveMarkerPointcloudAlignment::findInputTransform()
{
	while(!transform_received_ && !transform_accepted_ && ros::ok())
	{
		ros::spinOnce();
		ros::Duration(0.05).sleep();
	}	 
	return true;
}


bool InteractiveMarkerPointcloudAlignment::checkAcceptance()
{
	return true;
}



bool InteractiveMarkerPointcloudAlignment::displayTransformCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	visualization_msgs::InteractiveMarker marker;
	interactive_marker_server_.get("cloud_moving_interactive_marker", marker);
	last_transform_.transform.translation.x = marker.pose.position.x;
	last_transform_.transform.translation.y = marker.pose.position.y;
	last_transform_.transform.translation.z = marker.pose.position.z;
	last_transform_.transform.rotation 		= marker.pose.orientation;
	last_transform_.header 					= marker.header;

	ROS_INFO_STREAM("[InteractiveMarkerPointcloudAlignmnet] Received callback to display current transform in RViz - current transform: ");
	ROS_INFO_STREAM(last_transform_);
	transform_received_ = true;
	res.success = true;
	return true;
}


bool InteractiveMarkerPointcloudAlignment::acceptTransformCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	transform_accepted_ = true;
	res.success = true;
	return true;
}



bool InteractiveMarkerPointcloudAlignment::transformInputLoop(sensor_msgs::PointCloud2 &fixed_cloud, sensor_msgs::PointCloud2 &cloud_to_align, geometry_msgs::TransformStamped &output_transform)
{
	sensor_msgs::PointCloud2 new_aligned_cloud = cloud_to_align;

	while(ros::ok() && !transform_accepted_)
	{
		// Get a transform to try from the user 
		if( !findInputTransform() )
		{
			ROS_ERROR_STREAM("[ManualPointcloudAlignment] Attempt to load inputs failed. Exiting service call without changing the cloud...");
			return false;
		}
		transform_received_ = false;

		// Apply new transform to Cloud_To_Align
		tf2::doTransform(cloud_to_align, new_aligned_cloud, last_transform_);
		new_aligned_cloud.header = cloud_to_align.header;
		output_transform = last_transform_;

		// Update RViz display with transformed cloud and output transform to logging  
		displayClouds(fixed_cloud, new_aligned_cloud);
		ROS_INFO_STREAM("[ManualPointcloudAlignment] Received new transform.");
		ROS_INFO_STREAM("[ManualPointcloudAlignment] Translation: " << last_transform_.transform.translation.x << " " << last_transform_.transform.translation.y << " " << last_transform_.transform.translation.z);
		ROS_INFO_STREAM("[ManualPointcloudAlignment] Rotation:    " << last_transform_.transform.rotation.x << " " << last_transform_.transform.rotation.y << " " << last_transform_.transform.rotation.z << " " << last_transform_.transform.rotation.w);

		// Check with user whether new alignment is good
		//   checkAcceptance only returns false if the check itself fails to receive a response properly!
		//   if the user replies in the negative (transform is not good) then transform_accepted_ will be set to false and loop will repeat
		if( !checkAcceptance() )
		{
			ROS_ERROR_STREAM("[ManualPointcloudAlignment] Attempt to check transform acceptability failed. Exiting service call without changing the cloud...");
			return false;
		}
	}
	return true;
}



int main (int argc, char **argv)
{ 
	ros::init(argc, argv, "interactive_marker_pointcloud_alignment");
	
	InteractiveMarkerPointcloudAlignment cloud_aligner;

}