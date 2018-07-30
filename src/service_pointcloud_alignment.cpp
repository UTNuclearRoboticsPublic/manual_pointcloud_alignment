
#include "manual_pointcloud_alignment/service_pointcloud_alignment.h"

ServicePointcloudAlignment::ServicePointcloudAlignment()
{
	transform_input_server_ = nh_general_.advertiseService("manual_pointcloud_alignment/input_transform", &ServicePointcloudAlignment::inputTransformCallback, this);
	transform_acceptance_server_ = nh_general_.advertiseService("manual_pointcloud_alignment/accept_transform", &ServicePointcloudAlignment::acceptTransformCallback, this);

	while(ros::ok())
	{
		callback_queue_.callAvailable(ros::WallDuration());
		ros::Duration(0.05).sleep();
	}
}

// This function is implemented as a general interface to allow custom GUIs to use this service
//   Users should publish the input parameters (6 floats associated with XYZ translation and RPY rotation)
//   to the input topic subscribed to below
bool ServicePointcloudAlignment::findInputTransform()
{
	while(!transform_received_ && !transform_accepted_ && ros::ok())
	{
		ros::spinOnce();
		ros::Duration(0.05).sleep();
	}	 
	return true;
}


bool ServicePointcloudAlignment::checkAcceptance()
{
	// Kinda hacky way to do this right now... but should never reach this point unless 
	return true;
}


bool ServicePointcloudAlignment::inputTransformCallback(manual_pointcloud_alignment::transform_input_service::Request &req, manual_pointcloud_alignment::transform_input_service::Response &res)
{
	last_transform_ = req.transform;
	res.success = true;
	return true;
}

bool ServicePointcloudAlignment::acceptTransformCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	res.success = true;
	return true;
}


bool ServicePointcloudAlignment::transformInputLoop(sensor_msgs::PointCloud2 &fixed_cloud, sensor_msgs::PointCloud2 &cloud_to_align, geometry_msgs::TransformStamped &output_transform)
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
		transform_received_ = false;
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
	ros::init(argc, argv, "service_pointcloud_alignment");
	
	ServicePointcloudAlignment cloud_aligner;

}