
#include "manual_pointcloud_alignment/manual_pointcloud_alignment.h"

ManualPointcloudAlignment::ManualPointcloudAlignment()
{
	// Only the main server is placed on the 'main_service' nodehandle - isolates it from ros::spin calls INSIDE a service call
	server_ = nh_main_service_.advertiseService("manual_pointcloud_alignment/align_clouds", &ManualPointcloudAlignment::manualPointcloudAlignment, this);
	nh_main_service_.setCallbackQueue(&callback_queue_);	
	// All other ros::spin functions are run via a separate nodehandle, which can be spun without entering a new main service call
	//   This prevents us from starting a new service call before finishing one that's in progress
	//   Otherwise could keep going down new rabbitholes without ever finishing a call, if requests come too quickly 
	//   ...Alternatively, maybe the top-level service should be an action - think more about this later
	fixed_cloud_pub_ = nh_general_.advertise<sensor_msgs::PointCloud2>("manual_pointcloud_alignment/fixed_cloud", 1, this);
	aligned_cloud_pub_ = nh_general_.advertise<sensor_msgs::PointCloud2>("manual_pointcloud_alignment/aligned_cloud", 1, this);

	preprocessing_client_ = nh_general_.serviceClient<pointcloud_processing_server::pointcloud_process>("pointcloud_service");
	PointcloudTaskCreation::processFromYAML(&preprocess_, "manual_pointcloud_alignment_preprocess", "pointcloud_process");
}

// Preprocess input clouds in order to speed processing
bool ManualPointcloudAlignment::preprocess(sensor_msgs::PointCloud2 &input_cloud, sensor_msgs::PointCloud2 &output_cloud)
{
	preprocess_.request.pointcloud = input_cloud;
	while(ros::ok() && !preprocessing_client_.call(preprocess_))
	{
		ROS_ERROR_STREAM("[ManualPointcloudAlignment] Attempted to call preprocessing service, but failed - server is probably not up yet. Waiting and trying again...");
		ros::Duration(0.5).sleep();
	}	

	int preprocess_size = preprocess_.response.task_results.size();
	output_cloud = preprocess_.response.task_results[preprocess_size-1].task_pointcloud;
	
	return true;
}


// Updates the RViz display of the two clouds being aligned 
void ManualPointcloudAlignment::displayClouds(sensor_msgs::PointCloud2 fixed_cloud, sensor_msgs::PointCloud2 transformed_cloud)
{
	ROS_INFO_STREAM("[ManualPointcloudAlignment] Displaying new clouds.");

	fixed_cloud_pub_.publish(fixed_cloud);
	aligned_cloud_pub_.publish(transformed_cloud);

	ros::spinOnce();
}


void ManualPointcloudAlignment::multiplyTransforms(geometry_msgs::TransformStamped &current_transform, geometry_msgs::TransformStamped &new_transform)
{
	//ROS_DEBUG_STREAM("[ManualPointcloudAlignment] Transform Multiplication...")
	//ROS_DEBUG_STREAM("[ManualPointcloudAlignment] Current: " << current_transform.transform.translation.x << " " << current_transform.transform.translation.y << " " << current_transform.transform.translation.z);
	//ROS_DEBUG_STREAM("[ManualPointcloudAlignment] New:     " << new_transform.transform.translation.x << " " << new_transform.transform.translation.y << " " << new_transform.transform.translation.z);
	// Change Current Input to Homologous
	Eigen::Quaternionf current_rotation(current_transform.transform.rotation.w, current_transform.transform.rotation.x, current_transform.transform.rotation.y, current_transform.transform.rotation.z);
	Eigen::Matrix3f current_rotation_matrix = current_rotation.toRotationMatrix();
	Eigen::Matrix4f current_homologous;
	current_homologous << current_rotation_matrix(0,0), current_rotation_matrix(0,1), current_rotation_matrix(0,2), current_transform.transform.translation.x,
						  current_rotation_matrix(1,0), current_rotation_matrix(1,1), current_rotation_matrix(1,2), current_transform.transform.translation.y,
						  current_rotation_matrix(2,0), current_rotation_matrix(2,1), current_rotation_matrix(2,2), current_transform.transform.translation.z,
						    						 0,  						   0,  							 0,  										1;
	// Change New Input to Homologous
	Eigen::Quaternionf new_rotation(new_transform.transform.rotation.w, new_transform.transform.rotation.x, new_transform.transform.rotation.y, new_transform.transform.rotation.z);
	Eigen::Matrix3f new_rotation_matrix = new_rotation.toRotationMatrix();
	Eigen::Matrix4f new_homologous;
	new_homologous << 	new_rotation_matrix(0,0), new_rotation_matrix(0,1), new_rotation_matrix(0,2), new_transform.transform.translation.x,
						new_rotation_matrix(1,0), new_rotation_matrix(1,1), new_rotation_matrix(1,2), new_transform.transform.translation.y,
						new_rotation_matrix(2,0), new_rotation_matrix(2,1), new_rotation_matrix(2,2), new_transform.transform.translation.z,
						  					   0,  						 0,  					   0,  									  1;
 	// Combine Transforms, Change to Quaternion 
 	Eigen::Matrix4f updated_homologous = current_homologous * new_homologous;
 	Eigen::Matrix3f updated_rotation_matrix;
 	updated_rotation_matrix << 	updated_homologous(0,0), updated_homologous(0,1), updated_homologous(0,2), 
 								updated_homologous(1,0), updated_homologous(1,1), updated_homologous(1,2), 
 								updated_homologous(2,0), updated_homologous(2,1), updated_homologous(2,2);
	Eigen::Quaternionf updated_rotation(updated_rotation_matrix);

	current_transform.transform.translation.x = updated_homologous(0,3);
	current_transform.transform.translation.y = updated_homologous(1,3);
	current_transform.transform.translation.z = updated_homologous(2,3);
	current_transform.transform.rotation.x = 	updated_rotation.x();
	current_transform.transform.rotation.y = 	updated_rotation.y();
	current_transform.transform.rotation.z = 	updated_rotation.z();
	current_transform.transform.rotation.w = 	updated_rotation.w();
}



// Top-level Service Callback 
bool ManualPointcloudAlignment::manualPointcloudAlignment(manual_pointcloud_alignment::manual_alignment_service::Request &req, manual_pointcloud_alignment::manual_alignment_service::Response &res)
{
	ROS_INFO_STREAM("[ManualPointcloudAlignment] Received service call to perform pointcloud alignment.");
	transform_accepted_ = false;

	// Initialize Transform as Identity
	//   This is only necessary for algorithms in which each transform input is multiplied against the previous one 
	//   In other cases each input transform may be taken as standalone (eg Interactive Markers) 
	if(increment_transforms_)
	{
		res.transform.header.stamp = ros::Time::now();
		res.transform.transform.translation.x = 0;
		res.transform.transform.translation.y = 0;
		res.transform.transform.translation.z = 0;
		res.transform.transform.rotation.x    = 0;
		res.transform.transform.rotation.y    = 0;
		res.transform.transform.rotation.z    = 0;
		res.transform.transform.rotation.w    = 1;
		last_transform_ = res.transform;
	}

	// Perform preprocessing (voxelization, clipping, segmentation...)
	sensor_msgs::PointCloud2 input_cloud_preprocessed, aligned_cloud_preprocessed;
	preprocess(req.fixed_cloud, input_cloud_preprocessed);
	preprocess(req.cloud_to_align, aligned_cloud_preprocessed);

	// Update RViz display of preprocessed clouds
	displayClouds(input_cloud_preprocessed, aligned_cloud_preprocessed);
	
	// The following function is pure virtual, implemented on a case-by-case basis only in derived classes 
 	if( !transformInputLoop(input_cloud_preprocessed, aligned_cloud_preprocessed, res.transform) )
 	{
 		ROS_INFO_STREAM("[ManualPointcloudAlignment] Attempt to iteratively find an acceptable transform failed - exiting service as failure!");
 		return false;
 	}

 	// Set up final outputs based on transforms found
 	ROS_INFO_STREAM("[ManualPointcloudAlignment] Final cloud transform has been accepted! Performing final transforms, populating outputs, and returning service.");
	tf2::doTransform(req.cloud_to_align, res.aligned_cloud, res.transform);
	res.aligned_cloud.header = req.cloud_to_align.header;
	res.success = transform_accepted_;
	return true;
}