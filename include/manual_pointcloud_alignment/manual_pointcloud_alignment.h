
#include <ros/ros.h>
#include <iostream>
#include <ros/callback_queue.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_srvs/Trigger.h>

//#include <Eigen/Dense>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf/tf.h>

#include "manual_pointcloud_alignment/manual_alignment_service.h"
#include "manual_pointcloud_alignment/transform_input_service.h"
#include <pointcloud_processing_server/pointcloud_process.h>
#include <pointcloud_processing_server/pointcloud_task_creation.h>


class ManualPointcloudAlignment
{
public:
	ManualPointcloudAlignment();

	// These pure virtual functions are defined only within particular subclasses; there is no general implementation in this base class 
	virtual bool findInputTransform() = 0;
	virtual bool checkAcceptance() = 0;
	virtual bool transformInputLoop(sensor_msgs::PointCloud2 &fixed_cloud, sensor_msgs::PointCloud2 &cloud_to_align, geometry_msgs::TransformStamped &output_transform) = 0;

protected:
	// *** Service Callback ***
	//     - Top-level callback to entire alignmnet service 
	bool manualPointcloudAlignment(manual_pointcloud_alignment::manual_alignment_service::Request &req, manual_pointcloud_alignment::manual_alignment_service::Response &res);
	// *** Preprocessing  ***
	//     - Preprocess input clouds (eg downsamples them to lower densities) in order to speed processing
	//     - Depends on the Pointcloud_Processing_Server package
	bool preprocess(sensor_msgs::PointCloud2 &input_cloud, sensor_msgs::PointCloud2 &output_cloud);
	// *** Display Clouds ***
	//     - Updates the RViz display of the two clouds being aligned  
	void displayClouds(sensor_msgs::PointCloud2 fixed_cloud, sensor_msgs::PointCloud2 cloud_to_align);

	void multiplyTransforms(geometry_msgs::TransformStamped &total_transform, geometry_msgs::TransformStamped &last_transform);

	bool transform_received_;
	bool transform_accepted_;
	std::vector<float> leaf_sizes_;

	ros::NodeHandle nh_main_service_;		// This nodehandle is used for the main service callback (and incidentally, also all other ros::spin functionalities)
	ros::CallbackQueue callback_queue_; 	//   This queue is associated with the above nodehandle
	ros::NodeHandle nh_general_; 			// This nodehandle is used for all other ros::spin functionalities - this prevents receiving new main service callbacks while spinning inside a service
	
	ros::ServiceServer server_;
	ros::Publisher fixed_cloud_pub_;
	ros::Publisher aligned_cloud_pub_;
	ros::ServiceClient preprocessing_client_;

	geometry_msgs::TransformStamped last_transform_;
	bool increment_transforms_; 			// If this is true, each subsequent transform tested will be multiplied to the previous one. Else it starts over each time

	pointcloud_processing_server::pointcloud_process preprocess_;
};