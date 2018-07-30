
#include "manual_pointcloud_alignment/manual_pointcloud_alignment.h"

class ServicePointcloudAlignment : ManualPointcloudAlignment
{
public:
	ServicePointcloudAlignment();
	bool findInputTransform();
	bool checkAcceptance();
	bool transformInputLoop(sensor_msgs::PointCloud2 &fixed_cloud, sensor_msgs::PointCloud2 &cloud_to_align, geometry_msgs::TransformStamped &output_transform);
private: 
	bool inputTransformCallback(manual_pointcloud_alignment::transform_input_service::Request &req, manual_pointcloud_alignment::transform_input_service::Response &res);
	bool acceptTransformCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

	ros::ServiceServer transform_input_server_;
	ros::ServiceServer transform_acceptance_server_;
};