
#include "manual_pointcloud_alignment/manual_pointcloud_alignment.h"

class TerminalPointcloudAlignment : ManualPointcloudAlignment
{
public: 
	TerminalPointcloudAlignment();	
	bool findInputTransform();
	bool checkAcceptance();
	bool transformInputLoop(sensor_msgs::PointCloud2 &fixed_cloud, sensor_msgs::PointCloud2 &cloud_to_align, geometry_msgs::TransformStamped &output_transform);
};