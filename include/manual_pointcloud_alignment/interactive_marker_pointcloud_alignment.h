
#include "manual_pointcloud_alignment/manual_pointcloud_alignment.h"

// Interactive Markers
#include <interactive_markers/interactive_marker_server.h>

class InteractiveMarkerPointcloudAlignment : ManualPointcloudAlignment
{
public:
	InteractiveMarkerPointcloudAlignment();
	bool findInputTransform();
	bool checkAcceptance();
	bool transformInputLoop(sensor_msgs::PointCloud2 &fixed_cloud, sensor_msgs::PointCloud2 &cloud_to_align, geometry_msgs::TransformStamped &output_transform);
private: 
	void interactiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
	bool displayTransformCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
	bool acceptTransformCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
	void buildInteractiveMarker();

	visualization_msgs::Marker marker_;
	visualization_msgs::InteractiveMarkerControl marker_control_;

	ros::ServiceServer transform_acceptance_server_;
	ros::ServiceServer transform_display_server_;

	// Interactive Marker Stuff
	interactive_markers::InteractiveMarkerServer interactive_marker_server_;
	visualization_msgs::InteractiveMarker interactive_marker_;
	visualization_msgs::Marker visual_marker_;
	visualization_msgs::InteractiveMarkerControl visual_controller_;
	visualization_msgs::InteractiveMarkerControl marker_controller_;
};