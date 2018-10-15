
#include "manual_pointcloud_alignment/terminal_pointcloud_alignment.h"

TerminalPointcloudAlignment::TerminalPointcloudAlignment()
{
	//while(ros::ok())
	//{
		ros::spin();//callback_queue_.callAvailable(ros::WallDuration());
		ros::Duration(0.05).sleep();
	//}
}


// This function allows the user to specify alignment offsets using the terminal and iostream
bool TerminalPointcloudAlignment::findInputTransform()
{
	ROS_INFO_STREAM("[TerminalPointcloudAlignment] Prepped to take terminal input for alignment offset.");
	
	// Get XYZ Translation Inputs
	std::cin.clear();
	std::cout << "                            Enter X Translation:  ";
	std::cin >> last_transform_.transform.translation.x;
	std::cout << "                            Enter Y Translation:  ";
	std::cin >> last_transform_.transform.translation.y;
	std::cout << "                            Enter Z Translation:  ";
	std::cin >> last_transform_.transform.translation.z;

	// Get RPY Rotation Inputs
	float roll, pitch, yaw;
	std::cout << "                            Enter Roll:           ";
	std::cin >> roll;
	std::cout << "                            Enter Pitch:          ";
	std::cin >> pitch;
	std::cout << "                            Enter Yaw:            ";
	std::cin >> yaw;
	// Convert RPY to Quaternion
	tf::Quaternion quat_tf = tf::createQuaternionFromRPY(roll, pitch, yaw);
	last_transform_.transform.rotation.x = quat_tf.getAxis().getX()*sin(quat_tf.getAngle()/2);
	last_transform_.transform.rotation.y = quat_tf.getAxis().getY()*sin(quat_tf.getAngle()/2);
	last_transform_.transform.rotation.z = quat_tf.getAxis().getZ()*sin(quat_tf.getAngle()/2);
	last_transform_.transform.rotation.w = cos(quat_tf.getAngle()/2);

	transform_received_ = true;

	// Maybe later on add some std::fpclassify checks for input validity
	return true;
}


bool TerminalPointcloudAlignment::checkAcceptance()
{
	bool response_acceptable = false;
	while(ros::ok() && !response_acceptable)
	{
		ROS_INFO_STREAM("[TerminalPointcloudAlignment] Is the transform found acceptable?");
		std::cout <<    "                   Type Y or N, then enter: ";
		char acceptability;
		std::cin.clear();
		std::cin >> acceptability;
		switch (acceptability)
		{
			case 'Y':
				ROS_INFO_STREAM("[TerminalPointcloudAlignment] Transform Accepted.");
				transform_accepted_ = true;
				response_acceptable = true;
				break;
			case 'N':
				ROS_INFO_STREAM("[TerminalPointcloudAlignment] Transform not accepted.");
				transform_accepted_ = false;
				response_acceptable = true;
				break;
			default:
				ROS_ERROR_STREAM("[TerminalPointcloudAlignment] Value entered is invalid - not a Y or N. You entered: " << acceptability);
				response_acceptable = false;
		}
	}
	return true;
}



bool TerminalPointcloudAlignment::transformInputLoop(sensor_msgs::PointCloud2 &fixed_cloud, sensor_msgs::PointCloud2 &cloud_to_align, geometry_msgs::TransformStamped &output_transform)
{
	while(ros::ok() && !transform_accepted_)
	{
		// Get a transform to try from the user 
		transform_received_ = false;
		if( !findInputTransform() )
		{
			ROS_ERROR_STREAM("[ManualPointcloudAlignment] Attempt to load inputs failed. Exiting service call without changing the cloud...");
			return false;
		}

		// Apply new transform to Cloud_To_Align
		sensor_msgs::PointCloud2 new_aligned_cloud;
		tf2::doTransform(cloud_to_align, new_aligned_cloud, last_transform_);
		new_aligned_cloud.header = cloud_to_align.header;
		cloud_to_align = new_aligned_cloud;
		multiplyTransforms(output_transform, last_transform_);

		// Update RViz display with transformed cloud and output transform to logging  
		displayClouds(fixed_cloud, cloud_to_align);
		ROS_INFO_STREAM("[ManualPointcloudAlignment] Received new transform.");
		ROS_INFO_STREAM("[ManualPointcloudAlignment] Translation: " << last_transform_.transform.translation.x << " " << last_transform_.transform.translation.y << " " << last_transform_.transform.translation.z);
		ROS_INFO_STREAM("[ManualPointcloudAlignment] Rotation:    " << last_transform_.transform.rotation.x << " " << last_transform_.transform.rotation.y << " " << last_transform_.transform.rotation.z << " " << last_transform_.transform.rotation.w);
		ROS_INFO_STREAM("[ManualPointcloudAlignment] Current entire transform:");
		ROS_INFO_STREAM("[ManualPointcloudAlignment] Translation: " << output_transform.transform.translation.x << " " << output_transform.transform.translation.y << " " << output_transform.transform.translation.z);
		ROS_INFO_STREAM("[ManualPointcloudAlignment] Rotation:    " << output_transform.transform.rotation.x << " " << output_transform.transform.rotation.y << " " << output_transform.transform.rotation.z << " " << output_transform.transform.rotation.w);

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
	ros::init(argc, argv, "terminal_pointcloud_alignment");
	
	TerminalPointcloudAlignment cloud_aligner;

}