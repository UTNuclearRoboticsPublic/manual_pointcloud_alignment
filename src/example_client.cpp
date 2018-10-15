

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

// to introduce error
#include <geometry_msgs/TransformStamped.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <manual_pointcloud_alignment/manual_pointcloud_alignment.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

int main (int argc, char **argv)
{ 
	ros::init(argc, argv, "registration_example");

	ros::NodeHandle nh;

	std::string topic;
	std::string bag_name_1;
	std::string bag_name_2; 
	nh.param<std::string>("alignment_example/bag_topic", topic, "laser_mapper/local_dense_cloud");
	nh.param<std::string>("alignment_example/bag_name_1", bag_name_1, "./cloud_1.bag");
	nh.param<std::string>("alignment_example/bag_name_2", bag_name_2, "./cloud_2.bag");

	ros::Publisher first_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("alignment_example/first_cloud", 1);
	ros::Publisher second_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("alignment_example/second_cloud", 1);

	ros::Publisher final_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("alignment_example/final_cloud", 1);



	// ----------------------------------------------------------------------------------------
	// 										Load From Bags
	// ----------------------------------------------------------------------------------------
	sensor_msgs::PointCloud2 first_cloud, second_cloud;
	
	ROS_INFO_STREAM("[RegistrationClient] Loading clouds from bag files, using bag names: " << bag_name_1 << " and " << bag_name_2 << " and topic name " << topic << ".");
	rosbag::Bag bag_1; 
	bag_1.open(bag_name_1, rosbag::bagmode::Read);

	std::vector<std::string> topics;
	topics.push_back(topic);
	//topics.push_back("/rosout");
	rosbag::View view_1(bag_1, rosbag::TopicQuery(topics));

	BOOST_FOREACH(rosbag::MessageInstance const m, view_1)
    {
    	ROS_INFO_STREAM("actually entering the loop");
        sensor_msgs::PointCloud2::ConstPtr cloud_1_ptr = m.instantiate<sensor_msgs::PointCloud2>();
        if (cloud_1_ptr != NULL)
            first_cloud = *cloud_1_ptr;
        else
        	ROS_ERROR_STREAM("[RegistrationClient] Cloud caught for first cloud is null...");
    }
    bag_1.close();

    rosbag::Bag bag_2; 
	bag_2.open(bag_name_2, rosbag::bagmode::Read);

	rosbag::View view_2(bag_2, rosbag::TopicQuery(topics));

	foreach(rosbag::MessageInstance const m, view_2)
    {
        sensor_msgs::PointCloud2::ConstPtr cloud_2_ptr = m.instantiate<sensor_msgs::PointCloud2>();
        if (cloud_2_ptr != NULL)
            second_cloud = *cloud_2_ptr;
        else
        	ROS_ERROR_STREAM("[RegistrationClient] Cloud caught for second cloud is null...");
    }
    bag_2.close();

    ROS_INFO_STREAM("[RegistrationClient] Both clouds collected from bag files - cloud sizes are " << first_cloud.height*first_cloud.width << " and " << second_cloud.height*second_cloud.width << " points, respectively.");



    // ----------------------------------------------------------------------------------------
	// 										Alignment
	// ----------------------------------------------------------------------------------------
    ROS_INFO_STREAM("[RegistrationClient] Interposing manual error alignment.");
	ros::ServiceClient manual_alignment_server = nh.serviceClient<manual_pointcloud_alignment::manual_alignment_service>("manual_pointcloud_alignment/align_clouds");
	manual_pointcloud_alignment::manual_alignment_service alignment_service;
	alignment_service.request.fixed_cloud = first_cloud;
	alignment_service.request.cloud_to_align = second_cloud;
	while(ros::ok() && !manual_alignment_server.call(alignment_service))
	{
		ROS_ERROR_STREAM("[RegistrationClient] Attempt to call alignment service failed... prob not up yet. Waiting and trying again.");
		ros::Duration(0.5).sleep();
	}
	ROS_INFO_STREAM("[RegistrationClient] Final manual transform: " << alignment_service.response.transform);



	while(ros::ok())
	{
		first_cloud_pub.publish(first_cloud);
		second_cloud_pub.publish(second_cloud);
		final_cloud_pub.publish(alignment_service.response.aligned_cloud);
		ros::Duration(1.0).sleep();
	}
}