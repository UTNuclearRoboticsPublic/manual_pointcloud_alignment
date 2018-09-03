# Table of Contents
1. [About](#about)
2. [Parameter Setup](#parameter-setup)
3. [Usage](#usage)

## About
This package provides a utility to allow users to manually input an initial alignment for clouds to be registered algorithmically. The utility can also be used to provide a rough manual alignment INSTEAD of automated registration for cases were this is sufficient. 

The package is set up to be modular across different input interfaces. Currently, three formats are supported: terminal input, ros service input, and interactive marker input, with the last being the recommended method.

## Parameter Setup
Parameter setup for this package is fairly straightforward, following convention in the [pointcloud_processing_server](https://github.com/UTNuclearRoboticsPublic/pointcloud_processing_server) package. Preprocessing specifications are provided to allow the user to downsample and clip the cloud to make it more manageable for transforming and updating during alignment. It is recommended that a clipping task and a voxelization task are undertaken to downsample the input cloud prior to visualization and manual alignment in order to speed the process, at least for large clouds. This is not strictly necessary, though. Either way, the output clouds are created from the original fully dense versions of the clouds, transformed via the final transform selected by the user. 

## Usage
Regardless of the input type, transform specification works by the user inputting a transform, visualizing the resulting alignment, and then choosing whether it is acceptable. If not, a new transform is input until an acceptable output is found. Once a good alignment is selected, the original input cloud to be aligned is transformed and output. All that changes across input methods is the manner in which transforms are specified. 

Visualization of alignments is accomplished simply by publishing the aligned (downsampled) clouds to ROS topics. It is expected that the user use RViz to visualize the ROS messages in order to determine whether the alignment is acceptable. 

#### Client End
In all cases, alignments are run by a server which queries the user for transform inputs after it receives a ROS service request. The service request contains only the two clouds to be aligned - one that is fixed and the other to be transformed such that it aligns with the fixed cloud. 

The user should create a ros ServiceClient within their client file and use it to call a service on a service object of the type manual_pointcloud_alignment::manual_alignment_service, with the two requesite input clouds populated, using the service: 

/manual_pointcloud_alignment/align_clouds

The user will also need to rosrun a node of the input type required (eg, rosrun manual_pointcloud_alignment interactive_marker_pointcloud_alignment). 

#### Interactive Markers
In this input type, a 6 DoF interactive marker is generated when an alignment service request is received by the server. The user drags this marker around in space using the arrows and rotates it using the rotation circles in order to specify a transform. Once the marker is in a suitable location, the user sends an empty trigger service to the service: 

/manual_pointcloud_alignment/display_transform

This will force the system to display the new transform. If the transform is not acceptable, the user can repeat this process, dragging the marker around and repeatedly calling this service. Once an acceptable transform is found, the user accepts it by calling an empty trigger service to: 

/manual_pointcloud_alignment/accept_transform

#### Service Input
In this input type, the user inputs a transform by calling the service:

/manual_pointcloud_alignment/input_transform

using a service object of type manual_pointcloud_alignment::transform_input_service. This type has a geometry_msgs::TransformStamped as the only request field, and a simple success boolean as the only response field. Once this service is called, the system will automatically display the new alignment associated with the requested transform. If the user doesn't like this transform, they can call the input service again ad infinitum. Otherwise, they can call an empty trigger service to accept the alginment using the service: 

/manual_pointcloud_alignment/accept_transform

#### Terminal Input
In this input type, the server will request input via the terminal, using clear instructions that walk the user through inputting the 6 DoF transform requested. After each transform is input the alignment will be updated and published, and the user will be prompted to accept it or to enter a new transform. 
