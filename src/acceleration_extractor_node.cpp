#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <fstream>

std::ofstream accelerationFile;
double gravity;
bool firstImuMessageReceived = false;

void imuCallback(const sensor_msgs::Imu& msg)
{
	geometry_msgs::Vector3 msgBeforeTransform, msgAfterTransform;
	geometry_msgs::TransformStamped transformation;
	transformation.transform.rotation = msg.orientation;
	tf2::doTransform(msg.linear_acceleration, msgAfterTransform, transformation);
	
	if(!firstImuMessageReceived)
	{
		gravity = msgAfterTransform.z;
	}
	msgBeforeTransform.x = msgAfterTransform.x;
	msgBeforeTransform.y = msgAfterTransform.y;
	msgBeforeTransform.z = msgAfterTransform.z - gravity;
	
	tf2::Quaternion quaternion;
	tf2::fromMsg(msg.orientation, quaternion);
	transformation.transform.rotation = tf2::toMsg(quaternion.inverse());
	tf2::doTransform(msgBeforeTransform, msgAfterTransform, transformation);
	
	accelerationFile << msg.header.stamp << "," << msgAfterTransform.x << "," << msgAfterTransform.y << "," << msgAfterTransform.z << std::endl;
	firstImuMessageReceived = true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "acceleration_extractor_node");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	
	std::string accelerationFileName;
	pn.param<std::string>("acceleration_file_name", accelerationFileName, "accelerations.csv");
	accelerationFile.open(accelerationFileName);
	accelerationFile << "stamp,x,y,z" << std::endl;
	
	ros::Subscriber sub = n.subscribe("imu_topic", 1000, imuCallback);
	
	ros::spin();
	
	accelerationFile.close();
	
	return 0;
}
