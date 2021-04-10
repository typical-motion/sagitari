#include "Sagitari.h"
#include <ros/ros.h>
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "sagitari_node");
	ros::NodeHandle nh;//开进程
	ros::Rate rate(150);

	DeviceProvider *device;
#ifdef DEBUG
	device = new IODeviceProvider();
#else
	device = new ROSDeviceProvider();
#endif
		Sagitari sagitari(IdentityColor::IDENTITY_BLUE, device);

	cv::Mat target;
	while (ros::ok()) {
		ros::spinOnce();
        rate.sleep();
		*device >> target;
		sagitari << target;
		int key = cv::waitKey(1);
		if (key == 'q') {
			break;
		}
		else if (key == 'p') {
			cv::waitKey(0);
		}

	}
	return 0;
}
