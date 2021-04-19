#include "Sagitari.h"
#include <ros/ros.h>
#include <opencv2/core/utility.hpp>
#include <sagitari_debug/sagitari_img_debug.h>
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "sagitari");

	const cv::String keys =
		"{help h usage ? |      | print this message   }"
		"{usecam         |0     | Whether to use camera}";
	cv::CommandLineParser parser(argc, argv, keys);

	DeviceProvider *device;
	if (parser.has("help"))
	{
		parser.printMessage();
		return 0;
	}
	Sagitari sagitari(IdentityColor::IDENTITY_RED, device);
	if (parser.get<int>("usecam"))
	{
		device = new ROSDeviceProvider(&sagitari);
		sagitari.device = device;
	}
	else
	{
		ros::NodeHandle nh;
		image_transport::ImageTransport it(nh); 
		sagitari.debugImagePublisher = nh.advertise<sagitari_debug::sagitari_img_debug>("Sagitari/debugImage", 1);
		sagitari.originalImagePublisher = it.advertise("Sagitari/originalImage", 1);
		device = new IODeviceProvider();
		sagitari.device = device;
		cv::Mat target;
		while (ros::ok())
		{
			*device >> target;
			sagitari << target;
			cv::imshow("target", target);
			int key = cv::waitKey(10);
			if (key == 'q')
			{
				break;
			}
			else if (key == 'p')
			{
				cv::waitKey(0);
			}
		}
	}

	return 0;
}
