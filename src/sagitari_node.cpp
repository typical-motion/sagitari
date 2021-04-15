#include "Sagitari.h"
#include <ros/ros.h>
#include <opencv2/core/utility.hpp>
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
	Sagitari sagitari(IdentityColor::IDENTITY_BLUE, device);
	if (parser.get<int>("usecam"))
	{
		device = new ROSDeviceProvider(&sagitari);
		sagitari.device = device;
		std::cout<< "Hey I'm here" << std::endl;

	}
	else
	{
		ros::NodeHandle nh;
		image_transport::ImageTransport it(nh);
		sagitari.debugPublisher = it.advertise("Sagitari/debugImage",1);

		device = new IODeviceProvider();
		sagitari.device = device;
		cv::Mat target;
		while (ros::ok())
		{
			*device >> target;
			sagitari << target;
			int key = cv::waitKey(1);
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
