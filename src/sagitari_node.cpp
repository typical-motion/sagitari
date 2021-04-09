#include "Sagitari.h"
int main()
{

	DeviceProvider *device;
#ifdef DEBUG
	device = new IODeviceProvider();
#else
	device = new ROSDeviceProvider(); -
#endif
		Sagitari sagitari(IdentityColor::IDENTITY_BLUE, device);

	cv::Mat target;
	while (1) {
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
