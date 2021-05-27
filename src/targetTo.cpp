#include <Sagitari.h>
#include <message_filters/subscriber.h>
#include <uart_process_2/uart_send.h>

using namespace sensor_msgs;
using namespace message_filters;
float toFixed(double in) {
    float result = (int)(in * 100);
    return result / 100;
}
void Sagitari::targetTo(double yaw, double pitch, double distance, bool hasTarget) {
    uart_process_2::uart_send send_msg;
    send_msg.xdata = toFixed(-yaw);
    send_msg.ydata = toFixed(pitch);
    send_msg.zdata = distance;
    send_msg.tdata = hasTarget;
    send_msg.Cmdata = 0;
    this->uartPublisher.publish(send_msg);
    std::cout << "targetTo: yaw=" << yaw << ", pitch=" << pitch << std::endl;
}