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
    if(yaw > -1 && yaw < 1) yaw = 0;
    
    uart_process_2::uart_send send_msg;
    send_msg.xdata = toFixed(-yaw);
    send_msg.ydata = toFixed(pitch);
    send_msg.zdata = distance;
    send_msg.tdata = hasTarget;
    send_msg.Cmdata = suggestFire;
    this->uartPublisher.publish(send_msg);
    this->uartSent = send_msg;
    std::cout << "targetTo: yaw=" << yaw << ", pitch=" << pitch << std::endl;
}