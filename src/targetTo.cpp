#include <Sagitari.h>
#include <message_filters/subscriber.h>
#include <uart_process_2/uart_send.h>

using namespace sensor_msgs;
using namespace message_filters;

double Sagitari::averageFilter(double distance){
    queue<double> filterdistance;
    double sum=0;
    sum += distance;
    filterdistance.push(distance);
    if(filterdistance.size() > 10){
        double a = filterdistance.front();
        filterdistance.pop();
        sum -= a;
    }
    distance = sum / filterdistance.size();
    return distance;
}

float toFixed(double in) {
    float result = (int)(in * 100);
    return result / 100;
}
void Sagitari::targetTo(const EulerAngle& currentAngle, const EulerAngle& predictAngle, double distance, bool hasTarget, int predictLatency) {
    uart_process_2::uart_send send_msg;

    // send_msg.curYaw = 1;
    // send_msg.curPitch = 1;
    send_msg.curYaw = toFixed(-currentAngle.yaw);
    send_msg.curPitch = toFixed(currentAngle.pitch);
    send_msg.curDistance = distance;
    send_msg.time = hasTarget;
    send_msg.attackFlag = suggestFire;
    send_msg.predictYaw = predictAngle.yaw;
    send_msg.predictPitch = predictAngle.pitch;
    send_msg.predictLatency = predictLatency;
    this->uartPublisher.publish(send_msg);
    this->uartSent = send_msg;
}