#include "motor_control.h"
#include "stdlib.h"
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include<tf/transform_broadcaster.h>
#include<nav_msgs/Odometry.h>
#include "sensor_msgs/Joy.h"

MotorControl*mb;
bool js_control_mode;

void callback(const sensor_msgs::Joy& joy)
{
    if(joy.buttons[10] == 1){
        js_control_mode = true;
        mb->wheelEnable(2, 1);
        mb->wheelEnable(3, 1);
        mb->wheelEnable(4, 1);
        mb->wheelEnable(5, 1);
    }
    if(joy.buttons[9] == 1){
        js_control_mode = false;
        mb->wheelEnable(2, 0);
        mb->wheelEnable(3, 0);
        mb->wheelEnable(4, 0);
        mb->wheelEnable(5, 0);
    }
    if(js_control_mode)
        mb->setSpeedXYW(0.3*joy.axes[3]*abs(joy.axes[3]), 0.3*joy.axes[2]*abs(joy.axes[2]), 0.3*joy.axes[0]*abs(joy.axes[0]));
}

int main(int argc, char* argv[])
{
    js_control_mode = true;
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/joy", 1, callback);

    ros::Rate loop_rate(7);
    int times = 0;
    struct timeval t1,t2;
    double timeuse;
    int enable = atoi(argv[1]);
    __useconds_t sleeptime = 10000;
    int speed = 0;
    int dir = 1;
    bool add_plus = false;
    int16_t speed_1;
    int16_t speed_2;
    int16_t speed_3;
    int16_t speed_4;
    mb = new MotorControl("/dev/ttyUSB0", 57600);
    mb->wheelEnable(4, enable);
    mb->wheelEnable(5, enable);
    mb->wheelEnable(2, enable);
    mb->wheelEnable(3, enable);
    mb->setSpeedXYW(0, 0, 0);
    while(ros::ok()){
        gettimeofday(&t1, NULL);
        mb->readWheelSpeed(2, &speed_1);
        cout<<speed_1/30.0<<"rpm"<<endl;
        mb->readWheelSpeed(3, &speed_2);
        cout<<speed_2/30.0<<"rpm"<<endl;
        mb->readWheelSpeed(4, &speed_3);
        cout<<speed_3/30.0<<"rpm"<<endl;
        mb->readWheelSpeed(5, &speed_4);
        cout<<speed_4/30.0<<"rpm"<<endl;
        gettimeofday(&t2, NULL);
        ros::spin();
        cout<<t2.tv_sec - t1.tv_sec + (t2.tv_usec - t1.tv_usec)/1000000.0<<endl;
        loop_rate.sleep();
    }
    return 0;
}