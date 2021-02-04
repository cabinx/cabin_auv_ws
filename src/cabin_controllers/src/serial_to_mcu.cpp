#include "ros/ros.h"
#include "string"
#include "stdio.h"
#include "serial/serial.h"
#include "cabin_msgs/PwmStamped.h"
#include "std_msgs/Int64.h"

#define s_buffer_size 19
unsigned char s_buffer[s_buffer_size];
#define light_buffer_size 5
unsigned char light_buffer[light_buffer_size];

using namespace std;

serial::Serial ser;

int pwm_data_high = 0;
int pwm_data_low = 0;
int light_intensity_high = 0;
int light_intensity_low = 0;

// Dec to Hex
void pwmToHex(int pwm_input){
    pwm_data_high = pwm_input / (16 * 16);
    pwm_data_low = pwm_input % (16 * 16);
}

void lightToHex(int light_input){
    light_intensity_high = light_input / (16 * 16);
    light_intensity_low = light_input % (16 * 16);
}

void sendPWMData(const cabin_msgs::PwmStamped& msg){
    memset(s_buffer, 0, sizeof(s_buffer));
    s_buffer[0] = 0xAA;
    s_buffer[1] = 0xAA;
    s_buffer[2] = 0x01;

    pwmToHex(msg.pwm.vector_port_fwd);
    s_buffer[3] = 0x00 + pwm_data_high;
    s_buffer[4] = 0x00 + pwm_data_low;

    pwmToHex(msg.pwm.vector_stbd_fwd);
    s_buffer[5] = 0x00 + pwm_data_high;
    s_buffer[6] = 0x00 + pwm_data_low;

    pwmToHex(msg.pwm.vector_port_aft);
    s_buffer[7] = 0x00 + pwm_data_high;
    s_buffer[8] = 0x00 + pwm_data_low;

    pwmToHex(msg.pwm.vector_stbd_aft);
    s_buffer[9] = 0x00 + pwm_data_high;
    s_buffer[10] = 0x00 + pwm_data_low;

    pwmToHex(msg.pwm.heave_port_fwd);
    s_buffer[11] = 0x00 + pwm_data_high;
    s_buffer[12] = 0x00 + pwm_data_low;

    pwmToHex(msg.pwm.heave_stbd_fwd);
    s_buffer[13] = 0x00 + pwm_data_high;
    s_buffer[14] = 0x00 + pwm_data_low;

    pwmToHex(msg.pwm.heave_port_aft);
    s_buffer[15] = 0x00 + pwm_data_high;
    s_buffer[16] = 0x00 + pwm_data_low;

    pwmToHex(msg.pwm.heave_stbd_aft);
    s_buffer[17] = 0x00 + pwm_data_high;
    s_buffer[18] = 0x00 + pwm_data_low;

    ser.write(s_buffer, s_buffer_size);
}

void sendLightIntensity(std_msgs::Int64 msg){
    memset(light_buffer, 0, sizeof(light_buffer));
    light_buffer[0] = 0xAA;
    light_buffer[1] = 0xAA;
    light_buffer[2] = 0x02;
    
    lightToHex(msg.data);
    light_buffer[3] = 0x00 + light_intensity_high;
    light_buffer[4] = 0x00 + light_intensity_low;

    ser.write(light_buffer, light_buffer_size);
}
int main(int argc, char** argv){
    ros::init(argc, argv, "serial_to_mcu");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::string serial_port_;
    int baudrate_;
    nh_private.param<std::string>("serial_port", serial_port_, "/dev/ttyUSB0");
    nh_private.param<int>("baudrate", baudrate_, 115200);
    
    ros::Subscriber pwm_sub = nh.subscribe("/command/pwm", 1000, sendPWMData);
    ros::Subscriber light_sub = nh.subscribe("/command/light_intensity", 1000, sendLightIntensity);

    try{
        ser.setPort(serial_port_);
        ser.setBaudrate(baudrate_);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch(serial::IOException& e){
        ROS_ERROR_STREAM("Unable to open port!!!");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized...");
    }
    else{
        return -1;
    }

    ros::Rate loop_rate(50);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
}