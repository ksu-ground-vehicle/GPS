//#include "GPSSmartModule_V1_0.ino"
#include "gps_type_define.h"
#include "HalfDuplexSimpleSerial.h"
#include "GPSSmartModule.h"
#include "ParallaxSerialTerminal.h"
#include <ros.h>
#include <std_msgs/Int8MultiArray>
#include <std_msgs/String>


ros::NodeHandle nh;

void gps_send_cmd(uint8_t cmd_b, uint8_t cmd_s)
{  
  HDSS_write(0xA5);
  HDSS_write(cmd_b);
  HDSS_write(cmd_s);
  HDSS_write(0x5A);
}

int* getLatitude(){
    uint32_t rxdata =0;
    latitude_t latitude;
    int info[4];
    gps_send_cmd(CMD_INFO_BASIC, CMD_GET_LATITUDE);
    // The module will return 5 bytes, after received the command
    // Degree, minute, second(16-bit, MSB first), N/S
    for(i = 0; i < 5; i++)
    {
        rxdata = HDSS_read_time(8);
        if(rxdata == ERROR_RX_TIMEOUT)
        {
            return ERROR_RX_TIMEOUT;
        }
        latitude.buff[i] = rxdata;
    }
    PST_SET_X_Y(18, LATITUDE_INFO_AT_ROW);
    info[0] = latitude.degree;
    info[1] = latitude.minute;
    info[2] = latitude.second;
    info[3] = latitude.indicator;
    return info;
}

int* getLongitude(){
    uint32_t rxdata =0;
    longitude_t longitude;
    int info[4];
    gps_send_cmd(CMD_INFO_BASIC, CMD_GET_LONGITUDE);
    // The module will return 5 bytes, after received the command
    // Degree, minute, second(16-bit, MSB first), E/W
    
    for(i = 0; i < 5; i++)
    {
        rxdata = HDSS_read_time(8);
        if(rxdata == ERROR_RX_TIMEOUT)
        {
            return ERROR_RX_TIMEOUT;
        }
        longitude.buff[i] = rxdata;
    }
    PST_SET_X_Y(18, LONGITUDE_INFO_AT_ROW);
    info[0] = longitude.degree;
    info[1] = longitude.minute;
    info[2] = longitude.second;
    info[3] = longitude.indicator;
    return info;
}

void gps_callback(const std_msgs::String::ConstPtr& msg){
    ros::Publisher pub = nh.advertise<std_msgs::Int8MultiArray>("gps_info", 1000);
    if(msg->data.c_str == "long"){
        pub.publish(getLongitude());
    }
    else if(msg->data.c_str == "lat"){
        pub.publish(getLatitude());
    }
    else{
        ROS_INFO("error: wrong command format");
    }
    ros::spinOnce();
    loop_rate.sleep();
}

void setup(){
    ros::init("gps");
}

void loop(){
    ros::Subscriber sub = nh.subscribe("gps_cmd", 1000, gps_callback);
    ros::spin();
}
