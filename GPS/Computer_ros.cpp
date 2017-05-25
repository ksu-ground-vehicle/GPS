#include <ros.h>
#include <std_msgs/Int8MultiArray>
#include <std_msgs/String>
#include <std>
ros::NodeHandle nh;
int* recieved;

void info_callback(const std_msgs::Int8MultiArray::ConstPtr& msg){
    recieved = msg->data;
}

int* getLatitude(){
   ros::Publisher pub = nh.advertise<std_msgs::String>("gps_cmd", 1000); 
   pub.publish("lat");
   ros::Subscriber sub = nh.subscribe("gps_info", 1000, info_callback);
   ros::spinOnce();
   return recieved;
}

int* getLongitude(){
    ros::Publisher pub = nh.advertise<std_msgs::String>("gps_cmd", 1000); 
    pub.publish("long");
    ros::Subscriber sub = nh.subscribe("gps_info", 1000, info_callback);
    ros::spinOnce();
    return recieved;
}

void printIntArray(int* array){
    for(int i = 0; i < sizeof(array); i++){
        cout << array[i] << "\n";
    }
}

int main(int argc, char* argv){
    ros::init(argc, argv, "computer");
    printIntArray(getLatitude());
    printIntArray(getLongitude());
    return 0;
}