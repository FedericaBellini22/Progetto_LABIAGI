#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv){
		//inserire parametri per cmd_vel e laser_scan
		
		ros::init(argc, argv, "collision_avoidance");
		
		ros::NodeHandle n;
		ros::Rate loop_rate(10);
		
		//creo il publisher per inviare i comandi di velocità
		ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
		
		ROS_INFO("Ho avviato il publisher");
		
		//creo il subscriber per ricevere comandi laser scan 
		ros::Subscriber sub1 = n.subscribe("chatter", 1000, chatterCallback);
		
		//creo il subscriber per i comandi di velocità
		
		
		ROS_INFO("Inviare al topic /cmd_vel_mod il comando per far muovere il robot");
		ros::spin();
		
		return 0;


}
