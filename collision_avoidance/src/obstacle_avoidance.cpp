#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>

#include <sstream>

int main(int argc, char **argv){
		//inserire parametri per cmd_vel e laser_scan
		//...
		
		ros::init(argc, argv, "collision_avoidance");
		
		ros::NodeHandle n;
		ros::Rate loop_rate(10);
		
		//creo il publisher per inviare i comandi di velocità
		ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
		
		ROS_INFO("Ho avviato il publisher");
		
		//creo il subscriber per ricevere comandi laser scan 
		ros::Subscriber sub1 = n.subscribe("laser_scan", 1000, laserScanCallback);
		
		//creo il subscriber per ricevere comandi di velocità
		ros::Subscriber sub2 = n.subscribe("cmd_vel", 1000, cmdVelCallback);
		
		//pubblico il comando di velocità che non fa andare a sbattere
		//...

		ROS_INFO("Inviare al topic /cmd_vel il comando per far muovere il robot");
		ros::spin();
		
		return 0;


}
