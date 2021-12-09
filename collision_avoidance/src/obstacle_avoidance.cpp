#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>

#include <sstream>

bool cmd_received = false;
geometry_msgs::Twist vel_received;

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg){
	
	//ho ricevuto il comando di velocità 	
	cmd_received = true;
	
	
	vel_received =* msg;	


}

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
		//devo ricevere il comando di velocità
		if (!cmd_received) 
			return;

		cmd_received = false;
		
		
		tf::TransformListener listener;
		tf::StampedTransform transform_obstacle;
		laser_geometry::LaserProjection projector;
		sensor_msgs::PointCloud cloud;
		
		//funzione che converte un messaggio di tipo laser scan ad un messaggio di tipo point cloud
		projector.transformLaserScanToPointCloud("base_laser_link",*msg,cloud,listener);
		
		//try && catch
		
		
		//chiama transformOperations
		//chiama avoidanceOperations

}

void transformOperations(sensor_msgs::PointCloud cloud){
	//.........

}

void avoidanceOperations(){
	//TODO
}




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
