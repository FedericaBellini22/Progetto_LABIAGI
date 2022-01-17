#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include "geometry_utils_fd.h"
#include <sstream>

bool cmd_received = false;

geometry_msgs::Twist vel_received;

float vel_received_x = 0;
float vel_received_y = 0;
float vel_received_angular = 0;

ros::Publisher cmd_vel_pub;


void avoidanceOperations(float fx, float fy, float ob_dist) {
	
	geometry_msgs::Twist msg_final;
	//ROS_INFO("ob_dist= %f",ob_dist);
	
	float d = ob_dist/20;
	
	//robot decide di girare a questa distanza
	/*if (ob_dist > 3) {
	
		ROS_INFO("Voglio deviare l'ostacolo a questa distanza: %f\n",d);
	}*/

	//rotazione
	msg_final.angular.z = d * fy/60;
	msg_final.angular.z += vel_received_angular;
	//componenti lineari
	fx *= abs(vel_received_x)/600;
	// \sum_i fi + cmd_vel
	msg_final.linear.x = fx + vel_received_x;
	msg_final.linear.y = fy + vel_received_y;
	
	cmd_vel_pub.publish(msg_final);	
}


void transformOperations(sensor_msgs::PointCloud c, Eigen::Isometry2f lm) {

	Eigen::Vector2f obstacle_position;
	
	float force_x = 0.0;
	float force_y = 0.0;
	float obstacle_distance;
	
	for (auto& point: c.points) { //ciclo su tutti i punti della nuvola

		/*	
			p_i (x,y): posa ostacolo  ->  obstacle_position[2]
			t (x,y): posa robot
			t - p_i: direzione forza risultante			
			1/norm(t_i - p_i): modulo forza risultante
		*/

		obstacle_position(0) = point.x;
		obstacle_position(1) = point.y;
		obstacle_position = lm * obstacle_position;	//posizione dell'ostacolo nel robot frame 

		obstacle_distance = sqrt(point.x * point.x + point.y * point.y); //norm(t_i - p_i)
		
		force_x += (obstacle_position(0) / obstacle_distance) / obstacle_distance;	
		force_y += (obstacle_position(1) / obstacle_distance) / obstacle_distance; 
	}
	
	//prendiamo le forze uguali in modulo ma con verso opposto
	force_x = -force_x;
	force_y = -force_y;

	avoidanceOperations(force_x, force_y, obstacle_distance);
}


void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {

	//ho ricevuto il comando di velocità 	
	cmd_received = true;	
	vel_received = *msg;
	vel_received_x = msg->linear.x;
	vel_received_y = msg->linear.y;
	vel_received_angular = msg->angular.z;
	ROS_INFO("Ho ricevuto il comando!\nlinear_x = %f, linear_y = %f, angular = %f", vel_received_x, vel_received_y, vel_received_angular);
}


void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
		
	//ROS_INFO("ECCOMI");
		
	//devo ricevere il comando di velocità
	if (!cmd_received) 
		return;

	cmd_received = false;
		
	tf::TransformListener listener;
	tf::StampedTransform transform_obstacle;
	laser_geometry::LaserProjection projector;
	sensor_msgs::PointCloud cloud;
		
	//funzione che converte un messaggio di tipo laser scan ad un messaggio di tipo point cloud
	projector.transformLaserScanToPointCloud("base_laser_link", *msg, cloud, listener);
		
	try {
		//aspetto di avere una trasformata disponibile
		//parametri: sistema di riferimento di arrivo, sistema di riferimento di partenza, tempo, timeout
		listener.waitForTransform("base_footprint", "base_laser_link", ros::Time(0), ros::Duration(5.0));
		//estraggo la trasformata tra i due sistemi di riferimento e la memorizzo in transform_obstacle
		listener.lookupTransform("base_footprint", "base_laser_link", ros::Time(0), transform_obstacle);
	}
	catch (tf::TransformException &e) {
		ROS_ERROR("%s", e.what());
		ros::Duration(1.0).sleep();
		return;
	}

	Eigen::Isometry2f laser_matrix = convertPose2D(transform_obstacle);	//converto la trasformata in matrice 2D
		
	transformOperations(cloud, laser_matrix);
}


int main(int argc, char **argv) {
		
	ros::init(argc, argv, "collision_avoidance");
		
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
		
	ROS_INFO("Ho avviato il nodo...");
		
	//creo il subscriber per ricevere comandi laser scan 
	ros::Subscriber sub1 = n.subscribe("base_scan", 1000, laserScanCallback);
		
	//creo il subscriber per ricevere comandi di velocità
	ros::Subscriber sub2 = n.subscribe("avoid_cmd_vel", 1000, cmdVelCallback);
		
	//dico al publisher di inviare i comandi di velocità
	cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

	ROS_INFO("Inviare al topic /avoid_cmd_vel il comando per far muovere il robot");
	ros::spin();
		
	return 0;
}
