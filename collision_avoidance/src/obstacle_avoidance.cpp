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

#define CONSTANT_1 2

bool cmd_received = false;
geometry_msgs::Twist vel_received;

float vel_received_x = 0;
float vel_received_y = 0;
float vel_received_angular = 0;
float force_mod = 0;

ros::Publisher cmd_vel_pub;


void avoidanceOperations(float fx, float fy, float ob_dist, float fm) {
	
	geometry_msgs::Twist msg_final;
	ROS_INFO("ob_dist= %f",ob_dist);
	
	
	//robot decide di girare a questa distanza
	float target = ob_dist/4;
	ROS_INFO("Voglio deviare l'ostacolo a questa distanza: %f\n",target);
	
	//rotazione robot
	msg_final.angular.z = target * fy/40;
	fx *= abs(vel_received_x)/400;
	
	if (fx < 0){
			msg_final.linear.x = fx + vel_received_x;
	}
	
	else{
			msg_final.linear.x = -fx + vel_received_x;
	}
	
	msg_final.angular.z += vel_received_angular;
	
	msg_final.linear.y = fy + vel_received_y;
	
	cmd_vel_pub.publish(msg_final);
	
	
	
	
	

	/*ROS_INFO("CIAOOOO1");
	
	//messaggio per pubblicare la velocità modificata che non fa andare a sbattere il robot
	geometry_msgs::Twist msg_final;
	
	msg_final.linear.x = vel_received_x + fx;
	
	
	//azzero la velocità in prossimità dell'ostacolo
	if (signbit(msg_final.linear.x) != signbit(vel_received_x))
		msg_final.linear.x = 0;
	
	//operazioni per deviare ostacolo
	//componenti lineari
	
	msg_final.linear.y = vel_received_y + fy;
	msg_final.linear.z = vel_received.linear.z;
	 
	//componente angolare
	msg_final.angular = vel_received.angular ;
	
	if (ob_dist < 0.5){ 
		ROS_INFO("sono qui1");
		msg_final.angular.z = -fm * CONSTANT_1;
		
		cmd_vel_pub.publish(msg_final);
	}
	
	//ROS_INFO("sono qui2");
	
	else{
		cmd_vel_pub.publish(vel_received);
		ROS_INFO("Messaggio pubblicato correttamente");
	}
	
	
	ROS_INFO("Messaggio pubblicato correttamente");

	//pubblico il comando di velocità che non fa andare a sbattere
	//cmd_vel_pub.publish(msg_final);*/
	
}


void transformOperations(sensor_msgs::PointCloud c, Eigen::Isometry2f lm) {

	Eigen::Vector2f obstacle_position;
	Eigen::Vector2f obstacle_position_rframe;

	float force_x = 0.0;
	float force_y = 0.0;
	float obstacle_distance;
	
	for (auto& point: c.points) {	//ciclo su tutti i punti della radice

		/*	
			p_i (x,y): posa ostacolo	->	obstacle_position[2]
			t (x,y): posa robot
			t - p_i: direzione forza risultante			
			1/norm(t_i - p_i): modulo forza risultante
		*/

		obstacle_position(0) = point.x;
		obstacle_position(1) = point.y;
		
		obstacle_position_rframe = lm * obstacle_position;	//posizione dell'ostacolo nel robot frame, 

		obstacle_distance = sqrt(point.x * point.x + point.y * point.y);	//norm(t_i - p_i)
		force_mod = 1/(obstacle_distance * obstacle_distance);	//1/norm(t_i - p_i) modulo forza risultante
		
		force_x += obstacle_position(0) * force_mod;
		force_y += obstacle_position(1) * force_mod;

	}
	
	//prendiamo le forze uguali in modulo ma con verso opposto per far fermare il robot
	force_x = -force_x;
	force_y = -force_y;

	avoidanceOperations(force_x, force_y, obstacle_distance,force_mod);
}


void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg){
	
	
	
	//ho ricevuto il comando di velocità 	
	cmd_received = true;	
	vel_received = *msg;
	vel_received_x = msg->linear.x;
	vel_received_y = msg->linear.y;
	vel_received_angular = msg->angular.z;
	ROS_INFO("Ho ricevuto il comando!\nlinear_x = %f, linear_y = %f, angular = %f", vel_received_x, vel_received_y, vel_received_angular);
}


void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
		
		
		ROS_INFO("ECCOMI");
		
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
		
		try {
		//aspetto di avere una trasformata disponibile
		//parametri: sistema di riferimento di arrivo, sistema di riferimento di partenza, tempo, timeout
		listener.waitForTransform("base_footprint", "base_laser_link", ros::Time(0), ros::Duration(5.0));
		//estraggo la trasformata tra i due sistemi di riferimento e la memorizzo in transform_obstacle
		listener.lookupTransform("base_footprint", "base_laser_link", ros::Time(0), transform_obstacle);
		}
		catch (tf::TransformException &e) {
			ROS_ERROR("%s", e.what());
			//ros::Duration(1.0).sleep();
			return;
		}

		Eigen::Isometry2f laser_matrix = convertPose2D(transform_obstacle);	//converto la trasformata in matrice 2D
		
		//Chiama transformOperations(...)
		transformOperations(cloud, laser_matrix);
}


int main(int argc, char **argv){
		
		ros::init(argc, argv, "collision_avoidance");
		
		ros::NodeHandle n;
		ros::Rate loop_rate(10);
		
		ROS_INFO("Ho avviato il publisher");
		
		//creo il subscriber per ricevere comandi laser scan 
		ros::Subscriber sub1 = n.subscribe("base_scan", 1000, laserScanCallback);
		
		//creo il subscriber per ricevere comandi di velocità
		ros::Subscriber sub2 = n.subscribe("avoidance_cmd_vel", 1000, cmdVelCallback);
		//MEMO: rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=avoidance_cmd_vel
		
		//dico al publisher di inviare i comandi di velocità
		cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

		ROS_INFO("Inviare al topic /avoidance_cmd_vel il comando per far muovere il robot");
		ros::spin();
		
		return 0;
}

