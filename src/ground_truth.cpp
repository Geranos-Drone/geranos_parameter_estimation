#include "ros/ros.h"

#include <std_msgs/Float64.h>


#include <geranos_parameter_estimation/ground_truth.h>

#include <cmath>
#include <sstream>
#include <vector>

#include <nav_msgs/Odometry.h>


#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>


/*
This node calculates the ground truth mass, inertra xx = yy and center of mass z-position
Assumptions: 	
	roll = pitch = 0
	xy position drone =  xy position of grabbed pole
	only one pole at the time

*/

class Ground_truth{
public:

	/***ros stuff***/

	/*service*/
	ros::NodeHandle n;
	ros::ServiceServer grab_pole_1 = n.advertiseService("geranos/grab_pole_1_gt", &Ground_truth::get_p_1, this);
	ros::ServiceServer grab_pole_2 = n.advertiseService("geranos/grab_pole_2_gt", &Ground_truth::get_p_2, this);
	ros::ServiceServer grab_pole_3 = n.advertiseService("geranos/grab_pole_3_gt", &Ground_truth::get_p_3, this);
	ros::ServiceServer grab_pole_4 = n.advertiseService("geranos/grab_pole_4_gt", &Ground_truth::get_p_4, this);

	ros::ServiceServer drop_pole_1 = n.advertiseService("geranos/drop_pole_1_gt", &Ground_truth::drop_p_1, this);
	ros::ServiceServer drop_pole_2 = n.advertiseService("geranos/drop_pole_2_gt", &Ground_truth::drop_p_2, this);
	ros::ServiceServer drop_pole_3 = n.advertiseService("geranos/drop_pole_3_gt", &Ground_truth::drop_p_3, this);
	ros::ServiceServer drop_pole_4 = n.advertiseService("geranos/drop_pole_4_gt", &Ground_truth::drop_p_4, this);

	/*subscribe*/
	ros::Subscriber msf_sub = n.subscribe("geranos/msf_core/odometry", 1000, &Ground_truth::msf_callback, this);

	/*publish*/
	geranos_parameter_estimation::ground_truth gt_msg;
	ros::Publisher gt_publisher = n.advertise<geranos_parameter_estimation::ground_truth>("ground_truth_parameters", 1000);


	/*parameters*/
	const double mass_drone = 7.25;
	const double mass_pole1 = 0.5;
	const double mass_pole2 = 1.0;
	const double mass_pole3 = 1.5;
	const double mass_pole4 = 2.0;
	double mass;

	const double inertia_xy_drone = 0.5;
	const double inertia_xy_pole1 = 0.04166666;
	const double inertia_xy_pole2 = 0.1875;
	const double inertia_xy_pole3 = 0.5;
	const double inertia_xy_pole4 = 1.041666;
	double inertia;

	const double com_z_drone = -0.0662; // becaues of propellers on top
	const double com_z_pole1 = 0.5;
	const double com_z_pole2 = 0.75;
	const double com_z_pole3 = 1.0;
	const double com_z_pole4 = 1.25;
	double com_z;

	bool carrying_pole = false;

	double pos_z;
	double com_pole;
	double mass_pole;
	double inertia_pole;

	int pole_nr = 0;



	/*callback functions*/
	bool get_p_1(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
		std::cout << "GT grab pole 1" << std::endl;
		carrying_pole = true;
		com_pole = -pos_z + com_z_pole1;
		mass_pole = mass_pole1;
		inertia_pole = inertia_xy_pole1;
		update();
		pole_nr = 1;
		return true;
	}

	bool get_p_2(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
		std::cout << "GT grab pole 2" << std::endl;
		carrying_pole = true;
		com_pole = -pos_z + com_z_pole2;
		mass_pole = mass_pole2;
		inertia_pole = inertia_xy_pole2;
		update();
		pole_nr = 2;
		return true;
	}

	bool get_p_3(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
		std::cout << "GT grab pole 3" << std::endl;
		carrying_pole = true;
		com_pole = -pos_z + com_z_pole3;
		mass_pole = mass_pole3;
		inertia_pole = inertia_xy_pole3;
		update();
		pole_nr = 3;
		return true;
	}

	bool get_p_4(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
		std::cout << "GT grab pole 4" << std::endl;
		carrying_pole = true;
		com_pole = -pos_z + com_z_pole4;
		mass_pole = mass_pole4;
		inertia_pole = inertia_xy_pole4;
		update();
		pole_nr = 4;
		return true;
	}

	bool drop_p_1(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
		std::cout << "GT drop pole 1" << std::endl;
		if(pole_nr==1){
			carrying_pole = false;
		}
		update();
		return true;
	}

	bool drop_p_2(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
		std::cout << "GT drop pole 2" << std::endl;
		if(pole_nr==2){
			carrying_pole = false;
		}
		update();
		return true;
	}

	bool drop_p_3(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
		std::cout << "GT drop pole 3" << std::endl;
		if(pole_nr==3){
			carrying_pole = false;
		}
		update();
		return true;
	}

	bool drop_p_4(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
		std::cout << "GT drop pole 4" << std::endl;
		if(pole_nr==4){
			carrying_pole = false;
		}
		update();
		return true;
	}

	void msf_callback(const nav_msgs::Odometry message){
	  pos_z = message.pose.pose.position.z;
	}






	/*calculate mass:*/

	void calculate_mass(){
		if(carrying_pole){
			mass = mass_drone + mass_pole;
		}
		else{
			mass = mass_drone;
		}
	}

	/*calculate com_z*/
	void calculate_com_z(){ //position of com wtr. to body origin in body frame
		if(carrying_pole){
			com_z = (mass_pole*com_pole + mass_drone*com_z_drone) / (mass_drone + mass_pole);
		}
		else{
			com_z = com_z_drone;
		}
	}


	/*calculate inertia*/
	void calculate_inertia(){ // xx , yy yinertia in com
		if(carrying_pole){
			inertia = inertia_xy_drone + pow((com_z - com_z_drone),2)*mass_drone  +  inertia_pole + pow((com_z - com_pole),2) * mass_pole;
		}
		else{
			inertia = inertia_xy_drone;
		}
	}

	void update(){
		calculate_mass();
		calculate_com_z();
		calculate_inertia();
	}


	void get_gt(){
		
		gt_msg.gt_mass = mass;
		gt_msg.gt_Jxy = inertia;
		gt_msg.gt_com_z = com_z;
		gt_msg.header.stamp = ros::Time::now();	

		gt_publisher.publish(gt_msg);
	}

};












int main(int argc, char **argv)
{
  ros::init(argc, argv, "ground_truth_node");
  ros::NodeHandle n;
  
  Ground_truth gt;
  gt.update();

  ros::Rate loop_rate(10);

  while (ros::ok()){
  	gt.get_gt();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0; 
}
