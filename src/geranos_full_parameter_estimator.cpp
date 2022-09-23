#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <geranos_parameter_estimation/geranos_parameter_estimation.h>
#include <cmath>
#include "Eigen/Dense"
#include <sstream>
#include <vector>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <geranos_parameter_estimation/EKF_TuningConfig.h> 




// messages
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>
#include <rw_control_utils/WrenchSensorFilteredStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <mav_msgs/Actuators.h>


#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Core>


#include"functions.h"
#include"props_to_force.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::DiagonalMatrix;
using Eigen::seq;
using Eigen::Matrix3d;



// TODO: Jz estimatin does not work, plot variance of all states
// plot estimated torque and commanded torque


#define frequency 200.0000
// the mass and inertia estimator node subscribes to msf and the controller, and publihes the estimated mass and inertia with what rate.

class Mass_inertia_estimator{
public:

//----------setup variables---------------

	////////////////////////////////////////////
	////////system geometry parameters:
	int iterator = 0;
	//simulation parameters:
	float delta_t = 1/frequency;





	//gravity constant
	Vector3d g_vec;
	float gravity_constant = 9.8;



	Wrench wrench;
	Propeller_speeds_squared props_squared;
	std::vector<double> propeller_speeds;

	Props_to_wrench Wrench_calculator;

	////////////////////////////////////////////
	////////ekf parameters:

	// states and state vector, predictor vectors

  VectorXd x = VectorXd(18);           				
  VectorXd pos_com = VectorXd(3);
	VectorXd vel_com = VectorXd(3);
	VectorXd angles = VectorXd(3);
	Vector3d omega;// = VectorXd(3);
	Matrix3d J;// = MatrixXd(3,3);
	float mass;

	//predicted state
  VectorXd x_p = VectorXd(18);
	VectorXd pos_com_p = VectorXd(3);
	VectorXd vel_com_p = VectorXd(3);
	VectorXd angles_p = VectorXd(3);
	VectorXd omega_p = VectorXd(3);
		//position of center of mass in body frame

	Vector3d com;


	//measured pose and twist:
	VectorXd pos_com_m = VectorXd(3);
	VectorXd vel_com_m = VectorXd(3);
	VectorXd angles_m = VectorXd(3);
	VectorXd omega_m = VectorXd(3);

	
	Quaternion quaternion;
	EulerAngles eulerangles;

	float com_x;
	float com_y;
	float com_z;

	float roll;
	float pitch;
	float yaw;

	float omega_x;
	float omega_y;
	float omega_z;

	float Jxy;
	float Jz;


	//rotation matrices:
	MatrixXd R_roll = MatrixXd(3,3);
	MatrixXd R_pitch = MatrixXd(3,3);
	MatrixXd R_yaw = MatrixXd(3,3);
	MatrixXd R_total = MatrixXd(3,3);

	// angles dot to omega map:
	MatrixXd T = MatrixXd(3,3);

	//??????????? how to make angles = [roll, pitch, yaw] at all times, references, pointers???


	// measurement vector
	VectorXd z = VectorXd(12);
	VectorXd h = VectorXd(12); //(non) linear state to measurement map

	// input vector, forces and torques
	VectorXd u = VectorXd(8);
	Vector3d force_prop;
	Vector3d force_w;
	// Vector3d torque_props;
	Vector3d torque_o_prop;

	//---ekf tuning covariance matrices
	// measurement matrix R_ekf:
	Vector3d r_pos;
	Vector3d r_vel;
	Vector3d r_angles;
	Vector3d r_omega;
	
	VectorXd r_ekf = VectorXd(12);
	
	MatrixXd R_ekf = MatrixXd(12,12);

	//process noise covariance matrix Q_ekf
	Vector3d q_pos;
	Vector3d q_vel;
	Vector3d q_angles;
	Vector3d q_omega;
	float q_mass;
	float q_Jxy;
	float q_Jz;
	Vector3d q_com;
	
	VectorXd q_ekf = VectorXd(18);
	MatrixXd Q_ekf = MatrixXd(18,18);


	//----initialisation ekf matrices: 

	//predicted state covariance matrix
	MatrixXd P_ekf = MatrixXd::Identity(18,18)*1e2;

	//linearized state to measruement map
	MatrixXd H_ekf = MatrixXd::Identity(12,18);

	//linearized state update map:
	MatrixXd A_ekf = MatrixXd(18,18);

	//measurement resiudal:
	VectorXd y_ekf = VectorXd(12);

	//innovation covariance:
	MatrixXd S_ekf = MatrixXd(12,12);

	//kalman gain:
	MatrixXd K_ekf = MatrixXd(18,12);

	//identity matrix
	MatrixXd Identity_18 = MatrixXd::Identity(18,18);


	//------helper variables:
	//partial derivatives:
	float R_x_roll;
	float R_x_pitch;
	float R_x_yaw;

	float R_y_roll;
	float R_y_pitch;
	float R_y_yaw;

	float R_z_roll;
	float R_z_pitch;
	float R_z_yaw;

	//propeller force components in body frame:
	float f_x;
	float f_y;
	float f_z;

	//torques in body frame wtr to origin only from propellers
	float tau_x;
	float tau_y;
	float tau_z;

	//force components in world frame
	float Rfx;
	float Rfy;
	float Rfz;


	//trigonometric functins of roll pitch yaw:
	float sroll; 
	float spitch;
	float syaw;
	float croll; 
	float cpitch;
	float cyaw;



//----------setup ros inputs and outpus---------------
	ros::NodeHandle n;
	//ros messages
  geranos_parameter_estimation::geranos_parameter_estimation estimation_msg;


	//ros publishers
  ros::Publisher estimate_publisher = n.advertise<geranos_parameter_estimation::geranos_parameter_estimation>("parameter_estimation", 1000);

	//ros subscribers

	ros::Subscriber msf_sub = n.subscribe("geranos/msf_core/odometry", 1000, &Mass_inertia_estimator::msf_callback, this); // TODO correct topic name
	ros::Subscriber controller_sub = n.subscribe("geranos/command/motor_speed", 1000, &Mass_inertia_estimator::controller_callback, this); // /geranos/command/motor_speed
// Type: mav_msgs/Actuators


//------------dynamic reconfigure

  dynamic_reconfigure::Server<geranos_parameter_estimation::EKF_TuningConfig> server;
  dynamic_reconfigure::Server<geranos_parameter_estimation::EKF_TuningConfig>::CallbackType f;

  //calback
  void dynrcf_callback(geranos_parameter_estimation::EKF_TuningConfig& config, double_t level){
  	// ROS_INFO_STREAM("dynamic reconfigure called");


  	// update R_ekf
  	r_pos << config.R_pos, config.R_pos, config.R_pos;
		r_vel << config.R_vel, config.R_vel, config.R_vel;
		r_angles << config.R_orient, config.R_orient, config.R_orient;
		r_omega << config.R_angvel, config.R_angvel, config.R_angvel;
		r_ekf << r_pos, r_vel, r_angles, r_omega;
		R_ekf  = r_ekf.asDiagonal();


		// update Q_ekf
		q_pos << config.Q_pos, config.Q_pos, config.Q_pos;
		q_vel << config.Q_vel, config.Q_vel, config.Q_vel;
		q_angles << config.Q_orient, config.Q_orient, config.Q_orient;
		q_omega << config.Q_angvel, config.Q_angvel, config.Q_angvel;
		q_mass = config.Q_mass;
		q_Jxy = config.Q_Jxy;
		q_Jz = config.Q_Jz;
		q_com << config.Q_com, config.Q_com, config.Q_com;
		q_ekf << q_pos, q_vel, q_angles, q_omega, q_mass, q_Jxy, q_Jz, q_com;
		Q_ekf = q_ekf.asDiagonal();

		// std::cout << Q_ekf << std::endl;

  	ROS_INFO_STREAM(q_mass);
  }



//------------functions-----------------

	void publish(){
		estimation_msg.mass_estimate = mass;
		estimation_msg.mass_covariance = P_ekf(12,12);
		estimation_msg.Jxy_estimate=Jxy;
		estimation_msg.Jxy_covariance = P_ekf(13,13);
		estimation_msg.Jz_estimate=Jz;
		estimation_msg.Jz_covariance = P_ekf(14,14);

		estimation_msg.roll_estimate = roll;
		estimation_msg.pitch_estimate = pitch;
		estimation_msg.yaw_estimate = yaw;

		tf::vectorEigenToMsg(pos_com, estimation_msg.position_estimate);
		tf::vectorEigenToMsg(vel_com, estimation_msg.velocity_estimate);
		tf::vectorEigenToMsg(omega, estimation_msg.omega_estimate);
		tf::vectorEigenToMsg(com, estimation_msg.com_estimate);

		estimation_msg.header.stamp = ros::Time::now();	


		estimate_publisher.publish(estimation_msg);
	}



//calculate the full ekf steps
	void ekf_update_step(){//TODO




		//----------------------prediction update

		//calculate rotation matrix from roll, ptich and yaw
		//delta_t = 1/frequency;
		angles_to_rot_and_trans();

		//propeller forces in world frame
		force_w = R_total * force_prop;
		// std::cout << "\n force world:  " << force_w << std::endl;

		//-----------predict state: 
		pos_com_p = pos_com + delta_t * vel_com;
		vel_com_p = vel_com + delta_t * (force_w / mass + g_vec);
		angles_p = angles+delta_t*T.inverse() * omega;
		omega_p = omega + delta_t * J.inverse() * (torque_o_prop + force_w.cross(com) - omega.cross(J*omega));
		//mass_p = mass;
		//J_p = J;
		//com_p = com;

		// std::cout << "\n omega_p: " << omega << std::endl;
		// std::cout << "\n angles_p: " << angles << std::endl;
		// std::cout << "\n vel_com_p: " << vel_com_p << std::endl;
		// std::cout << "\n mass: " << mass << std::endl;

		//full predicted state vector
		x_p << pos_com_p, vel_com_p, angles_p, omega_p, mass, Jxy, Jz, com;



		//-----------precalculations for covariance prediction step:
		//components of propeller force in body frame:
		f_x = force_prop(0);
		f_y = force_prop(1);
		f_z = force_prop(2);

		//torque components in body frame:
		tau_x = torque_o_prop(0);
		tau_z = torque_o_prop(1);
		tau_y = torque_o_prop(2);

		//angular velocity components in body frame:
		omega_x = omega(0);
		omega_y = omega(1);
		omega_z = omega(2);


/*		if(mass==0 || Jxy==0 || Jz==0 || cpitch==0){//should never be called
			ROS_ERROR_STREAM("devision by zero");
			abort();
		}*/
		if(isnan(mass) || isnan(Jxy) || isnan(Jz) || isnan(cpitch)){
			ROS_ERROR_STREAM("nan value");
			abort();
		}

	
		A_ekf<<	1, 0, 0, delta_t,       0,       0,                                                                                                                                                                0,                                                                                              0,                                                                                                                                                 0,                                 0,                                0,                                 0,                                                                                                                                                               			   0,                                                                 		     0, 			                                             0,                 0,                  0,                  0,
						0, 1, 0,       0, delta_t,       0,                                                                                                                                                                0,                                                                                              0,                                                                                                                                                 0,                                 0,                                0,                                 0,                                                                                                                                                               			   0,                                                                 		     0, 			                                             0,                 0,                  0,                  0,
						0, 0, 1,       0,       0, delta_t,                                                                                                                                                                0,                                                                                              0,                                                                                                                                                 0,                                 0,                                0,                                 0,                                                                                                                                                               			   0,                                                                 		     0, 			                                             0,                 0,                  0,                  0,
						0, 0, 0,       1,       0,       0,                                                                                                                                                                0,            (delta_t*(f_z*cos(pitch) - f_x*cos(yaw)*sin(pitch) + f_y*sin(pitch)*sin(yaw)))/mass,                                                                                          -(delta_t*cos(pitch)*(f_y*cos(yaw) + f_x*sin(yaw)))/mass,                                 0,                                0,                                 0,                                                                             -(delta_t*(f_z*sin(pitch) + f_x*cos(pitch)*cos(yaw) - f_y*cos(pitch)*sin(yaw)))/pow(mass,2),                                                                 		     0, 			                                             0,                 0,                  0,                  0,
						0, 0, 0,       0,       1,       0, -(delta_t*(f_x*(sin(roll)*sin(yaw) - cos(roll)*cos(yaw)*sin(pitch)) + f_y*(cos(yaw)*sin(roll) + cos(roll)*sin(pitch)*sin(yaw)) + f_z*cos(pitch)*cos(roll)))/mass,  (delta_t*sin(roll)*(f_z*sin(pitch) + f_x*cos(pitch)*cos(yaw) - f_y*cos(pitch)*sin(yaw)))/mass, (delta_t*f_x*(cos(roll)*cos(yaw) - sin(pitch)*sin(roll)*sin(yaw)))/mass - (delta_t*f_y*(cos(roll)*sin(yaw) + cos(yaw)*sin(pitch)*sin(roll)))/mass,                                 0,                                0,                                 0, -(delta_t*(f_x*(cos(roll)*sin(yaw) + cos(yaw)*sin(pitch)*sin(roll)) + f_y*(cos(roll)*cos(yaw) - sin(pitch)*sin(roll)*sin(yaw)) - f_z*cos(pitch)*sin(roll)))/pow(mass,2),                                                                 		     0, 			                                             0,                 0,                  0,                  0,
						0, 0, 0,       0,       0,       1,  (delta_t*(f_x*(cos(roll)*sin(yaw) + cos(yaw)*sin(pitch)*sin(roll)) + f_y*(cos(roll)*cos(yaw) - sin(pitch)*sin(roll)*sin(yaw)) - f_z*cos(pitch)*sin(roll)))/mass, -(delta_t*cos(roll)*(f_z*sin(pitch) + f_x*cos(pitch)*cos(yaw) - f_y*cos(pitch)*sin(yaw)))/mass, (delta_t*f_x*(cos(yaw)*sin(roll) + cos(roll)*sin(pitch)*sin(yaw)))/mass - (delta_t*f_y*(sin(roll)*sin(yaw) - cos(roll)*cos(yaw)*sin(pitch)))/mass,                                 0,                                0,                                 0, -(delta_t*(f_x*(sin(roll)*sin(yaw) - cos(roll)*cos(yaw)*sin(pitch)) + f_y*(cos(yaw)*sin(roll) + cos(roll)*sin(pitch)*sin(yaw)) + f_z*cos(pitch)*cos(roll)))/pow(mass,2),                                                                 		     0, 			                                             0,                 0,                  0,                  0,
						0, 0, 0,       0,       0,       0,                                                                                                                                                                1,            -(delta_t*sin(pitch)*(omega_x*cos(yaw) - omega_y*sin(yaw)))/(pow(sin(pitch),2) - 1),                                                                                       -(delta_t*(omega_y*cos(yaw) + omega_x*sin(yaw)))/cos(pitch),     (delta_t*cos(yaw))/cos(pitch),   -(delta_t*sin(yaw))/cos(pitch),                                 0,                                                                                                                                                                		   0,                                                                 		     0, 			                                             0,                 0,                  0,                  0,
						0, 0, 0,       0,       0,       0,                                                                                                                                                                0,                                                                                              1,                                                                                                     delta_t*(omega_x*cos(yaw) - omega_y*sin(yaw)),                  delta_t*sin(yaw),                 delta_t*cos(yaw),                                 0,                                                                                                                                                                		   0,                                                                 		     0, 			                                             0,                 0,                  0,                  0,
						0, 0, 0,       0,       0,       0,                                                                                                                                                                0,                             -(delta_t*(omega_x*cos(yaw) - omega_y*sin(yaw)))/pow(cos(pitch),2),                                                                         (delta_t*sin(pitch)*(omega_y*cos(yaw) + omega_x*sin(yaw)))/cos(pitch) + 1,      -delta_t*cos(yaw)*tan(pitch),      delta_t*tan(pitch)*sin(yaw),                           delta_t,                                                                                                                                                                		   0,                                                                 		     0, 			                                             0,                 0,                  0,                  0,
						0, 0, 0,       0,       0,       0,                                                                                                                                                                0,                                                                                              0,                                                                                                                                                 0,                                 1, (delta_t*omega_z*(Jxy - Jz))/Jxy,  (delta_t*omega_y*(Jxy - Jz))/Jxy,                                                                                                                                                                		   0, -(delta_t*(tau_x + com_z*f_y - com_y*f_z - Jz*omega_y*omega_z))/pow(Jxy,2), 			                -(delta_t*omega_y*omega_z)/Jxy,                 0, -(delta_t*f_z)/Jxy,  (delta_t*f_y)/Jxy,
						0, 0, 0,       0,       0,       0,                                                                                                                                                                0,                                                                                              0,                                                                                                                                                 0, -(delta_t*omega_z*(Jxy - Jz))/Jxy,                                1, -(delta_t*omega_x*(Jxy - Jz))/Jxy,                                                                                                                                                                		   0, -(delta_t*(tau_y - com_z*f_x + com_x*f_z + Jz*omega_x*omega_z))/pow(Jxy,2), 			                 (delta_t*omega_x*omega_z)/Jxy, (delta_t*f_z)/Jxy,                  0, -(delta_t*f_x)/Jxy,
						0, 0, 0,       0,       0,       0,                                                                                                                                                                0,                                                                                              0,                                                                                                                                                 0,                                 0,                                0,                                 1,                                                                                                                                                                		   0,                                                                  			   0, -(delta_t*(tau_z + com_y*f_x - com_x*f_y))/pow(Jz,2), -(delta_t*f_y)/Jz,   (delta_t*f_x)/Jz,                  0,
						0, 0, 0,       0,       0,       0,                                                                                                                                                                0,                                                                                              0,                                                                                                                                                 0,                                 0,                                0,                                 0,                                                                                                                                                                		   1,                                                                  			   0,                                           		     0,                 0,                  0,                  0,
						0, 0, 0,       0,       0,       0,                                                                                                                                                                0,                                                                                              0,                                                                                                                                                 0,                                 0,                                0,                                 0,                                                                                                                                                                		   0,                                                                  			   1,                                           		     0,                 0,                  0,                  0,
						0, 0, 0,       0,       0,       0,                                                                                                                                                                0,                                                                                              0,                                                                                                                                                 0,                                 0,                                0,                                 0,                                                                                                                                                                		   0,                                                                  			   0,                                           		     1,                 0,                  0,                  0,
						0, 0, 0,       0,       0,       0,                                                                                                                                                                0,                                                                                              0,                                                                                                                                                 0,                                 0,                                0,                                 0,                                                                                                                                                                		   0,                                                                  			   0,                                           		     0,                 1,                  0,                  0,
						0, 0, 0,       0,       0,       0,                                                                                                                                                                0,                                                                                              0,                                                                                                                                                 0,                                 0,                                0,                                 0,                                                                                                                                                                		   0,                                                                  			   0,                                           		     0,                 0,                  1,                  0,
						0, 0, 0,       0,       0,       0,                                                                                                                                                                0,                                                                                              0,                                                                                                                                                 0,                                 0,                                0,                                 0,                                                                                                                                                                		   0,                                                                  			   0,                                           		     0,                 0,                  0,                  1;
 

	  //covariance update step:
    P_ekf = A_ekf * P_ekf * A_ekf.transpose() + Q_ekf;

     // std::cout << "A" <<A_ekf << "\n" << std::endl;


		//----------------------measurement update
		h=H_ekf * x_p; //only works because it's linear
		// std::cout << "x_p by h calc" << x_p << "\n" << std::endl;
    y_ekf = z - h;
    // std::cout << "z by y calc" << z << "\n" << std::endl;
    // std::cout << "h by y calc" << h << "\n" << std::endl;
    // std::cout << " \n y after calculation" << y_ekf << "\n" << std::endl;


 

    S_ekf = H_ekf * P_ekf *H_ekf.transpose() + R_ekf;

    K_ekf = P_ekf * H_ekf.transpose() * S_ekf.inverse();

    // std::cout << "K" << K_ekf << "\n" << std::endl;



    //measurement state update
    x = x_p + K_ekf * y_ekf;

    // std::cout << "angvel_z " << x(11) << "\n" << std::endl;
    // std::cout << "predicted state: \n" << x_p << "\n" << std::endl;
    // std::cout << "state: \n" << x << "\n" << std::endl;7

    // std::cout << "omega variance pre: \n" << P_ekf(9,9) << "\n" << P_ekf(10,10) << "\n" << P_ekf(11,11) << "\n" << std::endl;

    // std::cout << "com variance pre: \n" << P_ekf(15,15) << "\n" << P_ekf(16,16) << "\n" << P_ekf(17,17) << "\n" << std::endl;


    //covariance update step
    P_ekf = (Identity_18 - K_ekf * H_ekf) * P_ekf;

    // std::cout << "omega variance post: \n" << P_ekf(9) << "\n" << P_ekf(10) << "\n" << P_ekf(11) << "\n" << std::endl;

    // std::cout << "com variance post: \n" << P_ekf(15) << "\n" << P_ekf(16) << "\n" << P_ekf(17) << "\n" << std::endl;

    // prevent diverging state: if diverging, set state to 0
    if(abs(x(0))>100){ //position
    	x(0) = 0;
    }
    if(abs(x(1))>0){
    	x(1) = 0;
    }
    if(abs(x(2))>100){
    	x(2) = 0;
    }

		if(abs(x(3))>10){ //velocity
    	x(3) = 0;
    }
    if(abs(x(4))>0){
    	x(4) = 0;
    }
    if(abs(x(4))>10){
    	x(4) = 0;
    }

		if(abs(x(5))>5){ //rpy angles
    	x(5) = 0;
    }
    if(abs(x(6))>5){
    	x(6) = 0;
    }
    if(abs(x(7))>10){
    	x(7) = 0;
    }

    if(abs(x(9))>5){ //omega
    	x(9) = 0;
    }
    if(abs(x(10))>5){
    	x(10) = 0;
    }
    if(abs(x(11))>5){
    	x(11) = 0;
    }

    if(abs(x(12))>50){ //mass
    	x(12) = 8;
    }
    if(abs(x(13))>5){ //jxy
    	x(13) = 0.5;
    }
    if(abs(x(14))>5){//jz
    	x(14) = 0.1;
    }
    

    //prevent devision by zero:
		if(abs(x(12))<0.1){//mass
			x(12) = 0.1;
		}
		if(abs(x(13))<0.0001){//Jxy
			 x(13) = 0.0001;
		}
		if(abs(x(14))<0.0001){//Jz
			x(14) = 0.0001;
		}
		if(abs(cos(x(7)))<0.0001){//pitch
			x(7) = acos(0.0001);
		}


    //update seperate state vectors:
    pos_com = x(seq(0,2));

    vel_com = x(seq(3,5));

    roll = x(6);
    pitch = x(7);
    yaw = x(8);
    angles = x(seq(6,8));
    // std::cout << "angles " << angles << "\n" << std::endl;

    omega_x = x(9);
    omega_y = x(10);
    omega_z = x(11);
    omega = x(seq(9,11));



    mass = x(12);

    Jxy = x(13);
    Jz = x(14);
    J << Jxy, 0, 0,
    		 0, Jxy, 0,
    		 0, 0, Jz;

    com = x(seq(15,17));

  }




//calculate rotationmatrix and T from roll pitch yaw angles:
  
	void angles_to_rot_and_trans(){

	 	R_yaw << cos(yaw), -sin(yaw), 0,
	       	 	 sin(yaw), cos(yaw), 0,
	           0, 0, 1;

		R_pitch << cos(pitch), 0, sin(pitch),
	           0, 1, 0,
	         	 -sin(pitch), 0, cos(pitch);

		R_roll << 1, 0, 0,
	         		0, cos(roll), -sin(roll),
	         		0, sin(roll), cos(roll);

	  R_total = R_roll * R_pitch * R_yaw;


	  T << cos(yaw)*cos(pitch), sin(yaw), 0,
     		 -cos(pitch)*sin(yaw), cos(yaw), 0,
     		 sin(pitch), 0, 1;
	}
	


//---------------------constructor/initialization------------
	Mass_inertia_estimator(){

		g_vec << 0, 0, -gravity_constant;
		//initial state
		pos_com << 0,0,0;
		vel_com << 0,0,0;
		roll = 0;
		pitch = 0;
		yaw = 0;
		angles << roll, pitch, yaw;
		omega << 0,0,0;
		mass = 9;
		Jxy = 0.4;
		Jz = 0.6;
    J << Jxy, 0, 0,
    		 0, Jxy, 0,
    		 0, 0, Jz;

    //initial covariance
		P_ekf = MatrixXd::Identity(15,15)*1e6;

		//setup parameters:
		com << 0, 0, -0.07;


		// g_vec << 0, 0, -9.8; //-9.80665;



//tuning parameters R covariance:
    r_pos << 1, 1, 1;
		r_vel << 1, 1, 1;
		r_angles << 1, 1, 1;
		r_omega << 1, 1, 1;
		r_ekf << r_pos, r_vel, r_angles, r_omega;

		R_ekf  = r_ekf.asDiagonal();

//tuning parameters Q covariance:
		q_pos << 1, 1, 1;
		q_vel << 1, 1, 1;
		q_angles << 1, 1, 1;
		q_omega << 1, 1, 1;
		q_mass = 1;
		q_Jxy = 1;
		q_Jz = 1;
		q_ekf << q_pos, q_vel, q_angles, q_omega, q_mass, q_Jxy, q_Jz;
		Q_ekf = q_ekf.asDiagonal();

//initialisation of other variables:
		y_ekf << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

// dynamic reconfigure:
		f = boost::bind(&Mass_inertia_estimator::dynrcf_callback, this, _1, _2);
  	server.setCallback(f);

	}



//----------ros callback functions---------------
  void controller_callback(const mav_msgs::Actuators& message){// Type: Type: mav_msgs/Actuators

  	//read propeller speeds and calculate forces and torques in origin

  	propeller_speeds = message.angular_velocities;

  	props_squared.p0 = pow(propeller_speeds[0],2);
  	props_squared.p1 = pow(propeller_speeds[1],2);
  	props_squared.p2 = pow(propeller_speeds[2],2);
  	props_squared.p3 = pow(propeller_speeds[3],2);
  	props_squared.p4 = pow(propeller_speeds[4],2);
  	props_squared.p5 = pow(propeller_speeds[5],2);
  	props_squared.p6 = pow(propeller_speeds[6],2);
  	props_squared.p7 = pow(propeller_speeds[7],2);

  	wrench =  Wrench_calculator.props_to_wrench(props_squared);

  	force_prop << wrench.f_x, wrench.f_y, wrench.f_z;
  	torque_o_prop << wrench.tau_x, wrench.tau_y, wrench.tau_z;

  	// std::cout << "\n force: " << force_prop << std::endl;
  	// std::cout << "\n torque: " << torque_o_prop << std::endl;


	 
  	

	}


  void msf_callback(const nav_msgs::Odometry message){//Type: nav_msgs/Odometry
  	//creat measurement vector z

  	

	  pos_com_m << message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z;
		vel_com_m << message.twist.twist.linear.x, message.twist.twist.linear.y, message.twist.twist.linear.z;

		// std::cout << "pos_com \n" << pos_com << std::endl;
		// std::cout << "vel_com \n" << vel_com << std::endl;

		quaternion.x = message.pose.pose.orientation.x;
		quaternion.y = message.pose.pose.orientation.y; 
		quaternion.z = message.pose.pose.orientation.z; 
		quaternion.w = message.pose.pose.orientation.w;

		eulerangles = ToEulerAngles(quaternion);

		angles_m << eulerangles.roll, eulerangles.pitch, eulerangles.yaw;

		// std::cout << "angles_m \n" << angles_m << std::endl;

		omega_m << message.twist.twist.angular.x, message.twist.twist.angular.y, message.twist.twist.angular.z;
		

  	z << pos_com_m, vel_com_m, angles_m, omega_m;

  	// std::cout << "z \n" << z << std::endl;
	}

	//loop function, is called in main
	void iteration(){

		//std::cout <<"mass estimate is: " << mass << std::endl;
		//std::cout <<"measurement is: " << z << std::endl;

    if(iterator>250){
    	ekf_update_step();
			publish();
    	// std::cout << "\n NEW ITERATION \n" << std::endl;
    }


		++iterator;
	}


  //destructor
  ~Mass_inertia_estimator(){}

};













int main(int argc, char **argv)
{
  ros::init(argc, argv, "mass_inertia_com_estimator_node");
  ros::NodeHandle n;
  
  Mass_inertia_estimator My_estimator;

  ros::Rate loop_rate(frequency);

  while (ros::ok()){
    My_estimator.iteration();

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0; 
}
