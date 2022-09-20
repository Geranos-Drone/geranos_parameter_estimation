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

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <geranos_parameter_estimation/EKF_TuningConfig.h> 




// messages
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>
#include <rw_control_utils/WrenchSensorFilteredStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <mav_msgs/TorqueThrust.h>


#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Core>


#include"functions.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::DiagonalMatrix;
using Eigen::seq;
using Eigen::Matrix3d;

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

	//position of center of mass in body frame
/*	VectorXd x_com = VectorXd(3);*/



	//position of main propellers in body frame:
/*	VectorXd r_0 = VectorXd(3);
	VectorXd r_1 = VectorXd(3);
	VectorXd r_2 = VectorXd(3);
	VectorXd r_3 = VectorXd(3);*/




	//position of auxiliary propellers in body frame:
/*	VectorXd r_4 = VectorXd(3);
	VectorXd r_5 = VectorXd(3);
	VectorXd r_6 = VectorXd(3);
	VectorXd r_7 = VectorXd(3);*/




	//direction of main propellers in body frame:
/*	VectorXd d_0 = VectorXd(3);
	VectorXd d_1 = VectorXd(3);
	VectorXd d_2 = VectorXd(3);
	VectorXd d_3 = VectorXd(3);*/



	//direction of auxiliary propellers in body frame:
/*	VectorXd d_4 = VectorXd(3);
	VectorXd d_5 = VectorXd(3);
	VectorXd d_6 = VectorXd(3);
	VectorXd d_7 = VectorXd(3);*/



	//gravity constant
	Vector3d g_vec;
	float gravity_constant = 9.8;



	//force and moment constant of rotors
		//in newton/omega^2
/*	float cfa = 1.471 * 1e-6; // force constant auxiliary
	float cfm = 4.688 * 1e-5; // force constant main
		// in torque/newton
	float cta = 1.336 * 1e-2; // moment constant auxiliary
	float ctm = 2.382 * 1e-2; // moment constatn main
*/
	//force allocation
	// MatrixXd F_allocation=MatrixXd(3,8);




	////////////////////////////////////////////
	////////ekf parameters:

	// states and state vector, predictor vectors

  VectorXd x = VectorXd(15);           				
  VectorXd pos_com = VectorXd(3);
	VectorXd vel_com = VectorXd(3);
	VectorXd angles = VectorXd(3);
	Vector3d omega;// = VectorXd(3);
	Matrix3d J;// = MatrixXd(3,3);
	float mass;

	//predicted state
  VectorXd x_p = VectorXd(15);
	VectorXd pos_com_p = VectorXd(3);
	VectorXd vel_com_p = VectorXd(3);
	VectorXd angles_p = VectorXd(3);
	VectorXd omega_p = VectorXd(3);

	//measured pose and twist:
	VectorXd pos_com_m = VectorXd(3);
	VectorXd vel_com_m = VectorXd(3);
	VectorXd angles_m = VectorXd(3);
	VectorXd omega_m = VectorXd(3);

	
	Quaternion quaternion;
	EulerAngles eulerangles;

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
	Vector3d torque_m_thrust;
	Vector3d torque_a_thrust;
	Vector3d torque_m_motor;
	Vector3d torque_a_motor;
	Vector3d torque_props;
	Vector3d torque_com;

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
	
	VectorXd q_ekf = VectorXd(15);
	MatrixXd Q_ekf = MatrixXd(15,15);


	//----initialisation ekf matrices: 

	//predicted state covariance matrix
	MatrixXd P_ekf = MatrixXd::Identity(15,15)*1e6;

	//linearized state to measruement map
	MatrixXd H_ekf = MatrixXd::Identity(12,15);

	//linearized state update map:
	MatrixXd A_ekf = MatrixXd(15,15);

	//measurement resiudal:
	VectorXd y_ekf = VectorXd(12);

	//innovation covariance:
	MatrixXd S_ekf = MatrixXd(12,12);

	//kalman gain:
	MatrixXd K_ekf = MatrixXd(15,12);

	//identity matrix
	MatrixXd Identity_15 = MatrixXd::Identity(15,15);


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

	//force components in body frame:
	float fx;
	float fy;
	float fz;

	//force components in world frame
	float Rfx;
	float Rfy;
	float Rfz;

	//torque components:
	float torque_x;
	float torque_z;
	float torque_y;

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
	ros::Subscriber controller_sub = n.subscribe("geranos/wrench_target", 1000, &Mass_inertia_estimator::controller_callback, this); // TODO correct topic name

//------------dynamic reconfigure

  dynamic_reconfigure::Server<geranos_parameter_estimation::EKF_TuningConfig> server;
  dynamic_reconfigure::Server<geranos_parameter_estimation::EKF_TuningConfig>::CallbackType f;

  //calback
  void dynrcf_callback(geranos_parameter_estimation::EKF_TuningConfig& config, double_t level){


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
		q_ekf << q_pos, q_vel, q_angles, q_omega, q_mass, q_Jxy, q_Jz;
		Q_ekf = q_ekf.asDiagonal();

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

		estimation_msg.header.stamp = ros::Time::now();		


		estimate_publisher.publish(estimation_msg);
	}



//calculate the full ekf steps
	void ekf_update_step(){//TODO

		//----------------------prediction update

		//calculate rotation matrix from roll, ptich and yaw
		//delta_t = 1/frequency;
		angles_to_rot_and_trans();
		// std::cout << "angles: " << angles<< "\n" << std::endl;
		// std::cout << "T: " << T << "\n" << std::endl;
		// std::cout << "R: " << R_total << "\n" << std::endl;


		//propeller forces in world frame
		force_w = R_total * force_prop;
		//propeller force components in world frame
		Rfx = force_w(0);
		Rfy = force_w(1);
		Rfz = force_w(2);

		//-----------predict state: 
		pos_com_p = pos_com + delta_t * vel_com;
		vel_com_p = vel_com + delta_t * (force_w / mass + g_vec);
		angles_p = angles+delta_t*T.inverse() * omega;
		omega_p = omega + delta_t * J.inverse() * (torque_com-omega.cross(J*omega));
		//mass_p = mass;
		//J_p = J;
		// std::cout << "mass: " << mass << "\n" << std::endl;
		
		// std::cout << "J: " << J << "\n" << std::endl;

		//full predicted state vector
		x_p << pos_com_p, vel_com_p, angles_p, omega_p, mass, Jxy, Jz;


		//-----------precalculations for covariance prediction step:
		//components of propeller force in body frame:
		fx = force_prop(0);
		fy = force_prop(1);
		fz = force_prop(2);

		

		//torque components in body frame:
		torque_x = torque_com(0);
		torque_z = torque_com(1);
		torque_y = torque_com(2);

		//angular velocity components in body frame:
		omega_x = omega(0);
		omega_y = omega(1);
		omega_z = omega(2);

		//trigonometric functins of roll pitch yaw:
		sroll = sin(roll);
		spitch = sin(pitch);
		syaw = sin(yaw);
		croll = cos(roll);
		cpitch = cos(pitch);
		cyaw = cos(yaw);

				//partial derivatives of rotationmatrix*force wtr. to roll, pitch, yaw
		R_x_roll = 0;
		R_x_pitch = -fx*cyaw*spitch + fy*spitch*syaw + fz*cpitch;
		R_x_yaw = -fx* syaw*cpitch - fy*cpitch*cyaw;

		R_y_roll = fx*(-sroll*syaw + cyaw*croll*spitch) + fy*(-sroll*cyaw - croll*syaw*spitch) - fz*cpitch*croll;
		R_y_pitch = fx*cyaw*sroll*cpitch - fy*sroll*syaw*spitch + fz*spitch*sroll;
		R_y_yaw = fx*(croll*cyaw - syaw*sroll*spitch) + fy*(-croll*syaw - sroll*cyaw*spitch);

		R_z_roll = fx*(croll*syaw + sroll*cyaw*spitch) + fy*(cyaw*croll + sroll*syaw*spitch) - fz*sroll*cpitch;
		R_z_pitch = -fx*croll*cyaw*cpitch - fy*croll*syaw*cpitch -fz*croll*spitch;
		R_z_yaw = fx*(sroll*cyaw + croll*syaw*spitch) + fy*(-syaw*sroll -croll*cyaw*spitch);


		// prevent devision by zero
		// std::cout << "x_p: " <<x_p << "\n" << std::endl;
		std::cout << P_ekf << "\n" << std::endl;
		// std::cout << "x: " << x << "\n" << std::endl;
		if(mass==0 || Jxy==0 || Jz==0 || cpitch==0){
			ROS_ERROR_STREAM("devision by zero");
			abort();
		}
		if(isnan(mass) || isnan(Jxy) || isnan(Jz) || isnan(cpitch)){
			ROS_ERROR_STREAM("nan value");
			abort();
		}
		std::cout << "NEW ITERATION \n" << std::endl;

		

		// linearized state update matrix A_ekf
		A_ekf << 1, 0, 0, delta_t, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				     0, 1, 0, 0, delta_t, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				     0, 0, 1, 0, 0, delta_t, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				     0, 0, 0, 1, 0, 0, R_x_roll*delta_t/mass, R_x_pitch*delta_t/mass, R_x_yaw*delta_t/mass, 0, 0, 0, -Rfx*delta_t/(mass*mass), 0, 0,
				     0, 0, 0, 0, 1, 0, R_y_roll*delta_t/mass, R_y_pitch*delta_t/mass, R_y_yaw*delta_t/mass, 0, 0, 0, -Rfy*delta_t/(mass*mass), 0, 0,
				     0, 0, 0, 0, 0, 1, R_z_roll*delta_t/mass, R_z_pitch*delta_t/mass, R_z_yaw*delta_t/mass, 0, 0, 0, -Rfz*delta_t/(mass*mass), 0, 0,
				     0, 0, 0, 0, 0, 0, 1, delta_t*(spitch/(pow(cpitch,2)))*(cyaw*omega_x-syaw*omega_y), delta_t*((-syaw/cpitch)*omega_x-(cyaw/cpitch)*omega_y), delta_t*cyaw/cpitch, -delta_t*syaw/cpitch,  0, 0, 0, 0,
				     0, 0, 0, 0, 0, 0, 0, 1, delta_t*(cyaw*omega_x-syaw*omega_y), delta_t*syaw, delta_t*cyaw, 0, 0, 0, 0,
				     0, 0, 0, 0, 0, 0, 0, delta_t*(-cyaw*omega_x +syaw*omega_y)/(pow(cpitch,2)), 1+delta_t*(syaw*tan(pitch)*omega_x+cyaw*tan(pitch)*omega_y), -delta_t*cyaw*tan(pitch), delta_t*syaw*tan(pitch), -delta_t, 0, 0, 0,
				     0, 0, 0, 0, 0, 0, 0, 0, 0, 1, delta_t*(Jxy*omega_z-Jz*omega_z)/Jxy, delta_t*(Jxy*omega_y-Jz*omega_y)/Jxy, 0, -delta_t*(torque_x-Jz*omega_y*omega_z)/(Jxy*Jxy), -delta_t*omega_y*omega_z/Jxy,
				     0, 0, 0, 0, 0, 0, 0, 0, 0, delta_t*(Jz*omega_z-Jxy*omega_z)/Jxy, 1, delta_t*(Jz*omega_x-Jxy*omega_x)/Jxy, 0, -delta_t*(torque_y+Jz*omega_x*omega_z)/(Jxy*Jxy), delta_t*omega_x*omega_z/Jxy,
				     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, -delta_t*torque_z/(Jz*Jz),
				     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
				     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
				     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

	  //covariance update step:
    P_ekf = A_ekf * P_ekf * A_ekf.transpose() + Q_ekf;

    // std::cout << "A: " << A_ekf << "\n" << std::endl;

    


		//----------------------measurement update
		h=H_ekf * x_p; //only works because it's linear
    y_ekf = z - h;


 

    S_ekf = H_ekf * P_ekf *H_ekf.transpose() + R_ekf;

    // std::cout << "S: " << S_ekf << "\n" << std::endl;

    K_ekf = P_ekf * H_ekf.transpose() * S_ekf.inverse();

    // std::cout << "K: " << K_ekf << "\n" << std::endl;



    //measurement state update
    x = x_p + K_ekf * y_ekf;
    // std::cout << "y: " << y_ekf << "\n" << std::endl;
    // std::cout << "xp: " << x_p << "\n" << std::endl;
    std::cout << "angvel_z " << x(11) << "\n" << std::endl;

    // really bad fix for diverging omega_z: constraint: bounding omegaz at i
    if(abs(x(11))>1){
    	x(11)=abs(x(11))/x(11);
    	ROS_ERROR_STREAM("omega z  diverges");
    }



    //covariance update step
    P_ekf = (Identity_15 - K_ekf * H_ekf) * P_ekf;

    //update seperate state vectors:
    pos_com = x(seq(0,2));

    vel_com = x(seq(3,5));

    roll = x(6);
    pitch = x(7);
    yaw = x(8);
    angles = x(seq(6,8));


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
/*		x_com << 0, 0, -0.07;

		r_0 << -0.4189, -0.4189, 0;
		r_1 << -0.4189, 0.4189, 0;
		r_2 << 0.4189, 0.4189, 0;
		r_3 << 0.4189, -0.4189, 0;

		r_0 << -0.4465, 0, 0;
		r_0 << 0, 0.4465, 0;
		r_0 << 0.4465, 0, 0;
		r_0 << 0, -0.4465, 0;

		d_0 << 0, 0, 1;
		d_1 << 0, 0, 1;
		d_2 << 0, 0, 1;
		d_3 << 0, 0, 1;

		d_4 << -1, 0, 0;
		d_5 << 0, 1, 0;
		d_6 << 1, 0, 0;
		d_7 << 0, -1, 0;*/

		// g_vec << 0, 0, -9.8; //-9.80665;

/*		F_allocation << 0, 0, 0, 0, cfa, 0, -cfa, 0,
         				0, 0, 0, 0, 0, -cfa, 0, cfa,
         				cfm, cfm, cfm, cfm, 0, 0, 0, 0;*/

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
  void controller_callback(const mav_msgs::TorqueThrust& message){// Type: mav_msgs/TorqueThrust
  	//read commanded force and torque

  	force_prop << message.thrust.x, message.thrust.y, message.thrust.z;
  	torque_com << message.torque.x, message.torque.y, message.torque.z;
  	

	}


  void msf_callback(const nav_msgs::Odometry message){//Type: nav_msgs/Odometry
  	//creat measurement vector z

	  pos_com_m << message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z;
		vel_com_m << message.twist.twist.linear.x, message.twist.twist.linear.y, message.twist.twist.linear.z;

		quaternion.x = message.pose.pose.orientation.x;
		quaternion.y = message.pose.pose.orientation.y; 
		quaternion.z = message.pose.pose.orientation.z; 
		quaternion.w = message.pose.pose.orientation.w;

		eulerangles = ToEulerAngles(quaternion);

		angles_m << eulerangles.roll, eulerangles.pitch, eulerangles.yaw;

		omega_m << message.twist.twist.angular.x, message.twist.twist.angular.y, message.twist.twist.angular.z;
		

  	z << pos_com_m, vel_com_m, angles_m, omega_m;
	}

	//loop function, is called in main
	void iteration(){

		//std::cout <<"mass estimate is: " << mass << std::endl;
		//std::cout <<"measurement is: " << z << std::endl;

    if(iterator>250){
    	ekf_update_step();
			publish();
    	// std::cout << "state estimate = "<< x << std::endl;
    }


		++iterator;
	}


  //destructor
  ~Mass_inertia_estimator(){}

};













int main(int argc, char **argv)
{
  ros::init(argc, argv, "mass_and_inertia_estimator_node");
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
