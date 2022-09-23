
#pragma once

//source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

#include <cmath>
#include "Eigen/Dense"
#include <Eigen/Core>



struct Wrench {
    double f_x, f_y, f_z, tau_x, tau_y, tau_z;
};


struct Propeller_speeds_squared{
	double p0, p1, p2, p3, p4, p5, p6, p7;
};



class Props_to_wrench{
public:
	Wrench w;
	Propeller_speeds_squared p_struct;

	//force and moment constant of rotors
	//in newton/omega^2
	float cfa = 1.21542e-06;//1.471 * 1e-6; // force constant auxiliary

	// real cfm
	// float cfm = 16.56e-05;//4.688 * 1e-5; // force constant main

	// simulation cfm

	 float cfm = 8.58307e-05;
		// in torque/newton
	float cta = 0.011378057;//1.336 * 1e-2; // moment constant auxiliary
	float ctm = 0.02141697;//2.382 * 1e-2; // moment constatn main

	Eigen::VectorXd props = Eigen::VectorXd(8);

	//position of main propellers in body frame:
	Eigen::Vector3d r_0;
	Eigen::Vector3d r_1;
	Eigen::Vector3d r_2;
	Eigen::Vector3d r_3;

	//position of auxiliary propellers in body frame:
	Eigen::Vector3d r_4;
	Eigen::Vector3d r_5;
	Eigen::Vector3d r_6;
	Eigen::Vector3d r_7;

	//direction of main propellers in body frame:
	Eigen::Vector3d d_0;
	Eigen::Vector3d d_1;
	Eigen::Vector3d d_2;
	Eigen::Vector3d d_3;

	//direction of auxiliary propellers in body frame:
	Eigen::Vector3d d_4;
	Eigen::Vector3d d_5;
	Eigen::Vector3d d_6;
	Eigen::Vector3d d_7;

	//turning direction of propellers:
	const int n_0 = -1;
	const int n_1 = 1;
	const int n_2 = -1;
	const int n_3 = 1;
	const int n_4 = 1;
	const int n_5 = 1;
	const int n_6 = 1;
	const int n_7 = 1;




	// force and torque:
	Eigen::VectorXd f = Eigen::VectorXd(3);
	Eigen::VectorXd tau = Eigen::VectorXd(3);

		//force allocation
	Eigen::MatrixXd F_allocation=Eigen::MatrixXd(3,8);

	Props_to_wrench(){		
		r_0 << -0.418907, -0.418907, 0;
		r_1 << -0.418907, 0.418907, 0;
		r_2 << 0.418907, 0.418907, 0;
		r_3 << 0.418907, -0.418907, 0;

		r_4 << -0.455, 0, 0;
		r_5 << 0, 0.455, 0;
		r_6 << 0.455, 0, 0;
		r_7 << 0, -0.455, 0;

		d_0 << 0, 0, 1;
		d_1 << 0, 0, 1;
		d_2 << 0, 0, 1;
		d_3 << 0, 0, 1;

		d_4 << -1, 0, 0;
		d_5 << 0, 1, 0;
		d_6 << 1, 0, 0;
		d_7 << 0, -1, 0;

		F_allocation << 0, 0, 0, 0, cfa, 0, -cfa, 0,
 				0, 0, 0, 0, 0, -cfa, 0, cfa,
 				cfm, cfm, cfm, cfm, 0, 0, 0, 0;
	}


	void calculate_force(){
		f = F_allocation * props;

	}

	void calculate_torque(){
		tau = r_0.cross(d_0)*cfm*props(0) + ctm*d_0*props(0)*cfm*n_0 +
			  r_1.cross(d_1)*cfm*props(1) + ctm*d_1*props(1)*cfm*n_1 +
			  r_2.cross(d_2)*cfm*props(2) + ctm*d_2*props(2)*cfm*n_2 +
			  r_3.cross(d_3)*cfm*props(3) + ctm*d_3*props(3)*cfm*n_3 +
			  r_4.cross(d_4)*cfa*props(4) + cta*d_4*props(4)*cfa*n_4 +
			  r_5.cross(d_5)*cfa*props(5) + cta*d_5*props(5)*cfa*n_5 +
			  r_6.cross(d_6)*cfa*props(6) + cta*d_6*props(6)*cfa*n_6 +
			  r_7.cross(d_7)*cfa*props(7) + cta*d_7*props(7)*cfa*n_7;
	}

	Wrench 	props_to_wrench(Propeller_speeds_squared p){
		props << p.p0, p.p1, p.p2, p.p3, p.p4, p.p5, p.p6, p.p7;

		calculate_force();
		calculate_torque();

		w.f_x = f(0);
		w.f_y = f(1);
		w.f_z = f(2);
		w.tau_x = tau(0);
		w.tau_y = tau(1);
		w.tau_z = tau(2);

		return w;
	}



};


