/**
* Implementation of Dynamicalsystem class.
*
* @author: Miguelgd
* @date: 2022.12.3
*/

#pragma once

#include "DynamicalSystem.h"

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <stdio.h>

class DynamicalSystem
{
	
public:
	
	DynamicalSystem();
	~DynamicalSystem();
    DynamicalSystem(double dt, int n, int m, int l, const Eigen::MatrixXd R);
	
	void Initializer(Eigen::MatrixXd P0, Eigen::VectorXd x0, Eigen::VectorXd u0);
	void NonLinear_odometry(Eigen::VectorXd x, Eigen::VectorXd u);
	void NonLinear_observation(std::vector<Eigen::VectorXd> beaconsMapLocation);
	void GetDimensionality(int &n, int&l);
	
	Eigen::VectorXd GetKalmanPredictedState();
	Eigen::VectorXd GetOdometryPredictedState();
	Eigen::VectorXd GetObservationVector();
	Eigen::MatrixXd GetObservationMatrix();
	Eigen::MatrixXd GetMeasurementCovarianceMatrix();
	Eigen::MatrixXd GetProcessCovarianceMatrixEstimated();

	void SetKalmanPredictedState(Eigen::VectorXd); //innecesario a todas luces
	
private:
		
	void NonLinear_odometry();
	
	//dimensionality of the system 
	int    _n, _m, _l;  //let n be the dimensionality of the state vector, m the dimensionality of the control vector & l be the dimensionality of the measurement vector
	double _t0, _t, _dt;// Discrete time step
	bool   _initialized;// filter initialization flag
	
	Eigen::VectorXd _x;
	Eigen::VectorXd _x_predOdometry;
	Eigen::VectorXd _x_predKf;

	Eigen::VectorXd _zL; //_zL: laser observation vector
	Eigen::VectorXd _zO; //_zO: odometry observation vector
	Eigen::VectorXd _v;  //_v: Innovation vector

	Eigen::VectorXd _u; //control input vector
	
	Eigen::MatrixXd _A; //state transition matrix
	Eigen::MatrixXd _B; //control matrix
	Eigen::MatrixXd _H; //Observation matrix
	
	Eigen::MatrixXd _P_predOdometry; //covariance matrix associated with the process noise estimated by the odometry
	Eigen::MatrixXd _P_predKf; //covariance matrix associated with the process noise estimqated by the Kalman filter
	Eigen::MatrixXd _Q; //covariance matrix variance associated with the model noise
	Eigen::MatrixXd _R; //covariance matrix associated with the measurement noise
	
	//Eigen::MatrixXd _I1; // nxn-size identity
	//Eigen::MatrixXd _I2; // mxm-size identity
	//Eigen::MatrixXd _I3; // lxl-size identity
	
	
};

