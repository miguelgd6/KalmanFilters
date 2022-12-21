/**
* Implementation of ExtendedKalmanFilter class.
*
* @author: Miguelgd
* @date: 2022.12.3
*/

#pragma once

#include "ExtendedKalmanFilter.h"
#include "DynamicalSystem.h"
#include "KalmanFilterDataNoSql.h"

#include <iostream>
#include <Eigen/Dense>
#include <stdio.h>
#include <stdexcept>

#include <vector>
#include <algorithm>

class ExtendedKalmanFilter {

public:
    
    ExtendedKalmanFilter();
    ~ExtendedKalmanFilter();
	ExtendedKalmanFilter( DynamicalSystem dynamicalSystem, KalmanFilterDataNoSql kalmanFilterDataNoSql);
    
    void Initializer( Eigen::MatrixXd P0, Eigen::VectorXd x0, Eigen::VectorXd u0 ); //una variable en mayusculas es el mal
    void Prediction( Eigen::VectorXd );
    void Observation(Eigen::VectorXd *);
    void Comparation();
    void Correction();
    
    //void init(); //Initialize the filter with initial states as zero.
	//void init(double t0, const Eigen::VectorXd& x0); //Initialize the filter with a guess for initial states.
    //void update(const Eigen::VectorXd& y); // Update the estimated state based on measured values. The time step is assumed to remain constant.
    //void update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A); // Update the estimated state based on measured values, using the given time step and dynamics matrix.
	//Eigen::VectorXd state() { return x_hat; }; // Return the current state.
    //double time() { return t; };

private:

	int _n, _l;         //being n the dimensionality of the state vector & l the dimensionality of the measurement vector
    Eigen::VectorXd _v; //innovation vector 
	Eigen::MatrixXd _W; //Kalman Gain (dimensionality nxl)
	Eigen::MatrixXd _S; //Innovation covariance matrix (dimensionality lxl)
    
    DynamicalSystem _dynamicalSystem;//más adelante analizar la opción de usar punteros a función
    KalmanFilterDataNoSql _kalmanFilterData;
    
      
}; 
