/**
 *
 * @author: Miguel Garcia
 * @date: 2022.12.10
 * 
 */

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include "matplotlibcpp.h"

#include "DynamicalSystem.h"
#include "KalmanFilterDataNoSql.h"
#include "ExtendedKalmanFilter.h"

using namespace std;


int main(int argc, char* argv[]) {
    
    int                               n, m, l;    //dynamical system dimensionality
	double                            dt;         
    Eigen::VectorXd                   x_0, u_0;
    Eigen::MatrixXd                   R, P_0;     //Covariance matrix associated with laser measurements
	vector<Eigen::VectorXd>::iterator it_route;   //iterator for the robot actions
    KalmanFilterDataNoSql             kfd; //constructor itselfs reads data files
    
    //defining invariant parameters 
    dt = 0.1; //discrete time step for the odometry model estimations
    n = 3;    // Dimensionality of the state vector
    m = 2;    // Dimensionality of the control vector
    l = 2;    // Dimensionality of the measurement vector
    R << pow(0.016944, 2), 0, 0, 0, pow(0.016944, 2), 0, 0, 0, pow(0.016944, 2); //Covariance matrix associated with laser measurements
    
    //defining initial states 
    P_0 << pow(0.172341, 2), 0, 0, 0, pow(0.288014, 2), 0, 0, 0, pow(0.159464, 2);
	x_0 << 0, 0, 0;
	u_0 << 0, 0;
    
    DynamicalSystem PioneerRobot(dt, n, m, l, R); //Construct our Dynamical system with its invariant parameters
	ExtendedKalmanFilter ekf(PioneerRobot, kfd);  //Construct our Extended Kalman Filter with the dynamical system and the data from the database
    
	ekf.Initializer(x_0, P_0, u_0); //initialize the filter with the initial states of the robot
        
	for (it_route = kfd.GetRobotRealPose().begin(); it_route != kfd.GetRobotRealPose().end(); it_route++) {

		ekf.Prediction(*it_route); //predict the next state of the robot
		ekf.Observation(&(*it_route)); //creates the innovation vectors, taking signal from sensors
        //ekf.Observation();
        //ekf.Correction();
	}
    
    return 0;
}

