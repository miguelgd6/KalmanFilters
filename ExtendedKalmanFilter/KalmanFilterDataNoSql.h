
#pragma once

#include "KalmanFilterDataNoSql.h"

#include <iostream>
#include <vector>
#include <Eigen/Dense> //eigen vectors should only be used for algebraic operations
#include <stdio.h>

using namespace std;

//includes for data reading 
#include <fstream>

class KalmanFilterDataNoSql
{
	public:
		
		KalmanFilterDataNoSql();
		~KalmanFilterDataNoSql();
		
		int GetLaserBeaconsObservation(Eigen::VectorXd systemState, vector<double> *pBeacon);
		vector<Eigen::VectorXd> GetBeaconsPosition(const int*);
		
		vector<Eigen::VectorXd> GetRobotRealPose();
		vector<Eigen::VectorXd> GetInputVector(); //best with memory dirs?, //all gets are yet to be finished
		
	private: 
		
		bool _inputVectorReady = false;
		bool _beaconsRealPoseMapReady = false;
		bool _laserObservedBeaconsReady = false;
		bool _robotMatlabKalmanPoseReady = false;
		bool _robotRealPoseReady = false;

		vector<Eigen::VectorXd> _inputVector;
		vector<Eigen::VectorXd> _beaconsRealPoseMap;
		vector<vector<double>> _laserObservedBeacons;
		vector<Eigen::VectorXd> _robotMatlabKalmanPose;
		vector<Eigen::VectorXd> _robotRealPose;
		
		void ReadInputVector(); 
		void ReadBeaconsRealPoseMap();
		void ReadLaserObservedBeacons();
		void ReadRobotMatlabKalmanPose();
		void ReadRobotRealPose();
		
		
		vector<Eigen::VectorXd> GetBeaconsRealPoseMap();
		vector<vector<double>> GetLaserObservedBeacons();
		vector<Eigen::VectorXd> GetRobotMatlabKalmanPose();
		
};

