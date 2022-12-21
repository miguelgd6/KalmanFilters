#include "ExtendedKalmanFilter.h"

//Constructors
ExtendedKalmanFilter::ExtendedKalmanFilter() 
{
	
}

//-----------------------------------------------------------------------------------------------------------------------------

ExtendedKalmanFilter::~ExtendedKalmanFilter() 
{
	
}

//-----------------------------------------------------------------------------------------------------------------------------

ExtendedKalmanFilter::ExtendedKalmanFilter(
	DynamicalSystem dynamicalSystem, 
	KalmanFilterDataNoSql kalmanFilterData)
	:_dynamicalSystem(dynamicalSystem), _kalmanFilterData(kalmanFilterData)
{
	dynamicalSystem.GetDimensionality(_n, _l);
	
}

//-----------------------------------------------------------------------------------------------------------------------------

void ExtendedKalmanFilter::Initializer( Eigen::MatrixXd P0, Eigen::VectorXd x0, Eigen::VectorXd u0)
{
	
	_dynamicalSystem.Initializer(P0, x0, u0);

}

//-----------------------------------------------------------------------------------------------------------------------------

void ExtendedKalmanFilter::Prediction( Eigen::VectorXd systemState)
{
	
	Eigen::VectorXd x, u; 
	x = _dynamicalSystem.GetKalmanPredictedState();
	u << _kalmanFilterData.GetInputVector().at(systemState(0))(1), _kalmanFilterData.GetInputVector().at(systemState(0))(2);
	_dynamicalSystem.NonLinear_odometry( x, u );
	
}

//-----------------------------------------------------------------------------------------------------------------------------

void ExtendedKalmanFilter::Observation(Eigen::VectorXd *systemState )
{
	int beaconsNum, *beaconsId;
	size_t columnSort = 2;
	vector<double>*pBeacons;
	vector<vector<double>> beaconsObservedAux, beaconsObserved;
	vector<Eigen::VectorXd> beaconsMapLocation;//beacons map location vector, short name due its use in observation matrix construction
	Eigen::VectorXd zL, zO, b1, b2, b3, xO; //zL: laser observation vector, zO: odometry observation vector (constains angles), xO Odometry state vector
	
	//things should be moved from here:
	Eigen::MatrixXd H(3, 3); //Observation matrix, dimensionalisty should be defined by the dynamical system dimensions
	Eigen::VectorXd::iterator it;
	
	pBeacons = NULL; //Using deallocated memory would be a better a idea ?¿?
	beaconsNum = 0;
	xO = _dynamicalSystem.GetOdometryPredictedState();
	beaconsNum = _kalmanFilterData.GetLaserBeaconsObservation(*(systemState + 1), pBeacons);  //only if more than two beacons are observed, implement later
	
	if (beaconsNum > 2)
	{
		
		copy(pBeacons, pBeacons + beaconsNum, back_inserter(beaconsObservedAux));
		
		//https://stackoverflow.com/questions/72465968/how-to-sort-an-vector-based-on-any-column-value-by-passing-the-column-index-as-i
		sort(beaconsObservedAux.begin(), beaconsObservedAux.end(),
			[columnSort](vector<double> const& v1, vector<double> const& v2)
			{
				return v1[columnSort] < v2[columnSort]; //hay que indagar mas en esto, se puede acortar aqui el vector tambien?¿
			});
		
		beaconsObserved.push_back(beaconsObservedAux.at(0)); //horrible, but it works for now
		beaconsObserved.push_back(beaconsObservedAux.at(1));
		beaconsObserved.push_back(beaconsObservedAux.at(2));
				
		beaconsId = new int[beaconsNum]; 
		for (int i = 0; i < beaconsNum; i++)
		{
			beaconsId[i] = beaconsObserved.at(i).at(0);//we know id of beacon by just observing it?¿ wtf
		}
		
		//absolute positions of beacons are obtained
		beaconsMapLocation = _kalmanFilterData.GetBeaconsPosition(beaconsId);
		b1 = beaconsMapLocation.at(0);
		b2 = beaconsMapLocation.at(1);
		b3 = beaconsMapLocation.at(2);//onlye 3 are always used?¿
			
		_dynamicalSystem.NonLinear_observation( beaconsMapLocation );
		
		zL << beaconsObserved[0][3], beaconsObserved[1][3], beaconsObserved[2][3];
		zO = _dynamicalSystem.GetObservationVector();
		
		//comparation (move later) and other mathematical operations that should be reorganized later	
		_v = zL - zO;
		for (it == _v.cbegin(); it != _v.cend(); it++)
		{
			*it > EIGEN_PI ? *it -= 2 * EIGEN_PI : *it;
			*it < EIGEN_PI ? *it += 2 * EIGEN_PI : *it;
		}//clearer with a for_each statement

		delete[] beaconsId;
		beaconsId = NULL;
	}
}

//-----------------------------------------------------------------------------------------------------------------------------

void ExtendedKalmanFilter::Comparation() //it is assumed that is function is called only when the system have the needed data 
{
	//As comparation is the most complex part of the algorithm, comments are added to explain the process
	// 1- First we need to construct our innovation vector 
	// 2- By calculating covariance of that vector S matrix can be constructed
	// 3- Thus Kalman Gain is finally obtained 
	 	  
	//innovation vector (move it here later)

	
	//Covariance of the innovation vector
	// curioso lo que nos propone el copilot, usar el objeto kfd es buena 
	//_S = _dynamicalSystem.GetObservationMatrix() * _dynamicalSystem.GetCovarianceMatrix() * _dynamicalSystem.GetObservationMatrix().transpose() + _kalmanFilterData.GetObservationNoiseCovarianceMatrix();
	_S = _dynamicalSystem.GetObservationMatrix() * _dynamicalSystem.GetProcessCovarianceMatrixEstimated() * _dynamicalSystem.GetObservationMatrix().transpose() + _dynamicalSystem.GetMeasurementCovarianceMatrix();
	
	//Kalman Fucking Gain estimation 
	_W = _dynamicalSystem.GetProcessCovarianceMatrixEstimated() * _dynamicalSystem.GetObservationMatrix().transpose() * _S.inverse();
	

	
	delete[] beaconsId;
	beaconsId = NULL;
	

}

//-----------------------------------------------------------------------------------------------------------------------------

//void ExtendedKalmanFilter::Correction(Eigen::VectorXd* systemState)
//{
//
//}

//-----------------------------------------------------------------------------------------------------------------------------