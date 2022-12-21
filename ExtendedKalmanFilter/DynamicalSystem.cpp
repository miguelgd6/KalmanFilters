#include "DynamicalSystem.h"

//Constructors
DynamicalSystem::DynamicalSystem()
{
    
}

//-----------------------------------------------------------------------------------------------------------------------------

DynamicalSystem::~DynamicalSystem()
{
    //sin definir aún
}

//-----------------------------------------------------------------------------------------------------------------------------

DynamicalSystem::DynamicalSystem(
    const double dt,
    const int n,
    const int m,
    const int l,
    const Eigen::MatrixXd R)
    :_dt(dt), _n(n), _m(m), _l(l), _R(R), _initialized(false)
{
    
    _A.setIdentity(_n, _n); //sure that with this all dimensiones are correct?
    _B.setIdentity(_n, _m);
    _H.setIdentity(_l, _l);
    _P.setIdentity(_n, _n);
    _Q.setIdentity(_n, _m);
    
}

//-----------------------------------------------------------------------------------------------------------------------------

void DynamicalSystem::Initializer(Eigen::MatrixXd P0, Eigen::VectorXd x0, Eigen::VectorXd u0)
{
   
    //añadir seguridad dimensional
    _x = _x_predOdometry = _x_predKf = x0;
    _u << u0;
    _t = _t0 = 0;
    
    _P = P0;
    this->NonLinear_odometry();
    
    _initialized;//= true?
}

//-----------------------------------------------------------------------------------------------------------------------------

void DynamicalSystem::NonLinear_odometry()
{

    //añadir seguridad dimensional
    _A << 1, 0, (-_u(1) * sin(_x(3) + _u(2) / 2)),
        0, 1, (_u(1) * cos(_x(3) + _u(2) / 2)),
        0, 0, 1;

    //Odometry prediction
    _B << (cos(_x(3) + _u(2) / 2)), (-0.5 * _u(1) * sin(_x(3) + _u(2) / 2)),
        (sin(_x(3) + _u(2) / 2)), (0.5 * _u(1) * cos(_x(3) + _u(2) / 2)),
        0, 1;

    //Odometry covariance, depends on control input
    _Q << pow((0.05 * _u(1) * _u(2) / 0.1), 2), 0,
        0, (0.05 * _u(2));

}   

//-----------------------------------------------------------------------------------------------------------------------------

void DynamicalSystem::NonLinear_odometry( Eigen::VectorXd x, Eigen::VectorXd u )
{
    
	//añadir seguridad dimensional
    _A << 1, 0, (-_u(1) * sin(_x(3) + _u(2) / 2)),
           0, 1, (_u(1) * cos(_x(3) + _u(2) / 2)),
           0, 0, 1;

    //Odometry prediction
    _B << (cos(_x(3) + _u(2) / 2)), (-0.5 * _u(1) * sin(_x(3) + _u(2) / 2)),
           (sin(_x(3) + _u(2) / 2)), (0.5 * _u(1) * cos(_x(3) + _u(2) / 2)),
           0, 1;

    //Odometry covariance, depends on control input
    _Q << pow((0.05 * _u(1) * _u(2) / 0.1), 2), 0,
           0, (0.05 * _u(2));
    
    //Odometry prediction
    _x_predOdometry << (_x(1) + _u(1) * cos(_x(3) + (_u(2) / 2))),
                       (_x(2) + _u(1) * sin(_x(3) + (_u(2) / 2))),
		               (_x(3) + _u(2)); 
    
    _x_predOdometry(3) -= 2 * EIGEN_PI * floor((_x_predOdometry(3) + EIGEN_PI) * (1. / (2 * EIGEN_PI)));//Investigar mas adelante
        
}

//-----------------------------------------------------------------------------------------------------------------------------
void DynamicalSystem::NonLinear_observation(std::vector<Eigen::VectorXd> beaconsMapLocation) //the obervation estimation should be here?
{
    Eigen::VectorXd b1, b2, b3;
    
    b1 = beaconsMapLocation.at(0);
    b2 = beaconsMapLocation.at(1);
    b3 = beaconsMapLocation.at(2);//onlye 3 are always used?¿
    
    //observation matrix construction
    //If we are in first iteration, _x_predKf does not yet exists, an alternative must be found later (today)
    _H << (b1(1) - _x_predKf(1)) / (pow(b1(0) - _x_predKf(0), 2) + pow(b1(1) - _x_predKf(1), 2)),
        -(b1(0) - _x_predKf(0)) / (pow(b1(0) - _x_predKf(0), 2) + pow(b1(1) - _x_predKf(1), 2)),
        -1, //first row
        (b2(1) - _x_predKf(1)) / (pow(b2(0) - _x_predKf(0), 2) + pow(b2(1) - _x_predKf(1), 2)),
        -(b2(0) - _x_predKf(0)) / (pow(b2(0) - _x_predKf(0), 2) + pow(b2(1) - _x_predKf(1), 2)),
        -1, //second row
        (b3(1) - _x_predKf(1)) / (pow(b3(0) - _x_predKf(0), 2) + pow(b3(1) - _x_predKf(1), 2)),
        -(b3(0) - _x_predKf(0)) / (pow(b3(0) - _x_predKf(0), 2) + pow(b3(1) - _x_predKf(1), 2)),
        -1; //third row

    _zO << atan2(b1(1) - _x_predKf(1), b1(0) - _x_predKf(0)) - _x_predKf(2),//first row
        atan2(b2(1) - _x_predKf(1), b2(0) - _x_predKf(0)) - _x_predKf(2),//second row
        atan2(b3(1) - _x_predKf(1), b3(0) - _x_predKf(0)) - _x_predKf(2);//third row
}

//-----------------------------------------------------------------------------------------------------------------------------

void DynamicalSystem::GetDimensionality(int &n, int &l) //which is the best way of doing this?¿//by reference for now, smart pointers later on
{
    
    n = _n;
    l = _l;
    
}

//-----------------------------------------------------------------------------------------------------------------------------

Eigen::VectorXd DynamicalSystem::GetOdometryPredictedState()
{

    return _x_predOdometry;

}

//-----------------------------------------------------------------------------------------------------------------------------

Eigen::VectorXd DynamicalSystem::GetKalmanPredictedState()
{

    return _x_predKf;

}

//-----------------------------------------------------------------------------------------------------------------------------

void DynamicalSystem::SetKalmanPredictedState(Eigen::VectorXd x)
{

    _x_predKf = x;

}

//-----------------------------------------------------------------------------------------------------------------------------
//
//void DynamicalSystem::SetOdometryPredictedState(Eigen::VectorXd x)
//{
//
//	_x_predOdometry = x;
//
//}

//-----------------------------------------------------------------------------------------------------------------------------

Eigen::VectorXd DynamicalSystem::GetObservationVector()
{

	return _zO;

}

//-----------------------------------------------------------------------------------------------------------------------------

Eigen::MatrixXd DynamicalSystem::GetObservationMatrix()
{

    return _H;

}

//-----------------------------------------------------------------------------------------------------------------------------

Eigen::MatrixXd DynamicalSystem::GetMeasurementCovarianceMatrix()
{

    return _R;

}

//-----------------------------------------------------------------------------------------------------------------------------

Eigen::MatrixXd DynamicalSystem::GetProcessCovarianceMatrixEstimated()
{

    return _P_predOdometry;

}

