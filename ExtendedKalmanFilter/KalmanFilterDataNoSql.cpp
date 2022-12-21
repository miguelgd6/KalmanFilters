#include "KalmanFilterDataNoSql.h"

KalmanFilterDataNoSql::KalmanFilterDataNoSql() 
{
	cout << "started data reading from files (no Database)" << endl; //add time measumerements
    ReadInputVector();
    ReadBeaconsRealPoseMap();
    ReadLaserObservedBeacons();
    ReadRobotMatlabKalmanPose();
    ReadRobotRealPose();
    cout << "finished data reading from files" << endl; 
}

//-----------------------------------------------------------------------------------------------------------------------------

KalmanFilterDataNoSql::~KalmanFilterDataNoSql()
{
    //sin definir aún
}
//-----------------------------------------------------------------------------------------------------------------------------

void KalmanFilterDataNoSql::ReadInputVector() 
{
    double word_aux;
    string line, word;
    fstream fin;
    vector<double> row;
    Eigen::VectorXd inputVectorAux(3); //[pathIteration], [V], [theta]

    // opens an existing csv file or creates a new file.
    fin.open("InputVector.csv", ios::in);

    while (!fin.eof()) {

        getline(fin, line);
        stringstream s(line);

        while (getline(s, word, ';')) {
            word_aux = stod(word);
            row.push_back(word_aux);
        }

        if (!row.empty())
        {
            inputVectorAux << row.at(0), row.at(1), row.at(2);
            _inputVector.push_back(inputVectorAux);

            row.clear();
            cout << inputVectorAux << endl;
        }
        
        _inputVectorReady = _inputVector.size() != 0 ? true : false;
    }
}

//-----------------------------------------------------------------------------------------------------------------------------

void KalmanFilterDataNoSql::ReadBeaconsRealPoseMap() 
{
    double word_aux;
    string line, word;
    fstream fin;
    vector<double> row;
    Eigen::VectorXd BeaconsRealPoseMapAux(2); //[x], [y]
    
    // opens an existing csv file or creates a new file.
    fin.open("BeaconsRealPoseMap.csv", ios::in);

    while (!fin.eof()) {

        getline(fin, line);
        stringstream s(line);

        while (getline(s, word, ';')) {
            word_aux = stod(word);
            row.push_back(word_aux);
        }

        if (!row.empty())
        {
            BeaconsRealPoseMapAux << row.at(0), row.at(1);
            _beaconsRealPoseMap.push_back(BeaconsRealPoseMapAux);
            
            row.clear();
            cout << BeaconsRealPoseMapAux << endl;
        }
        
        _beaconsRealPoseMapReady = _beaconsRealPoseMap.size() != 0 ? true : false;
    }
}

//-----------------------------------------------------------------------------------------------------------------------------

void KalmanFilterDataNoSql::ReadLaserObservedBeacons() 
{
    double word_aux;
    string line, word;
    fstream fin;
    vector<double> row;
    //vector<double> laserObservedBeaconsAux(4); //[pathIteration], [beacon_id], [distance], [angle]
    
    // opens an existing csv file or creates a new file.
    fin.open("LaserObservedBeacons.csv", ios::in);

    while (!fin.eof()) {

        getline(fin, line);
        stringstream s(line);

        while (getline(s, word, ';')) {
            word_aux = stod(word);
            row.push_back(word_aux);
        }

        if (!row.empty())
        {
            //laserObservedBeaconsAux << row.at(0), row.at(1), row.at(2), row.at(3);
            //_laserObservedBeacons.push_back(laserObservedBeaconsAux);
            _laserObservedBeacons.push_back(row);

            cout << "[pathIteration]:" << row.at(0) << "[beacon_id]:" << row.at(1) 
                 << "[distance]:" << row.at(2) <<"[angle]:" << row.at(2) << endl;
            
            row.clear();
        }
        
        _laserObservedBeaconsReady = _laserObservedBeacons.size() != 0 ? true : false;
    }
}

//-----------------------------------------------------------------------------------------------------------------------------

void KalmanFilterDataNoSql::ReadRobotMatlabKalmanPose() 
{
    double word_aux;
    string line, word;
    fstream fin;
    vector<double> row;
    Eigen::VectorXd robotMatlabKalmanPoseAux(4); //[pathIteration], [x], [y], [z]

    // opens an existing csv file or creates a new file.
    fin.open("ReadRobotMatlabKalmanPose.csv", ios::in);

    while (!fin.eof()) {

        getline(fin, line);
        stringstream s(line);

        while (getline(s, word, ';')) {
            word_aux = stod(word);
            row.push_back(word_aux);
        }

        if (!row.empty())
        {
            robotMatlabKalmanPoseAux << row.at(0), row.at(1), row.at(2), row.at(3);
            _robotMatlabKalmanPose.push_back(robotMatlabKalmanPoseAux);

            row.clear();
            cout << robotMatlabKalmanPoseAux << endl;
        }
        
        _robotMatlabKalmanPoseReady = _robotMatlabKalmanPose.size() != 0 ? true : false;
    }
}

//-----------------------------------------------------------------------------------------------------------------------------

void KalmanFilterDataNoSql::ReadRobotRealPose() //this should be state, not pose
{
    double word_aux;
    string line, word;
    fstream fin;
    vector<double> row;
    Eigen::VectorXd robotRealPoseAux(4); //[pathIteration], [x], [y], [z]

    // opens an existing csv file or creates a new file.
    fin.open("RobotRealPose.csv", ios::in);

    while (!fin.eof()) {

        getline(fin, line);
        stringstream s(line);

        while (getline(s, word, ';')) {
            word_aux = stod(word);
            row.push_back(word_aux);
        }

        if (!row.empty())
        {
            robotRealPoseAux << row.at(0), row.at(1), row.at(2), row.at(3);
            _robotRealPose.push_back(robotRealPoseAux);

            row.clear();
            cout << robotRealPoseAux << endl;
        }
        _robotRealPoseReady = _robotRealPose.size() != 0 ? true : false;
    }
}

//-----------------------------------------------------------------------------------------------------------------------------

vector<Eigen::VectorXd> KalmanFilterDataNoSql::GetInputVector()
{
	if (_inputVectorReady)
	{
		return _inputVector;
	}
	else
	{
		cout << "inputVector is not ready" << endl;
		return _inputVector;
	}
    
}

//-----------------------------------------------------------------------------------------------------------------------------

vector<Eigen::VectorXd> KalmanFilterDataNoSql::GetBeaconsRealPoseMap()
{
    if (_beaconsRealPoseMapReady)
    {
        return _beaconsRealPoseMap;
    }
    else
    {
        cout << "beaconsRealPoseMap is not ready" << endl;
        return _beaconsRealPoseMap;
    }
}

//-----------------------------------------------------------------------------------------------------------------------------

vector<vector<double>> KalmanFilterDataNoSql::GetLaserObservedBeacons()
{
    if (_laserObservedBeaconsReady)
    {
        return _laserObservedBeacons;
    }
	else
	{
		cout << "laserObservedBeacons is not ready" << endl;
		return _laserObservedBeacons;
	}
}

//-----------------------------------------------------------------------------------------------------------------------------

vector<Eigen::VectorXd> KalmanFilterDataNoSql::GetRobotMatlabKalmanPose()
{
	if (_robotMatlabKalmanPoseReady)
	{
		return _robotMatlabKalmanPose;
	}
    else
    {
        cout << "robotMatlabKalmanPose is not ready" << endl;
        return _robotMatlabKalmanPose;
    }
    
}

//-----------------------------------------------------------------------------------------------------------------------------

vector<Eigen::VectorXd> KalmanFilterDataNoSql::GetRobotRealPose()
{
	if (_robotRealPoseReady)
	{
		return _robotRealPose;
	}
	else
	{
		cout << "robotRealPose is not ready" << endl;
		return _robotRealPose;
	}
}

//-----------------------------------------------------------------------------------------------------------------------------

int KalmanFilterDataNoSql::GetLaserBeaconsObservation(Eigen::VectorXd systemState, vector<double> *pBeacon)
{
    // given a robot state, laser observed beacons are returned
    // search of sensor data must be way better than this 
    // binary search, for instance, is a faster option

    vector<vector<double>>::iterator it;
    vector<double> *pBeacon_aux; //global variable?
    int beaconsNumber;

    beaconsNumber = 0;

    if (_laserObservedBeaconsReady)
    {
        for (it = _laserObservedBeacons.begin(); it != _laserObservedBeacons.end(); it++)
        {
            if ((*it).at(0) == (double)systemState(0)) //is this okey? needs a cast?¿
            {
                pBeacon = &((vector<double>)(*it));
                break;
            }
        }
        pBeacon_aux = pBeacon;
        while ((*pBeacon_aux).at(0) == (double)systemState(0))
        {
            pBeacon_aux++;
            beaconsNumber++;
        }
        return beaconsNumber;
    }
    else
    {
        cout << "No Beacons observed" << endl;
        return 0;
    }

}

//-----------------------------------------------------------------------------------------------------------------------------

vector<Eigen::VectorXd> KalmanFilterDataNoSql::GetBeaconsPosition(const int* identifiers) //pointer because it is an array, but we aim not to change it 
{
    
    vector<Eigen::VectorXd> beaconsVector{ _beaconsRealPoseMap.at(identifiers[0]), _beaconsRealPoseMap.at(identifiers[1]), _beaconsRealPoseMap.at(identifiers[2]) };
    return beaconsVector;
    
}