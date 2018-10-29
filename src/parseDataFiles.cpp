
#include<parseDataFiles.h>


void DataFiles::getNext(ifstream &file, mocapData &mocap, imuData &imu, int &type){

    string str;
    std::getline(readerMixed,str,',');
    if(!str.compare("IMU")){
        type = isImuData;

        std::getline(file,str,',');
        imu.accel[0] = atof(str.c_str());

        std::getline(file,str,',');
        imu.accel[1] = atof(str.c_str());

        std::getline(file,str,',');
        imu.accel[2] = atof(str.c_str());

        std::getline(file,str,',');
        imu.gyro[0] = atof(str.c_str());

        std::getline(file,str,',');
        imu.gyro[1] = atof(str.c_str());

        std::getline(file,str,',');
        imu.gyro[2] = atof(str.c_str());

        std::getline(file,str,',');
        imu.stamp.sec = atoi(str.c_str());

        std::getline(file,str,'\n');
        imu.stamp.nsec = atoi(str.c_str());

    }
    else{
        type = isMocapData;


        std::getline(file,str,',');
        mocap.pos[0] = atof(str.c_str());

        std::getline(file,str,',');
        mocap.pos[1] = atof(str.c_str());

        std::getline(file,str,',');
        mocap.pos[2] = atof(str.c_str());

        std::getline(file,str,',');
        float qx = atof(str.c_str());

        std::getline(file,str,',');
        float qy = atof(str.c_str());

        std::getline(file,str,',');
        float qz = atof(str.c_str());

        std::getline(file,str,',');
        float qw = atof(str.c_str());

        mocap.quat = Quaternionf(qw,qx,qy,qz);

        std::getline(file,str,',');
        mocap.stamp.sec = atoi(str.c_str());

        std::getline(file,str,',');
        mocap.stamp.nsec = atoi(str.c_str());

        std::getline(file,str,',');
        mocap.receivedTime.sec = atoi(str.c_str());

        std::getline(file,str,'\n');
        mocap.receivedTime.nsec = atoi(str.c_str());
    }

    return;

}



DataFiles::DataFiles(std::string path)

{

    //init files and waste the header line.
    stringstream ssMixed;
    ssMixed << path << "/mixedTimeSeries.txt";
    readerMixed.open(ssMixed.str());
    char var[256];
    readerMixed.getline(var,sizeof(var),'\n');



    stringstream ssMocap;
    ssMocap << path << "/mocapTimeSeries.txt";
    readerMocap.open(ssMocap.str());
    readerMocap.getline(var,sizeof(var),'\n');

    stringstream ssIMU;
    ssIMU << path << "/accelGyroTimeSeries.txt";
    readerIMU.open(ssIMU.str());
    readerIMU.getline(var,sizeof(var),'\n');
}


