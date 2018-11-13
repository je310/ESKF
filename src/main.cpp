#include <ESKF.h>
#include <iostream>
#include <chrono>
#include <parseDataFiles.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <lTime.h>
#define SQ(x) (x*x)
#define GRAVITY 	9.812  // London g value.
#define I_3 (Eigen::Matrix3f::Identity())

using namespace Eigen;
using namespace std;

void postTF(ESKF eskf,tf::TransformBroadcaster tb,std::string name);
double difference(ESKF eskfRef,ESKF eskfTest);

int main(int argc, char** argv) {

    float sigma_accel = 0.124; // [m/s^2]  (value derived from Noise Spectral Density in datasheet)
    float sigma_gyro = 0.00276; // [rad/s] (value derived from Noise Spectral Density in datasheet)
    float sigma_accel_drift = 0.001f*sigma_accel; // [m/s^2 sqrt(s)] (Educated guess, real value to be measured)
    float sigma_gyro_drift = 0.001f*sigma_gyro; // [rad/s sqrt(s)] (Educated guess, real value to be measured)

    float sigma_init_pos = 1.0; // [m]
    float sigma_init_vel = 0.1; // [m/s]
    float sigma_init_dtheta = 1.0; // [rad]
    float sigma_init_accel_bias = 1000*sigma_accel_drift; // [m/s^2]
    float sigma_init_gyro_bias = 1000*sigma_gyro_drift; // [rad/s]

    float sigma_mocap_pos = 0.0003; // [m]
    float sigma_mocap_rot = 0.003; // [rad]
    //initialise many ESKF's for testing:
    // eskfSpoof - Will be fed data as if there is no lag, this is the comparison case
    // eskfAsArrive - No time correction method used, just fed lagged data.
    // eskfAverageIMU - Uses the average of the IMU measurements as representative since the delayed measurment should have arrived
    // eskfNewIMU - Uses newest IMU measurement as representative since the delayed measurment should have arrived
    // eskfFullLarson - Uses the method as described by Larson et al (same as above, but uses all IMU measurments rather than simplificationis)
    // eskfUpdateToNew - Keeps the state in a buffer, calculates the update that would have been made if there was no lag, then applies directly to current state.
    ESKF eskfSpoof(
            Vector3f(0, 0, -GRAVITY), // Acceleration due to gravity in global frame
            ESKF::makeState(
                Vector3f(0, 0, 0), // init pos
                Vector3f(0, 0, 0), // init vel
                Quaternionf(AngleAxisf(0.5f, Vector3f(1, 0, 0))), // init quaternion
                Vector3f(0, 0, 0), // init accel bias
                Vector3f(0, 0, 0) // init gyro bias
            ),
            ESKF::makeP(
                SQ(sigma_init_pos) * I_3,
                SQ(sigma_init_vel) * I_3,
                SQ(sigma_init_dtheta) * I_3,
                SQ(sigma_init_accel_bias) * I_3,
                SQ(sigma_init_gyro_bias) * I_3
            ),
            SQ(sigma_accel),
            SQ(sigma_gyro),
            SQ(sigma_accel_drift),
            SQ(sigma_gyro_drift),
            ESKF::delayTypes::noMethod,100);
    ESKF eskfAsArrive(
            Vector3f(0, 0, -GRAVITY), // Acceleration due to gravity in global frame
            ESKF::makeState(
                Vector3f(0, 0, 0), // init pos
                Vector3f(0, 0, 0), // init vel
                Quaternionf(AngleAxisf(0.5f, Vector3f(1, 0, 0))), // init quaternion
                Vector3f(0, 0, 0), // init accel bias
                Vector3f(0, 0, 0) // init gyro bias
            ),
            ESKF::makeP(
                SQ(sigma_init_pos) * I_3,
                SQ(sigma_init_vel) * I_3,
                SQ(sigma_init_dtheta) * I_3,
                SQ(sigma_init_accel_bias) * I_3,
                SQ(sigma_init_gyro_bias) * I_3
            ),
            SQ(sigma_accel),
            SQ(sigma_gyro),
            SQ(sigma_accel_drift),
            SQ(sigma_gyro_drift),
            ESKF::delayTypes::noMethod,100);
    ESKF eskfAverageIMU(
            Vector3f(0, 0, -GRAVITY), // Acceleration due to gravity in global frame
            ESKF::makeState(
                Vector3f(0, 0, 0), // init pos
                Vector3f(0, 0, 0), // init vel
                Quaternionf(AngleAxisf(0.5f, Vector3f(1, 0, 0))), // init quaternion
                Vector3f(0, 0, 0), // init accel bias
                Vector3f(0, 0, 0) // init gyro bias
            ),
            ESKF::makeP(
                SQ(sigma_init_pos) * I_3,
                SQ(sigma_init_vel) * I_3,
                SQ(sigma_init_dtheta) * I_3,
                SQ(sigma_init_accel_bias) * I_3,
                SQ(sigma_init_gyro_bias) * I_3
            ),
            SQ(sigma_accel),
            SQ(sigma_gyro),
            SQ(sigma_accel_drift),
            SQ(sigma_gyro_drift),
            ESKF::delayTypes::larsonAverageIMU,100);
    ESKF eskfNewIMU(
            Vector3f(0, 0, -GRAVITY), // Acceleration due to gravity in global frame
            ESKF::makeState(
                Vector3f(0, 0, 0), // init pos
                Vector3f(0, 0, 0), // init vel
                Quaternionf(AngleAxisf(0.5f, Vector3f(1, 0, 0))), // init quaternion
                Vector3f(0, 0, 0), // init accel bias
                Vector3f(0, 0, 0) // init gyro bias
            ),
            ESKF::makeP(
                SQ(sigma_init_pos) * I_3,
                SQ(sigma_init_vel) * I_3,
                SQ(sigma_init_dtheta) * I_3,
                SQ(sigma_init_accel_bias) * I_3,
                SQ(sigma_init_gyro_bias) * I_3
            ),
            SQ(sigma_accel),
            SQ(sigma_gyro),
            SQ(sigma_accel_drift),
            SQ(sigma_gyro_drift),
            ESKF::delayTypes::larsonNewestIMU,100);
    ESKF eskfFullLarson(
            Vector3f(0, 0, -GRAVITY), // Acceleration due to gravity in global frame
            ESKF::makeState(
                Vector3f(0, 0, 0), // init pos
                Vector3f(0, 0, 0), // init vel
                Quaternionf(AngleAxisf(0.5f, Vector3f(1, 0, 0))), // init quaternion
                Vector3f(0, 0, 0), // init accel bias
                Vector3f(0, 0, 0) // init gyro bias
            ),
            ESKF::makeP(
                SQ(sigma_init_pos) * I_3,
                SQ(sigma_init_vel) * I_3,
                SQ(sigma_init_dtheta) * I_3,
                SQ(sigma_init_accel_bias) * I_3,
                SQ(sigma_init_gyro_bias) * I_3
            ),
            SQ(sigma_accel),
            SQ(sigma_gyro),
            SQ(sigma_accel_drift),
            SQ(sigma_gyro_drift),
            ESKF::delayTypes::larsonFull,100);
    ESKF eskfUpdateToNew(
            Vector3f(0, 0, -GRAVITY), // Acceleration due to gravity in global frame
            ESKF::makeState(
                Vector3f(0, 0, 0), // init pos
                Vector3f(0, 0, 0), // init vel
                Quaternionf(AngleAxisf(0.5f, Vector3f(1, 0, 0))), // init quaternion
                Vector3f(0, 0, 0), // init accel bias
                Vector3f(0, 0, 0) // init gyro bias
            ),
            ESKF::makeP(
                SQ(sigma_init_pos) * I_3,
                SQ(sigma_init_vel) * I_3,
                SQ(sigma_init_dtheta) * I_3,
                SQ(sigma_init_accel_bias) * I_3,
                SQ(sigma_init_gyro_bias) * I_3
            ),
            SQ(sigma_accel),
            SQ(sigma_gyro),
            SQ(sigma_accel_drift),
            SQ(sigma_gyro_drift),
            ESKF::delayTypes::applyUpdateToNew,100);

    // Start section that manages collecting data from files and feeding them to the ESKFs.
    // eskfSpoof is a special case that uses separated files for IMU and Mocap, it finds the next one based on timestamp (of data generration)
    // The others use the data as they arrived during data collection (randomised delay of the mocap ~ 18ms delay)
    // All comparisons are made on the arrival of IMU data, as this is the same for all of them including the spoof case.

    ros::init(argc, argv, "TestESKF");

    ros::NodeHandle node;

    ros::Publisher posePub = node.advertise<geometry_msgs::PoseWithCovariance>("predictiedLocation",1);

    tf::TransformBroadcaster tb;

    DataFiles filesObj("../timeSeriesNov/shakeWave");

    int flag = 0;
    int spoofIMUcount = 0 ;
    int testIMUCount = 0;
    double asArriveErrorAcc = 0;
    double asArriveError  = 0;
    double averageIMUErrorAcc = 0;
    double averageIMUError  = 0;
    double newIMUErrorAcc = 0;
    double newIMUError  = 0;
    double fullLarsonErrorAcc = 0;
    double fullLarsonError  = 0;
    double upToNewError = 0;
    double upToNewErrorAcc =0;

    while(!flag){
        imuData imu;
        mocapData mocap;
        int type;

        if(spoofIMUcount < testIMUCount){
        //do spoof.
        flag = filesObj.getNextTimeCorrected(filesObj.readerMocap,filesObj.readerIMU,mocap,imu,type); // emulates lag free data
        if(type == isImuData){
            static ros::Time oldTime;
            ros::Duration diff = imu.stamp - oldTime;
            oldTime = imu.stamp;
            lTime stamp(imu.stamp.sec,imu.stamp.nsec);
            if(diff.toSec() > 1999) diff.fromSec(0.001);
            spoofIMUcount ++;
            eskfSpoof.predictIMU(imu.accel, imu.gyro, diff.toSec(),stamp);
        }

        if(type == isMocapData){
            lTime stamp(mocap.stamp.sec,mocap.stamp.nsec);
            lTime now(mocap.receivedTime.sec,mocap.receivedTime.nsec);
            eskfSpoof.measurePos(mocap.pos, SQ(sigma_mocap_pos)*I_3,stamp,now);
            eskfSpoof.measureQuat(mocap.quat, SQ(sigma_mocap_rot)*I_3,stamp,now);


            tf::StampedTransform meas;
            meas.setOrigin(tf::Vector3(mocap.pos[0],mocap.pos[1],mocap.pos[2]));
            meas.setRotation(tf::Quaternion(mocap.quat.x(),mocap.quat.y(),mocap.quat.z(),mocap.quat.w()));
            meas.stamp_ = ros::Time::now();
            meas.frame_id_ = "mocha_world";
            meas.child_frame_id_ = "meas";
            tb.sendTransform(meas);
            cout << "mocap" << endl;
        }
        }
        else{
        //do candidates.
        flag = filesObj.getNextNotCorrected(filesObj.readerMixed,mocap,imu,type);
        if(type == isImuData){
            testIMUCount ++;
            static ros::Time oldTime;
            ros::Duration diff = imu.stamp - oldTime;
            oldTime = imu.stamp;
            lTime stamp(imu.stamp.sec,imu.stamp.nsec);
            if(diff.toSec() > 1999) diff.fromSec(0.001);
            eskfAsArrive.predictIMU(imu.accel, imu.gyro, diff.toSec(),stamp);
            eskfAverageIMU.predictIMU(imu.accel, imu.gyro, diff.toSec(),stamp);
            eskfNewIMU.predictIMU(imu.accel, imu.gyro, diff.toSec(),stamp);
            eskfFullLarson.predictIMU(imu.accel, imu.gyro, diff.toSec(),stamp);
            eskfUpdateToNew.predictIMU(imu.accel, imu.gyro, diff.toSec(),stamp);
        }

        if(type == isMocapData){

            lTime stamp(mocap.stamp.sec,mocap.stamp.nsec);

            lTime now(mocap.receivedTime.sec,mocap.receivedTime.nsec);

            eskfAsArrive.measurePos(mocap.pos, SQ(sigma_mocap_pos)*I_3,stamp,now);
            eskfAsArrive.measureQuat(mocap.quat, SQ(sigma_mocap_rot)*I_3,stamp,now);

            eskfAverageIMU.measurePos(mocap.pos, SQ(sigma_mocap_pos)*I_3,stamp,now);
            eskfAverageIMU.measureQuat(mocap.quat, SQ(sigma_mocap_rot)*I_3,stamp,now);

            eskfNewIMU.measurePos(mocap.pos, SQ(sigma_mocap_pos)*I_3,stamp,now);
            eskfNewIMU.measureQuat(mocap.quat, SQ(sigma_mocap_rot)*I_3,stamp,now);

            eskfFullLarson.measurePos(mocap.pos, SQ(sigma_mocap_pos)*I_3,stamp,now);
            eskfFullLarson.measureQuat(mocap.quat, SQ(sigma_mocap_rot)*I_3,stamp,now);

            eskfUpdateToNew.measurePos(mocap.pos, SQ(sigma_mocap_pos)*I_3,stamp,now);
            eskfUpdateToNew.measureQuat(mocap.quat, SQ(sigma_mocap_rot)*I_3,stamp,now);
        }
        }

        if(testIMUCount == spoofIMUcount){
            postTF(eskfSpoof,tb,"spoof");
            postTF(eskfAsArrive,tb,"asArrive");
            postTF(eskfAverageIMU,tb,"amIMU");
            postTF(eskfNewIMU,tb,"newIMU");
            postTF(eskfFullLarson,tb,"Larson");
            postTF(eskfUpdateToNew,tb,"UpToNew");
            asArriveErrorAcc += difference(eskfSpoof,eskfAsArrive);
            averageIMUErrorAcc += difference(eskfSpoof,eskfAverageIMU);
            newIMUErrorAcc += difference(eskfSpoof,eskfNewIMU);
            fullLarsonErrorAcc += difference(eskfSpoof,eskfFullLarson);
            upToNewErrorAcc += difference(eskfSpoof,eskfUpdateToNew);
        }
    }
    asArriveError = asArriveErrorAcc / testIMUCount;
    averageIMUError = averageIMUErrorAcc / testIMUCount;
    newIMUError = newIMUErrorAcc / testIMUCount;
    fullLarsonError = fullLarsonErrorAcc / testIMUCount;
    upToNewError = upToNewErrorAcc / testIMUCount;
    cout << "asArrive pos error average = " << asArriveError << endl;
    cout << "averageIMUError pos error average = " << averageIMUError << endl;
    cout << "newIMUError pos error average = " << newIMUError << endl;
    cout << "fullLarsonError pos error average = " << fullLarsonError << endl;
    cout << "upToNewError pos error average = " << upToNewError << endl;


}

double difference(ESKF eskfRef,ESKF eskfTest){
    Vector3f posRef = eskfRef.getPos();
    Vector3f posTest = eskfTest.getPos();
    Vector3f out = posTest - posRef;
    return out.norm();
}

void postTF(ESKF eskf,tf::TransformBroadcaster tb,std::string name){
    tf::StampedTransform pred;
    Vector3f pos = eskf.getPos();
    Quaternionf quat = eskf.getQuat();
    pred.setOrigin(tf::Vector3(pos[0],pos[1],pos[2]));
    pred.setRotation(tf::Quaternion(quat.x(),quat.y(),quat.z(),quat.w()));
    pred.stamp_ = ros::Time::now();
    pred.frame_id_ = "mocha_world";
    pred.child_frame_id_ = name;
    tb.sendTransform(pred);
}
