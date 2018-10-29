#include <ESKF.h>
#include <iostream>
#include <chrono>
#include <parseDataFiles.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <tf/transform_broadcaster.h>
#include <math.h>

#define SQ(x) (x*x)
#define GRAVITY 	9.812  // London g value.

using namespace Eigen;
using namespace std;

int main(int argc, char** argv) {

    float sigma_accel = 0.124; // [m/s^2]  (value derived from Noise Spectral Density in datasheet)
    float sigma_gyro = 0.00276; // [rad/s] (value derived from Noise Spectral Density in datasheet)
    float sigma_accel_drift = 0.0025; // [m/s^2 sqrt(s)] (Educated guess, real value to be measured)
    float sigma_gyro_drift = 5e-5; // [rad/s sqrt(s)] (Educated guess, real value to be measured)

    float sigma_init_pos = 1.0; // [m]
    float sigma_init_vel = 0.1; // [m/s]
    float sigma_init_dtheta = 1.0; // [rad]
    float sigma_init_accel_bias = 100*sigma_accel_drift; // [m/s^2]
    float sigma_init_gyro_bias = 100*sigma_gyro_drift; // [rad/s]

    float sigma_mocap_pos = 0.001; // [m]
    float sigma_mocap_rot = 0.001; // [rad]

    ESKF eskf(
            0.001f, // delta_t
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
            SQ(sigma_gyro_drift));

    //dirty debugging area for josh START:

    ros::init(argc, argv, "TestESKF");

    ros::NodeHandle node;

    ros::Publisher posePub = node.advertise<geometry_msgs::PoseWithCovariance>("predictiedLocation",1);

    tf::TransformBroadcaster tb;

    DataFiles filesObj("/home/josh/newDownloads/DeltaRobotFirmware/build/timeSeries/gentleWave");

    int flag = 0;

    while(!flag){
        imuData imu;
        mocapData mocap;
        int type;
        //filesObj.getNext(filesObj.readerMixed,mocap,imu,type);
        flag = filesObj.getNextTimeCorrected(filesObj.readerMocap,filesObj.readerIMU,mocap,imu,type);
        if(type == isImuData){
            eskf.predictIMU(GRAVITY * imu.accel, M_PI*imu.gyro/180);

            tf::StampedTransform pred;
            Vector3f pos = eskf.getPos();
            Quaternionf quat = eskf.getQuat();
            pred.setOrigin(tf::Vector3(pos[0],pos[1],pos[2]));
            pred.setRotation(tf::Quaternion(quat.x(),quat.y(),quat.z(),quat.w()));
            pred.stamp_ = ros::Time::now();
            pred.frame_id_ = "mocha_world";
            pred.child_frame_id_ = "pred";
            tb.sendTransform(pred);
        }
        if(type == isMocapData){
            eskf.measurePos(mocap.pos, SQ(sigma_mocap_pos)*I_3);
            eskf.measureQuat(mocap.quat, SQ(sigma_mocap_rot)*I_3);

            tf::StampedTransform meas;
            meas.setOrigin(tf::Vector3(mocap.pos[0],mocap.pos[1],mocap.pos[2]));
            meas.setRotation(tf::Quaternion(mocap.quat.x(),mocap.quat.y(),mocap.quat.z(),mocap.quat.w()));
            meas.stamp_ = ros::Time::now();
            meas.frame_id_ = "mocha_world";
            meas.child_frame_id_ = "meas";
            tb.sendTransform(meas);
        }
    }


    //dirty debugging area for josh END:


    //timing some key things just for some insight. Accell is 1000hz and mocap is 100hz.
    float sim_data_duration = 1000.0f; //seconds
    auto start = std::chrono::system_clock::now();
    for (int ms = 0; ms < 1000*sim_data_duration; ms++) {

        // Simulated static true pos/orientation
        Vector3f pos_true = Vector3f(0, 0, 0);
        Quaternionf q_true = Quaternionf(AngleAxisf(0.0f, Vector3f(1, 0, 0)));
        // Quaternionf q_true = Quaternionf(AngleAxisf(0.001f*ms, Vector3f(1, 0, 0)));
        Matrix3f R_true = q_true.toRotationMatrix();

        // Fake accel/gyro
        Vector3f acc_true = R_true.transpose() * Vector3f(0, 0, GRAVITY);
        Vector3f acc = acc_true + sigma_accel * Vector3f::Random();
        Vector3f gyro = sigma_gyro * Vector3f::Random();
        // Input our accel/gyro data
        eskf.predictIMU(acc, gyro);


        //optional ros viz JOSH MESSY
        tf::StampedTransform pred;
        Vector3f pos = eskf.getPos();
        Quaternionf quat = eskf.getQuat();
        pred.setOrigin(tf::Vector3(pos[0],pos[1],pos[2]));
        pred.setRotation(tf::Quaternion(quat.x(),quat.y(),quat.z(),quat.w()));
        pred.stamp_ = ros::Time::now();
        pred.frame_id_ = "mocha_world";
        pred.child_frame_id_ = "pred";
        tb.sendTransform(pred);
        //end optional ros viz JOSH MESSY

        // 10 accel/gyro measurements per motion capture input.
        if (ms % 10 == 0) {
            // Fake mocap data
            Vector3f pos_meas = pos_true + sigma_mocap_pos * Vector3f::Random();
            Quaternionf q_meas = q_true;
            // input our motion capture data
            eskf.measurePos(pos_meas, SQ(sigma_mocap_pos)*I_3);
            eskf.measureQuat(q_meas, SQ(sigma_mocap_rot)*I_3);
        }
    }
    auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << elapsed.count() << std::endl;

    std::cout << "Pos: " << std::endl << eskf.getPos() << std::endl;
    std::cout << "Vel: " << std::endl << eskf.getVel() << std::endl;
    std::cout << "QuatVector: " << std::endl << eskf.getQuatVector() << std::endl;
    std::cout << "AccelBias: " << std::endl << eskf.getAccelBias() << std::endl;
    std::cout << "GyroBias: " << std::endl << eskf.getGyroBias() << std::endl;

}
