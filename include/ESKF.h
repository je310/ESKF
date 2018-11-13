#ifndef ESKF_H
#define ESKF_H

// Malloc is really bad on embedded platform
#define EIGEN_NO_MALLOC
#include <Core.h>
#include <Geometry.h>
#include <iostream>
#include <lTime.h>

#define SUPPORT_STDIOSTREAM

#define POS_IDX (0)
#define VEL_IDX (POS_IDX + 3)
#define QUAT_IDX (VEL_IDX + 3)
#define AB_IDX (QUAT_IDX + 4)
#define GB_IDX (AB_IDX + 3)
#define STATE_SIZE (GB_IDX + 3)

#define dPOS_IDX (0)
#define dVEL_IDX (dPOS_IDX + 3)
#define dTHETA_IDX (dVEL_IDX + 3)
#define dAB_IDX (dTHETA_IDX + 3)
#define dGB_IDX (dAB_IDX + 3)
#define dSTATE_SIZE (dGB_IDX + 3)

//the main ESKF class
class ESKF {
public:
    ESKF() {};
    // takes as input the  variance of the acceleration and gyro, where _n is the measurement noise, and _w is the pertibations of the system.
    ESKF(Eigen::Vector3f a_gravity,
            const Eigen::Matrix<float, STATE_SIZE, 1>& initialState,
            const Eigen::Matrix<float, dSTATE_SIZE, dSTATE_SIZE>& initalP,
            float var_acc, float var_omega, float var_acc_bias, float var_omega_bias,
            int delayHandling, int bufferL);

    // Concatenates relevant vectors to one large vector.
    static Eigen::Matrix<float, STATE_SIZE, 1> makeState(
            const Eigen::Vector3f& p,
            const Eigen::Vector3f& v,
            const Eigen::Quaternionf& q,
            const Eigen::Vector3f& a_b,
            const Eigen::Vector3f& omega_b);
    // Inserts relevant parts of the block-diagonal of the P matrix
    static Eigen::Matrix<float, dSTATE_SIZE, dSTATE_SIZE> makeP(
        const Eigen::Matrix3f& cov_pos,
        const Eigen::Matrix3f& cov_vel,
        const Eigen::Matrix3f& cov_dtheta,
        const Eigen::Matrix3f& cov_a_b,
        const Eigen::Matrix3f& cov_omega_b);

    // The quaternion convention in the document is "Hamilton" convention.
    // Eigen has a different order of components, so we need conversion
    static Eigen::Quaternionf quatFromHamilton(const Eigen::Vector4f& qHam);
    static Eigen::Vector4f quatToHamilton(const Eigen::Quaternionf& q);
    static Eigen::Matrix3f rotVecToMat(const Eigen::Vector3f& in);
    static Eigen::Quaternionf rotVecToQuat(const Eigen::Vector3f& in);
    static Eigen::Vector3f quatToRotVec(const Eigen::Quaternionf& q);
    static Eigen::Matrix3f getSkew(const Eigen::Vector3f& in);



    // Acessors of nominal state
    inline Eigen::Vector3f getPos() { return nominalState_.block<3, 1>(POS_IDX, 0); }
    inline Eigen::Vector3f getVel() { return nominalState_.block<3, 1>(VEL_IDX, 0); }
    inline Eigen::Vector4f getQuatVector() { return nominalState_.block<4, 1>(QUAT_IDX, 0); }
    inline Eigen::Quaternionf getQuat() { return quatFromHamilton(getQuatVector()); }
    inline Eigen::Vector3f getAccelBias() { return nominalState_.block<3, 1>(AB_IDX, 0); }
    inline Eigen::Vector3f getGyroBias() { return nominalState_.block<3, 1>(GB_IDX, 0); }

    // Called when there is a new measurment from the IMU.
    // dt is the integration time of this sample, nominally the IMU sample period
    void predictIMU(const Eigen::Vector3f& a_m, const Eigen::Vector3f& omega_m, const float dt, lTime stamp);

    // Called when there is a new measurment from an absolute position reference.
    // Note that this has no body offset, i.e. it assumes exact observation of the center of the IMU.
    void measurePos(const Eigen::Vector3f& pos_meas, const Eigen::Matrix3f& pos_covariance, lTime stamp, lTime now);

    // Called when there is a new measurment from an absolute position reference.
    // The measurement is with respect to some location on the body that is not at the IMU center in general.
    // pos_ref_body should specify the reference location in the body frame.
    // For example, this would be the location of the GPS antenna on the body.
    // NOT YET IMPLEMENTED
    // void measurePosWithOffset(Eigen::Vector3f pos_meas, Matrix3f pos_covariance,
    //        Eigen::Vector3f pos_ref_body);

    // Called when there is a new measurment from an absolute orientation reference.
    // The uncertianty is represented as the covariance of a rotation vector in the body frame
    void measureQuat(const Eigen::Quaternionf& q_meas, const Eigen::Matrix3f& theta_covariance, lTime stamp, lTime now);

    Eigen::Matrix3f getDCM();

    enum delayTypes{
        noMethod,           //apply updates  as if they are new.
        applyUpdateToNew,   //Keep buffer of states, calculate what the update would have been, and apply to current state.
        larsonAverageIMU,   //Method as described by Larson et al. Though a buffer of IMU values is kept, and a single update taking the average of these values is used.
        larsonNewestIMU,    //As above, though no buffer kept, use most recent value as representing the average.
        larsonFull          //As above, though the buffer is applied with the correct time steps, fully as described by Larson.
    };
    struct imuMeasurement{
        Eigen::Vector3f acc;
        Eigen::Vector3f gyro;
        lTime time;
    };

private:
    Eigen::Matrix<float, 4, 3> getQ_dtheta(); // eqn 280, page 62
    void update_3D(
        const Eigen::Vector3f& delta_measurement,
        const Eigen::Matrix3f& meas_covariance,
        const Eigen::Matrix<float, 3, dSTATE_SIZE>& H);
    void injectErrorState(const Eigen::Matrix<float, dSTATE_SIZE, 1>& error_state);

    //get best time from history of state
    int getClosestTime(std::vector<std::pair<lTime,Eigen::Matrix<float, STATE_SIZE, 1>>>* ptr, lTime stamp);

    //get best time from history of imu
    int getClosestTime(std::vector<imuMeasurement>*  ptr, lTime stamp);

    // IMU Noise values, used in prediction
    float var_acc_;
    float var_omega_;
    float var_acc_bias_;
    float var_omega_bias_;
    // Acceleration due to gravity in global frame
    Eigen::Vector3f a_gravity_; // [m/s^2]
    // State vector of the filter
    Eigen::Matrix<float, STATE_SIZE, 1> nominalState_;
    // Covariance of the (error) state
    Eigen::Matrix<float, dSTATE_SIZE, dSTATE_SIZE> P_;
    // Jacobian of the state transition: page 59, eqn 269
    // Note that we precompute the static parts in the constructor,
    // and update the dynamic parts in the predict function
    Eigen::Matrix<float, dSTATE_SIZE, dSTATE_SIZE> F_x_;


    int delayHandling_;
    int bufferL_;
    int recentPtr;
    //pointers to structures that are allocated only after choosing a time delay handling method.
    std::vector<std::pair<lTime,Eigen::Matrix<float, STATE_SIZE, 1>>>* stateHistoryPtr_;
    std::vector<imuMeasurement>* imuHistoryPtr_;
    imuMeasurement lastImu_;
    lTime firstMeasTime;
    lTime lastMeasurement;

};

#endif /* ESKF_H */
