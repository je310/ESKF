#include <Core.h>
#include <Geometry.h>
#include <iostream>

#define GRAVITY 	9.812  // London g value.
#define SUPPORT_STDIOSTREAM

#define POS_IDX (0)
#define VEL_IDX (POS_IDX + 3)
#define QUAT_IDX (VEL_IDX + 3)
#define AB_IDX (QUAT_IDX + 4)
#define GB_IDX (AB_IDX + 3)
#define GRAV_IDX (GB_IDX + 3)
#define STATE_SIZE (GRAV_IDX + 3)

#define dPOS_IDX (0)
#define dVEL_IDX (dPOS_IDX + 3)
#define dTHETA_IDX (dVEL_IDX + 3)
#define dAB_IDX (dTHETA_IDX + 3)
#define dGB_IDX (dAB_IDX + 3)
#define dGRAV_IDX (dGB_IDX + 3)
#define dSTATE_SIZE (dGRAV_IDX + 3)

#define I_3 (Eigen::Matrix3f::Identity())

//the main ESKF class
class ESKF {
public:
    // takes as input the  variance of the acceleration and gyro, where _n is the measurement noise, and _w is the pertibations of the system.
    ESKF(Eigen::Matrix<float, STATE_SIZE, 1> initialState,
            float sig2_a_n_, float sig2_omega_n_, float sig2_a_w_, float sig2_omega_w_);
    // Concatenates relevant vectors to one large vector.
    static Eigen::Matrix<float, STATE_SIZE, 1> makeState(
            Eigen::Vector3f p,
            Eigen::Vector3f v,
            Eigen::Quaternionf q,
            Eigen::Vector3f a_b,
            Eigen::Vector3f omega_b,
            Eigen::Vector3f g);

    // Called when there is a new measurment from the IMU.
    void predictIMU(Eigen::Vector3f a, Eigen::Vector3f omega, float delta_t);

    // Called when there is a new measurment from an absolute position reference (such as Motion Capture, GPS, map matching etc )
    void observeErrorState(Eigen::Vector3f pos, Eigen::Quaternionf rot);

    //returns the combination of the nominal state and the error state.
    Eigen::Matrix<float, STATE_SIZE, 1> getTrueState();

    inline Eigen::Vector3f getPos() { return nominalState.block<3, 1>(POS_IDX, 0); }
    inline Eigen::Vector3f getVel() { return nominalState.block<3, 1>(VEL_IDX, 0); }
    inline Eigen::Vector4f getQuatVector() { return nominalState.block<4, 1>(QUAT_IDX, 0); }
    inline Eigen::Quaternionf getQuat() { 
        return Eigen::Quaternionf(getQuatVector());
    }
    inline Eigen::Vector3f getAccelBias() { return nominalState.block<3, 1>(AB_IDX, 0); }
    inline Eigen::Vector3f getGyroBias() { return nominalState.block<3, 1>(GB_IDX, 0); }

    //private:
    Eigen::Matrix3f getRotationMatrixFromState(Eigen::Matrix<float, STATE_SIZE, 1> state);
    Eigen::Matrix3f getSkew(Eigen::Vector3f in);
    Eigen::Matrix3f rotVecToMat(Eigen::Vector3f in);
    Eigen::Matrix<float, STATE_SIZE, 1> measurementFunc(Eigen::Matrix<float, STATE_SIZE, 1> in);
    void composeTrueState();
    void injectObservedError();
    void resetError();

    // states
    Eigen::Matrix<float, STATE_SIZE, 1> trueState;
    Eigen::Matrix<float, dSTATE_SIZE, 1> errorState;
    Eigen::Matrix<float, STATE_SIZE, 1> nominalState;


    //covarience matrices as defined on page 59
    Eigen::Matrix3f V_i;
    Eigen::Matrix3f THETA_i;
    Eigen::Matrix3f A_i;
    Eigen::Matrix3f OMEGA_i;
    float sig2_a_n;
    float sig2_a_w;
    float sig2_omega_n;
    float sig2_omega_w;


    //jacobians of f() as defined on page 59// not sure if should be STATE_SIZE in size, the quaternion seems to be a rotation matrix here.
    // Matrix<float, dSTATE_SIZE,dSTATE_SIZE> F_x;
    Eigen::Matrix<float, dSTATE_SIZE, 12> F_i;
    //covariances matrix of the perturbation impulses.
    Eigen::Matrix <float, 12, 12> Q_i;

    // the P matrix
    Eigen::Matrix<float, dSTATE_SIZE, dSTATE_SIZE> P;

    // the K matrix
    Eigen::Matrix<float, dSTATE_SIZE, STATE_SIZE> K;


    //Jacobian of the true state with respect to the error state. Page 61.
    Eigen::Matrix<float, STATE_SIZE, dSTATE_SIZE> H;
    Eigen::Matrix<float, STATE_SIZE, STATE_SIZE> H_x;
    Eigen::Matrix<float, STATE_SIZE, dSTATE_SIZE> X_dx;

    Eigen::Matrix<float, 4, 3> Q_dTheta;

    // Jacobian matrix defined on page 63, can have a simple implementation as an Identity, or one that has a correction term
    Eigen::Matrix<float, dSTATE_SIZE, dSTATE_SIZE> G;

};
