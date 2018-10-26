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
    ESKF(float delta_t, Eigen::Matrix<float, STATE_SIZE, 1> initialState,
            float sig2_a_n, float sig2_omega_n, float sig2_a_w, float sig2_omega_w);
    // Concatenates relevant vectors to one large vector.
    static Eigen::Matrix<float, STATE_SIZE, 1> makeState(
            Eigen::Vector3f p,
            Eigen::Vector3f v,
            Eigen::Quaternionf q,
            Eigen::Vector3f a_b,
            Eigen::Vector3f omega_b,
            Eigen::Vector3f g);

    // The quaternion convention in the document is "Hamilton" convention.
    // Eigen has a different order of components, so we need conversion
    static Eigen::Quaternionf quatFromHamilton(Eigen::Vector4f qHam);
    static Eigen::Vector4f quatToHamilton(Eigen::Quaternionf q);
    static Eigen::Matrix3f rotVecToMat(Eigen::Vector3f in);
    static Eigen::Quaternionf rotVecToQuat(Eigen::Vector3f in);
    static Eigen::Matrix3f getSkew(Eigen::Vector3f in);

    // Acessors of nominal state
    inline Eigen::Vector3f getPos() { return nominalState.block<3, 1>(POS_IDX, 0); }
    inline Eigen::Vector3f getVel() { return nominalState.block<3, 1>(VEL_IDX, 0); }
    inline Eigen::Vector4f getQuatVector() { return nominalState.block<4, 1>(QUAT_IDX, 0); }
    inline Eigen::Quaternionf getQuat() { return quatFromHamilton(getQuatVector()); }
    inline Eigen::Vector3f getAccelBias() { return nominalState.block<3, 1>(AB_IDX, 0); }
    inline Eigen::Vector3f getGyroBias() { return nominalState.block<3, 1>(GB_IDX, 0); }
    inline Eigen::Vector3f getGravity() { return nominalState.block<3, 1>(GRAV_IDX, 0); }

    // Called when there is a new measurment from the IMU.
    void predictIMU(Eigen::Vector3f a_m, Eigen::Vector3f omega_m);

    // Called when there is a new measurment from an absolute position reference.
    // Note that this has no body offset, i.e. it assumes exact observation of the center of the IMU.
    void measurePos(Eigen::Vector3f pos_meas);

    // Called when there is a new measurment from an absolute position reference.
    // The measurement is with respect to some location on the body that is not at the IMU center in general.
    // pos_ref_body should specify the reference location in the body frame.
    // For example, this would be the location of the GPS antenna on the body.
    // NOT YET IMPLEMENTED
    // void measurePosWithOffset(Eigen::Vector3f pos_meas, Eigen::Vector3f pos_ref_body);

    // Called when there is a new measurment from an absolute orientation reference.
    void measureQuat(Eigen::Quaternionf q_meas);

    // Called when there is a new measurment from an absolute position reference (such as Motion Capture, GPS, map matching etc )
    void observeErrorState(Eigen::Vector3f pos, Eigen::Quaternionf rot);

    //returns the combination of the nominal state and the error state.
    Eigen::Matrix<float, STATE_SIZE, 1> getTrueState();
    Eigen::Matrix3f getDCM();

private:
    Eigen::Matrix<float, 4, 3> getQ_dtheta(); // eqn 280, page 62
    Eigen::Matrix<float, STATE_SIZE, 1> measurementFunc(Eigen::Matrix<float, STATE_SIZE, 1> in);
    void composeTrueState();
    void injectObservedError();
    void resetError();

    // We assume a fixed dt, so we can precompute many matrices
    const float dt_;
    // Process noise, stored as a vector of the diagonal
    Eigen::Matrix<float, 4*3, 1> Q_diag_;

    // states
    Eigen::Matrix<float, STATE_SIZE, 1> trueState;
    Eigen::Matrix<float, dSTATE_SIZE, 1> errorState;
    Eigen::Matrix<float, STATE_SIZE, 1> nominalState;



    // Jacobian of the state transition: page 59, eqn 269
    // Note that we precompute the static parts in the constructor,
    // and only update the dynamic parts in the predict function
    Eigen::Matrix<float, dSTATE_SIZE,dSTATE_SIZE> F_x_;

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
