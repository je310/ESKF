#include <Core.h>
#include <Geometry.h>
#include <iostream>


#define GRAVITY 	9.812  // London g value.
#define SUPPORT_STDIOSTREAM

using namespace Eigen;
using namespace std;

//the main ESKF class
class ESKF{
public:
    // takes as input the  variance of the acceleration and gyro, where _n is the measurement noise, and _w is the pertibations of the system.
    ESKF(Matrix<float, 19,1> initialState, float sig2_a_n_, float sig2_omega_n_,float sig2_a_w_, float sig2_omega_w_);
    ESKF();
    // concatonates  relevant vectors to one large vector.
    Matrix<float, 19,1> makeState(Vector3f p,Vector3f v, Quaternionf q, Vector3f a_b, Vector3f omega_b,Vector3f g );

    // Called when there is a new measurment from the IMU.
    void updateStateIMU(Vector3f a, Vector3f omega, float delta_t);

    // Called when there is a new measurment from an absolute position reference (such as Motion Capture, GPS, map matching etc )
    void observeErrorState(Vector3f pos, Quaternionf rot);
//private:

    Matrix<float, 3,3> getRotationMatrixFromState(Matrix<float, 19,1> state);
    Matrix<float,3,3> getSkew(Vector3f in);
    Matrix<float,3,3> AngAxToMat(Vector3f in);
    Matrix<float,19,1> measurementFunc(Matrix<float,19,1> in);
    void composeTrueState();
    void injectObservedError();
    void resetError();

    void predictionUpdate(Vector3f a, Vector3f omega, float delta_t);

    // states
    Matrix<float, 19,1> trueState;
    Matrix<float, 18,1> errorState;
    Matrix<float, 19,1> nominalState;


    //covarience matrices as defined on page 59
    Matrix<float, 3,3> V_i;
    Matrix<float, 3,3> PHI_i;
    Matrix<float, 3,3> A_i;
    Matrix<float, 3,3> OMEGA_i;
    float sig2_a_n;
    float sig2_a_w;
    float sig2_omega_n;
    float sig2_omega_w;


    //jacobians of f() as defined on page 59// not sure if should be 19 in size, the quaternion seems to be a rotation matrix here.
    Matrix<float, 18,18> F_x;
    Matrix<float, 18,12> F_i;
    //covariances matrix of the perturbation impulses.
    Matrix <float, 12,12> Q_i;

    // the P matrix
    Matrix<float, 18,18> P;

    // the K matrix
    Matrix<float, 18,19> K;


    //Jacobian of the true state with respect to the error state. Page 61.
    Matrix<float, 19,18> H;
    Matrix<float, 19, 19> H_x;
    Matrix<float, 19, 18> X_dx;

    Matrix<float, 4, 3> Q_dTheta;

    // Jacobian matrix defined on page 63, can have a simple implementation as an Identity, or one that has a correction term
    Matrix<float, 18,18> G;

};
