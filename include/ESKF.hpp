#include <Matrix.hpp>

#define GRAVITY 	9.812  // London g value.

using namespace matrix;
//the main ESKF class
class ESKF{
public:
    // takes as input the  variance of the acceleration and gyro, where _n is the measurement noise, and _w is the pertibations of the system.
    ESKF(Vector<float, 19> initialState, Vector3<float> sig2_a_n, Vector3<float> sig2_omega_n,Vector3<float> sig2_a_w, Vector3<float> sig2_omega_w);

    // concatonates  relevant vectors to one large vector.
    Vector<float, 19> makeState(Vector3<float> p,Vector3<float> v, Quaternion<float> q, Vector3<float> a_b, Vector3<float> omega_b,Vector3<float> g );

    // Called when there is a new measurment from the IMU.
    void updateStateIMU(Vector3<float> a, Vector3<float> omega, float delta_t);

    // Called when there is a new measurment from an absolute position reference (such as Motion Capture, GPS, map matching etc )
    void updateStateAbsPos(Vector3<float> pos, Quaternion<float> rot);
private:

    // states
    Vector<float, 19> acutalState;
    Vector<float, 19> errorState;


    //covarience matrices as defined on page 59
    SquareMatrix<float, 3> V_i;
    SquareMatrix<float, 3> PHI_i;
    SquareMatrix<float, 3> A_i;
    SquareMatrix<float, 3> OMEGA_i;

    //jacobians of f() as defined on page 59// not sure if should be 19 in size, the quaternion seems to be a rotation matrix here.
    SquareMatrix<float, 18> F_x;
    Matrix<float, 18,12> F_i;
    //covariances matrix of the perturbation impulses.
    SquareMatrix <float, 12> Q_i;

    // the P matrix
    SquareMatrix<float, 18> P;

    // the K matrix
    SquareMatrix<float, 18> K;


    //Jacobian of the true state with respect to the error state. Page 61.
    SquareMatrix<float, 18> H;
    Matrix<float, 18, 19> H_x;
    Matrix<float, 19, 18> X_dx;

    Matrix<float, 4, 3> Q_dTheta;

    // Jacobian matrix defined on page 63, can have a simple implementation as an Identity, or one that has a correction term
    SquareMatrix<float, 18> G;

};
