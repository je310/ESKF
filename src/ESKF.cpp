#include <ESKF.h>

using namespace Eigen;
using namespace std;

ESKF::ESKF(Matrix<float, STATE_SIZE, 1> initialState, 
        float sig2_a_n_, float sig2_omega_n_, float sig2_a_w_, float sig2_omega_w_) {
    trueState = initialState;
    sig2_a_n = sig2_a_n_;
    sig2_omega_n = sig2_omega_n_;
    sig2_a_w = sig2_a_w_;
    sig2_omega_w = sig2_omega_w_;

    //Build F_i,
    F_i.setZero();
    F_i.block<12, 12>(3, 0).setIdentity();
}

Matrix<float, STATE_SIZE, 1> ESKF::makeState(Vector3f p, Vector3f v, Quaternionf q, 
            Vector3f a_b, Vector3f omega_b, Vector3f g) {
    Matrix<float, STATE_SIZE, 1> out;
    out << p, v, q.coeffs(), a_b, omega_b, g;
    return out;
}

Matrix3f ESKF::getRotationMatrixFromState(Matrix<float, STATE_SIZE, 1> state) {
    Matrix<float, 4, 1> mat = state.block<4, 1>(6, 0);
    Quaternionf quat(mat);
    return quat.matrix();
}

Matrix3f ESKF::getSkew(Vector3f in) {
    Matrix3f out;
    out << 0, -in(2), in(1),
        in(2), 0, -in(0),
        -in(1), in(0), 0;
    return out;
}

Matrix3f ESKF::rotVecToMat(Vector3f in) {
    float angle = in.norm();
    Vector3f axis = in.normalized();
    if (angle == 0) axis = Vector3f(1, 0, 0);


    AngleAxisf angAx(angle, axis);
    return angAx.toRotationMatrix();
}

void ESKF::predictIMU(Vector3f a, Vector3f omega, float delta_t) {
    // Build F_x
    Matrix<float, dSTATE_SIZE, dSTATE_SIZE> F_x;
    F_x.setZero();
    Matrix3f rotation = getRotationMatrixFromState(nominalState);
    
    // page 59, eqn 269
    // dPos row
    F_x.block<3, 3>(dPOS_IDX, dPOS_IDX) = I_3;
    F_x.block<3, 3>(dPOS_IDX, dVEL_IDX) = I_3 * delta_t;
    // dVel row
    F_x.block<3, 3>(dVEL_IDX, dVEL_IDX) = I_3;
    F_x.block<3, 3>(dVEL_IDX, dTHETA_IDX) = 
            -rotation * getSkew(a - nominalState.block<3, 1>(AB_IDX, 0)) * delta_t;
    F_x.block<3, 3>(dVEL_IDX, dAB_IDX) = -rotation * delta_t;
    F_x.block<3, 3>(dVEL_IDX, dGRAV_IDX) = I_3 * delta_t;
    // dTheta row
    Vector3f delta_theta = (omega - nominalState.block<3, 1>(GB_IDX, 0))*delta_t;
    F_x.block<3, 3>(dTHETA_IDX, dTHETA_IDX) = rotVecToMat(delta_theta).transpose();
    F_x.block<3, 3>(dTHETA_IDX, dGB_IDX) = -I_3 * delta_t;
    // dGyroBias row
    F_x.block<3, 3>(dAB_IDX, dAB_IDX) = I_3;
    // dAccelBias row
    F_x.block<3, 3>(dGB_IDX, dGB_IDX) = I_3;
    // dGravity row
    F_x.block<3, 3>(dGRAV_IDX, dGRAV_IDX) = I_3;



    // build Q_i, this is only a diagonal matrix augmented by a scalar, so could be more efficient to for loop the relevant entries.

    Q_i.setZero();
    Q_i.block<3, 3>(0, 0) = sig2_a_n*delta_t*delta_t * I_3;
    Q_i.block<3, 3>(3, 3) = sig2_omega_n*delta_t*delta_t * I_3;
    Q_i.block<3, 3>(6, 6) = sig2_a_w*delta_t * I_3;
    Q_i.block<3, 3>(9, 9) = sig2_omega_w*delta_t * I_3;

    //probably unnecessary copying here. Need to check if things are done inplace or otherwise. //.eval should fix this issue This is by far the most expensive line (roughly 30% cpu alocation on mbed)
    P = F_x*P*F_x.transpose() + F_i*Q_i*F_i.transpose();


    //this line is apparently not needed, according to the document. // I suspect it only meant in the first iteration?????
    errorState = F_x * errorState;



}

Matrix<float, STATE_SIZE, 1> ESKF::measurementFunc(Matrix<float, STATE_SIZE, 1> in) {
    Matrix<float, STATE_SIZE, 1> func;
    func <<
        1, 1, 1,
        0, 0, 0,
        1, 1, 1, 1,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0;
    return (in.array()*func.array()).matrix();
}

void ESKF::composeTrueState() {

}


// this function puts the errorstate into the nominal state. as per page 62
void ESKF::injectObservedError() {

    nominalState = getTrueState();
}

Matrix<float, STATE_SIZE, 1> ESKF::getTrueState() {
    Matrix<float, STATE_SIZE, 1> newState;
    // compose position
    newState.block<3, 1>(0, 0) = nominalState.block<3, 1>(0, 0) + errorState.block<3, 1>(0, 0);
    // compose Velocity
    newState.block<3, 1>(3, 0) = nominalState.block<3, 1>(3, 0) + errorState.block<3, 1>(3, 0);

    // compose Quaternion - probably this can be done in less lines.
    Matrix<float, 3, 1>  angAxMat = errorState.block<3, 1>(6, 0);
    AngleAxisf AngAx(angAxMat.norm(), angAxMat.normalized());
    Quaternionf qError(AngAx);
    Matrix<float, 4, 1> qMat = nominalState.block<4, 1>(6, 0);
    Quaternionf qNom(qMat);
    newState.block<4, 1>(6, 0) = (qNom*qError).coeffs();

    //compose accelerometer drift
    newState.block<3, 1>(10, 0) = nominalState.block<3, 1>(10, 0) + errorState.block<3, 1>(9, 0);

    //compose gyro drift.
    newState.block<3, 1>(13, 0) = nominalState.block<3, 1>(13, 0) + errorState.block<3, 1>(12, 0);

    //compose gravity. (I don't think it changes anything.)
    newState.block<3, 1>(16, 0) = nominalState.block<3, 1>(16, 0) + errorState.block<3, 1>(15, 0);
    return newState;

}

void ESKF::resetError() {
    // set the errorState to zero
    errorState.Zero();

    // set up G matrix, can be simply an identity or with a more compicated term for the rotation section.
    G.setIdentity();
    Matrix3f rotCorrection;
    rotCorrection = -getSkew(0.5*errorState.block<3, 1>(6, 0));
    G.block<3, 3>(6, 6) = G.block<3, 3>(6, 6) + rotCorrection;
    P = G * P * G.transpose();

}


// this function is called when you have a reference to correct the error state, in this case a mocap system.
void ESKF::observeErrorState(Vector3f pos, Quaternionf rot) {
    Matrix<float, STATE_SIZE, 1> y;
    y.Zero();
    y.block<3, 1>(0, 0) = pos;
    y.block<4, 1>(6, 0) << rot.coeffs();

    // setup X_dx, essensially an identity, with some quaternion stuff in the middle. Optimise by initilising everything elsewhere.
    X_dx.Zero();
    Matrix<float, 6, 6> I6;
    I6 = I6.Identity();
    Matrix<float, 9, 9> I9;
    I9 = I9.Identity();
    X_dx.block<6, 6>(0, 0) = I6;
    X_dx.block<9, 9>(10, 9) = I9;
    Matrix<float, 4, 1> q(nominalState.block<4, 1>(6, 0)); // getting quaternion, though in a mat, so we can divide by 2.
    q = q / 2;
    X_dx.block<4, 3>(6, 6) <<
        -q.x(), -q.y(), -q.z(),
        q.w(), -q.z(), q.y(),
        q.z(), q.w(), -q.x(),
        -q.y(), q.x(), q.w();

    // then set up H_x, though this is not told to us directly, it is described as:
    /*"Here, Hx , ∂h∂xt|x
    is the standard Jacobian of h() with respect to its own argument (i.e.,
    the Jacobian one would use in a regular EKF). This first part of the chain rule depends on
    the measurement function of the particular sensor used, and is not presented here.*/

    //I believe that if I am only providing position and a quaternion then the position part will be an identity, the quaternion part will be identity?
    // I think I might need to use a gyro reading to compute the quaternion gradient?
    H_x = H_x.Identity();

    //compose the two halves of the hessian
    H = H_x * X_dx;
    Matrix<float, STATE_SIZE, STATE_SIZE> V; //  the covariance of the measurement function.
    V.Zero();

    K = P * H.transpose() * (H*P*H.transpose() + V).inverse();

    Matrix<float, dSTATE_SIZE, 1> d_x_hat;
    composeTrueState();
    d_x_hat = K * (y - measurementFunc(trueState));
    Matrix<float, dSTATE_SIZE, dSTATE_SIZE> I18;
    I18 = I18.Identity();

    // simple form
    //P = (I18 - K*H)*P;
    //Joseph form
    Matrix<float, dSTATE_SIZE, dSTATE_SIZE> IKH = I18 - K * H;
    P = IKH * P*IKH.transpose() + K * V*K.transpose();

    injectObservedError();

    resetError();




}
