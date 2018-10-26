#include <ESKF.h>

using namespace Eigen;
using namespace std;

ESKF::ESKF(float delta_t, Matrix<float, STATE_SIZE, 1> initialState,
        float var_a_n, float var_omega_n, float var_a_w, float var_omega_w)
        : dt_(delta_t),
        trueState(initialState) {
    
    // Jacobian of the state transition: page 59, eqn 269
    // Precompute constant part only
    F_x_.setZero();
    // dPos row
    F_x_.block<3, 3>(dPOS_IDX, dPOS_IDX) = I_3;
    F_x_.block<3, 3>(dPOS_IDX, dVEL_IDX) = I_3 * dt_;
    // dVel row
    F_x_.block<3, 3>(dVEL_IDX, dVEL_IDX) = I_3;
    F_x_.block<3, 3>(dVEL_IDX, dGRAV_IDX) = I_3 * dt_;
    // dTheta row
    F_x_.block<3, 3>(dTHETA_IDX, dGB_IDX) = -I_3 * dt_;
    // dGyroBias row
    F_x_.block<3, 3>(dAB_IDX, dAB_IDX) = I_3;
    // dAccelBias row
    F_x_.block<3, 3>(dGB_IDX, dGB_IDX) = I_3;
    // dGravity row
    F_x_.block<3, 3>(dGRAV_IDX, dGRAV_IDX) = I_3;

    // Precompute Q
    Q_diag_ <<
        (var_a_n*dt_*dt_     * I_3).diagonal(),
        (var_omega_n*dt_*dt_ * I_3).diagonal(),
        (var_a_w*dt_         * I_3).diagonal(),
        (var_omega_w*dt_     * I_3).diagonal();
}


Matrix<float, STATE_SIZE, 1> ESKF::makeState(Vector3f p, Vector3f v, Quaternionf q, 
            Vector3f a_b, Vector3f omega_b, Vector3f g) {
    Matrix<float, STATE_SIZE, 1> out;
    out << p, v, quatToHamilton(q), a_b, omega_b, g;
    return out;
}

Matrix3f ESKF::getDCM() {
    return getQuat().matrix();
}

Quaternionf ESKF::quatFromHamilton(Eigen::Vector4f qHam) {
    return Quaternionf(
        (Vector4f() <<
            qHam.block<3, 1>(1, 0), // x, y, z
            qHam.block<1, 1>(0, 0) // w
        ).finished());
}

Vector4f ESKF::quatToHamilton(Eigen::Quaternionf q){
    return (Vector4f() <<
            q.coeffs().block<1, 1>(3, 0), // w
            q.coeffs().block<3, 1>(0, 0) // x, y, z
        ).finished();
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
    Vector3f axis = (angle == 0) ? Vector3f(1, 0, 0) : in.normalized();
    AngleAxisf angAx(angle, axis);
    return angAx.toRotationMatrix();
}

Quaternionf ESKF::rotVecToQuat(Vector3f in) {
    float angle = in.norm();
    Vector3f axis = (angle == 0) ? Vector3f(1, 0, 0) : in.normalized();
    return Quaternionf(AngleAxisf(angle, axis));
}

void ESKF::predictIMU(Vector3f a_m, Vector3f omega_m) {
    // DCM of current state
    Matrix3f Rot = getDCM();
    // Accelerometer measurement
    Vector3f acc_body = a_m - getAccelBias();
    Vector3f acc_global = Rot * acc_body;
    // Gyro measruement
    Vector3f omega = omega_m - getGyroBias();
    Vector3f theta = omega * dt_;
    Quaternionf q_theta = rotVecToQuat(theta);
    Matrix3f R_theta = q_theta.toRotationMatrix();

    // Nominal state kinematics (eqn 259, pg 58)
    Vector3f delta_pos = getVel()*dt_ + 0.5f*(acc_global + getGravity())*dt_*dt_;
    nominalState.block<3, 1>(POS_IDX, 0) += delta_pos;
    nominalState.block<3, 1>(VEL_IDX, 0) += (acc_global + getGravity())*dt_;
    nominalState.block<4, 1>(QUAT_IDX, 0) = quatToHamilton(getQuat()*q_theta);

    // Jacobian of the state transition (eqn 269, page 59)
    // Update dynamic parts only
    // dVel row
    F_x_.block<3, 3>(dVEL_IDX, dTHETA_IDX) = -Rot * getSkew(acc_body) * dt_;
    F_x_.block<3, 3>(dVEL_IDX, dAB_IDX) = -Rot * dt_;
    // dTheta row
    Vector3f delta_theta = omega * dt_;
    F_x_.block<3, 3>(dTHETA_IDX, dTHETA_IDX) = R_theta.transpose();

    // Predict P and optimized inject variance
    P = F_x_*P*F_x_.transpose();
    P.diagonal().block<4*3, 1>(dVEL_IDX, 0) += Q_diag_;

}

// eqn 280, page 62
Matrix<float, 4, 3> ESKF::getQ_dtheta() {
    Vector4f qby2 = 0.5f*getQuatVector();
    // Assing to letters for readability. Note Hamilton order.
    float w = qby2[0];
    float x = qby2[1];
    float y = qby2[2];
    float z = qby2[3];
    Matrix<float, 4, 3>Q_dtheta;
    Q_dtheta <<
        -x, -y, -z,
        w, -z, y,
        z, w, -x,
        -y, x, w;
    return Q_dtheta;
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
    newState.block<4, 1>(6, 0) = quatToHamilton(getQuat()*qError);

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
    y.block<4, 1>(6, 0) << quatToHamilton(rot);

    // setup X_dx, essensially an identity, with some quaternion stuff in the middle. Optimise by initilising everything elsewhere.
    X_dx.Zero();
    Matrix<float, 6, 6> I6;
    I6 = I6.Identity();
    Matrix<float, 9, 9> I9;
    I9 = I9.Identity();
    X_dx.block<6, 6>(0, 0) = I6;
    X_dx.block<9, 9>(10, 9) = I9;


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
