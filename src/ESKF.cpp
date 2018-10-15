#include <ESKF.h>


ESKF::ESKF(Matrix<float, 19,1> initialState, float sig2_a_n_, float sig2_omega_n_,float sig2_a_w_, float sig2_omega_w_){
    trueState = initialState;
    sig2_a_n = sig2_a_n_;
    sig2_omega_n = sig2_omega_n_;
    sig2_a_w = sig2_a_w_;
    sig2_omega_w = sig2_omega_w_;

    //Build F_i,
    F_i.setZero();
    F_i.block(3,0,12,12).setIdentity();
}

ESKF::ESKF(){

}


Matrix<float, 19,1> ESKF::makeState(Vector3f p,Vector3f v, Quaternionf q, Vector3f a_b, Vector3f omega_b,Vector3f g ){

}

void ESKF::updateStateIMU(Vector3f a, Vector3f omega, float delta_t){


}

Matrix<float, 3,3> ESKF::getRotationMatrixFromState(Matrix<float, 19,1> state){
    Matrix<float,4,1> mat = state.block(6,0,4,1);
    Quaternionf quat(mat);
    return quat.matrix();
}

Matrix<float,3,3> ESKF::getSkew(Vector3f in){
   Matrix<float,3,3> out;
   out << 0, -in(2), in(1),
           in(2), 0, -in(0),
           -in(1), in(0), 0;
   return out;
}

Matrix<float,3,3> ESKF::AngAxToMat(Vector3f in){
    float angle = in.norm();
    Vector3f axis = in.normalized();
    if(angle == 0) axis = Vector3f(1,0,0);


    AngleAxisf angAx(angle,axis);
    return angAx.toRotationMatrix();
}

void ESKF::predictionUpdate(Vector3f a, Vector3f omega, float delta_t){
    // build F_x
    Matrix<float, 3,3> I3 , I3dt;
    F_x = F_x.Zero(18,18);
    //page 59
    I3 = I3.Identity();
    I3dt = delta_t * I3;
    F_x.block(0,0,3,3) = I3;
    F_x.block(3,3,3,3) = I3;
    F_x.block(9,9,3,3) = I3;
    F_x.block(12,12,3,3) = I3;
    F_x.block(15,15,3,3) = I3;
    F_x.block(0,3,3,3) = I3dt;
    F_x.block(3,15,3,3) = I3dt;
    F_x.block(6,12,3,3) = -I3dt;

    Matrix<float, 3,3> rotation;
    rotation = getRotationMatrixFromState(nominalState);
    F_x.block(3,9,3,3) = -rotation*delta_t;

    // for the 2nd row and 3rd column
    F_x.block(3,6,3,3) = - rotation * getSkew(a - nominalState.block(9,0,3,1)) * delta_t;

    // for the 3rd row 3rd column
    F_x.block(6,6,3,3) = AngAxToMat((omega - nominalState.block(12,0,3,1))*delta_t).transpose();


    // build Q_i, this is only a diagonal matrix augmented by a scalar, so could be more efficient to for loop the relevant entries.

    Q_i.setZero();
    Q_i.block(0,0,3,3) =  delta_t * delta_t * sig2_a_n  * I3 ;
    Q_i.block(3,3,3,3) = I3 * sig2_omega_n * delta_t * delta_t;
    Q_i.block(6,6,3,3) = I3 * sig2_a_w * delta_t;
    Q_i.block(9,9,3,3) = I3 * sig2_omega_w * delta_t;

    //probably unnecessary copying here. Need to check if things are done inplace or otherwise. //.eval should fix this issue
     P = (F_x*P*F_x.transpose() + F_i*Q_i*F_i.transpose()).eval();


    //this line is apparently not needed, according to the document. // I suspect it only meant in the first iteration?????
    //Matrix<float,18,18> errorState_new = F_x * errorState;
    //errorState = errorState_new;


}

Matrix<float,19,1> ESKF::measurementFunc(Matrix<float,19,1> in){
    Matrix<float,19,1> func;
    func << 1,1,1
            ,0,0,0
            ,1,1,1,1
            ,0,0,0
            ,0,0,0
            ,0,0,0;
    return (in.array()*func.array()).matrix();
}

void ESKF::composeTrueState(){

}


// this function puts the errorstate into the nominal state. as per page 62
void ESKF::injectObservedError(){
    Matrix<float,19,1> newState;
    // compose position
    newState.block(0,0,3,1) = nominalState.block(0,0,3,1) + errorState.block(0,0,3,1);
    // compose Velocity
    newState.block(3,0,3,1) = nominalState.block(3,0,3,1) + errorState.block(3,0,3,1);

    // compose Quaternion - probably this can be done in less lines.
    Matrix<float,3,1>  angAxMat = errorState.block(6,0,3,1);
    AngleAxisf AngAx(angAxMat.norm(),angAxMat.normalized());
    Quaternionf qError(AngAx);
    Matrix<float,4,1> qMat =  nominalState.block(6,0,4,1);
    Quaternionf qNom(qMat);
    newState.block(6,0,4,1) = (qNom*qError).coeffs();

    //compose accelerometer drift
    newState.block(10,0,3,1) = nominalState.block(10,0,3,1) + errorState.block(9,0,3,1);

    //compose gyro drift.
    newState.block(13,0,3,1) = nominalState.block(13,0,3,1) + errorState.block(12,0,3,1);

    //compose gravity. (I don't think it changes anything.)
    newState.block(16,0,3,1) = nominalState.block(16,0,3,1) + errorState.block(15,0,3,1);

    //update the nominal state (I am doing a copy to be sure there is no aliasing problems etc)
    nominalState = newState;
}

void ESKF::resetError(){
    // set the errorState to zero
    errorState.Zero();

    // set up G matrix, can be simply an identity or with a more compicated term for the rotation section.
    G.setIdentity();
    Matrix<float,3,3> rotCorrection;
    rotCorrection = - getSkew(0.5*errorState.block(6,0,3,1));
    G.block(6,6,3,3) = (G.block(6,6,3,3) + rotCorrection).eval();

}


// this function is called when you have a reference to correct the error state, in this case a mocap system.
void ESKF::observeErrorState(Vector3f pos, Quaternionf rot){
    Matrix<float,19,1> y;
    y.Zero();
    y.block(0,0,3,1) = pos;
    y.block(6,0,4,1) <<rot.x() , rot.y() , rot.z() , rot.w();

    // setup X_dx, essensially an identity, with some quaternion stuff in the middle. Optimise by initilising everything elsewhere.
    X_dx.Zero();
    Matrix<float, 6,6> I6;
    I6 = I6.Identity();
    Matrix<float,9,9> I9;
    I9 = I9.Identity();
    X_dx.block(0,0,6,6) = I6;
    X_dx.block(10,9,9,9) = I9;
    Matrix<float,4,1> q(nominalState.block(6,0,4,1)); // getting quaternion, though in a mat, so we can divide by 2.
    q = q/2;
    X_dx.block(6,6,4,3) <<   -q.x() , -q.y() , -q.z(),
                              q.w() , -q.z() ,  q.y(),
                              q.z() ,  q.w() , -q.x(),
                             -q.y() ,  q.x() ,  q.w();

    // then set up H_x, though this is not told to us directly, it is described as:
    /*"Here, Hx , ∂h∂xt|x
    is the standard Jacobian of h() with respect to its own argument (i.e.,
    the Jacobian one would use in a regular EKF). This first part of the chain rule depends on
    the measurement function of the particular sensor used, and is not presented here.*/

    //I believe that if I am only providing position and a quaternion then the position part will be an identity, the quaternion part will be identity?

    H_x = H_x.Identity();

    //compose the two halves of the hessian
    H = H_x*X_dx;
    Matrix<float,19,19> V; //  the covariance of the measurement function.
    V.Zero();
    K = P * H.transpose() * (H*P*H.transpose() + V).inverse();

    Matrix<float,18,1> d_x_hat;
    composeTrueState();
    d_x_hat = K *(y - measurementFunc(trueState));
    Matrix<float,18,18> I18;
    I18 = I18.Identity();
    P = ((I18 - K*H)*P).eval();

    injectObservedError();

    resetError();




}
