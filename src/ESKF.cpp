#include <ESKF.h>


ESKF::ESKF(Matrix<float, 19,1> initialState, Vector3f sig2_a_n_, Vector3f sig2_omega_n_,Vector3f sig2_a_w_, Vector3f sig2_omega_w_){
    trueState = initialState;
    sig2_a_n = sig2_a_n_;
    sig2_omega_n = sig2_omega_n_;
    sig2_a_w = sig2_a_w_;
    sig2_omega_w = sig2_omega_w_;
}

ESKF::ESKF(){

}


Matrix<float, 19,1> ESKF::makeState(Vector3f p,Vector3f v, Quaternionf q, Vector3f a_b, Vector3f omega_b,Vector3f g ){

}

void ESKF::updateStateIMU(Vector3f a, Vector3f omega, float delta_t){


}

Matrix<float, 3,3> ESKF::getRotationMatrixFromState(Matrix<float, 19,1> state){
    Quaternionf quat(state.block(6,0,4,1));
    return quat.matrix();
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
    cout << F_x << endl;

    // for the 2nd row and 3rd column
    Matrix<float, 3,3> rotation;
    rotation = getRotationMatrixFromState(nominalState);

}
