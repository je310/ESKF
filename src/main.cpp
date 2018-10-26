#include <ESKF.h>
#include <iostream>
#include <chrono>

// this is the main file that will be used to test the ESKF class.
Matrix<float, 19,1> makeState(Vector3f p,Vector3f v, Quaternionf q, Vector3f a_b, Vector3f omega_b,Vector3f g ){
    Matrix<float,19,1> out;
    out.block(0,0,3,1) = p;
    out.block(3,0,3,1) = v;
    out.block(6,0,4,1) = q.coeffs();
    out.block(10,0,3,1) = a_b;
    out.block(13,0,3,1) = omega_b;
    out.block(15,0,3,1) = g;
    return out;
}

int main(int argc, char** argv){

    Matrix<float,19,1> initState = makeState(Vector3f(0,0,0),Vector3f(0,0,0),Quaternionf(1,0,0,0),Vector3f(0,0,0),Vector3f(0,0,0),Vector3f(0,0,-GRAVITY));
    ESKF ourESKF(initState, 1,1,1,1);
    Vector3f acc, gyro,pos;
    Quaternionf quat(0.25,0.25,0.25,0.25);
    acc = Vector3f::Random();
    gyro = Vector3f::Random();
    pos = Vector3f::Random();

    //timing some key things just for some insight. Accell is 1000hz and mocap is 100hz.
    auto start = std::chrono::system_clock::now();
    for(int i = 0; i < 10000; i ++){
        // we will have about 10 accel/gyro measurements per motion capture input.
        for(int f = 0; f < 10; f ++){
            //input our accel/gyro into the system
            ourESKF.predictionUpdate(acc,gyro, 0.001);
            //Get an update of where we are.
            ourESKF.getTrueState();
            // do something with the info.
        }

        // input our motion capture data.
        ourESKF.observeErrorState(pos,quat);
    }
    auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << elapsed.count() << '\n';

}
