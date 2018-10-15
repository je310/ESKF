#include <ESKF.h>
#include <iostream>
#include <chrono>

// this is the main file that will be used to test the ESKF class.

int main(int argc, char** argv){

    ESKF ourESKF;
    Vector3f acc, gyro,pos;
    Quaternionf quat(0.25,0.25,0.25,0.25);
    acc = Vector3f::Random();
    gyro = Vector3f::Random();
    pos = Vector3f::Random();

    //timing some key things just for some insight. Accell is 1000hz and mocap is 100hz.
    auto start = std::chrono::system_clock::now();
    for(int i = 0; i < 1000; i ++){
        ourESKF.predictionUpdate(acc,gyro, 0.001);
    }
    for(int i = 0; i < 100; i ++){
        ourESKF.observeErrorState(pos,quat);
    }
    auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << elapsed.count() << '\n';

}
