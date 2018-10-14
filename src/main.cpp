#include <ESKF.h>
#include <iostream>

// this is the main file that will be used to test the ESKF class.

int main(int argc, char** argv){

    ESKF ourESKF;
    Vector3f acc, gyro;
    ourESKF.predictionUpdate(acc,gyro, 0.001);


}
