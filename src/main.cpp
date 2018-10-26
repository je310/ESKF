#include <ESKF.h>
#include <iostream>
#include <chrono>

using namespace Eigen;
using namespace std;


int main(int argc, char** argv) {

    Matrix<float, STATE_SIZE, 1> initState = ESKF::makeState(
        Vector3f(0, 0, 0),
        Vector3f(0, 0, 0),
        Quaternionf(1, 0, 0, 0),
        Vector3f(0, 0, 0),
        Vector3f(0, 0, 0),
        // Vector3f(0, 0, -GRAVITY));
        Vector3f(0, 0, 0));
    ESKF ourESKF(initState, 0.1, 0.1, 1e-6, 1e-6);

    //timing some key things just for some insight. Accell is 1000hz and mocap is 100hz.
    auto start = std::chrono::system_clock::now();
    for (int i = 0; i < 50000; i++) {
        // we will have about 10 accel/gyro measurements per motion capture input.
        for (int f = 0; f < 10; f++) {
            Vector3f acc = Vector3f::Random();
            Vector3f gyro = Vector3f::Random();
            //input our accel/gyro into the system
            ourESKF.predictIMU(acc, gyro, 0.001);
            //Get an update of where we are.
            ourESKF.getTrueState();
            // do something with the info.
        }
        Vector3f pos = Vector3f::Random();
        Quaternionf quat(1.0f, 0.0f, 0.0f, 0.0f);
        // input our motion capture data.
        ourESKF.observeErrorState(pos, quat);
    }
    auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << elapsed.count() << '\n';

}
