#include <ESKF.h>
#include <iostream>
#include <chrono>

using namespace Eigen;
using namespace std;


int main(int argc, char** argv) {

    float std_dev_accel = 0.1;
    float std_dev_gyro = 0.1;
    float std_dev_accel_drift = 1e-6;
    float std_dev_gyro_drift = 1e-6;

    Matrix<float, STATE_SIZE, 1> initState = ESKF::makeState(
        Vector3f(0, 0, 0),
        Vector3f(0, 0, 0),
        ESKF::quatFromHamilton(Vector4f(1, 0, 0, 0)),
        Vector3f(0, 0, 0),
        Vector3f(0, 0, 0),
        // Vector3f(0, 0, -GRAVITY));
        Vector3f(0, 0, 0));
    ESKF eskf(
            0.001f, initState,
            std_dev_accel*std_dev_accel,
            std_dev_gyro*std_dev_gyro,
            std_dev_accel_drift*std_dev_accel_drift,
            std_dev_gyro_drift*std_dev_gyro_drift);

    //timing some key things just for some insight. Accell is 1000hz and mocap is 100hz.
    auto start = std::chrono::system_clock::now();
    for (int i = 0; i < 50000; i++) {
        // we will have about 10 accel/gyro measurements per motion capture input.
        for (int f = 0; f < 10; f++) {
            Vector3f acc = std_dev_accel * Vector3f::Random() + Vector3f(0,0,GRAVITY);
            Vector3f gyro = std_dev_gyro * Vector3f::Random();
            //input our accel/gyro into the system
            eskf.predictIMU(acc, gyro);
            //Get an update of where we are.
            // eskf.getTrueState();
            // do something with the info.
        }
        Vector3f pos = Vector3f::Random();
        Quaternionf quat(1.0f, 0.0f, 0.0f, 0.0f);
        // input our motion capture data.
        eskf.observeErrorState(pos, quat);
    }
    auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << elapsed.count() << std::endl;

    // Inspect with debugger
    Vector3f p = eskf.getPos();
    Vector3f v = eskf.getVel();
    Quaternionf q = eskf.getQuat();
    Vector3f a_b = eskf.getAccelBias();
    Vector3f omega_b = eskf.getGyroBias();
    Vector3f g = eskf.getGravity();

}
