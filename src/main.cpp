#include <ESKF.h>
#include <iostream>
#include <chrono>

using namespace Eigen;
using namespace std;


int main(int argc, char** argv) {

    float sigma_accel = 0.1;
    float sigma_gyro = 0.1;
    float sigma_accel_drift = 1e-6;
    float sigma_gyro_drift = 1e-6;

    float sigma_init_pos = 0.1;
    float sigma_init_vel = 0.1;
    float sigma_init_dtheta = 1e-6;
    float sigma_init_accel_bias = 1e-6;
    float sigma_init_gyro_bias = 1e-6;
    float sigma_init_gravity = 1e-6;

    ESKF eskf(
            0.001f, // delta_t
            ESKF::makeState(
                Vector3f(0, 0, 0), // init pos
                Vector3f(0, 0, 0), // init vel
                ESKF::quatFromHamilton(Vector4f(1, 0, 0, 0)), // init quaternion
                Vector3f(0, 0, 0), // init accel bias
                Vector3f(0, 0, 0), // init gyro bias
                // Vector3f(0, 0, -GRAVITY) // init gravity
                Vector3f(0, 0, 0) // init gravity
            ),
            ESKF::makeP(
                sigma_init_pos*sigma_init_pos * I_3,
                sigma_init_vel*sigma_init_vel * I_3,
                sigma_init_dtheta*sigma_init_dtheta * I_3,
                sigma_init_accel_bias*sigma_init_accel_bias * I_3,
                sigma_init_gyro_bias*sigma_init_gyro_bias * I_3,
                sigma_init_gravity*sigma_init_gravity * I_3
            ),
            sigma_accel*sigma_accel,
            sigma_gyro*sigma_gyro,
            sigma_accel_drift*sigma_accel_drift,
            sigma_gyro_drift*sigma_gyro_drift);

    //timing some key things just for some insight. Accell is 1000hz and mocap is 100hz.
    auto start = std::chrono::system_clock::now();
    for (int i = 0; i < 50000; i++) {
        // we will have about 10 accel/gyro measurements per motion capture input.
        for (int f = 0; f < 10; f++) {
            // Fake accel/gyro
            Vector3f acc = sigma_accel * Vector3f::Random() + Vector3f(0,0,GRAVITY);
            Vector3f gyro = sigma_gyro * Vector3f::Random();
            // Input our accel/gyro data
            eskf.predictIMU(acc, gyro);
        }
        // Fake mocap data
        Vector3f pos = Vector3f::Random();
        Quaternionf quat = ESKF::quatFromHamilton(Vector4f(1, 0, 0, 0));
        // input our motion capture data
        eskf.measurePos(pos, I_3);
        eskf.measureQuat(quat, I_3);
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
