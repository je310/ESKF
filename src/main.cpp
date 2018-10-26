#include <ESKF.h>
#include <iostream>
#include <chrono>

#define SQ(x) (x*x)

using namespace Eigen;
using namespace std;

int main(int argc, char** argv) {

    float sigma_accel = 0.1; // [m/s^2]
    float sigma_gyro = 0.01; // [rad/s]
    float sigma_accel_drift = 1e-6; // [m/s^2 sqrt(s)]
    float sigma_gyro_drift = 1e-6; // [rad/s sqrt(s)]

    float sigma_init_pos = 1.0; // [m]
    float sigma_init_vel = 0.1; // [m/s]
    float sigma_init_dtheta = 1.0; // [rad]
    float sigma_init_accel_bias = 1.0; // [m/s^2]
    float sigma_init_gyro_bias = 1.0; // [rad/s]
    float sigma_init_gravity = 10; // kill me now

    float sigma_mocap_pos = 0.01; // [m]
    float sigma_mocap_rot = 0.1; // [rad]

    ESKF eskf(
            0.001f, // delta_t
            ESKF::makeState(
                Vector3f(0, 0, 0), // init pos
                Vector3f(0, 0, 0), // init vel
                Quaternionf(AngleAxisf(0.5f, Vector3f(1, 0, 0))), // init quaternion
                Vector3f(0, 0, 0), // init accel bias
                Vector3f(0, 0, 0), // init gyro bias
                // Vector3f(0, 0, -GRAVITY) // init gravity
                Vector3f(0, 0, 0) // init gravity
            ),
            ESKF::makeP(
                SQ(sigma_init_pos) * I_3,
                SQ(sigma_init_vel) * I_3,
                SQ(sigma_init_dtheta) * I_3,
                SQ(sigma_init_accel_bias) * I_3,
                SQ(sigma_init_gyro_bias) * I_3,
                SQ(sigma_init_gravity) * I_3
            ),
            SQ(sigma_accel),
            SQ(sigma_gyro),
            SQ(sigma_accel_drift),
            SQ(sigma_gyro_drift));

    //timing some key things just for some insight. Accell is 1000hz and mocap is 100hz.
    float sim_data_duration = 1000.0f; //seconds
    auto start = std::chrono::system_clock::now();
    for (int ms = 0; ms < 1000*sim_data_duration; ms++) {

        // Simulated static true pos/orientation
        Vector3f pos_true = Vector3f(0, 0, 0);
        Quaternionf q_true = Quaternionf(AngleAxisf(0.0f, Vector3f(1, 0, 0)));
        Matrix3f R_true = q_true.toRotationMatrix();

        // Fake accel/gyro
        Vector3f acc_true = R_true.transpose() * Vector3f(0, 0, GRAVITY);
        Vector3f acc = acc_true + sigma_accel * Vector3f::Random();
        Vector3f gyro = sigma_gyro * Vector3f::Random();
        // Input our accel/gyro data
        eskf.predictIMU(acc, gyro);

        // 10 accel/gyro measurements per motion capture input.
        if (ms % 10 == 0) {
            // Fake mocap data
            Vector3f pos_meas = pos_true + sigma_mocap_pos * Vector3f::Random();
            Quaternionf q_meas = q_true;
            // input our motion capture data
            eskf.measurePos(pos_meas, SQ(sigma_mocap_pos)*I_3);
            eskf.measureQuat(q_meas, SQ(sigma_mocap_rot)*I_3);
        }
    }
    auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << elapsed.count() << std::endl;

    std::cout << "Pos: " << std::endl << eskf.getPos() << std::endl;
    std::cout << "Vel: " << std::endl << eskf.getVel() << std::endl;
    std::cout << "QuatVector: " << std::endl << eskf.getQuatVector() << std::endl;
    std::cout << "AccelBias: " << std::endl << eskf.getAccelBias() << std::endl;
    std::cout << "GyroBias: " << std::endl << eskf.getGyroBias() << std::endl;
    std::cout << "Gravity: " << std::endl << eskf.getGravity() << std::endl;

}
