#include <Arduino.h>
#include <LSM6DS3.h> // IMU driver
#include "Adafruit_AHRS_NXPFusion.h"
// #include <Kalman.cpp> // Kalman filter
// #include <Control.h> // servo control algorithm
// #include <DataWrite.h> // record measurement data

// constants
const float G = 9.81; // gravity
float PID_GAINS[3] = {1,0.1,1}; // PID gains
const float dt = 0.1; // integration timestep

// instantiate IMU (I2C device address 0x6A)
LSM6DS3 myIMU(I2C_MODE, 0x6A);

struct IMUReading {
    float a[3] = {0,0,0}; // acceleration
    float w[3] = {0,0,0}; // angular velocity
};

struct State {
    float x[3] = {0,0,0}; // position
    float v[3] = {0,0,0}; // velocity
    float theta[3] = {0,0,0}; // orientation (roll, pitch, yaw)
};

struct ServoError {
    float e = 0; // error
    float e_dot = 0; // error derivative
    float e_int = 0; // error integral
};

float accel_zero_calibration[3] = {0,0,0};
float gyro_zero_calibration[3] = {0,0,0};

IMUReading readIMU(void) {
    IMUReading imu_reading;
    imu_reading.a[0] = myIMU.readFloatAccelX() - accel_zero_calibration[0];
    imu_reading.a[1] = myIMU.readFloatAccelY() - accel_zero_calibration[1];
    imu_reading.a[2] = myIMU.readFloatAccelZ() - accel_zero_calibration[2];
    imu_reading.w[0] = myIMU.readFloatGyroX() - gyro_zero_calibration[0];
    imu_reading.w[1] = myIMU.readFloatGyroY() - gyro_zero_calibration[1];
    imu_reading.w[2] = myIMU.readFloatGyroZ() - gyro_zero_calibration[2];

    return imu_reading;
}

void update_servo(float current_pitch, float target_pitch, ServoError& error_state, float* pid_gains) {
    // calculate error
    float e_new = target_pitch - current_pitch;
    error_state.e_dot = (e_new - error_state.e)/dt;
    error_state.e_int += e_new*dt;
    error_state.e = e_new;

    // calculate control signal
    float u = pid_gains[0]*error_state.e + pid_gains[1]*error_state.e_dot + pid_gains[2]*error_state.e_int;

    // update servo
}

Adafruit_NXPSensorFusion kf;

void setup() {
    Serial.begin(9600);
    while(!Serial);

    // init IMU
    if (myIMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
    }

    // init gyro offset
    int n_steps = 100;
    float accel_calibration_tmp[3] = {0,0,0};
    float gyro_calibration_tmp[3] = {0,0,0};
    for(int i=0; i<n_steps; i++) {
        IMUReading imu_init = readIMU();

        for(int j=0; j<3; j++) {
            accel_calibration_tmp[j] += imu_init.a[j];
            gyro_calibration_tmp[j] += imu_init.w[j];
        }

        delay(10);
    }

    // take mean
    for(int j=0; j<3; j++) {
        accel_zero_calibration[j] = accel_calibration_tmp[j] / n_steps;
        gyro_zero_calibration[j] = gyro_calibration_tmp[j] / n_steps;
    }
    
    // add gravity offset
    accel_zero_calibration[2] -= 1;

    // init kalman filter
    kf.begin(100);
}

// global variables
// IMUReading imu_current; // current IMU reading
// IMUReading imu_prev; // previous IMU reading
State state_current; // current state
ServoError error_state; // servo error

void loop() {
  // read IMU
    IMUReading imu_data = readIMU();

    // kalman update
    kf.update(imu_data.w[0], imu_data.w[1], imu_data.w[2],
        imu_data.a[0], imu_data.a[1], imu_data.a[2], 
        0.0, 0.0, 0.0);
    
    // Print the orientation filter output
    float roll = kf.getRoll();
    float pitch = kf.getPitch();
    // float yaw = kf.getYaw();
    Serial.print(millis());
    Serial.print(" - Orientation: ");
    Serial.print(roll);
    Serial.print(" ");
    Serial.println(pitch);
    // Serial.print(" ");
    // Serial.println(yaw);

    // Serial.print("Accel X: "); Serial.print(imu_data.a[0], 3);
    // Serial.print(" Y: "); Serial.print(imu_data.a[1], 3);
    // Serial.print(" Z: "); Serial.print(imu_data.a[2], 3);
    // Serial.print(" | Gyro X: "); Serial.print(imu_data.w[0], 3);
    // Serial.print(" Y: "); Serial.print(imu_data.w[1], 3);
    // Serial.print(" Z: "); Serial.println(imu_data.w[2], 3);

    // control loop for pitch
    update_servo(roll, 0, error_state, PID_GAINS);

    delay(10);
}