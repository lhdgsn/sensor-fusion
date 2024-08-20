#include <Arduino.h>
#include <Servo.h>
#include <LSM6DS3.h> // IMU driver
#include "Adafruit_AHRS_NXPFusion.h"
// #include <Kalman.cpp> // Kalman filter
// #include <Control.h> // servo control algorithm
// #include <DataWrite.h> // record measurement data

// constants
const float G = 9.81; // gravity
const float PID_GAINS[3] = {1, 0.01, 0.01}; // PID gains
const float SERVO_LOOP_FREQ = 100; // Hz

// instantiate IMU (I2C device address 0x6A)
LSM6DS3 myIMU(I2C_MODE, 0x6A);
Servo laser_servo;

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

float servo_angle = 0;
float update_servo(float measured_pitch, float target_pitch, ServoError& error_state) {
    // measured pitch is angle of housing relative to ground
    // target pitch is laser angle relative to ground
    // servo angle is relative to housing (assume this is updated instantaneously)
    // so want target pitch = measured pitch + servo angle

    // calculate error
    float pitch_error = target_pitch - (servo_angle + measured_pitch);
    float alpha = 0.5;
    error_state.e_dot = alpha * error_state.e_dot + (1 - alpha) * (pitch_error - error_state.e) * SERVO_LOOP_FREQ;
    error_state.e_int += pitch_error / SERVO_LOOP_FREQ;
    error_state.e = pitch_error;

    // calculate control signal
    float update = PID_GAINS[0]*error_state.e + PID_GAINS[1]*error_state.e_dot + PID_GAINS[2]*error_state.e_int;

    // update servo
    servo_angle += update;
    if(servo_angle > 180)
        servo_angle = 180;
    else if(servo_angle < 0)
        servo_angle = 0;
    laser_servo.write(int(servo_angle));

    return servo_angle;
}

// init Kalman filter
Adafruit_NXPSensorFusion kf;

void setup() {
    Serial.begin(9600);
    while(!Serial);

    // init IMU
    if (myIMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
    }

    // attach servo (digital pin 9)
    // initialize to midpoint
    laser_servo.attach(8);
    laser_servo.write(90);

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
ServoError error_state; // servo error
unsigned long t_servo = 0; // track last servo update
unsigned long t_kalman = 0; // track last kalman update
unsigned long t = 0;
float command_angle = -1;

void loop() {
    // read IMU
    IMUReading imu_data = readIMU();

    // kalman update
    // t = millis();
    // float dt = t - t_kalman; // for flexible kalman update (TODO)
    kf.update(imu_data.w[0], imu_data.w[1], imu_data.w[2],
        imu_data.a[0], imu_data.a[1], imu_data.a[2], 
        0.0, 0.0, 0.0);
    
    float roll = kf.getRoll();
    float pitch = kf.getPitch();
    // float yaw = kf.getYaw();

    // Serial.print("Accel X: "); Serial.print(imu_data.a[0], 3);
    // Serial.print(" Y: "); Serial.print(imu_data.a[1], 3);
    // Serial.print(" Z: "); Serial.print(imu_data.a[2], 3);
    // Serial.print(" | Gyro X: "); Serial.print(imu_data.w[0], 3);
    // Serial.print(" Y: "); Serial.print(imu_data.w[1], 3);
    // Serial.print(" Z: "); Serial.println(imu_data.w[2], 3);

    // control loop for pitch (lower frequency than sensor update)
    // t = millis();
    // if(t - t_servo > 50) {
    //     int target_angle = 90;
    //     command_angle = update_servo(roll, target_angle, error_state);
    //     t_servo = t; // reset servo update timer
    // }
    int target_angle = 90;
    command_angle = update_servo(roll, target_angle, error_state);

    // print estimated orientation
    Serial.print(millis());
    Serial.print(" - Orientation: ");
    Serial.print(roll);
    Serial.print(" ");
    Serial.println(pitch);
    Serial.print(" - Command angle: ");
    Serial.println(command_angle);

    delay(10);
}