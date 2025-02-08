#pragma once

#include <cstdint>

#include <cmath>

const int ESTIMATOR_FREQ = 100; //250;        // frequency to run state estimator at [Hz]
const float WHEEL_DIAMETER = 0.0618;  // [m]
const uint ENCODER_PULSES_PER_REVOLUTION = 700*2;   // detects rising and falling edge of each pulse
const float DISTANCE_PER_PULSE = M_PI * WHEEL_DIAMETER / ENCODER_PULSES_PER_REVOLUTION; 

// serial tx control characters
const char STX = '!';   // start of frame
const char ETX = '@';   // end of frame

enum ControlMode { AUTO, MANUAL };    // choose whether control system controls the motors (AUTO) or user manually sets wheel movements (MANUAL)


typedef struct {
  long long packetID;

  int64_t microSecondsSinceBoot;
} __attribute__((packed)) PacketHeader_t;

// raw IMU readings
typedef struct {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    int16_t temp;
} __attribute__((packed)) IMURawPacket_t;

// IMU readings converted into standard units
typedef struct {
    float ax;   // [m/s^2]
    // float ay;
    float az;
    // float gx;   // [rad/s]
    float gy;
    // float gz;
    float temp; // [deg C]
} __attribute__((packed)) IMUPacket_t;


// debugging info for calculation of pitch angle
typedef struct {
  float gyroOffsetY;  // [rad/s]

  // gyro angular velocity measurement
  float pitchVelocityGyro;  // [rad/s]

  bool isCalibrated;  // true if IMU values calibrated

  float pitchAccelRaw; // [rad] - uncalibrated
  float pitchAccel;  // [rad] - calibrated
  float pitchGyro;   // [rad]
  float pitchEst;    // [rad]
} __attribute__((packed)) PitchAngleCalcPacket_t;


typedef struct {
  float pitch_est;    // estimated pitch angle [rad]

  // wheel encoder measurements
  // float motor1DistanceMeas;
  int32_t motor1EncoderPulses;
  int32_t motor1EncoderPulsesDelta;
  // unsigned char motor1DirMeas;

  // float motor2DistanceMeas;
  int32_t motor2EncoderPulses;
  int32_t motor2EncoderPulsesDelta;
  // unsigned char motor2DirMeas;

  // gyro angular velocity measurement
  float pitch_velocity_gyro;

  bool estimatesValid;  // true if state estimates are valid

} __attribute__((packed)) StateEstimatePacket_t;

typedef struct {
    float pitch_setpoint;
    float pitch_current;
    float pitch_error;

    float motorSpeed;
    int8_t motorDir;      // TODO: use an enum?
    uint8_t dutyCycle;

    ControlMode controlMode;
    uint8_t dutyCycle1;
    uint8_t dutyCycle2;
    uint8_t dutyCycle2Calibrated;

} __attribute__((packed)) ControlPacket_t;



typedef struct {
  IMUPacket_t imu;
  PitchAngleCalcPacket_t pitchInfo;
  StateEstimatePacket_t state;
} __attribute__((packed)) DataPacket_t;