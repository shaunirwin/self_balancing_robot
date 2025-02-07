#pragma once

#include <cstdint>


typedef struct {
  float testVal;
  char c;
  bool b;
} __attribute__((packed)) TestPacket_t;


typedef struct {
  long long packetID;

  int64_t microSecondsSinceBoot;
} __attribute__((packed)) PacketHeader_t;


typedef struct {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
} __attribute__((packed)) IMUPacket_t;


// the following struct is used to show debugging info
typedef struct {
  float ax;   // [m/s^2]
  float az;   // [m/s^2]
  float gy;   // [rad/s]

  float gyroOffsetY;  // [rad/s]

  // gyro angular velocity measurement
  float pitchVelocityGyro;  // [rad/s]

  bool isCalibrated;  // true if IMU values calibrated

  float pitchAccel;  // [rad]
  float pitchGyro;   // [rad]
  float pitchEst;    // [rad]
} __attribute__((packed)) PitchAngleCalcPacket_t;


typedef struct {
  // IMU estimates
  float pitch_est;    // [rad]

  // wheel encoder measurements
  // float motor1DistanceMeas;
  signed long motor1EncoderPulses;
  signed long motor1EncoderPulsesDelta;
  // unsigned char motor1DirMeas;

  // float motor2DistanceMeas;
  signed long motor2EncoderPulses;
  signed long motor2EncoderPulsesDelta;
  // unsigned char motor2DirMeas;

  // gyro angular velocity measurement
  float pitch_velocity_gyro;

} __attribute__((packed)) StateEstimatePacket_t;

typedef struct {
    float pitch_setpoint;
    float pitch_current;
    float pitch_error;

    float motorSpeed;
    int motorDir;
    uint8_t dutyCycle;

} __attribute__((packed)) ControlPacket_t;



typedef struct {
  PitchAngleCalcPacket_t pitchInfo;
  StateEstimatePacket_t state;
} __attribute__((packed)) DataPacket_t;