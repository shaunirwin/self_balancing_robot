#include <iostream>
#include <string>
#include <algorithm>
#include <cstdint>
#include <atomic>
#include <cstring>

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "driver/pcnt.h"
// #include "driver/pulse_cnt.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

#include "data_structs.h"

// #include "sd_read_write.h"
// #include "SD_MMC.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include "secrets.h"
#include "pid.h"

MPU6050 imu;
AsyncWebServer server(80);

// // NB: it looks like these correspond to pin numbers, rather than GPIO numbers (therefore GPIOs 32, 33, 34)
// #define SD_MMC_CMD 38 //Please do not modify it.
// #define SD_MMC_CLK 39 //Please do not modify it. 
// #define SD_MMC_D0  40 //Please do not modify it.

const int PIN_LED_PWM = 2;
const int PIN_MOTOR1_SLEEP = 42;
const int PIN_MOTOR2_SLEEP = 41;
const int PIN_MOTOR1_DIR = 40;
const int PIN_MOTOR2_DIR = 39;
// Keep encoder numbering aligned with the corresponding motor/PWM channel.
const int PIN_ENCODER1A = 16;
const int PIN_ENCODER1B = 15;
const int PIN_ENCODER2A = 11;
const int PIN_ENCODER2B = 12;
const int PIN_MOTOR1_PWM = 35;
const int PIN_MOTOR2_PWM = 45;
const int PIN_I2C_SDA = 14;
const int PIN_I2C_SCL = 13;
// const int PIN_PULSE_INPUT = 11;   // testing PCNT module for wheel encoder position/speed measurement
const auto PCNT_UNIT = PCNT_UNIT_0;

const int PWM_FREQ = 30000;         // frequency to run PWM at [Hz]
const int MOTOR1_PWM_CHANNEL = 0;   // set the PWM channel
const int MOTOR2_PWM_CHANNEL = 1;   // set the PWM channel
const int PWM_RESOLUTION = 8;       // set PWM resolution

const bool MOTOR_1_DIR_INVERT = false;
const bool MOTOR_2_DIR_INVERT = true;
const bool MOTOR_COAST = false;

bool ledStatus = true;

// state estimation
int16_t pcnt_encoder_pulse_count;  // for PCNT module
uint32_t intr_status;       // for PCNT module
volatile signed long motor1EncoderPulses = 0;
volatile int motor1DirMeas = 0;   // 0: stopped, 1: forward, -1: backward
volatile signed long motor2EncoderPulses = 0;
volatile int motor2DirMeas = 0;

long long packetID = 0;


float accel_resolution = 0;
float gyro_resolution = 0;
float pitchAngleGyro = 0;             // [rad]
float pitchAngleEst = 0;              // [rad]

// hardware timer
const float ALPHA = 0.98;             // gyro weight for complementary filter
hw_timer_t *hwTimer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
QueueHandle_t queueIMU;  // queue of IMU measurements
QueueHandle_t queueStateEstimates;  // queue of state estimates
QueueHandle_t queueTelemetry;  // latest telemetry snapshot waiting for serial transmission
QueueHandle_t queueControlCommands;  // commands from the I/O core to the control core
std::atomic<bool> resetControlTimingRequested {false};
std::atomic<bool> emergencyStopRequested {false};

const uint TELEMETRY_DECIMATION = 5;  // transmit at 20 Hz when the controller runs at 100 Hz

// Keep the time-critical control pipeline away from Wi-Fi and web-server work.
constexpr BaseType_t IO_CORE = 0;
constexpr BaseType_t CONTROL_CORE = 1;
constexpr UBaseType_t IO_TASK_PRIORITY = 1;
constexpr UBaseType_t CONTROL_PIPELINE_PRIORITY = 3;
constexpr UBaseType_t MOTOR_CONTROL_PRIORITY = 4;
constexpr float AUTO_ARM_MAX_PITCH_ERROR = 5.0f * M_PI / 180.0f;

enum class ControlCommandType : uint8_t {
  SET_PID_KP,
  SET_PID_KI,
  SET_PID_KD,
  SET_PID_SETPOINT,
  SET_DUTY_CYCLE_MIN,
  SET_DUTY_CYCLE_MAX,
  SET_PITCH_ERROR_MAX,
  SET_PITCH_ERROR_MIN,
  SET_CONTROL_MODE,
  SET_MOTOR1_DIRECTION,
  SET_MOTOR2_DIRECTION,
  SET_MOTOR1_DUTY,
  SET_MOTOR2_DUTY,
  START_RECORDING,
  STOP_RECORDING,
};

typedef struct {
  ControlCommandType type;
  float floatValue;
  uint32_t uintValue;
  ControlMode controlMode;
  MotorDirection motorDirection;
} ControlCommand_t;

typedef struct {
  PacketHeader_t header;
  DataPacket_t data;
} TelemetryPacket_t;

MotorDirection dirDrive { MotorDirection::FORWARD };
MotorDirection motor1DirManual { MotorDirection::FORWARD }; // motor 1 direction when in manual mode
MotorDirection motor2DirManual { MotorDirection::FORWARD };
uint8_t dutyCycle1Manual {0};             // motor 1 duty cycle when in manual mode
uint8_t dutyCycle2Manual {0};

// Always boot disarmed. AUTO must be selected explicitly after calibration and
// the live state estimate have been checked.
ControlMode controlMode {MANUAL};
uint DUTY_CYCLE_MIN = 35;
uint DUTY_CYCLE_MAX = 160;      // conservative limit for initial floor tests
float PITCH_ANGLE_ERROR_MAX = 10.f*M_PI/180.f;   // disarm AUTO after a fall begins
float PITCH_ANGLE_ERROR_MIN = 0.2f*M_PI/180.f;   // minimumpitch angle error before motors cut off

// PID variables
float pitch_angle_setpoint = 0; //-3.7*M_PI/180;  // desired pitch angle [rad]
float pitch_angle_current = 0;        // current pitch angle [rad]

// Fixed-size, control-task-owned telemetry recording. The buffer is frozen
// before the I/O core is allowed to stream it over HTTP.
enum class RecordingState : uint8_t {
  EMPTY,
  RECORDING,
  READY,
  DOWNLOADING,
};

typedef struct {
  uint32_t elapsed_us;
  float pitch_rad;
  float gyro_rad_s;
  float pid_output;
  int32_t motor1_encoder_pulses;
  int32_t motor2_encoder_pulses;
  uint32_t control_interval_us;
  uint8_t motor1_pwm;
  uint8_t motor2_pwm;
  uint8_t flags;
  uint8_t reserved;
} RecordingRecord_t;

static_assert(sizeof(RecordingRecord_t) == 32,
              "RecordingRecord_t file format changed");

typedef struct {
  char magic[8];
  uint16_t version;
  uint16_t header_size;
  uint16_t record_size;
  uint16_t sample_rate_hz;
  uint32_t record_count;
  uint32_t capacity;
  float pid_kp;
  float pid_ki;
  float pid_kd;
  float pitch_setpoint_rad;
  float pitch_error_min_rad;
  float pitch_error_max_rad;
  uint8_t duty_cycle_min;
  uint8_t duty_cycle_max;
  uint16_t flags;
  uint32_t reserved[3];
} RecordingFileHeader_t;

static_assert(sizeof(RecordingFileHeader_t) == 64,
              "RecordingFileHeader_t file format changed");

constexpr uint32_t RECORDING_DURATION_SECONDS = 20;
constexpr uint32_t RECORDING_CAPACITY =
    RECORDING_DURATION_SECONDS * ESTIMATOR_FREQ;
RecordingRecord_t recordingBuffer[RECORDING_CAPACITY];
RecordingFileHeader_t recordingHeader {};
std::atomic<RecordingState> recordingState {RecordingState::EMPTY};
std::atomic<uint32_t> recordingCount {0};
int64_t recordingStartTimeUs = 0;
bool recordingSawAuto = false;

// PID controller
PropIntDiff pid(-1.f, 1.f, 1.f);

typedef struct {
  float pidKp;
  float pidKi;
  float pidKd;
  float pitchSetpoint;
  float pitchCurrent;
  uint dutyCycleMin;
  uint dutyCycleMax;
  float pitchErrorMax;
  float pitchErrorMin;
  ControlMode controlMode;
  MotorDirection motor1Direction;
  MotorDirection motor2Direction;
  uint dutyCycle1Manual;
  uint dutyCycle2Manual;
  bool estimatesValid;
  bool autoArmAllowed;
} ControlStatusSnapshot_t;

portMUX_TYPE controlStatusMux = portMUX_INITIALIZER_UNLOCKED;
ControlStatusSnapshot_t controlStatusSnapshot {};



void IRAM_ATTR stateEstimatorTimer(){
  // Give the semaphore to unblock the task
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
}

const char *recordingStateToStr(const RecordingState state) {
  switch (state) {
    case RecordingState::EMPTY: return "EMPTY";
    case RecordingState::RECORDING: return "RECORDING";
    case RecordingState::READY: return "READY";
    case RecordingState::DOWNLOADING: return "DOWNLOADING";
  }
  return "UNKNOWN";
}

bool startRecording() {
  if (recordingState.load(std::memory_order_acquire)
      == RecordingState::DOWNLOADING) {
    return false;
  }

  std::memcpy(recordingHeader.magic, "SBRLOG1", 8);
  recordingHeader.version = 1;
  recordingHeader.header_size = sizeof(RecordingFileHeader_t);
  recordingHeader.record_size = sizeof(RecordingRecord_t);
  recordingHeader.sample_rate_hz = ESTIMATOR_FREQ;
  recordingHeader.record_count = 0;
  recordingHeader.capacity = RECORDING_CAPACITY;
  recordingHeader.pid_kp = pid.kp;
  recordingHeader.pid_ki = pid.ki;
  recordingHeader.pid_kd = pid.kd;
  recordingHeader.pitch_setpoint_rad = pitch_angle_setpoint;
  recordingHeader.pitch_error_min_rad = PITCH_ANGLE_ERROR_MIN;
  recordingHeader.pitch_error_max_rad = PITCH_ANGLE_ERROR_MAX;
  recordingHeader.duty_cycle_min = static_cast<uint8_t>(DUTY_CYCLE_MIN);
  recordingHeader.duty_cycle_max = static_cast<uint8_t>(DUTY_CYCLE_MAX);
  recordingHeader.flags = 0;
  std::memset(recordingHeader.reserved, 0, sizeof(recordingHeader.reserved));

  recordingCount.store(0, std::memory_order_relaxed);
  recordingStartTimeUs = esp_timer_get_time();
  recordingSawAuto = false;
  recordingState.store(RecordingState::RECORDING, std::memory_order_release);
  return true;
}

void stopRecording() {
  if (recordingState.load(std::memory_order_acquire)
      != RecordingState::RECORDING) {
    return;
  }

  recordingHeader.record_count =
      recordingCount.load(std::memory_order_acquire);
  recordingState.store(RecordingState::READY, std::memory_order_release);
}

uint8_t correctMotor2DutyCycle(const uint8_t dutyCycle) {
  // correct motor 2's commanded duty cycle to ensure resultant speed is same as motor 1 when commanded to have the same duty cycle

  const float m1 = 0.0014799539967724955f;   // slope motor 1 graph of speed as a function of duty cycle
  const float b1 = -0.0036162999881075696f;   // intercept motor 1 graph of speed as a function of duty cycle
  const float m2 = 0.0014670524848993663f;   // slope motor 2 graph of speed as a function of duty cycle
  const float b2 = -0.004342195129313476f;  // intercept motor 2 graph of speed as a function of duty cycle

  const float dutyCycleCorrected = ((m1 * dutyCycle + b1) - b2) / m2;

  return static_cast<uint8_t>(std::max(std::min(std::round(dutyCycleCorrected), 255.f), 0.f));
}

// Task to be executed periodically
void taskReadIMURawValues(void * parameter) {
    for(;;) {
        // Wait for the semaphore from the timer ISR
        if(xSemaphoreTake(timerSemaphore, portMAX_DELAY) == pdTRUE) {
            IMURawPacket_t imuRawPacket;
            imu.getMotion6(&(imuRawPacket.ax), &(imuRawPacket.ay), &(imuRawPacket.az), &(imuRawPacket.gx), &(imuRawPacket.gy), &(imuRawPacket.gz));
            imuRawPacket.temp = imu.getTemperature();

            IMUPacket_t imuPacket;

            // invert accelerometer readings to account for IMU mounted upside down
            imuPacket.ax = -imuRawPacket.ax * accel_resolution / 2;   // [m/s^2]
            imuPacket.az = -imuRawPacket.az * accel_resolution / 2;
            imuPacket.gy = imuRawPacket.gy * gyro_resolution / 2  * PI / 180.;    // [rad/s]

            imuPacket.temp = (float)(imuRawPacket.temp / 340.0 + 36.53);  // formula from datasheet

            if (xQueueSend(queueIMU, &imuPacket, portMAX_DELAY) != pdPASS) {
                Serial.println("Failed to send to IMU packet queue");
            }
        }
    }
}

void taskEstimateState(void * parameter) {
    IMUPacket_t imuPacket;
    StateEstimatePacket_t stateEstimatePacket;
    bool gyroOffsetCalculated = false;
    uint numIMUCalibSamples = 0;
    const uint totalIMUCalibSamples = 600;
    float gyroOffsetY = 0;     
    float pitchAccelOffset = 0;                      // [deg/s]
    long motor1EncoderPulsesLastUpdate = 0;       // pulses since last state estimator update
    long motor2EncoderPulsesLastUpdate = 0;

    const uint WHEEL_VELOCITY_ESTIMATOR_TIME_STEPS = 25;  // number of time steps of the estimator period used to calculate wheel velocity over
    uint wheel_velocity_estimator_step_count = WHEEL_VELOCITY_ESTIMATOR_TIME_STEPS;   // keep track of how many steps since last velocity measurement taken
    long motor1EncoderPulsesDelta = 0;
    long motor2EncoderPulsesDelta = 0;
    uint txCount = 0;    // used for keeping track of frequency of transmitting over serial
    const uint TX_PERIOD = 1; //10;
    int64_t previousIMUSampleTimeUs = 0;

    for (;;) {
        // Wait until data is available in the queue
        if (xQueueReceive(queueIMU, &imuPacket, portMAX_DELAY) == pdPASS) {

            const int64_t imuSampleTimeUs = esp_timer_get_time();
            const float elapsedTime = previousIMUSampleTimeUs == 0
                ? 0.0f
                : static_cast<float>(imuSampleTimeUs - previousIMUSampleTimeUs) * 1e-6f;  // [s]
            previousIMUSampleTimeUs = imuSampleTimeUs;

            const float pitchAngleAccelRaw = atan2(imuPacket.ax, imuPacket.az);            // [rad]
            
            if (!gyroOffsetCalculated)
            {
                // calculate mean iteratively over a period
                numIMUCalibSamples ++;
                gyroOffsetY = (imuPacket.gy + (numIMUCalibSamples - 1) * gyroOffsetY) / numIMUCalibSamples;
                pitchAccelOffset = (pitchAngleAccelRaw + (numIMUCalibSamples - 1) * pitchAccelOffset) / numIMUCalibSamples;

                if (numIMUCalibSamples == totalIMUCalibSamples) {
                  gyroOffsetCalculated = true;
                }
            }

            const float pitchAngleAccel = pitchAngleAccelRaw - pitchAccelOffset;

            const float pitchAngularRateGyro = imuPacket.gy - gyroOffsetY;          // [rad/s]
            const float pitchAngularVelocityGyro = -pitchAngularRateGyro;            // [rad/s]
            const float deltaPitchAngleGyro = pitchAngularVelocityGyro * elapsedTime; // [rad]

            pitchAngleGyro += deltaPitchAngleGyro;

            pitchAngleEst = ALPHA * (pitchAngleEst + deltaPitchAngleGyro) + (1-ALPHA) * pitchAngleAccel;   // [rad]

            // calculate angular velocity of each wheel
            wheel_velocity_estimator_step_count--;
            if (wheel_velocity_estimator_step_count == 0)
            {
              motor1EncoderPulsesDelta = motor1EncoderPulses - motor1EncoderPulsesLastUpdate;
              motor2EncoderPulsesDelta = motor2EncoderPulses - motor2EncoderPulsesLastUpdate;
              motor1EncoderPulsesLastUpdate = motor1EncoderPulses;
              motor2EncoderPulsesLastUpdate = motor2EncoderPulses;

              // reset counter
              wheel_velocity_estimator_step_count = WHEEL_VELOCITY_ESTIMATOR_TIME_STEPS;
            }

            // stateEstimatePacket.pitch_accel = pitchAngleAccel;
            // stateEstimatePacket.pitch_gyro = pitchAngleGyro;
            stateEstimatePacket.pitch_est = pitchAngleEst;
            stateEstimatePacket.motor1EncoderPulses = motor1EncoderPulses;
            stateEstimatePacket.motor1EncoderPulsesDelta = motor1EncoderPulsesDelta;
            // stateEstimatePacket.motor1DistanceMeas = motor1EncoderPulses * DISTANCE_PER_PULSE;
            // stateEstimatePacket.motor1DirMeas = static_cast<signed char>(motor1DirMeas);
            stateEstimatePacket.motor2EncoderPulses = motor2EncoderPulses;
            stateEstimatePacket.motor2EncoderPulsesDelta = motor2EncoderPulsesDelta;
            // stateEstimatePacket.motor2DistanceMeas = motor2EncoderPulses * DISTANCE_PER_PULSE;
            // stateEstimatePacket.motor2DirMeas = static_cast<signed char>(motor2DirMeas);
            stateEstimatePacket.pitch_velocity_gyro = pitchAngularVelocityGyro;
            stateEstimatePacket.estimatesValid = gyroOffsetCalculated;  // valid once IMU readings calibrated
            
            txCount ++;
            if (txCount == TX_PERIOD) {
              PacketHeader_t packetHeader;
              packetHeader.packetID = packetID;
              packetHeader.microSecondsSinceBoot = esp_timer_get_time();

              PitchAngleCalcPacket_t pitchAngleCalcPacket;
              pitchAngleCalcPacket.gyroOffsetY = gyroOffsetY;
              pitchAngleCalcPacket.pitchVelocityGyro = pitchAngularRateGyro;
              pitchAngleCalcPacket.isCalibrated = gyroOffsetCalculated;
              pitchAngleCalcPacket.pitchAccelRaw = pitchAngleAccelRaw;
              pitchAngleCalcPacket.pitchAccel = pitchAngleAccel;
              pitchAngleCalcPacket.pitchGyro = pitchAngleGyro;
              pitchAngleCalcPacket.pitchEst = pitchAngleEst;

              // DataPacket_t dataPacket;
              // dataPacket.imu = imuPacket;
              // dataPacket.pitchInfo = pitchAngleCalcPacket;
              // dataPacket.state = stateEstimatePacket;

              // Serial.write(STX);
              // Serial.write( (uint8_t *) &packetHeader, sizeof( packetHeader ) );
              // Serial.write( (uint8_t *) &dataPacket, sizeof( dataPacket ) );
              // Serial.write(ETX);

              // appendFile(SD_MMC, "/data_log.bin", "World!\n");

              txCount = 0;
              // packetID += 1;
            }

            if (xQueueSend(queueStateEstimates, &stateEstimatePacket, portMAX_DELAY) != pdPASS) {
                Serial.println("Failed to send to state estimate queue");
            }
        }
    }
}

PIDControlPacket_t calcPID(const float pitch_angle_current, const float pitch_velocity_gyro) {

  // Compute the PID output
  pid.calculate(pitch_angle_setpoint, pitch_angle_current, pitch_velocity_gyro);

  const float motorPerc = pid.Output;
  const MotorDirection motorDir = motorPerc < 0 ? MotorDirection::FORWARD : MotorDirection::REVERSE;

  const bool pitch_error_exceeded = abs(pitch_angle_current - pitch_angle_setpoint) >= PITCH_ANGLE_ERROR_MAX;
  const bool pitch_error_small = abs(pitch_angle_current - pitch_angle_setpoint) <= PITCH_ANGLE_ERROR_MIN;
  const bool motor_power_too_low = abs(motorPerc) < 0.05;
  const bool disable_motors = pitch_error_exceeded || pitch_error_small || motor_power_too_low;
  const uint8_t dutyCycle = disable_motors ? 0 : static_cast<uint8_t>(static_cast<float>(DUTY_CYCLE_MAX - DUTY_CYCLE_MIN) * abs(motorPerc) + static_cast<float>(DUTY_CYCLE_MIN));

  PIDControlPacket_t pidPacket {
    .pitch_setpoint = pitch_angle_setpoint,
    .pitch_current = pitch_angle_current,
    .pitch_error = pitch_angle_current - pitch_angle_setpoint,
    .motorSpeed = motorPerc,
    .motorDir = motorDir,
    .dutyCycle = dutyCycle
  };

  return pidPacket;
}


MotorOutput_t calcMotorOutput(const MotorDirection motor1dir, 
                              const MotorDirection motor2dir,
                              uint8_t dutyCycle1,
                              uint8_t dutyCycle2,
                              const bool estimatesValid) {
  // receive the motor commands and format them to the physical motors

  // invert direction to specific motors in case wires are switched
  bool motor1dirActual = motor1dir == MotorDirection::FORWARD;
  bool motor2dirActual = motor2dir == MotorDirection::FORWARD;
  if (MOTOR_1_DIR_INVERT) {
    motor1dirActual = !motor1dirActual;
  }
  if (MOTOR_2_DIR_INVERT) {
    motor2dirActual = !motor2dirActual;
  }

  // correct motor2's duty cycle to ensure motors spin at same speed when same duty cycle commanded
  uint8_t dutyCycle2Calibrated = correctMotor2DutyCycle(dutyCycle2);   // TODO: check that this is applied to correct motor

  if (!estimatesValid) {
    dutyCycle1 = 0;
    dutyCycle2Calibrated = 0;
  }

  MotorOutput_t motorOutput {
    .dutyCycle1 = dutyCycle1,
    .dutyCycle2 = dutyCycle2,
    .dutyCycle2Calibrated = dutyCycle2Calibrated,
    .motor1dir = motor1dirActual,
    .motor2dir = motor2dirActual
  };

  return motorOutput;
}

void stopMotors() {
  motor1DirManual = MotorDirection::FORWARD;
  motor2DirManual = MotorDirection::FORWARD;
  dutyCycle1Manual = 0;
  dutyCycle2Manual = 0;

  controlMode = ControlMode::MANUAL;
}

void applyControlCommand(const ControlCommand_t &command,
                         const StateEstimatePacket_t &stateEstimatePacket) {
  switch (command.type) {
    case ControlCommandType::SET_PID_KP:
      pid.kp = command.floatValue;
      break;
    case ControlCommandType::SET_PID_KI:
      pid.ki = command.floatValue;
      break;
    case ControlCommandType::SET_PID_KD:
      pid.kd = command.floatValue;
      break;
    case ControlCommandType::SET_PID_SETPOINT:
      pitch_angle_setpoint = command.floatValue;
      break;
    case ControlCommandType::SET_DUTY_CYCLE_MIN:
      if (command.uintValue <= DUTY_CYCLE_MAX) {
        DUTY_CYCLE_MIN = command.uintValue;
      }
      break;
    case ControlCommandType::SET_DUTY_CYCLE_MAX:
      if (command.uintValue >= DUTY_CYCLE_MIN) {
        DUTY_CYCLE_MAX = command.uintValue;
      }
      break;
    case ControlCommandType::SET_PITCH_ERROR_MAX:
      PITCH_ANGLE_ERROR_MAX = command.floatValue;
      break;
    case ControlCommandType::SET_PITCH_ERROR_MIN:
      PITCH_ANGLE_ERROR_MIN = command.floatValue;
      break;
    case ControlCommandType::SET_CONTROL_MODE:
      if (command.controlMode == ControlMode::MANUAL) {
        stopMotors();
        pid.initialize();
      }
      else if (command.controlMode == ControlMode::AUTO) {
        const float pitchError =
            stateEstimatePacket.pitch_est - pitch_angle_setpoint;
        const bool safeToArm = stateEstimatePacket.estimatesValid
            && abs(pitchError) <= AUTO_ARM_MAX_PITCH_ERROR;

        // Clear latent manual commands whether AUTO arming succeeds or fails.
        stopMotors();
        pid.initialize();
        if (safeToArm) {
          controlMode = ControlMode::AUTO;
        }
      }
      break;
    case ControlCommandType::SET_MOTOR1_DIRECTION:
      motor1DirManual = command.motorDirection;
      break;
    case ControlCommandType::SET_MOTOR2_DIRECTION:
      motor2DirManual = command.motorDirection;
      break;
    case ControlCommandType::SET_MOTOR1_DUTY:
      dutyCycle1Manual = static_cast<uint8_t>(command.uintValue);
      break;
    case ControlCommandType::SET_MOTOR2_DUTY:
      dutyCycle2Manual = static_cast<uint8_t>(command.uintValue);
      break;
    case ControlCommandType::START_RECORDING:
      startRecording();
      break;
    case ControlCommandType::STOP_RECORDING:
      stopRecording();
      break;
  }
}

void publishControlStatus(const StateEstimatePacket_t &stateEstimatePacket) {
  const float pitchError = stateEstimatePacket.pitch_est - pitch_angle_setpoint;
  ControlStatusSnapshot_t snapshot {
    .pidKp = pid.kp,
    .pidKi = pid.ki,
    .pidKd = pid.kd,
    .pitchSetpoint = pitch_angle_setpoint,
    .pitchCurrent = stateEstimatePacket.pitch_est,
    .dutyCycleMin = DUTY_CYCLE_MIN,
    .dutyCycleMax = DUTY_CYCLE_MAX,
    .pitchErrorMax = PITCH_ANGLE_ERROR_MAX,
    .pitchErrorMin = PITCH_ANGLE_ERROR_MIN,
    .controlMode = controlMode,
    .motor1Direction = motor1DirManual,
    .motor2Direction = motor2DirManual,
    .dutyCycle1Manual = dutyCycle1Manual,
    .dutyCycle2Manual = dutyCycle2Manual,
    .estimatesValid = stateEstimatePacket.estimatesValid,
    .autoArmAllowed = stateEstimatePacket.estimatesValid
        && abs(pitchError) <= AUTO_ARM_MAX_PITCH_ERROR,
  };

  portENTER_CRITICAL(&controlStatusMux);
  controlStatusSnapshot = snapshot;
  portEXIT_CRITICAL(&controlStatusMux);
}

ControlStatusSnapshot_t getControlStatus() {
  portENTER_CRITICAL(&controlStatusMux);
  const ControlStatusSnapshot_t snapshot = controlStatusSnapshot;
  portEXIT_CRITICAL(&controlStatusMux);
  return snapshot;
}

void recordControlSample(const int64_t controlTimeUs,
                         const StateEstimatePacket_t &stateEstimatePacket,
                         const ControlPacket_t &controlPacket,
                         const ControlTimingPacket_t &controlTimingPacket,
                         const bool autoWasActiveBeforeSafety) {
  if (recordingState.load(std::memory_order_acquire)
      != RecordingState::RECORDING) {
    return;
  }

  recordingSawAuto = recordingSawAuto
      || autoWasActiveBeforeSafety
      || controlPacket.controlMode == ControlMode::AUTO;

  const uint32_t index = recordingCount.load(std::memory_order_relaxed);
  if (index >= RECORDING_CAPACITY) {
    stopRecording();
    return;
  }

  uint8_t flags = 0;
  if (controlPacket.motorOutput.motor1dir) flags |= 1U << 0;
  if (controlPacket.motorOutput.motor2dir) flags |= 1U << 1;
  flags |= (static_cast<uint8_t>(controlPacket.controlMode) & 0x03U) << 2;
  if (stateEstimatePacket.estimatesValid) flags |= 1U << 4;
  if (controlPacket.motorOutput.motor1dir != MOTOR_1_DIR_INVERT) flags |= 1U << 5;
  if (controlPacket.motorOutput.motor2dir != MOTOR_2_DIR_INVERT) flags |= 1U << 6;

  recordingBuffer[index] = {
    .elapsed_us = static_cast<uint32_t>(controlTimeUs - recordingStartTimeUs),
    .pitch_rad = stateEstimatePacket.pitch_est,
    .gyro_rad_s = stateEstimatePacket.pitch_velocity_gyro,
    .pid_output = controlPacket.controlMode == ControlMode::AUTO
        ? controlPacket.pid.motorSpeed
        : 0.0f,
    .motor1_encoder_pulses = stateEstimatePacket.motor1EncoderPulses,
    .motor2_encoder_pulses = stateEstimatePacket.motor2EncoderPulses,
    .control_interval_us = controlTimingPacket.interval_us,
    .motor1_pwm = controlPacket.motorOutput.dutyCycle1,
    .motor2_pwm = controlPacket.motorOutput.dutyCycle2Calibrated,
    .flags = flags,
    .reserved = 0,
  };

  recordingCount.store(index + 1, std::memory_order_release);

  if (index + 1 >= RECORDING_CAPACITY
      || (recordingSawAuto
          && controlPacket.controlMode != ControlMode::AUTO)) {
    stopRecording();
  }
}

ManualControlPacket_t stepMotors(const uint dutyCycleMin, const uint dutyCycleMax, const uint period, bool changeDir) {
  // period is given in estimator packets

  ManualControlPacket_t p;

  p.dutyCycle1 = dutyCycleMax;

  if (packetID % period < period / 2) {
    p.dutyCycle1 = dutyCycleMin;
  }

  p.dutyCycle2 = p.dutyCycle1;

  // if (changeDir && packetID % period == 0) {
  //   p.motor1dir = !p.motor1dir;
  // }

  // p.motor2dir = p.motor1dir;

  return p;
}

void taskControlMotors(void * parameter) {
  StateEstimatePacket_t stateEstimatePacket;
  ControlPacket_t controlPacket {};
  ControlTimingPacket_t controlTimingPacket {};
  int64_t previousControlTimeUs = 0;
  uint32_t controlTimingSampleCount = 0;
  const uint32_t targetControlIntervalUs = 1000000U / ESTIMATOR_FREQ;
  
  for (;;) {
    // Wait until data is available in the queue
    if (xQueueReceive(queueStateEstimates, &stateEstimatePacket, portMAX_DELAY) == pdPASS) {

      const int64_t controlTimeUs = esp_timer_get_time();
      if (resetControlTimingRequested.exchange(false)) {
        previousControlTimeUs = controlTimeUs;
        controlTimingSampleCount = 0;
        controlTimingPacket = {};
      }
      else if (previousControlTimeUs != 0) {
        const uint32_t controlIntervalUs = static_cast<uint32_t>(controlTimeUs - previousControlTimeUs);
        const uint32_t absJitterUs = controlIntervalUs >= targetControlIntervalUs
            ? controlIntervalUs - targetControlIntervalUs
            : targetControlIntervalUs - controlIntervalUs;

        controlTimingSampleCount++;
        controlTimingPacket.interval_us = controlIntervalUs;
        controlTimingPacket.average_interval_us +=
            (static_cast<float>(controlIntervalUs) - controlTimingPacket.average_interval_us)
            / controlTimingSampleCount;
        controlTimingPacket.average_abs_jitter_us +=
            (static_cast<float>(absJitterUs) - controlTimingPacket.average_abs_jitter_us)
            / controlTimingSampleCount;
        controlTimingPacket.max_abs_jitter_us =
            std::max(controlTimingPacket.max_abs_jitter_us, absJitterUs);
      }
      previousControlTimeUs = controlTimeUs;

      ControlCommand_t command;
      while (xQueueReceive(queueControlCommands, &command, 0) == pdPASS) {
        applyControlCommand(command, stateEstimatePacket);
      }

      const bool autoWasActiveBeforeSafety = controlMode == ControlMode::AUTO;

      // Emergency stop wins over every queued command received in this cycle.
      if (emergencyStopRequested.exchange(false)) {
        stopMotors();
        pid.initialize();
      }

      // A fall or invalid estimate disarms AUTO instead of allowing it to
      // restart unexpectedly when the robot is later moved back upright.
      if (controlMode == ControlMode::AUTO) {
        const float pitchError =
            stateEstimatePacket.pitch_est - pitch_angle_setpoint;
        if (!stateEstimatePacket.estimatesValid
            || abs(pitchError) >= PITCH_ANGLE_ERROR_MAX) {
          stopMotors();
          pid.initialize();
        }
      }

      controlPacket.manual = {
        .dutyCycle1 = dutyCycle1Manual,
        .dutyCycle2 = dutyCycle2Manual,
        .motor1dir = motor1DirManual,
        .motor2dir = motor2DirManual,
      };

      if (controlMode == ControlMode::FUNCTION) {
        // turn the motors on and off with given period
        controlPacket.manual = stepMotors(0, 255, 2000, 0);

        // controlPacket.manual.dutyCycle1 = 100; // TODO: testing
      }

      uint dutyCycle1 = controlPacket.manual.dutyCycle1;
      uint dutyCycle2 = controlPacket.manual.dutyCycle2;
      MotorDirection motor1dir = controlPacket.manual.motor1dir;
      MotorDirection motor2dir = controlPacket.manual.motor2dir;

      if (controlMode == ControlMode::AUTO) {
        controlPacket.pid = calcPID(stateEstimatePacket.pitch_est, stateEstimatePacket.pitch_velocity_gyro);

        dutyCycle1 = controlPacket.pid.dutyCycle;
        dutyCycle2 = controlPacket.pid.dutyCycle;
        motor1dir = controlPacket.pid.motorDir;
        motor2dir = controlPacket.pid.motorDir;
      }

      controlPacket.controlMode = controlMode;

      controlPacket.motorOutput = 
        calcMotorOutput(motor1dir, motor2dir, dutyCycle1, dutyCycle2, stateEstimatePacket.estimatesValid);

      ledcWrite(MOTOR1_PWM_CHANNEL, controlPacket.motorOutput.dutyCycle1);
      ledcWrite(MOTOR2_PWM_CHANNEL, controlPacket.motorOutput.dutyCycle2Calibrated);
      digitalWrite(PIN_MOTOR1_DIR, controlPacket.motorOutput.motor1dir);
      digitalWrite(PIN_MOTOR2_DIR, controlPacket.motorOutput.motor2dir);

      recordControlSample(controlTimeUs, stateEstimatePacket, controlPacket,
                          controlTimingPacket, autoWasActiveBeforeSafety);

      publishControlStatus(stateEstimatePacket);

      if (packetID % TELEMETRY_DECIMATION == 0) {
        TelemetryPacket_t telemetryPacket;
        telemetryPacket.header.packetID = packetID;
        telemetryPacket.header.microSecondsSinceBoot = esp_timer_get_time();
        telemetryPacket.data.state = stateEstimatePacket;
        telemetryPacket.data.control = controlPacket;
        telemetryPacket.data.controlTiming = controlTimingPacket;

        // Never wait for telemetry: replace an unsent snapshot with the latest one.
        xQueueOverwrite(queueTelemetry, &telemetryPacket);
      }

      packetID ++;

    }
  }
}

void taskTransmitTelemetry(void * parameter) {
  TelemetryPacket_t telemetryPacket;
  constexpr size_t telemetryFrameSize = 1 + sizeof(TelemetryPacket_t) + 1;
  uint8_t telemetryFrame[telemetryFrameSize];

  for (;;) {
    if (xQueueReceive(queueTelemetry, &telemetryPacket, portMAX_DELAY) == pdPASS) {
      size_t frameOffset = 0;
      telemetryFrame[frameOffset++] = static_cast<uint8_t>(STX);
      memcpy(telemetryFrame + frameOffset, &telemetryPacket, sizeof(telemetryPacket));
      frameOffset += sizeof(telemetryPacket);
      telemetryFrame[frameOffset] = static_cast<uint8_t>(ETX);

      // Normally this completes in one call. If the USB driver accepts only part
      // of the frame, finish that same frame before taking another queue item.
      size_t bytesWritten = 0;
      while (bytesWritten < telemetryFrameSize) {
        const size_t writeCount = Serial.write(
            telemetryFrame + bytesWritten,
            telemetryFrameSize - bytesWritten);
        if (writeCount == 0) {
          vTaskDelay(pdMS_TO_TICKS(1));
          continue;
        }
        bytesWritten += writeCount;
      }
    }
  }
}

// wheel encoder interrupts

void init_pcnt() {
  pcnt_config_t pcnt_config = {
        // .pulse_gpio_num = PIN_ENCODER1A,
        .ctrl_gpio_num = PCNT_PIN_NOT_USED, //,
        .lctrl_mode = PCNT_MODE_KEEP,  // KEEP, REVERSE, DISABLE, MAX: Control mode when control signal is low
        .hctrl_mode = PCNT_MODE_KEEP,  // KEEP, REVERSE, DISABLE, MAX: Control mode when control signal is high
        .pos_mode = PCNT_COUNT_INC,  // Count up on the positive edge
        .neg_mode = PCNT_COUNT_DIS,  // INC, DIS, KEEP: Do nothing on negative edge.
        .counter_h_lim = 16384,  // Maximum count value
        .counter_l_lim = 0, // Minimum count value
        .unit = PCNT_UNIT,  // PCNT unit
        .channel = PCNT_CHANNEL_0
    };

    // Initialize PCNT unit
    pcnt_unit_config(&pcnt_config);

    // pcnt_chan_config_t chan_a_config = {
    //     .edge_gpio_num = PIN_ENCODER1A, // EXAMPLE_EC11_GPIO_A,
    //     .level_gpio_num = PIN_ENCODER1B // EXAMPLE_EC11_GPIO_B,
    // };
    // pcnt_channel_handle_t pcnt_chan_a = NULL;
    // ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
    // pcnt_chan_config_t chan_b_config = {
    //     .edge_gpio_num = PIN_ENCODER1B, //EXAMPLE_EC11_GPIO_B,
    //     .level_gpio_num = PIN_ENCODER1A //EXAMPLE_EC11_GPIO_A,
    // };
    // pcnt_channel_handle_t pcnt_chan_b = NULL;
    // ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

    // Set the filter value for the pulse input
    pcnt_set_filter_value(PCNT_UNIT, 10);
    pcnt_filter_enable(PCNT_UNIT);

    pcnt_counter_pause(PCNT_UNIT);
    pcnt_counter_clear(PCNT_UNIT);

    // Start counting
    pcnt_counter_resume(PCNT_UNIT);

    delay(10);
}

void IRAM_ATTR handleMotor1EncoderA() {
  // Channel B determines direction whenever channel A changes state.
  const int stateA = digitalRead(PIN_ENCODER1A);
  const int stateB = digitalRead(PIN_ENCODER1B);
  motor1DirMeas = stateA == stateB ? -1 : 1;
  motor1EncoderPulses += motor1DirMeas;
}

// void handleMotor1EncoderB() {
//   // Read the state of channel A
//   int stateA = digitalRead(PIN_ENCODER1A);

//   // Determine the direction
//   if (digitalRead(PIN_ENCODER1B) == HIGH) {
//     motor1DirMeas = (stateA == HIGH) ? 1 : -1; // Forward if A is HIGH, backward if A is LOW
//   } else {
//     motor1DirMeas = (stateA == LOW) ? 1 : -1; // Forward if A is LOW, backward if A is HIGH
//   }

//   // Update pulse count
//   motor1EncoderPulses += motor1DirMeas;
// }

void IRAM_ATTR handleMotor2EncoderA() {
  // Channel B determines direction whenever channel A changes state.
  const int stateA = digitalRead(PIN_ENCODER2A);
  const int stateB = digitalRead(PIN_ENCODER2B);
  motor2DirMeas = stateA == stateB ? -1 : 1;
  motor2EncoderPulses += motor2DirMeas;
}

void handleMotor2EncoderB() {
  // Read the state of channel A
  int stateA = digitalRead(PIN_ENCODER2A);

  // Determine the direction
  if (digitalRead(PIN_ENCODER2B) == HIGH) {
    motor2DirMeas = (stateA == HIGH) ? 1 : -1; // Forward if A is HIGH, backward if A is LOW
  } else {
    motor2DirMeas = (stateA == LOW) ? 1 : -1; // Forward if A is LOW, backward if A is HIGH
  }

  // Update pulse count
  motor2EncoderPulses += motor2DirMeas;
}

void initWiFi() {
  WiFi.mode(WIFI_STA);    //Set Wi-Fi Mode as station
  WiFi.begin(ssid, password);   

  Serial.println("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);

    ledcWrite(PIN_LED_PWM, ledStatus);
    ledStatus = !ledStatus;
  }

  Serial.println(WiFi.localIP());
  Serial.print("RRSI: ");
  Serial.println(WiFi.RSSI());

  ledStatus = true;
  ledcWrite(PIN_LED_PWM, ledStatus);
}

bool convertStringToDouble(const String &value, double &result) {
  char* end;
  result = strtod(value.c_str(), &end);

  // Check if the conversion was successful
  if (*end == '\0') {
    return true;
  }

  return false;
}

bool convertStringToFloat(const String &value, float &result) {
  char* end;
  result = strtof(value.c_str(), &end);

  // Check if the conversion was successful
  if (*end == '\0') {
    return true;
  }

  return false;
}


// Does not currently handle negative values: will return a huge number
bool convertStringToUint(const String &value, uint &result) {
  char* end;
  unsigned long tempResult = strtoul(value.c_str(), &end, 10);

  // Check if there were any non-numeric characters in the string
  if (*end != '\0') {
    Serial.println("Error: The string contains non-numeric characters.");
    return false;
  }

  // Check if the result fits into an unsigned int
  if (tempResult > std::numeric_limits<unsigned int>::max()) {
    Serial.println("Error: The value is too large to fit in an unsigned int.");
    return false;
  }

  result = static_cast<unsigned int>(tempResult);
  return true;
}

// serialisation of boolean variables

String controlModeToStr(const ControlMode controlMode) {
  switch (controlMode) {
    case ControlMode::AUTO:
      return "AUTO";
    case ControlMode::FUNCTION:
      return "FUNCTION";
    case ControlMode::MANUAL:
    default:
      return "MANUAL";
  }
}

ControlMode strToControlMode(String str) {
  return str == "AUTO" ? ControlMode::AUTO : ControlMode::MANUAL;
}

String motorDirToStr(const MotorDirection motorDir) {
  return motorDir == MotorDirection::FORWARD ? "FORWARD" : "REVERSE";
}

MotorDirection strToMotorDir(String str) {
  return str == "FORWARD" ? MotorDirection::FORWARD : MotorDirection::REVERSE;
}

bool enqueueControlCommand(const ControlCommand_t &command) {
  return xQueueSend(queueControlCommands, &command, 0) == pdPASS;
}


void initWebserver() {
    // server.setTimeout(60); // Set timeout to 60 seconds

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    // Create a JSON document
    JsonDocument jsonDoc;
    jsonDoc["message"] = "Hello, world!";
    
    // Serialize JSON document to a string
    String jsonString;
    serializeJson(jsonDoc, jsonString);
    
    // Send JSON response
    request->send(200, "application/json", jsonString);
  });

  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request){
    const ControlStatusSnapshot_t status = getControlStatus();
    JsonDocument jsonDoc;
    jsonDoc["PID_Kp"] = status.pidKp;
    jsonDoc["PID_Ki"] = status.pidKi;
    jsonDoc["PID_Kd"] = status.pidKd;
    jsonDoc["PID_setpoint"] = status.pitchSetpoint;
    jsonDoc["pitch_angle_current"] = status.pitchCurrent;
    jsonDoc["MOTOR_DUTY_CYCLE_MIN"] = status.dutyCycleMin;
    jsonDoc["MOTOR_DUTY_CYCLE_MAX"] = status.dutyCycleMax;
    jsonDoc["PITCH_ANGLE_ERROR_MAX"] = status.pitchErrorMax;
    jsonDoc["PITCH_ANGLE_ERROR_MIN"] = status.pitchErrorMin;
    jsonDoc["CONTROL_MODE"] = controlModeToStr(status.controlMode);
    jsonDoc["MOTOR_1_DIR_MANUAL"] = motorDirToStr(status.motor1Direction);
    jsonDoc["MOTOR_2_DIR_MANUAL"] = motorDirToStr(status.motor2Direction);
    jsonDoc["MOTOR_1_DUTY_CYCLE_MANUAL"] = status.dutyCycle1Manual;
    jsonDoc["MOTOR_2_DUTY_CYCLE_MANUAL"] = status.dutyCycle2Manual;
    jsonDoc["ESTIMATES_VALID"] = status.estimatesValid;
    jsonDoc["AUTO_ARM_ALLOWED"] = status.autoArmAllowed;
    jsonDoc["AUTO_ARM_MAX_PITCH_ERROR"] = AUTO_ARM_MAX_PITCH_ERROR;

    String jsonString;
    serializeJson(jsonDoc, jsonString);

    // Send JSON response
    request->send(200, "application/json", jsonString);
  });

  server.on("/recording/start", HTTP_POST, [](AsyncWebServerRequest *request){
    if (recordingState.load(std::memory_order_acquire)
        == RecordingState::DOWNLOADING) {
      request->send(409, "application/json",
                    "{\"status\":\"error\",\"message\":\"Recording download in progress\"}");
      return;
    }

    ControlCommand_t command {};
    command.type = ControlCommandType::START_RECORDING;
    if (!enqueueControlCommand(command)) {
      request->send(503, "application/json",
                    "{\"status\":\"error\",\"message\":\"Control command queue full\"}");
      return;
    }

    request->send(202, "application/json",
                  "{\"status\":\"success\",\"message\":\"Recording start queued\"}");
  });

  server.on("/recording/stop", HTTP_POST, [](AsyncWebServerRequest *request){
    ControlCommand_t command {};
    command.type = ControlCommandType::STOP_RECORDING;
    if (!enqueueControlCommand(command)) {
      request->send(503, "application/json",
                    "{\"status\":\"error\",\"message\":\"Control command queue full\"}");
      return;
    }

    request->send(202, "application/json",
                  "{\"status\":\"success\",\"message\":\"Recording stop queued\"}");
  });

  server.on("/recording/status", HTTP_GET, [](AsyncWebServerRequest *request){
    const RecordingState state =
        recordingState.load(std::memory_order_acquire);
    const uint32_t count = recordingCount.load(std::memory_order_acquire);

    JsonDocument jsonDoc;
    jsonDoc["state"] = recordingStateToStr(state);
    jsonDoc["record_count"] = count;
    jsonDoc["capacity"] = RECORDING_CAPACITY;
    jsonDoc["sample_rate_hz"] = ESTIMATOR_FREQ;
    jsonDoc["record_size_bytes"] = sizeof(RecordingRecord_t);
    jsonDoc["bytes_available"] =
        sizeof(RecordingFileHeader_t) + count * sizeof(RecordingRecord_t);
    jsonDoc["duration_seconds"] =
        static_cast<float>(count) / static_cast<float>(ESTIMATOR_FREQ);
    jsonDoc["max_duration_seconds"] = RECORDING_DURATION_SECONDS;
    jsonDoc["download_ready"] = state == RecordingState::READY;

    String jsonString;
    serializeJson(jsonDoc, jsonString);
    request->send(200, "application/json", jsonString);
  });

  server.on("/recording/download", HTTP_GET, [](AsyncWebServerRequest *request){
    RecordingState expected = RecordingState::READY;
    if (!recordingState.compare_exchange_strong(
            expected, RecordingState::DOWNLOADING,
            std::memory_order_acq_rel)) {
      request->send(409, "application/json",
                    "{\"status\":\"error\",\"message\":\"No completed recording is ready\"}");
      return;
    }

    const uint32_t count = recordingCount.load(std::memory_order_acquire);
    if (count == 0) {
      recordingState.store(RecordingState::READY, std::memory_order_release);
      request->send(409, "application/json",
                    "{\"status\":\"error\",\"message\":\"Recording is empty\"}");
      return;
    }

    RecordingFileHeader_t header = recordingHeader;
    header.record_count = count;
    const size_t recordsSize = count * sizeof(RecordingRecord_t);
    const size_t totalSize = sizeof(RecordingFileHeader_t) + recordsSize;

    AsyncWebServerResponse *response = request->beginResponse(
        "application/octet-stream", totalSize,
        [header, recordsSize, totalSize](uint8_t *buffer, size_t maxLen,
                                        size_t index) -> size_t {
          if (index >= totalSize) {
            recordingState.store(RecordingState::READY,
                                 std::memory_order_release);
            return 0;
          }

          size_t copied = 0;
          if (index < sizeof(RecordingFileHeader_t)) {
            const size_t headerBytes = std::min(
                maxLen, sizeof(RecordingFileHeader_t) - index);
            std::memcpy(buffer,
                        reinterpret_cast<const uint8_t *>(&header) + index,
                        headerBytes);
            copied += headerBytes;
          }

          const size_t absoluteOffset = index + copied;
          if (copied < maxLen
              && absoluteOffset >= sizeof(RecordingFileHeader_t)) {
            const size_t recordOffset =
                absoluteOffset - sizeof(RecordingFileHeader_t);
            if (recordOffset < recordsSize) {
              const size_t recordBytes = std::min(
                  maxLen - copied, recordsSize - recordOffset);
              std::memcpy(buffer + copied,
                          reinterpret_cast<const uint8_t *>(recordingBuffer)
                              + recordOffset,
                          recordBytes);
              copied += recordBytes;
            }
          }

          if (index + copied >= totalSize) {
            recordingState.store(RecordingState::READY,
                                 std::memory_order_release);
          }
          return copied;
        });
    response->addHeader("Content-Disposition",
                        "attachment; filename=balance-recording.bin");
    response->addHeader("Cache-Control", "no-store");
    request->onDisconnect([](){
      RecordingState downloading = RecordingState::DOWNLOADING;
      recordingState.compare_exchange_strong(
          downloading, RecordingState::READY, std::memory_order_acq_rel);
    });
    request->send(response);
  });

  server.on("/logs", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(410, "application/json",
                  "{\"status\":\"error\",\"message\":\"Use /recording endpoints\"}");
  });

  server.on("/set-value", HTTP_POST, [](AsyncWebServerRequest *request) {
    // std::set<String> keys {"PID_Kp", "PID_Ki", "PID_Kd"}; //, "MOTOR_DUTY_CYCLE_MIN", "MOTOR_DUTY_CYCLE_MAX"};
    String key {""};
    String value {""};

    {
      if (!request->hasParam("key", true)) {
        request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing key parameter\"}");
        return;   // is this needed?
      }
      AsyncWebParameter* p = request->getParam("key", true);
      key = p->value();
    }

    {
      if (!request->hasParam("value", true)) {
        request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing value parameter\"}");
        return;   // is this needed?
      }
      AsyncWebParameter* p = request->getParam("value", true);
      value = p->value();
    }

    JsonDocument jsonDoc;
    jsonDoc["status"] = "success";
    jsonDoc["message"] = "Variable updated";
    jsonDoc["key"] = key;
    bool convertedSuccessfully {false};
    bool commandRequired {false};
    ControlCommand_t command {};

    if (key == "PID_Kp") {
      float temp;
      convertedSuccessfully = convertStringToFloat(value, temp);

      if (convertedSuccessfully) {
        command.type = ControlCommandType::SET_PID_KP;
        command.floatValue = temp;
        commandRequired = true;
        jsonDoc["value"] = temp;
      }
    }
    else if (key == "PID_Ki") {
      float temp;
      convertedSuccessfully = convertStringToFloat(value, temp);

      if (convertedSuccessfully) {
        command.type = ControlCommandType::SET_PID_KI;
        command.floatValue = temp;
        commandRequired = true;
        jsonDoc["value"] = temp;
      }
    }
    else if (key == "PID_Kd") {
      float temp;
      convertedSuccessfully = convertStringToFloat(value, temp);

      if (convertedSuccessfully) {
        command.type = ControlCommandType::SET_PID_KD;
        command.floatValue = temp;
        commandRequired = true;
        jsonDoc["value"] = temp;
      }
    }
    else if (key == "PID_setpoint") {
      float temp;
      convertedSuccessfully = convertStringToFloat(value, temp)
          && (temp >= -25.0f * M_PI / 180.0f)
          && (temp <= 25.0f * M_PI / 180.0f);

      if (convertedSuccessfully) {
        command.type = ControlCommandType::SET_PID_SETPOINT;
        command.floatValue = temp;
        commandRequired = true;
        jsonDoc["value"] = temp;
      }
    }
    else if (key == "MOTOR_DUTY_CYCLE_MIN") {
      uint temp;
      convertedSuccessfully = convertStringToUint(value, temp) && (temp >= 0) && (temp <= 255);

      if (convertedSuccessfully) {
        command.type = ControlCommandType::SET_DUTY_CYCLE_MIN;
        command.uintValue = temp;
        commandRequired = true;
        jsonDoc["value"] = temp;
      }
    }
    else if (key == "MOTOR_DUTY_CYCLE_MAX") {
      uint temp;
      convertedSuccessfully = convertStringToUint(value, temp) && (temp >= 0) && (temp <= 255);

      if (convertedSuccessfully) {
        command.type = ControlCommandType::SET_DUTY_CYCLE_MAX;
        command.uintValue = temp;
        commandRequired = true;
        jsonDoc["value"] = temp;
      }
    }
    else if (key == "PITCH_ANGLE_ERROR_MAX") {
      float temp;
      convertedSuccessfully = convertStringToFloat(value, temp)
          && (temp >= 5.0f * M_PI / 180.0f)
          && (temp <= 35.0f * M_PI / 180.0f);

      if (convertedSuccessfully) {
        command.type = ControlCommandType::SET_PITCH_ERROR_MAX;
        command.floatValue = temp;
        commandRequired = true;
        jsonDoc["value"] = temp;
      }
    }
    else if (key == "PITCH_ANGLE_ERROR_MIN") {
      float temp;
      convertedSuccessfully = convertStringToFloat(value, temp)
          && (temp >= 0.0f)
          && (temp <= 45.0f * M_PI / 180.0f);

      if (convertedSuccessfully) {
        command.type = ControlCommandType::SET_PITCH_ERROR_MIN;
        command.floatValue = temp;
        commandRequired = true;
        jsonDoc["value"] = temp;
      }
    }
    else if (key == "CONTROL_MODE") {
      if (value == "AUTO") {
        convertedSuccessfully = true;
        command.type = ControlCommandType::SET_CONTROL_MODE;
        command.controlMode = ControlMode::AUTO;
        commandRequired = true;
        jsonDoc["value"] = "AUTO";
        jsonDoc["message"] = "AUTO requested; confirm active mode in telemetry or /status";
      }
      else if (value == "MANUAL") {
        convertedSuccessfully = true;
        command.type = ControlCommandType::SET_CONTROL_MODE;
        command.controlMode = ControlMode::MANUAL;
        commandRequired = true;
        jsonDoc["value"] = "MANUAL";
      }
    }
    else if (key == "MOTOR_1_DIR_MANUAL") {
      if (value == "FORWARD") {
        convertedSuccessfully = true;
        command.type = ControlCommandType::SET_MOTOR1_DIRECTION;
        command.motorDirection = MotorDirection::FORWARD;
        commandRequired = true;
        jsonDoc["value"] = "FORWARD";
      }
      else if (value == "REVERSE") {
        convertedSuccessfully = true;
        command.type = ControlCommandType::SET_MOTOR1_DIRECTION;
        command.motorDirection = MotorDirection::REVERSE;
        commandRequired = true;
        jsonDoc["value"] = "REVERSE";
      }
    }
    else if (key == "MOTOR_2_DIR_MANUAL") {
      if (value == "FORWARD") {
        convertedSuccessfully = true;
        command.type = ControlCommandType::SET_MOTOR2_DIRECTION;
        command.motorDirection = MotorDirection::FORWARD;
        commandRequired = true;
        jsonDoc["value"] = "FORWARD";
      }
      else if (value == "REVERSE") {
        convertedSuccessfully = true;
        command.type = ControlCommandType::SET_MOTOR2_DIRECTION;
        command.motorDirection = MotorDirection::REVERSE;
        commandRequired = true;
        jsonDoc["value"] = "REVERSE";
      }
    }
    else if (key == "MOTOR_1_DUTY_CYCLE_MANUAL") {
      uint temp;
      convertedSuccessfully = convertStringToUint(value, temp) && (temp >= 0) && (temp <= 255);

      if (convertedSuccessfully) {
        command.type = ControlCommandType::SET_MOTOR1_DUTY;
        command.uintValue = temp;
        commandRequired = true;
        jsonDoc["value"] = temp;
      }
    }
    else if (key == "MOTOR_2_DUTY_CYCLE_MANUAL") {
      uint temp;
      convertedSuccessfully = convertStringToUint(value, temp) && (temp >= 0) && (temp <= 255);

      if (convertedSuccessfully) {
        command.type = ControlCommandType::SET_MOTOR2_DUTY;
        command.uintValue = temp;
        commandRequired = true;
        jsonDoc["value"] = temp;
      }
    }
    else if (key == "EMERGENCY_STOP") {
      emergencyStopRequested.store(true);
      convertedSuccessfully = true;
      jsonDoc["value"] = true;
      jsonDoc["message"] = "Emergency stop requested";
    }
    else if (key == "START_LOGGING") {
      convertedSuccessfully = true;
      command.type = ControlCommandType::START_RECORDING;
      commandRequired = true;
      jsonDoc["value"] = "queued";
    }
    else if (key == "STOP_LOGGING") {
      convertedSuccessfully = true;
      command.type = ControlCommandType::STOP_RECORDING;
      commandRequired = true;
      jsonDoc["value"] = "queued";
    }
    else {
      request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Key invalid\"}");
      return;   // is this needed?
    }

    if (!convertedSuccessfully) {
      request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid value type\"}");
      return;   // is this needed?
    }

    if (commandRequired && !enqueueControlCommand(command)) {
      request->send(503, "application/json", "{\"status\":\"error\",\"message\":\"Control command queue full\"}");
      return;
    }

    // Serialize JSON document to a string
    String jsonString;
    serializeJson(jsonDoc, jsonString);

    // Send JSON response
    request->send(200, "application/json", jsonString);
    
  });

  // Start the server
  server.begin();
}

// void sdCardSetup() {
//   SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0);
//     if (!SD_MMC.begin("/sdcard", true, true, SDMMC_FREQ_DEFAULT, 5)) {
//       Serial.println("Card Mount Failed");
//       return;
//     }
//     uint8_t cardType = SD_MMC.cardType();
//     if(cardType == CARD_NONE){
//         Serial.println("No SD_MMC card attached");
//         return;
//     }

//     Serial.print("SD_MMC Card Type: ");
//     if(cardType == CARD_MMC){
//         Serial.println("MMC");
//     } else if(cardType == CARD_SD){
//         Serial.println("SDSC");
//     } else if(cardType == CARD_SDHC){
//         Serial.println("SDHC");
//     } else {
//         Serial.println("UNKNOWN");
//     }

//     uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
//     Serial.printf("SD_MMC Card Size: %lluMB\n", cardSize);
// }
 
void setup(){
  Serial.begin(115200);

  Wire.setPins(PIN_I2C_SDA, PIN_I2C_SCL); // Set the I2C pins before begin

  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      // Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties. - Shaun: Not sure whether I need this?
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  imu.initialize();
  accel_resolution = imu.get_acce_resolution();
  gyro_resolution = imu.get_gyro_resolution();

  // Create the semaphore
  timerSemaphore = xSemaphoreCreateBinary();

  queueIMU = xQueueCreate(10, sizeof(IMUPacket_t));
  if (queueIMU == NULL) {
      Serial.println("Failed to create queue");
      while (1);
  }
  queueStateEstimates = xQueueCreate(10, sizeof(StateEstimatePacket_t));
  if (queueStateEstimates == NULL) {
      Serial.println("Failed to create queue");
      while (1);
  }
  queueTelemetry = xQueueCreate(1, sizeof(TelemetryPacket_t));
  if (queueTelemetry == NULL) {
      Serial.println("Failed to create telemetry queue");
      while (1);
  }
  queueControlCommands = xQueueCreate(16, sizeof(ControlCommand_t));
  if (queueControlCommands == NULL) {
      Serial.println("Failed to create control command queue");
      while (1);
  }

  // Create the task that will be executed periodically
  xTaskCreatePinnedToCore(
      taskReadIMURawValues, "Read Raw IMU Values", 10000, NULL,
      CONTROL_PIPELINE_PRIORITY, NULL, CONTROL_CORE);
  xTaskCreatePinnedToCore(
      taskEstimateState, "Estimate State", 2048, NULL,
      CONTROL_PIPELINE_PRIORITY, NULL, CONTROL_CORE);
  xTaskCreatePinnedToCore(
      taskControlMotors, "Control Motors", 2048, NULL,
      MOTOR_CONTROL_PRIORITY, NULL, CONTROL_CORE);
  xTaskCreatePinnedToCore(
      taskTransmitTelemetry, "Transmit Telemetry", 2048, NULL,
      IO_TASK_PRIORITY, NULL, IO_CORE);

  hwTimer = timerBegin(/* timer num */ 0, /* clock divider */ 80, /* count up */true);
  timerAttachInterrupt(hwTimer, &stateEstimatorTimer, /* edge */ true);
  const uint64_t ALARM_PERIOD = 1e6 / ESTIMATOR_FREQ;               // (1 million / 250 Hz = 4000)
  timerAlarmWrite(hwTimer, ALARM_PERIOD, /* periodic */true);
  timerAlarmEnable(hwTimer);

  // verify connection
  // Serial.println("Testing device connections...");
  // Serial.println(imu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  pinMode(PIN_MOTOR1_SLEEP, OUTPUT);
  pinMode(PIN_MOTOR2_SLEEP, OUTPUT);
  pinMode(PIN_MOTOR1_DIR, OUTPUT);
  pinMode(PIN_MOTOR2_DIR, OUTPUT);
  pinMode(PIN_LED_PWM, OUTPUT);

  // enable motors
  digitalWrite(PIN_MOTOR1_SLEEP, !MOTOR_COAST);
  digitalWrite(PIN_MOTOR2_SLEEP, !MOTOR_COAST);

  // is ledc the best way to do PWM on the ESP32? Should we not use MCPWM, since that is for motor control?
  ledcSetup(MOTOR1_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);  // define the PWM Setup
  ledcSetup(MOTOR2_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PIN_MOTOR1_PWM, MOTOR1_PWM_CHANNEL);
  ledcAttachPin(PIN_MOTOR2_PWM, MOTOR2_PWM_CHANNEL);
//   ledcAttachPin(PIN_LED_PWM, MOTOR1_PWM_CHANNEL);

  // Count both edges of channel A and read channel B to determine direction.
  // ENCODER_PULSES_PER_REVOLUTION is defined for this 2x decoding scheme.
  pinMode(PIN_ENCODER1A, INPUT);
  pinMode(PIN_ENCODER1B, INPUT);
  pinMode(PIN_ENCODER2A, INPUT);
  pinMode(PIN_ENCODER2B, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER1A), handleMotor1EncoderA, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(PIN_ENCODER1B), handleMotor1EncoderB, CHANGE);   // use only half the possible pules for now, since angular resolution should be sufficient
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER2A), handleMotor2EncoderA, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(PIN_ENCODER2B), handleMotor2EncoderB, CHANGE);

  // PID constants
  pid.kp = 18.0;
  pid.ki = 0.0;
  pid.kd = 0.4;
  pid.inAuto = true;

  initWiFi();
  initWebserver();
  resetControlTimingRequested.store(true);
}
 
// void loop(){
//     digitalWrite(PIN_MOTOR1_SLEEP, HIGH);     // inveted, so HIGH should be not sleeping/coasting?
//     digitalWrite(PIN_MOTOR1_DIR, dirDrive);

//     // ledcWrite(MOTOR1_PWM_CHANNEL, 0);        // set the Duty cycle out of 255
//     // delay(1000);
//     //ledcWrite(MOTOR1_PWM_CHANNEL, 50);
//     // delay(1000);
//     // ledcWrite(MOTOR1_PWM_CHANNEL, 80);
//     // delay(1000);

//     // dirDrive = !dirDrive;

//     int16_t ax, ay, az;
// int16_t gx, gy, gz;

//     IMUPacket_t imuPacket;

//     // imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

//     imu.getMotion6(&(imuPacket.ax), &(imuPacket.ay), &(imuPacket.az), &(imuPacket.gx), &(imuPacket.gy), &(imuPacket.gz));
//     // imuPacket.gz = 246;
//     //Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF)); Serial.write((uint8_t)('\r')); Serial.write((uint8_t)('\n'));
//     Serial.write(STX);
//     Serial.write( (uint8_t *) &imuPacket, sizeof( imuPacket ) );
//     Serial.write(ETX);

//     // Serial.print("a/g:\t");
//     // Serial.print("a:\t");
//     //     Serial.print(ax); Serial.print("\t");
//     //     Serial.print(ay); Serial.print("\t");
//     //     Serial.print(az); Serial.print("\t");
//         // Serial.print(gx); Serial.print("\t");
//         // Serial.print(gy); Serial.print("\t");
//         // Serial.println(gz);
//         // Serial.print("\n");
//     delay(50);
// }



void loop() {

  // loopTask is pinned to the control core by the Arduino framework. Yield it
  // while all application work is performed by the dedicated tasks above.
  delay(1000);

  // digitalWrite(PIN_MOTOR1_DIR, true);
  // digitalWrite(PIN_MOTOR2_DIR, false);

  // // ledcWrite(MOTOR1_PWM_CHANNEL, 50);        // set the Duty cycle out of 255
  // auto  pwm = 100; //50*1;
  // uint pwmCalib = correctMotor2DutyCycle(pwm);
  // ledcWrite(MOTOR1_PWM_CHANNEL, pwm); //pwmCalib);
  // ledcWrite(MOTOR2_PWM_CHANNEL, pwm);
  // delay(2000);

  // ledcWrite(MOTOR2_PWM_CHANNEL, 0);
  // ledcWrite(MOTOR1_PWM_CHANNEL, 0);

  // delay(2000);

  // digitalWrite(PIN_MOTOR1_DIR, false);
  // digitalWrite(PIN_MOTOR2_DIR, false);
  // ledcWrite(MOTOR2_PWM_CHANNEL, 23);
  // ledcWrite(MOTOR1_PWM_CHANNEL, 23);

  // delay(2000);

  // ledcWrite(MOTOR2_PWM_CHANNEL, 0);
  // ledcWrite(MOTOR1_PWM_CHANNEL, 0);

  // delay(2000);

}
