#include <iostream>
#include <string>
#include <algorithm>
#include <cstdint>

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <Base64.h>
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
const int PIN_ENCODER2A = 16;
const int PIN_ENCODER2B = 15;
const int PIN_ENCODER1A = 11;
const int PIN_ENCODER1B = 12;
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

const uint TELEMETRY_DECIMATION = 5;  // transmit at 20 Hz when the controller runs at 100 Hz

typedef struct {
  PacketHeader_t header;
  DataPacket_t data;
} TelemetryPacket_t;

MotorDirection dirDrive { MotorDirection::FORWARD };
MotorDirection motor1DirManual { MotorDirection::FORWARD }; // motor 1 direction when in manual mode
MotorDirection motor2DirManual { MotorDirection::FORWARD };
uint dutyCycle1Manual {0};                // motor 1 duty cycle when in manual mode
uint dutyCycle2Manual {0};

// ControlMode controlMode {MANUAL}; //AUTO};
// ControlMode controlMode {FUNCTION}; //AUTO};
ControlMode controlMode {AUTO};
uint DUTY_CYCLE_MIN = 15;
uint DUTY_CYCLE_MAX = 253;      // conservative for now. Can be as high as 255
float PITCH_ANGLE_ERROR_MAX = 25.f*M_PI/180.f;   // maximum pitch angle error before motors cut off
float PITCH_ANGLE_ERROR_MIN = 0.2f*M_PI/180.f;   // minimumpitch angle error before motors cut off

// PID variables
float pitch_angle_setpoint = 0; //-3.7*M_PI/180;  // desired pitch angle [rad]
float pitch_angle_current = 0;        // current pitch angle [rad]

// for logging data and sending to base station afterwards
typedef struct {
    float pitch_current;
    float pitch_gyro;

    float motorSpeed;
    int motorDir1;
    uint dutyCycle1;
    // uint dutyCycle2;

} LogPacket_t;

const long NUM_LOG_PACKETS = 30 * ESTIMATOR_FREQ; // log duration [s] * packets/sec
LogPacket_t logPackets[NUM_LOG_PACKETS];
bool enableLogging = false;
uint logIndex = 0;
// DataLogger dataLogger();

// PID controller
PropIntDiff pid(-1.f, 1.f, 1.f);



void IRAM_ATTR stateEstimatorTimer(){
  // Give the semaphore to unblock the task
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
}

void startLogging() {
  enableLogging = true;
  logIndex = 0;
}

void stopLogging() {
  enableLogging = false;
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

            pcnt_get_counter_value(PCNT_UNIT, &pcnt_encoder_pulse_count);

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
            stateEstimatePacket.motor1EncoderPulses = pcnt_encoder_pulse_count; //motor1EncoderPulses;
            stateEstimatePacket.motor1EncoderPulsesDelta = 0; //motor1EncoderPulsesDelta;
            // stateEstimatePacket.motor1DistanceMeas = motor1EncoderPulses * DISTANCE_PER_PULSE;
            // stateEstimatePacket.motor1DirMeas = static_cast<signed char>(motor1DirMeas);
            stateEstimatePacket.motor2EncoderPulses = 0; //motor2EncoderPulses;
            stateEstimatePacket.motor2EncoderPulsesDelta = 0; //motor2EncoderPulsesDelta;
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
      if (previousControlTimeUs != 0) {
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

      // log data to RAM
      if (enableLogging) {
        if (logIndex == NUM_LOG_PACKETS) {
          stopLogging();
        }
        // logPackets[logIndex].pitch_current = pitch_angle_current;
        // logPackets[logIndex].pitch_gyro = pitchAngleGyro;
        // logPackets[logIndex].dutyCycle1 = dutyCycle1;
        // logPackets[logIndex].motorDir1 = motor1dirActual;
        // logPackets[logIndex].motorSpeed = pid.Output;

        logIndex ++;
      }
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

// void handleMotor1EncoderA() {
//   // Read the state of channel B
//   int stateB = digitalRead(PIN_ENCODER1B);

//   // Determine the direction
//   if (digitalRead(PIN_ENCODER1A) == HIGH) {
//     motor1DirMeas = (stateB == LOW) ? 1 : -1; // Forward if B is LOW, backward if B is HIGH
//   } else {
//     motor1DirMeas = (stateB == HIGH) ? 1 : -1; // Forward if B is HIGH, backward if B is LOW
//   }

//   // Update pulse count
//   motor1EncoderPulses += motor1DirMeas;

// }

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

void handleMotor2EncoderA() {
  // Read the state of channel B
  int stateB = digitalRead(PIN_ENCODER2B);

  // Determine the direction
  if (digitalRead(PIN_ENCODER2A) == HIGH) {
    motor2DirMeas = (stateB == LOW) ? 1 : -1; // Forward if B is LOW, backward if B is HIGH
  } else {
    motor2DirMeas = (stateB == HIGH) ? 1 : -1; // Forward if B is HIGH, backward if B is LOW
  }

  // Update pulse count
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
  return controlMode == ControlMode::AUTO ? "AUTO" : "MANUAL";
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
    JsonDocument jsonDoc;
    jsonDoc["PID_Kp"] = pid.kp;
    jsonDoc["PID_Ki"] = pid.ki;
    jsonDoc["PID_Kd"] = pid.kd;
    jsonDoc["PID_setpoint"] = pitch_angle_setpoint;
    jsonDoc["pitch_angle_current"] = pitch_angle_current;
    jsonDoc["MOTOR_DUTY_CYCLE_MIN"] = DUTY_CYCLE_MIN;
    jsonDoc["MOTOR_DUTY_CYCLE_MAX"] = DUTY_CYCLE_MAX;
    jsonDoc["PITCH_ANGLE_ERROR_MAX"] = PITCH_ANGLE_ERROR_MAX;
    jsonDoc["PITCH_ANGLE_ERROR_MIN"] = PITCH_ANGLE_ERROR_MIN;
    jsonDoc["CONTROL_MODE"] = controlModeToStr(controlMode);
    jsonDoc["MOTOR_1_DIR_MANUAL"] = motorDirToStr(motor1DirManual);
    jsonDoc["MOTOR_2_DIR_MANUAL"] = motorDirToStr(motor2DirManual);
    jsonDoc["MOTOR_1_DUTY_CYCLE_MANUAL"] = dutyCycle1Manual;
    jsonDoc["MOTOR_2_DUTY_CYCLE_MANUAL"] = dutyCycle2Manual;

    String jsonString;
    serializeJson(jsonDoc, jsonString);

    // Send JSON response
    request->send(200, "application/json", jsonString);
  });

  server.on("/logs", HTTP_GET, [](AsyncWebServerRequest *request){
    // stop logging before getting the data
    stopLogging();



    // Calculate the size of the byte array
    const size_t numPackets = sizeof(logPackets) / sizeof(LogPacket_t);
    const size_t packetSize = sizeof(LogPacket_t);
    const size_t byteArraySize = numPackets * packetSize;

    // Convert the array of structs to a byte array
    // uint8_t byteArray[byteArraySize];
    char byteArray[byteArraySize];
    memcpy(byteArray, logPackets, byteArraySize);

    // Encode the byte array to a base64 string
    char base64Str[Base64.encodedLength(byteArraySize)];
    Base64.encode(base64Str, byteArray, byteArraySize);

    // Create a JSON document
    JsonDocument jsonDoc;
    jsonDoc["data"] = base64Str;
    jsonDoc["num_packets_logged"] = logIndex;
    jsonDoc["num_bytes_logged"] = sizeof(logPackets);

    // Convert JSON document to string
    String jsonString;
    serializeJson(jsonDoc, jsonString);





    // JsonDocument jsonDoc;
    // (uint8_t *) &microSecondsSinceBoot, sizeof( microSecondsSinceBoot )
    // jsonDoc["data"] = JsonString{(char *) &logPackets, sizeof(logPackets)};
    // jsonDoc["num_packets_logged"] = logIndex;
    // jsonDoc["num_bytes_logged"] = sizeof(logPackets);

    // String jsonString;
    // serializeJson(jsonDoc, jsonString);

    // Send JSON response
    request->send(200, "application/json", jsonString);
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

    if (key == "PID_Kp") {
      float temp;
      convertedSuccessfully = convertStringToFloat(value, temp);

      if (convertedSuccessfully) {
        pid.kp = temp;
        jsonDoc["value"] = pid.kp;
      }
    }
    else if (key == "PID_Ki") {
      float temp;
      convertedSuccessfully = convertStringToFloat(value, temp);

      if (convertedSuccessfully) {
        pid.ki = temp;
        jsonDoc["value"] = pid.ki;
      }
    }
    else if (key == "PID_Kd") {
      float temp;
      convertedSuccessfully = convertStringToFloat(value, temp);

      if (convertedSuccessfully) {
        pid.kd = temp;
        jsonDoc["value"] = pid.kd;
      }
    }
    else if (key == "PID_setpoint") {
      float temp;
      convertedSuccessfully = convertStringToFloat(value, temp) && (temp >= -25.) && (temp <= 25.);

      if (convertedSuccessfully) {
        pitch_angle_setpoint = temp;
        jsonDoc["value"] = pitch_angle_setpoint;
      }
    }
    else if (key == "MOTOR_DUTY_CYCLE_MIN") {
      uint temp;
      convertedSuccessfully = convertStringToUint(value, temp) && (temp >= 0) && (temp <= 255);

      if (convertedSuccessfully) {
        DUTY_CYCLE_MIN = temp;
        jsonDoc["value"] = DUTY_CYCLE_MIN;
      }
    }
    else if (key == "MOTOR_DUTY_CYCLE_MAX") {
      uint temp;
      convertedSuccessfully = convertStringToUint(value, temp) && (temp >= 0) && (temp <= 255);

      if (convertedSuccessfully) {
        DUTY_CYCLE_MAX = temp;
        jsonDoc["value"] = DUTY_CYCLE_MAX;
      }
    }
    else if (key == "PITCH_ANGLE_ERROR_MAX") {
      float temp;
      convertedSuccessfully = convertStringToFloat(value, temp) && (temp >= 5.) && (temp <= 35.);

      if (convertedSuccessfully) {
        PITCH_ANGLE_ERROR_MAX = temp;
        jsonDoc["value"] = PITCH_ANGLE_ERROR_MAX;
      }
    }
    else if (key == "PITCH_ANGLE_ERROR_MIN") {
      float temp;
      convertedSuccessfully = convertStringToFloat(value, temp) && (temp >= 0) && (temp <= 45.);

      if (convertedSuccessfully) {
        PITCH_ANGLE_ERROR_MIN = temp;
        jsonDoc["value"] = PITCH_ANGLE_ERROR_MIN;
      }
    }
    else if (key == "CONTROL_MODE") {
      if (value == "AUTO") {
        controlMode = ControlMode::AUTO;
        convertedSuccessfully = true;
        jsonDoc["value"] = controlMode;
      }
      else if (value == "MANUAL") {
        motor1DirManual = MotorDirection::FORWARD;
        motor2DirManual = MotorDirection::FORWARD;
        dutyCycle1Manual = 0;
        dutyCycle2Manual = 0;

        controlMode = ControlMode::MANUAL;
        convertedSuccessfully = true;
        jsonDoc["value"] = controlModeToStr(controlMode);
      }
    }
    else if (key == "MOTOR_1_DIR_MANUAL") {
      if (value == "FORWARD") {
        motor1DirManual = MotorDirection::FORWARD;
        convertedSuccessfully = true;
        jsonDoc["value"] = motorDirToStr(motor1DirManual);
      }
      else if (value == "REVERSE") {
        motor1DirManual = MotorDirection::REVERSE;
        convertedSuccessfully = true;
        jsonDoc["value"] = motorDirToStr(motor1DirManual);
      }
    }
    else if (key == "MOTOR_2_DIR_MANUAL") {
      if (value == "FORWARD") {
        motor2DirManual = MotorDirection::FORWARD;
        convertedSuccessfully = true;
        jsonDoc["value"] = motorDirToStr(motor2DirManual);
      }
      else if (value == "REVERSE") {
        motor2DirManual = MotorDirection::REVERSE;
        convertedSuccessfully = true;
        jsonDoc["value"] = motorDirToStr(motor2DirManual);
      }
    }
    else if (key == "MOTOR_1_DUTY_CYCLE_MANUAL") {
      uint temp;
      convertedSuccessfully = convertStringToUint(value, temp) && (temp >= 0) && (temp <= 255);

      if (convertedSuccessfully) {
        dutyCycle1Manual = temp;
        jsonDoc["value"] = dutyCycle1Manual;
      }
    }
    else if (key == "MOTOR_2_DUTY_CYCLE_MANUAL") {
      uint temp;
      convertedSuccessfully = convertStringToUint(value, temp) && (temp >= 0) && (temp <= 255);

      if (convertedSuccessfully) {
        dutyCycle2Manual = temp;
        jsonDoc["value"] = dutyCycle2Manual;
      }
    }
    else if (key == "EMERGENCY_STOP") {
      stopMotors();
      convertedSuccessfully = true;
      jsonDoc["value"] = "";
    }
    else if (key == "START_LOGGING") {
      startLogging();
      convertedSuccessfully = true;
      jsonDoc["value"] = enableLogging;
    }
    else if (key == "STOP_LOGGING") {
      stopLogging();
      convertedSuccessfully = true;
      jsonDoc["value"] = enableLogging;
    }
    else {
      request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Key invalid\"}");
      return;   // is this needed?
    }

    if (!convertedSuccessfully) {
      request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid value type\"}");
      return;   // is this needed?
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

  // Create the task that will be executed periodically
  xTaskCreate(taskReadIMURawValues, "Read Raw IMU Values", 10000, NULL, 1, NULL);
  xTaskCreate(taskEstimateState, "Estimate State", 2048, NULL, 1, NULL);
  xTaskCreate(taskControlMotors, "Control Motors", 2048, NULL, 2, NULL);
  xTaskCreate(taskTransmitTelemetry, "Transmit Telemetry", 2048, NULL, 1, NULL);

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

  // Set encoder pins as inputs
  init_pcnt();
  // pinMode(PIN_ENCODER1A, INPUT);
  // pinMode(PIN_ENCODER1B, INPUT);
  pinMode(PIN_ENCODER2A, INPUT);
  pinMode(PIN_ENCODER2B, INPUT);
  // attachInterrupt(digitalPinToInterrupt(PIN_ENCODER1A), handleMotor1EncoderA, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(PIN_ENCODER1B), handleMotor1EncoderB, CHANGE);   // use only half the possible pules for now, since angular resolution should be sufficient
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER2A), handleMotor2EncoderA, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(PIN_ENCODER2B), handleMotor2EncoderB, CHANGE);

  // PID constants
  pid.kp = 18.0;
  pid.ki = 0.0;
  pid.kd = 0.4;
  pid.inAuto = true;

  // initialise storage file
  // sdCardSetup();
  // deleteFile(SD_MMC, "/data_log.bin");

  // logPackets[0].pitch_current = 7.f;    // TODO: testing

  for (uint i = 0; i < NUM_LOG_PACKETS; i++) {    // TODO: testing
    logPackets[i].pitch_current = 7.f;
    logPackets[i].pitch_gyro = 8.f;
    logPackets[i].motorSpeed = 9.f;
    logPackets[i].motorDir1 = 2;
    logPackets[i].dutyCycle1 = 3;
  }

  // initWiFi();
  // initWebserver();
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
