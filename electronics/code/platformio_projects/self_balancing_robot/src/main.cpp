#include <iostream>
#include <string>

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include "secrets.h"
#include "pid.h"

MPU6050 imu;
AsyncWebServer server(80);

const int PIN_LED_PWM = 2;
const int PIN_MOTOR1_SLEEP = 42;
const int PIN_MOTOR2_SLEEP = 41;
const int PIN_MOTOR1_DIR = 40;
const int PIN_MOTOR2_DIR = 39;
const int PIN_ENCODER2A = 15;
const int PIN_ENCODER2B = 16;
const int PIN_ENCODER1A = 11;
const int PIN_ENCODER1B = 12;
const int PIN_MOTOR1_PWM = 35;
const int PIN_MOTOR2_PWM = 45;
const int PIN_I2C_SDA = 14;
const int PIN_I2C_SCL = 13;

const int PWM_FREQ = 30000;         // frequency to run PWM at [Hz]
const int MOTOR1_PWM_CHANNEL = 0;   // set the PWM channel
const int MOTOR2_PWM_CHANNEL = 1;   // set the PWM channel
const int PWM_RESOLUTION = 8;       // set PWM resolution

const bool MOTOR_1_DIR_INVERT = false;
const bool MOTOR_2_DIR_INVERT = true;
const bool MOTOR_COAST = false;
const uint ENCODER_PULSES_PER_REVOLUTION = 464;

bool ledStatus = true;

// serial tx control characters
const char STX = '!'; //'\x002';   // start of frame
const char ETX = '@'; //'\x003';   // end of frame

// state estimation
volatile long motor1EncoderPulses = 0;
volatile int motor1DirMeas = 0;   // 0: stopped, 1: forward, -1: backward
volatile long motor2EncoderPulses = 0;
volatile int motor2DirMeas = 0;

long long packetID = 0;

typedef struct {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
} IMUPacket_t;

typedef struct {
  // IMU estimates
  float pitch_accel;
  float pitch_gyro;
  float pitch_est;

  // wheel encoder measurements
  // float motor1DistanceMeas;
  long motor1EncoderPulses;
  long motor1EncoderPulsesDelta;
  // unsigned char motor1DirMeas;

  // float motor2DistanceMeas;
  long motor2EncoderPulses;
  long motor2EncoderPulsesDelta;
  // unsigned char motor2DirMeas;

  // gyro angular velocity measurement
  float pitch_velocity_gyro;

} StateEstimatePacket_t;

typedef struct {
    float pitch_setpoint;
    float pitch_current;
    float pitch_error;

    float motorSpeed;
    int motorDir;
    uint dutyCycle;

} ControlPacket_t;

float accel_resolution = 0;
float gyro_resolution = 0;
float pitchAngleGyro = 0;             // [rad]
float pitchAngleEst = 0;              // [rad]

// hardware timer
const int ESTIMATOR_FREQ = 200; //250;        // frequency to run state estimator at [Hz]
const float ALPHA = 0.98;             // gyro weight for complementary filter
hw_timer_t *hwTimer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
QueueHandle_t queueIMURaw;  // queue of raw IMU measurements
QueueHandle_t queueStateEstimates;  // queue of state estimates

enum MotorDirection { FORWARD, REVERSE }; 
MotorDirection dirDrive { MotorDirection::FORWARD };
MotorDirection motor1DirManual { MotorDirection::FORWARD }; // motor 1 direction when in manual mode
MotorDirection motor2DirManual { MotorDirection::FORWARD };
uint dutyCycle1Manual {0};                // motor 1 duty cycle when in manual mode
uint dutyCycle2Manual {0};

enum ControlMode { AUTO, MANUAL };    // choose whether control system controls the motors (AUTO) or user manually sets wheel movements (MANUAL)
ControlMode controlMode {AUTO};
uint DUTY_CYCLE_MIN = 35;
uint DUTY_CYCLE_MAX = 255;      // conservative for now. Can be as high as 255
float PITCH_ANGLE_ERROR_MAX = 25.f*PI/180.f;   // maximum pitch angle error before motors cut off
float PITCH_ANGLE_ERROR_MIN = 0.5f*PI/180.f;   // minimumpitch angle error before motors cut off

// PID variables
float pitch_angle_setpoint = -3.7*PI/180;  // desired pitch angle [rad]
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

uint correctMotor2DutyCycle(const uint dutyCycle) {
  // correct motor 2's commanded duty cycle to ensure resultant speed is same as motor 1 when commanded to have the same duty cycle

  const float m1 = 0.01676954f;   // slope motor 1 graph of speed as a function of duty cycle
  const float b1 = -0.0466465f;   // intercept motor 1 graph of speed as a function of duty cycle
  const float m2 = 0.01765459f;   // slope motor 2 graph of speed as a function of duty cycle
  const float b2 = -0.09230133f;  // intercept motor 2 graph of speed as a function of duty cycle

  const float dutyCycleCorrected = ((m1 * dutyCycle + b1) - b2) / m2;

  return static_cast<uint>(dutyCycleCorrected);
}

// Task to be executed periodically
void taskReadIMURawValues(void * parameter) {
    for(;;) {
        // Wait for the semaphore from the timer ISR
        if(xSemaphoreTake(timerSemaphore, portMAX_DELAY) == pdTRUE) {
            IMUPacket_t imuPacket;
            imu.getMotion6(&(imuPacket.ax), &(imuPacket.ay), &(imuPacket.az), &(imuPacket.gx), &(imuPacket.gy), &(imuPacket.gz));

            if (xQueueSend(queueIMURaw, &imuPacket, portMAX_DELAY) != pdPASS) {
                Serial.println("Failed to send to IMU packet queue");
            }
        }
    }
}

void taskEstimateState(void * parameter) {
    IMUPacket_t imuPacket;
    StateEstimatePacket_t stateEstimatePacket;
    bool gyroOffsetCalculated = false;
    float gyroOffsetY = 0;                            // [deg/s]
    long motor1EncoderPulsesLastUpdate = 0;       // pulses since last state estimator update
    long motor2EncoderPulsesLastUpdate = 0;

    const float WHEEL_DIAMETER = 0.0815;  // [m]
    const float DISTANCE_PER_PULSE = PI * WHEEL_DIAMETER / ENCODER_PULSES_PER_REVOLUTION;
    const uint WHEEL_VELOCITY_ESTIMATOR_TIME_STEPS = 25;  // number of time steps of the estimator period used to calculate wheel velocity over
    uint wheel_velocity_estimator_step_count = WHEEL_VELOCITY_ESTIMATOR_TIME_STEPS;   // keep track of how many steps since last velocity measurement taken
    long motor1EncoderPulsesDelta = 0;
    long motor2EncoderPulsesDelta = 0;
    uint txCount = 0;    // used for keeping track of frequency of transmitting over serial
    const uint TX_PERIOD = 10;

    for (;;) {
        // Wait until data is available in the queue
        if (xQueueReceive(queueIMURaw, &imuPacket, portMAX_DELAY) == pdPASS) {

            // invert accelerometer readings to account for IMU mounted upside down
            const float ax = -imuPacket.ax * accel_resolution / 2;   // [m/s^2]
            const float az = -imuPacket.az * accel_resolution / 2;
            const float gy = imuPacket.gy * gyro_resolution / 2;    // [deg/s]

            const float pitchAngleAccel = atan2(ax, az);            // [rad]
            
            if (!gyroOffsetCalculated)
            {
                // TODO: calculate average over a period
                gyroOffsetY = gy;
                gyroOffsetCalculated = true;
            }

            const float pitchAngularRateGyro = (gy - gyroOffsetY) * PI / 180.;          // [rad/s]
            const float deltaAngularRateGyro = -pitchAngularRateGyro / ESTIMATOR_FREQ;

            pitchAngleGyro += deltaAngularRateGyro;

            pitchAngleEst = ALPHA * (pitchAngleEst + deltaAngularRateGyro) + (1-ALPHA) * pitchAngleAccel;   // [rad]

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

            stateEstimatePacket.pitch_accel = pitchAngleAccel;
            stateEstimatePacket.pitch_gyro = pitchAngleGyro;
            stateEstimatePacket.pitch_est = pitchAngleEst;
            stateEstimatePacket.motor1EncoderPulses = motor1EncoderPulses;
            stateEstimatePacket.motor1EncoderPulsesDelta = motor1EncoderPulsesDelta;
            // stateEstimatePacket.motor1DistanceMeas = motor1EncoderPulses * DISTANCE_PER_PULSE;
            // stateEstimatePacket.motor1DirMeas = static_cast<signed char>(motor1DirMeas);
            stateEstimatePacket.motor2EncoderPulses = motor2EncoderPulses;
            stateEstimatePacket.motor2EncoderPulsesDelta = motor2EncoderPulsesDelta;
            // stateEstimatePacket.motor2DistanceMeas = motor2EncoderPulses * DISTANCE_PER_PULSE;
            // stateEstimatePacket.motor2DirMeas = static_cast<signed char>(motor2DirMeas);
            stateEstimatePacket.pitch_velocity_gyro = deltaAngularRateGyro;   // TODO: should it be -pitchAngularRateGyro instead?
            
            txCount ++;
            const int64_t microSecondsSinceBoot = esp_timer_get_time();
            if (txCount == TX_PERIOD) {
              Serial.write(STX);
              Serial.write( (uint8_t *) &packetID, sizeof( packetID ));
              Serial.write( (uint8_t *) &microSecondsSinceBoot, sizeof( microSecondsSinceBoot ));
              Serial.write( (uint8_t *) &stateEstimatePacket, sizeof( stateEstimatePacket ) );
              Serial.write(ETX);

              txCount = 0;
              packetID += 1;
            }

            if (xQueueSend(queueStateEstimates, &stateEstimatePacket, portMAX_DELAY) != pdPASS) {
                Serial.println("Failed to send to state estimate queue");
            }
        }
    }
}

void taskControlMotors(void * parameter) {
  StateEstimatePacket_t stateEstimatePacket;
  ControlPacket_t controlPacket;
  
  for (;;) {
    // Wait until data is available in the queue
    if (xQueueReceive(queueStateEstimates, &stateEstimatePacket, portMAX_DELAY) == pdPASS) {
      pitch_angle_current = stateEstimatePacket.pitch_est;

      // Compute the PID output
      pid.calculate(pitch_angle_setpoint, pitch_angle_current, stateEstimatePacket.pitch_velocity_gyro);

      const float motorPerc = pid.Output;
      const MotorDirection motorDirAuto = motorPerc < 0 ? MotorDirection::FORWARD : MotorDirection::REVERSE;

      const bool pitch_error_exceeded = abs(pitch_angle_current - pitch_angle_setpoint) >= PITCH_ANGLE_ERROR_MAX;
      const bool pitch_error_small = abs(pitch_angle_current - pitch_angle_setpoint) <= PITCH_ANGLE_ERROR_MIN;
      const bool motor_power_too_low = abs(motorPerc) < 0.05;
      const bool disable_motors = pitch_error_exceeded || pitch_error_small || motor_power_too_low;
      const uint dutyCycleAuto = disable_motors ? 0 : static_cast<uint>(static_cast<float>(DUTY_CYCLE_MAX - DUTY_CYCLE_MIN) * abs(motorPerc) + static_cast<float>(DUTY_CYCLE_MIN));

      controlPacket.pitch_setpoint = pitch_angle_setpoint;
      controlPacket.pitch_current = pitch_angle_current;
      controlPacket.pitch_error = pitch_angle_current - pitch_angle_setpoint;
      controlPacket.motorSpeed = motorPerc;
      controlPacket.motorDir = static_cast<int>(motorDirAuto);
      controlPacket.dutyCycle = dutyCycleAuto;

      // Serial.write(STX);
      // Serial.write( (uint8_t *) &packetID, sizeof( packetID ));
      // Serial.write( (uint8_t *) &microSecondsSinceBoot, sizeof( microSecondsSinceBoot ));
      // Serial.write( (uint8_t *) &controlPacket, sizeof( controlPacket ) );
      // Serial.write(ETX);
      // packetID += 1;

      // adjust the PWM to each motor independently to ensure speed is equal
      const uint dutyCycle1Auto = dutyCycleAuto; // + dutyCycleDiff;
      const uint dutyCycle2Auto = dutyCycleAuto; // - dutyCycleDiff;

      uint dutyCycle1 = dutyCycle1Manual;
      uint dutyCycle2 = dutyCycle2Manual;
      MotorDirection motor1dir = motor1DirManual;
      MotorDirection motor2dir = motor2DirManual;

      if (controlMode == ControlMode::AUTO) {
        dutyCycle1 = dutyCycle1Auto;
        dutyCycle2 = dutyCycle2Auto;
        motor1dir = motorDirAuto;
        motor2dir = motorDirAuto;
      }

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
      uint dutyCycle2Calibrated = correctMotor2DutyCycle(dutyCycle2);

      ledcWrite(MOTOR1_PWM_CHANNEL, dutyCycle1);
      ledcWrite(MOTOR2_PWM_CHANNEL, dutyCycle2Calibrated);
      digitalWrite(PIN_MOTOR1_DIR, motor1dirActual);
      digitalWrite(PIN_MOTOR2_DIR, motor2dirActual);

      // log data to RAM
      if (enableLogging) {
        if (logIndex == NUM_LOG_PACKETS) {
          stopLogging();
        }
        logPackets[logIndex].pitch_current = pitch_angle_current;
        logPackets[logIndex].pitch_gyro = pitchAngleGyro;
        logPackets[logIndex].dutyCycle1 = dutyCycle1;
        logPackets[logIndex].motorDir1 = motor1dirActual;
        logPackets[logIndex].motorSpeed = pid.Output;

        logIndex ++;
      }
    }
  }
}

void handleMotor1EncoderA() {
  // Read the state of channel B
  int stateB = digitalRead(PIN_ENCODER1B);

  // Determine the direction
  if (digitalRead(PIN_ENCODER1A) == HIGH) {
    motor1DirMeas = (stateB == LOW) ? 1 : -1; // Forward if B is LOW, backward if B is HIGH
  } else {
    motor1DirMeas = (stateB == HIGH) ? 1 : -1; // Forward if B is HIGH, backward if B is LOW
  }

  // Update pulse count
  motor1EncoderPulses += motor1DirMeas;

    if (motor1EncoderPulses % ENCODER_PULSES_PER_REVOLUTION == 0) {
    ledStatus = !ledStatus;
    digitalWrite(PIN_LED_PWM, ledStatus);
  }
}

void handleMotor1EncoderB() {
  // Read the state of channel A
  int stateA = digitalRead(PIN_ENCODER1A);

  // Determine the direction
  if (digitalRead(PIN_ENCODER1B) == HIGH) {
    motor1DirMeas = (stateA == HIGH) ? 1 : -1; // Forward if A is HIGH, backward if A is LOW
  } else {
    motor1DirMeas = (stateA == LOW) ? 1 : -1; // Forward if A is LOW, backward if A is HIGH
  }

  // Update pulse count
  motor1EncoderPulses += motor1DirMeas;

  if (motor1EncoderPulses % ENCODER_PULSES_PER_REVOLUTION == 0) {
    ledStatus = !ledStatus;
    digitalWrite(PIN_LED_PWM, ledStatus);
  }
}

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

void stopMotors() {
  motor1DirManual = MotorDirection::FORWARD;
  motor2DirManual = MotorDirection::FORWARD;
  dutyCycle1Manual = 0;
  dutyCycle2Manual = 0;

  controlMode = ControlMode::MANUAL;
}

void initWebserver() {
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

    JsonDocument jsonDoc;
    // (uint8_t *) &microSecondsSinceBoot, sizeof( microSecondsSinceBoot )
    jsonDoc["data"] = JsonString{(char *) &logPackets, sizeof(logPackets)};
    jsonDoc["num_packets_logged"] = logIndex;

    String jsonString;
    serializeJson(jsonDoc, jsonString);

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

  queueIMURaw = xQueueCreate(10, sizeof(IMUPacket_t));
  if (queueIMURaw == NULL) {
      Serial.println("Failed to create queue");
      while (1);
  }
  queueStateEstimates = xQueueCreate(10, sizeof(StateEstimatePacket_t));
  if (queueStateEstimates == NULL) {
      Serial.println("Failed to create queue");
      while (1);
  }

  // Create the task that will be executed periodically
  xTaskCreate(taskReadIMURawValues, "Read Raw IMU Values", 10000, NULL, 1, NULL);
  xTaskCreate(taskEstimateState, "Estimate State", 2048, NULL, 1, NULL);
  xTaskCreate(taskControlMotors, "Estimate State", 2048, NULL, 1, NULL);

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

  ledcSetup(MOTOR1_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);  // define the PWM Setup
  ledcSetup(MOTOR2_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PIN_MOTOR1_PWM, MOTOR1_PWM_CHANNEL);
  ledcAttachPin(PIN_MOTOR2_PWM, MOTOR2_PWM_CHANNEL);
//   ledcAttachPin(PIN_LED_PWM, MOTOR1_PWM_CHANNEL);

  // Set encoder pins as inputs
  pinMode(PIN_ENCODER1A, INPUT);
  pinMode(PIN_ENCODER1B, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER1A), handleMotor1EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER1B), handleMotor1EncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER2A), handleMotor2EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER2B), handleMotor2EncoderB, CHANGE);

  // PID constants
  pid.kp = 3.0;
  pid.ki = 0.0;
  pid.kd = 0.0;
  pid.inAuto = true;

  initWiFi();
  initWebserver();
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

  // ledcWrite(MOTOR1_PWM_CHANNEL, 0);        // set the Duty cycle out of 255
  // ledcWrite(MOTOR2_PWM_CHANNEL, 0);
  // delay(1000);
  // ledcWrite(MOTOR1_PWM_CHANNEL, 50);
  //  ledcWrite(MOTOR2_PWM_CHANNEL, 50);
  // delay(1000);
  // ledcWrite(MOTOR1_PWM_CHANNEL, 80);
  // ledcWrite(MOTOR2_PWM_CHANNEL, 80);
  // delay(1000);

}

