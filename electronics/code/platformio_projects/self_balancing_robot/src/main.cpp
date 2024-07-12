#include <Arduino.h>

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"
#include <PID_v1.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 imu;

const int PIN_LED_PWM = 2;
const int PIN_MOTOR1_SLEEP = 42;
const int PIN_MOTOR2_SLEEP = 41;
const int PIN_MOTOR1_DIR = 40;
const int PIN_MOTOR2_DIR = 39;
const int PIN_ENCODER1A = 15;
const int PIN_ENCODER1B = 16;
const int PIN_ENCODER2A = 11;
const int PIN_ENCODER2B = 12;
const int PIN_MOTOR1_PWM = 35;
const int PIN_MOTOR2_PWM = 45;
const int PIN_I2C_SDA = 14;
const int PIN_I2C_SCL = 13;

const int PWM_FREQ = 30000;         // frequency to run PWM at [Hz]
const int MOTOR1_PWM_CHANNEL = 0;   // set the PWM channel
const int MOTOR2_PWM_CHANNEL = 1;   // set the PWM channel
const int PWM_RESOLUTION = 8;       // set PWM resolution

const bool MOTOR_1_FORWARD = true;
const bool MOTOR_2_FORWARD = false;
const bool MOTOR_COAST = false;
const uint ENCODER_PULSES_PER_REVOLUTION = 464;

bool dirDrive = MOTOR_1_FORWARD;
bool ledStatus = true;

// serial tx control characters
const char STX = '!'; //'\x002';   // start of frame
const char ETX = '@'; //'\x003';   // end of frame

// state estimation
volatile long motor1EncoderPulses = 0;
volatile int motor1DirMeas = 0;   // 0: stopped, 1: forward, -1: backward
volatile long motor2EncoderPulses = 0;
volatile int motor2DirMeas = 0;

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
const int ESTIMATOR_FREQ = 250;        // frequency to run state estimator at [Hz]
const float ALPHA = 0.98;             // gyro weight for complementary filter
hw_timer_t *hwTimer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
QueueHandle_t queueIMURaw;  // queue of raw IMU measurements
QueueHandle_t queueStateEstimates;  // queue of state estimates

// PID constants
double Kp = 5.0;
double Ki = 0.0;
double Kd = 0.0;

// PID variables
double pitch_angle_setpoint = -6.0*PI/180;  // desired pitch angle [rad]
double pitch_angle_current = 0;        // current pitch angle [rad]
double pid_output = 0;       // PID output (speed of motors)

// PID controller
PID myPID(&pitch_angle_current, &pid_output, &pitch_angle_setpoint, Kp, Ki, Kd, DIRECT);

void IRAM_ATTR stateEstimatorTimer(){
  // Give the semaphore to unblock the task
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
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
            
            // Serial.write(STX);
            // Serial.write( (uint8_t *) &stateEstimatePacket, sizeof( stateEstimatePacket ) );
            // Serial.write(ETX);

            if (xQueueSend(queueStateEstimates, &stateEstimatePacket, portMAX_DELAY) != pdPASS) {
                Serial.println("Failed to send to state estimate queue");
            }
        }
    }
}

void taskControlMotors(void * parameter) {
  StateEstimatePacket_t stateEstimatePacket;
  ControlPacket_t controlPacket;

  const uint DUTY_CYCLE_MIN = 35;
  const uint DUTY_CYCLE_MAX = 150;      // conservative for now. Can be as high as 255
  const float PITCH_ANGLE_ERROR_MAX = 28.f*PI/180.f;   // maximum pitch angle error before motors cut off
  
  for (;;) {
        // Wait until data is available in the queue
        if (xQueueReceive(queueStateEstimates, &stateEstimatePacket, portMAX_DELAY) == pdPASS) {
            const float pitchAngleEst = stateEstimatePacket.pitch_est;

            pitch_angle_current = static_cast<double>(pitchAngleEst);

            // Compute the PID output
            myPID.Compute();

            double motorPerc = pid_output;
            const bool motorDir = motorPerc < 0;

            const bool pitch_error_exceeded = abs(pitch_angle_current - pitch_angle_setpoint) >= PITCH_ANGLE_ERROR_MAX;
            const bool motor_power_too_low = abs(motorPerc) < 0.05;
            const bool disable_motors = pitch_error_exceeded || motor_power_too_low;
            const uint dutyCycle = disable_motors ? 0 : static_cast<uint>(static_cast<double>(DUTY_CYCLE_MAX - DUTY_CYCLE_MIN) * abs(motorPerc) + static_cast<double>(DUTY_CYCLE_MIN));

            controlPacket.pitch_setpoint = static_cast<float>(pitch_angle_setpoint);
            controlPacket.pitch_current = static_cast<float>(pitch_angle_current);
            controlPacket.pitch_error = static_cast<float>(pitch_angle_current - pitch_angle_setpoint);
            controlPacket.motorSpeed = static_cast<float>(motorPerc);
            controlPacket.motorDir = static_cast<int>(motorDir);
            controlPacket.dutyCycle = dutyCycle;

            Serial.write(STX);
            Serial.write( (uint8_t *) &controlPacket, sizeof( controlPacket ) );
            Serial.write(ETX);
            // invert direction to specific motors in case wires are switched
            const bool motor1dir = motorDir;
            const bool motor2dir = !motorDir;

            // adjust the PWM to each motor independently to ensure speed is equal
            const uint dutyCycle1 = dutyCycle; // + dutyCycleDiff;
            const uint dutyCycle2 = dutyCycle; // - dutyCycleDiff;

            ledcWrite(MOTOR1_PWM_CHANNEL, dutyCycle1);
            ledcWrite(MOTOR2_PWM_CHANNEL, dutyCycle2);
            digitalWrite(PIN_MOTOR1_DIR, motor1dir);
            digitalWrite(PIN_MOTOR2_DIR, motor2dir);
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

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-1., 1);  // -100% to 100% for motor power
  myPID.SetSampleTime(4);   // ms
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

