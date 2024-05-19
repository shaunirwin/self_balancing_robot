#include <Arduino.h>

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 imu;

const int PIN_LED_PWM = 2;
const int PIN_MOTOR1_SLEEP = 47;
const int PIN_MOTOR1_DIR = 48;
const int PIN_ENCODER1A = 11;
const int PIN_ENCODER1B = 12;
const int PIN_MOTOR1_PWM = 45;      // the PWM pin the motor 1 PWM is attached to
const int PIN_I2C_SDA = 21; // GPIO21 as I2C SDA (this is the default for ESP32 devices)
const int PIN_I2C_SCL = 20; // GPIO17 as I2C SCL (GPIO22 is the default)

const int PWM_FREQ = 30000;       // frequency to run PWM at [Hz]
const int MOTOR1_PWM_CHANNEL = 0;   // set the PWM channel
const int PWM_RESOLUTION = 8;         // set PWM resolution

const bool MOTOR_FORWARD = true;

bool dirDrive = MOTOR_FORWARD;
bool ledStatus = true;

// serial tx control characters
const char STX = '!'; //'\x002';   // start of frame
const char ETX = '@'; //'\x003';   // end of frame

// state estimation
volatile long motor1Position = 0;
volatile bool motor1Dir = MOTOR_FORWARD;
typedef struct {
   int16_t ax;
   int16_t ay;
   int16_t az;
   int16_t gx;
   int16_t gy;
   int16_t gz;
} IMUPacket_t;

typedef struct {
   float pitch_accel;
   float pitch_gyro;
   float pitch_est;
} StateEstimatePacket_t;

float accel_resolution = 0;
float gyro_resolution = 0;
float pitchAngleGyro = 0;             // [rad]
float pitchAngleEst = 0;              // [rad]

// hardware timer
const int ESTIMATOR_FREQ = 250;        // frequency to run state estiamtor at [Hz]
const float ALPHA = 0.98;             // gyro weight for complementary filter
hw_timer_t *hwTimer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
QueueHandle_t queueIMURaw;  // queue of raw IMU measurements
//uint16_t pulses_count = 0;    // for testing

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
                Serial.println("Failed to send to queue");
            }
        }
    }
}

void taskEstimateState(void * parameter) {
    IMUPacket_t imuPacket;
    StateEstimatePacket_t stateEstimatePacket;
    for (;;) {
        // Wait until data is available in the queue
        if (xQueueReceive(queueIMURaw, &imuPacket, portMAX_DELAY) == pdPASS) {

            const float ax = imuPacket.ax * accel_resolution / 2;   // [m/s^2]
            const float az = imuPacket.az * accel_resolution / 2;
            const float gy = imuPacket.gy * gyro_resolution / 2;    // [deg/s]

            const float pitchAngleAccel = atan2(ax, az);            // [rad]

            // TODO: calculate gyro offsets
            float gyroOffsetY = 0;                            // [deg/s]

            const float pitchAngularRateGyro = (gy - gyroOffsetY) * PI / 180.;          // [rad/s]
            const float deltaAngularRateGyro = -pitchAngularRateGyro / ESTIMATOR_FREQ;

            pitchAngleGyro += deltaAngularRateGyro;

            pitchAngleEst = ALPHA * (pitchAngleEst + deltaAngularRateGyro) + (1-ALPHA) * pitchAngleAccel;   // [rad]

            stateEstimatePacket.pitch_accel = pitchAngleAccel;
            stateEstimatePacket.pitch_gyro = pitchAngleGyro;
            stateEstimatePacket.pitch_est = pitchAngleEst;
            
            Serial.write(STX);
            Serial.write( (uint8_t *) &stateEstimatePacket, sizeof( stateEstimatePacket ) );
            Serial.write(ETX);
        }
    }
}

void updateMotor1Position(){
  if (digitalRead(PIN_ENCODER1B) != digitalRead(PIN_ENCODER1A)) {
    motor1Position ++;
    motor1Dir = MOTOR_FORWARD;
  }
  else {
    motor1Position --;
    motor1Dir = !MOTOR_FORWARD;
  }

  if (motor1Position % 464 == 0) {
    ledStatus = !ledStatus;
    digitalWrite(PIN_LED_PWM, ledStatus);
  }
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

  // Create the task that will be executed periodically
  xTaskCreate(taskReadIMURawValues, "Read Raw IMU Values", 10000, NULL, 1, NULL);
  xTaskCreate(taskEstimateState, "Estimate State", 2048, NULL, 1, NULL);

  hwTimer = timerBegin(/* timer num */ 0, /* clock divider */ 80, /* count up */true);
  timerAttachInterrupt(hwTimer, &stateEstimatorTimer, /* edge */ true);
  const uint64_t ALARM_PERIOD = 1e6 / ESTIMATOR_FREQ;               // (1 million / 250 Hz = 4000)
  timerAlarmWrite(hwTimer, ALARM_PERIOD, /* periodic */true);
  timerAlarmEnable(hwTimer);

  // verify connection
  // Serial.println("Testing device connections...");
  // Serial.println(imu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  pinMode(PIN_MOTOR1_SLEEP, OUTPUT);
  pinMode(PIN_MOTOR1_DIR, OUTPUT);
  pinMode(PIN_LED_PWM, OUTPUT);

  ledcSetup(MOTOR1_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);  // define the PWM Setup
  ledcAttachPin(PIN_MOTOR1_PWM, MOTOR1_PWM_CHANNEL);
  //ledcAttachPin(PIN_LED_PWM, MOTOR1_PWM_CHANNEL);

  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER1A), updateMotor1Position, CHANGE);
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
  digitalWrite(PIN_LED_PWM, ledStatus);
}

