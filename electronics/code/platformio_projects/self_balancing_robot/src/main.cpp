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

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high
//MPU6050 accelgyro(0x68, &Wire1); // <-- use for AD0 low, but 2nd Wire (TWI/I2C) object

// // forward declaration
// void toggleLED(void* parameter);


// // const int led2 = 2;   // pin of the LED
// // int count = 0;

// // void setup()
// // {
// //     pinMode(led2, OUTPUT);

// //     Serial.begin(9600);

// //     xTaskCreate(
// //         toggleLED,
// //         "Toggle LED",   // name of task, for debugging
// //         1000,         // stack size [bytes]
// //         NULL,   // param to pass
// //         1,    // task priority
// //         NULL  // task handle
// //     );
// // }


// // void toggleLED(void* parameter)
// // {
// //     for (;;)    // infinite loop
// //     {
// //         digitalWrite(led2, HIGH);

// //         Serial.print('h');

// //         // pause for 500ms
// //         vTaskDelay(500 / portTICK_PERIOD_MS);

// //         digitalWrite(led2, LOW);

// //         Serial.print('i');

// //         // pause for 500ms
// //         vTaskDelay(500 / portTICK_PERIOD_MS);
// //     }
// // }


// // void loop() {
// //   while(1) {
// //     // must insure increment is atomic
// //     // in case of context switch for print
// //     noInterrupts();
// //     count++;
// //     interrupts();
// //   }
// // }







const int ledPWMPin = 2;
const int motor1SleepPin = 47;
const int motor1DirPin = 48;
const int encoder1APin = 11;
const int encoder1BPin = 12;
const int motor1PWMPin = 45;      // the PWM pin the motor 1 PWM is attached to
const int sda_pin = 21; // GPIO21 as I2C SDA (this is the default for ESP32 devices)
const int scl_pin = 20; // GPIO17 as I2C SCL (GPIO22 is the default)

const int freq = 30000;            // set the frequency to 30kHz
const int motor1PWMChannel = 0;   // set the PWM channel
const int resolution = 8;         // set PWM resolution

const bool MOTOR_FORWARD = true;

bool dirDrive = true;
bool ledStatus = true;

// measured values
volatile long motor1Position = 0;
volatile bool motor1Dir = true;

// hardware timer
hw_timer_t *hw_timer = NULL;
uint16_t pulses_count = 0;    // for testing

void IRAM_ATTR state_estimator_timer(){
  // ledStatus = !ledStatus;
  pulses_count += 1;

  if (pulses_count == 250) {
    ledStatus = !ledStatus;
    pulses_count = 0;
  }
}

void updateMotor1Position(){
  if (digitalRead(encoder1BPin) != digitalRead(encoder1APin)) {
    motor1Position ++;
    motor1Dir = MOTOR_FORWARD;
  }
  else {
    motor1Position --;
    motor1Dir = !MOTOR_FORWARD;
  }

  if (motor1Position % 464 == 0) {
    ledStatus = !ledStatus;
    digitalWrite(ledPWMPin, ledStatus);
  }
}
 
void setup(){
  Serial.begin(115200);

  Wire.setPins(sda_pin, scl_pin); // Set the I2C pins before begin

  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      // Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties. - Shaun: Not sure whether I need this?
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  accelgyro.initialize();

  hw_timer = timerBegin(/* timer num */ 0, /* divider */ 80, /* count up */true);
  timerAttachInterrupt(hw_timer, &state_estimator_timer, /* edge */ true);
  // timerAlarmWrite(hw_timer, 1000000, true);   // 1kHz
  timerAlarmWrite(hw_timer, 4000, /* periodic */true);      // 250Hz (1 million / 250 = 4000)
  timerAlarmWrite(hw_timer, 4000, true);      // 250Hz
  timerAlarmEnable(hw_timer);

  // verify connection
  // Serial.println("Testing device connections...");
  // Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  pinMode(motor1SleepPin, OUTPUT);
  pinMode(motor1DirPin, OUTPUT);
  pinMode(ledPWMPin, OUTPUT);

  ledcSetup(motor1PWMChannel, freq, resolution);  // define the PWM Setup
  ledcAttachPin(motor1PWMPin, motor1PWMChannel);
  //ledcAttachPin(ledPWMPin, motor1PWMChannel);

  attachInterrupt(digitalPinToInterrupt(encoder1APin), updateMotor1Position, CHANGE);
}

// serial tx control characters
const char STX = '!'; //'\x002';   // start of frame
const char ETX = '@'; //'\x003';   // end of frame

struct IMUPacket {
   int16_t ax;
   int16_t ay;
   int16_t az;
   int16_t gx;
   int16_t gy;
   int16_t gz;
};
 
// void loop(){
//     digitalWrite(motor1SleepPin, HIGH);     // inveted, so HIGH should be not sleeping/coasting?
//     digitalWrite(motor1DirPin, dirDrive);

//     // ledcWrite(motor1PWMChannel, 0);        // set the Duty cycle out of 255
//     // delay(1000);
//     //ledcWrite(motor1PWMChannel, 50);
//     // delay(1000);
//     // ledcWrite(motor1PWMChannel, 80);
//     // delay(1000);

//     // dirDrive = !dirDrive;

//     int16_t ax, ay, az;
// int16_t gx, gy, gz;

//     struct IMUPacket imuPacket;

//     // accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

//     accelgyro.getMotion6(&(imuPacket.ax), &(imuPacket.ay), &(imuPacket.az), &(imuPacket.gx), &(imuPacket.gy), &(imuPacket.gz));
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
  digitalWrite(ledPWMPin, ledStatus);
}











// // MPU6050 offset-finder, based on Jeff Rowberg's MPU6050_RAW
// // 2016-10-19 by Robert R. Fenichel (bob@fenichel.net)

// // I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// // 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// // Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
// //
// // Changelog:
// //      2019-07-11 - added PID offset generation at begninning Generates first offsets 
// //                 - in @ 6 seconds and completes with 4 more sets @ 10 seconds
// //                 - then continues with origional 2016 calibration code.
// //      2016-11-25 - added delays to reduce sampling rate to ~200 Hz
// //                   added temporizing printing during long computations
// //      2016-10-25 - requires inequality (Low < Target, High > Target) during expansion
// //                   dynamic speed change when closing in
// //      2016-10-22 - cosmetic changes
// //      2016-10-19 - initial release of IMU_Zero
// //      2013-05-08 - added multiple output formats
// //                 - added seamless Fastwire support
// //      2011-10-07 - initial release of MPU6050_RAW

// /* ============================================
// I2Cdev device library code is placed under the MIT license
// Copyright (c) 2011 Jeff Rowberg

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

//   If an MPU6050 
//       * is an ideal member of its tribe, 
//       * is properly warmed up, 
//       * is at rest in a neutral position, 
//       * is in a location where the pull of gravity is exactly 1g, and 
//       * has been loaded with the best possible offsets, 
// then it will report 0 for all accelerations and displacements, except for 
// Z acceleration, for which it will report 16384 (that is, 2^14).  Your device 
// probably won't do quite this well, but good offsets will all get the baseline 
// outputs close to these target values.

//   Put the MPU6050 on a flat and horizontal surface, and leave it operating for 
// 5-10 minutes so its temperature gets stabilized.

//   Run this program.  A "----- done -----" line will indicate that it has done its best.
// With the current accuracy-related constants (NFast = 1000, NSlow = 10000), it will take 
// a few minutes to get there.

//   Along the way, it will generate a dozen or so lines of output, showing that for each 
// of the 6 desired offsets, it is 
//       * first, trying to find two estimates, one too low and one too high, and
//       * then, closing in until the bracket can't be made smaller.

//   The line just above the "done" line will look something like
//     [567,567] --> [-1,2]  [-2223,-2223] --> [0,1] [1131,1132] --> [16374,16404] [155,156] --> [-1,1]  [-25,-24] --> [0,3] [5,6] --> [0,4]
// As will have been shown in interspersed header lines, the six groups making up this
// line describe the optimum offsets for the X acceleration, Y acceleration, Z acceleration,
// X gyro, Y gyro, and Z gyro, respectively.  In the sample shown just above, the trial showed
// that +567 was the best offset for the X acceleration, -2223 was best for Y acceleration, 
// and so on.

//   The need for the delay between readings (usDelay) was brought to my attention by Nikolaus Doppelhammer.
// ===============================================
// */

// // I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// // for both classes must be in the include path of your project
// #include "I2Cdev.h"
// #include "MPU6050.h"

// // Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// // is used in I2Cdev.h
// #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//     #include "Wire.h"
// #endif

// // class default I2C address is 0x68
// // specific I2C addresses may be passed as a parameter here
// // AD0 low = 0x68 (default for InvenSense evaluation board)
// // AD0 high = 0x69
// MPU6050 accelgyro;
// //MPU6050 accelgyro(0x69); // <-- use for AD0 high


// const char LBRACKET = '[';
// const char RBRACKET = ']';
// const char COMMA    = ',';
// const char BLANK    = ' ';
// const char PERIOD   = '.';

// const int iAx = 0;
// const int iAy = 1;
// const int iAz = 2;
// const int iGx = 3;
// const int iGy = 4;
// const int iGz = 5;

// const int usDelay = 3150;   // empirical, to hold sampling to 200 Hz
// const int NFast =  1000;    // the bigger, the better (but slower)
// const int NSlow = 10000;    // ..
// const int LinesBetweenHeaders = 5;
//       int LowValue[6];
//       int HighValue[6];
//       int Smoothed[6];
//       int LowOffset[6];
//       int HighOffset[6];
//       int Target[6];
//       int LinesOut;
//       int N;

// const int sda_pin = 21; // GPIO21 as I2C SDA (this is the default for ESP32 devices)
// const int scl_pin = 20; // GPIO17 as I2C SCL (GPIO22 is the default)
      
// void ForceHeader()
//   { LinesOut = 99; }
    
// void GetSmoothed()
//   { int16_t RawValue[6];
//     int i;
//     long Sums[6];
//     for (i = iAx; i <= iGz; i++)
//       { Sums[i] = 0; }
// //    unsigned long Start = micros();

//     for (i = 1; i <= N; i++)
//       { // get sums
//         accelgyro.getMotion6(&RawValue[iAx], &RawValue[iAy], &RawValue[iAz], 
//                              &RawValue[iGx], &RawValue[iGy], &RawValue[iGz]);
//         if ((i % 500) == 0)
//           Serial.print(PERIOD);
//         delayMicroseconds(usDelay);
//         for (int j = iAx; j <= iGz; j++)
//           Sums[j] = Sums[j] + RawValue[j];
//       } // get sums
// //    unsigned long usForN = micros() - Start;
// //    Serial.print(" reading at ");
// //    Serial.print(1000000/((usForN+N/2)/N));
// //    Serial.println(" Hz");
//     for (i = iAx; i <= iGz; i++)
//       { Smoothed[i] = (Sums[i] + N/2) / N ; }
//   } // GetSmoothed

// void SetAveraging(int NewN)
//   { N = NewN;
//     Serial.print("averaging ");
//     Serial.print(N);
//     Serial.println(" readings each time");
//    } // SetAveraging

// void Initialize()
//   {
//       Wire.setPins(sda_pin, scl_pin); // Set the I2C pins before begin

//     // join I2C bus (I2Cdev library doesn't do this automatically)
//     #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//         Wire.begin();
//     #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
//         Fastwire::setup(400, true);
//     #endif

//     Serial.begin(115200);

//     // initialize device
//     Serial.println("Initializing I2C devices...");
//     accelgyro.initialize();

//     // verify connection
//     Serial.println("Testing device connections...");
//     Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
//     Serial.println("PID tuning Each Dot = 100 readings");
//   /*A tidbit on how PID (PI actually) tuning works. 
//     When we change the offset in the MPU6050 we can get instant results. This allows us to use Proportional and 
//     integral of the PID to discover the ideal offsets. Integral is the key to discovering these offsets, Integral 
//     uses the error from set-point (set-point is zero), it takes a fraction of this error (error * ki) and adds it 
//     to the integral value. Each reading narrows the error down to the desired offset. The greater the error from 
//     set-point, the more we adjust the integral value. The proportional does its part by hiding the noise from the 
//     integral math. The Derivative is not used because of the noise and because the sensor is stationary. With the 
//     noise removed the integral value lands on a solid offset after just 600 readings. At the end of each set of 100 
//     readings, the integral value is used for the actual offsets and the last proportional reading is ignored due to 
//     the fact it reacts to any noise.
//   */
//         accelgyro.CalibrateAccel(6);
//         accelgyro.CalibrateGyro(6);
//         Serial.println("\nat 600 Readings");
//         accelgyro.PrintActiveOffsets();
//         Serial.println();
//         accelgyro.CalibrateAccel(1);
//         accelgyro.CalibrateGyro(1);
//         Serial.println("700 Total Readings");
//         accelgyro.PrintActiveOffsets();
//         Serial.println();
//         accelgyro.CalibrateAccel(1);
//         accelgyro.CalibrateGyro(1);
//         Serial.println("800 Total Readings");
//         accelgyro.PrintActiveOffsets();
//         Serial.println();
//         accelgyro.CalibrateAccel(1);
//         accelgyro.CalibrateGyro(1);
//         Serial.println("900 Total Readings");
//         accelgyro.PrintActiveOffsets();
//         Serial.println();    
//         accelgyro.CalibrateAccel(1);
//         accelgyro.CalibrateGyro(1);
//         Serial.println("1000 Total Readings");
//         accelgyro.PrintActiveOffsets();
//      Serial.println("\n\n Any of the above offsets will work nice \n\n Lets proof the PID tuning using another method:"); 
//   } // Initialize

// void SetOffsets(int TheOffsets[6])
//   { accelgyro.setXAccelOffset(TheOffsets [iAx]);
//     accelgyro.setYAccelOffset(TheOffsets [iAy]);
//     accelgyro.setZAccelOffset(TheOffsets [iAz]);
//     accelgyro.setXGyroOffset (TheOffsets [iGx]);
//     accelgyro.setYGyroOffset (TheOffsets [iGy]);
//     accelgyro.setZGyroOffset (TheOffsets [iGz]);
//   } // SetOffsets

// void ShowProgress()
//   { if (LinesOut >= LinesBetweenHeaders)
//       { // show header
//         Serial.println("\tXAccel\t\t\tYAccel\t\t\t\tZAccel\t\t\tXGyro\t\t\tYGyro\t\t\tZGyro");
//         LinesOut = 0;
//       } // show header
//     Serial.print(BLANK);
//     for (int i = iAx; i <= iGz; i++)
//       { Serial.print(LBRACKET);
//         Serial.print(LowOffset[i]),
//         Serial.print(COMMA);
//         Serial.print(HighOffset[i]);
//         Serial.print("] --> [");
//         Serial.print(LowValue[i]);
//         Serial.print(COMMA);
//         Serial.print(HighValue[i]);
//         if (i == iGz)
//           { Serial.println(RBRACKET); }
//         else
//           { Serial.print("]\t"); }
//       }
//     LinesOut++;
//   } // ShowProgress

// void PullBracketsIn()
//   { boolean AllBracketsNarrow;
//     boolean StillWorking;
//     int NewOffset[6];
  
//     Serial.println("\nclosing in:");
//     AllBracketsNarrow = false;
//     ForceHeader();
//     StillWorking = true;
//     while (StillWorking) 
//       { StillWorking = false;
//         if (AllBracketsNarrow && (N == NFast))
//           { SetAveraging(NSlow); }
//         else
//           { AllBracketsNarrow = true; }// tentative
//         for (int i = iAx; i <= iGz; i++)
//           { if (HighOffset[i] <= (LowOffset[i]+1))
//               { NewOffset[i] = LowOffset[i]; }
//             else
//               { // binary search
//                 StillWorking = true;
//                 NewOffset[i] = (LowOffset[i] + HighOffset[i]) / 2;
//                 if (HighOffset[i] > (LowOffset[i] + 10))
//                   { AllBracketsNarrow = false; }
//               } // binary search
//           }
//         SetOffsets(NewOffset);
//         GetSmoothed();
//         for (int i = iAx; i <= iGz; i++)
//           { // closing in
//             if (Smoothed[i] > Target[i])
//               { // use lower half
//                 HighOffset[i] = NewOffset[i];
//                 HighValue[i] = Smoothed[i];
//               } // use lower half
//             else
//               { // use upper half
//                 LowOffset[i] = NewOffset[i];
//                 LowValue[i] = Smoothed[i];
//               } // use upper half
//           } // closing in
//         ShowProgress();
//       } // still working
   
//   } // PullBracketsIn

// void PullBracketsOut()
//   { boolean Done = false;
//     int NextLowOffset[6];
//     int NextHighOffset[6];

//     Serial.println("expanding:");
//     ForceHeader();
 
//     while (!Done)
//       { Done = true;
//         SetOffsets(LowOffset);
//         GetSmoothed();
//         for (int i = iAx; i <= iGz; i++)
//           { // got low values
//             LowValue[i] = Smoothed[i];
//             if (LowValue[i] >= Target[i])
//               { Done = false;
//                 NextLowOffset[i] = LowOffset[i] - 1000;
//               }
//             else
//               { NextLowOffset[i] = LowOffset[i]; }
//           } // got low values
      
//         SetOffsets(HighOffset);
//         GetSmoothed();
//         for (int i = iAx; i <= iGz; i++)
//           { // got high values
//             HighValue[i] = Smoothed[i];
//             if (HighValue[i] <= Target[i])
//               { Done = false;
//                 NextHighOffset[i] = HighOffset[i] + 1000;
//               }
//             else
//               { NextHighOffset[i] = HighOffset[i]; }
//           } // got high values
//         ShowProgress();
//         for (int i = iAx; i <= iGz; i++)
//           { LowOffset[i] = NextLowOffset[i];   // had to wait until ShowProgress done
//             HighOffset[i] = NextHighOffset[i]; // ..
//           }
//      } // keep going
//   } // PullBracketsOut


// void setup()
//   { Initialize();
//     for (int i = iAx; i <= iGz; i++)
//       { // set targets and initial guesses
//         Target[i] = 0; // must fix for ZAccel 
//         HighOffset[i] = 0;
//         LowOffset[i] = 0;
//       } // set targets and initial guesses
//     Target[iAz] = 16384;
//     SetAveraging(NFast);
    
//     PullBracketsOut();
//     PullBracketsIn();
    
//     Serial.println("-------------- done --------------");
//   } // setup
 
// void loop()
//   {
//   } // loop