#include <Arduino.h>

// // put function declarations here:
// int myFunction(int, int);

// void setup() {
//   // put your setup code here, to run once:
//   int result = myFunction(2, 3);
// }

// void loop() {
//   // put your main code here, to run repeatedly:
// }

// // put function definitions here:
// int myFunction(int x, int y) {
//   return x + y;
// }



// #define LED_BUILTIN  2
// // the setup function runs once when you press reset or power the board
// void setup() {
//   // initialize digital pin LED_BUILTIN as an output.
//   pinMode(LED_BUILTIN, OUTPUT);
// }

// // the loop function runs over and over again forever
// void loop() {
//   digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
//   delay(40);                       // wait for a second
//   digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
//   delay(40);                       // wait for a second
// }



// // forward declaration
// void toggleLED(void* parameter);


// const int led2 = 2;   // pin of the LED
// int count = 0;

// void setup()
// {
//     pinMode(led2, OUTPUT);

//     Serial.begin(9600);

//     xTaskCreate(
//         toggleLED,
//         "Toggle LED",   // name of task, for debugging
//         1000,         // stack size [bytes]
//         NULL,   // param to pass
//         1,    // task priority
//         NULL  // task handle
//     );
// }


// void toggleLED(void* parameter)
// {
//     for (;;)    // infinite loop
//     {
//         digitalWrite(led2, HIGH);

//         Serial.print('h');

//         // pause for 500ms
//         vTaskDelay(500 / portTICK_PERIOD_MS);

//         digitalWrite(led2, LOW);

//         Serial.print('i');

//         // pause for 500ms
//         vTaskDelay(500 / portTICK_PERIOD_MS);
//     }
// }


// void loop() {
//   while(1) {
//     // must insure increment is atomic
//     // in case of context switch for print
//     noInterrupts();
//     count++;
//     interrupts();
//   }
// }




// // #include <Wire.h>

// // const int sda_pin = 21; // GPIO21 as I2C SDA (this is the default for ESP32 devices)
// // const int scl_pin = 20; // GPIO17 as I2C SCL (GPIO22 is the default)

// void setup()
// {
//     // Wire.setPins(sda_pin, scl_pin); // Set the I2C pins before begin
//     // Wire.begin(); // join i2c bus (address optional for master)
//     Serial.begin(9600);  // start serial for output
// }


// void loop() {
//   while(1) {
//     // // must insure increment is atomic
//     // // in case of context switch for print
//     // noInterrupts();
//     // count++;
//     // interrupts();
//   }
// }








// String inputString = "";      //a String to hold incoming data
// bool stringComplete = false;  // whether the string is complete

// void setup() {
//   Serial.begin(115200);
//   Serial.println(String("\nESP32S3 initialization completed!\r\n")
//                 + String("Please input some characters,\r\n")
//                 + String("select \"Newline\" below and Ctrl + Enter to send message to ESP32S3. \r\n"));
// }

// void loop() {
//   if (Serial.available()) {         // judge whether data has been received
//     char inChar = Serial.read();         // read one character
//     inputString += inChar;
//     if (inChar == '\n') {
//       stringComplete = true;
//     }
//   }
//   if (stringComplete) {
//     Serial.printf("inputString: %s \r\n", inputString);
//     inputString = "";
//     stringComplete = false;
//   }
// }




const int ledPWMPin = 2;
const int motor1PWMPin = 45;      // the PWM pin the motor 1 PWM is attached to
const int freq = 30000;            // set the frequency to 30kHz
const int motor1PWMChannel = 0;   // set the PWM channel
const int resolution = 8;         // set PWM resolution

const int motor1SleepPin = 47;
const int motor1DirPin = 48;
const int encoder1APin = 11;
const int encoder1BPin = 12;

bool dirDrive = true;
const bool MOTOR_FORWARD = true;

// measured values
volatile long motor1Position = 0;
volatile bool motor1Dir = true;

void updateMotor1Position(){
  if (digitalRead(encoder1BPin) != digitalRead(encoder1APin)) {
    motor1Position ++;
    motor1Dir = MOTOR_FORWARD;
  }
  else {
    motor1Position --;
    motor1Dir = !MOTOR_FORWARD;
  }

  digitalWrite(ledPWMPin, motor1Dir);
}
 
void setup(){
  pinMode(motor1SleepPin, OUTPUT);
  pinMode(motor1DirPin, OUTPUT);
  pinMode(ledPWMPin, OUTPUT);

  ledcSetup(motor1PWMChannel, freq, resolution);  // define the PWM Setup
  ledcAttachPin(motor1PWMPin, motor1PWMChannel);
  //ledcAttachPin(ledPWMPin, motor1PWMChannel);

  attachInterrupt(digitalPinToInterrupt(encoder1APin), updateMotor1Position, CHANGE);
}


 
void loop(){
    digitalWrite(motor1SleepPin, HIGH);     // inveted, so HIGH should be not sleeping/coasting?
    digitalWrite(motor1DirPin, dirDrive);

    ledcWrite(motor1PWMChannel, 0);        // set the Duty cycle out of 255
    delay(1000);
    ledcWrite(motor1PWMChannel, 50);
    delay(1000);
    ledcWrite(motor1PWMChannel, 80);
    delay(1000);

    dirDrive = !dirDrive;
}