#include <Arduino.h>

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







const int ledPWMPin = 2;
const int motor1SleepPin = 47;
const int motor1DirPin = 48;
const int encoder1APin = 11;
const int encoder1BPin = 12;

const int motor1PWMPin = 45;      // the PWM pin the motor 1 PWM is attached to
const int freq = 30000;            // set the frequency to 30kHz
const int motor1PWMChannel = 0;   // set the PWM channel
const int resolution = 8;         // set PWM resolution

const bool MOTOR_FORWARD = true;

bool dirDrive = true;
bool ledStatus = true;

int16_t ax = 314;

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

  if (motor1Position % 464 == 0) {
    ledStatus = !ledStatus;
    digitalWrite(ledPWMPin, ledStatus);
  }
}
 
void setup(){
  Serial.begin(115200);

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

struct point {
   uint8_t x;
   uint8_t y;
   uint8_t z;
};

struct pointInt {
   int x;
   int y;
   int z;
};
 
void loop(){
    digitalWrite(motor1SleepPin, HIGH);     // inveted, so HIGH should be not sleeping/coasting?
    digitalWrite(motor1DirPin, dirDrive);

    // ledcWrite(motor1PWMChannel, 0);        // set the Duty cycle out of 255
    // delay(1000);
    //ledcWrite(motor1PWMChannel, 50);
    // delay(1000);
    // ledcWrite(motor1PWMChannel, 80);
    // delay(1000);

    // dirDrive = !dirDrive;

    //Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF)); Serial.write((uint8_t)('\r')); Serial.write((uint8_t)('\n'));
    struct pointInt data = { 356726, 57, -15675 };
    Serial.write(STX);
    Serial.write( (uint8_t *) &data, sizeof( data ) );
    Serial.write(ETX);
    delay(50);
}