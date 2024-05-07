const int ledPin = 2;      // the PWM pin the LED is attached to
const int freq = 5000;      // set the frequency for 5kHz
const int ledChannel = 0;   // set the PWM channel
const int resolution = 8;   // set PWM resolution
 
void setup(){
  ledcSetup(ledChannel, freq, resolution);  // define the PWM Setup
  ledcAttachPin(ledPin, ledChannel);
}
 
void loop(){
    ledcWrite(ledChannel, 25);        // set the Duty cycle to 50 out of 255
    delay(500);
    ledcWrite(ledChannel, 50);        // set the Duty cycle to 50 out of 255
    delay(500);
    ledcWrite(ledChannel, 100);        // set the Duty cycle to 50 out of 255
    delay(500);
}