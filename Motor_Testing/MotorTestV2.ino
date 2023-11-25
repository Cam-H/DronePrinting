#include <Servo.h> //include the servo library

//define the bldc motor as a servo
Servo Motor;

int potIn; //define value for pot
  
void setup() {

  Serial.begin(115200); //Start debugging comms
  //attach the servo object to pin 6
  //min pusle width 1000us max 2000us
  Motor.attach(6, 1000, 2000);

}

void loop() {
  //Get vlaue of pot, map to 0 - 180
  //0 - 180 corresponds to 1000 - 2000 pulse width
  potIn = analogRead(A0);
  potIn = map(potIn, 0, 700, 0, 180);
  Motor.write(potIn); //output to the motor
  
  Serial.print(potIn); //output for debugging
  Serial.print(" Throttle Percent: ");
  Serial.println(potIn*100/180);
}
