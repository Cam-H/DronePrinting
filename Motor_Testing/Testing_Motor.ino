//set vars
uint32_t currTick;
uint32_t prevTick = 0; //Initilaize to 0
const int freq = 50; //Carrier Frequency is to be 50Hz for the BLDC's
double potIn;
uint32_t onTime;


void setup() {
  //Start the serial log
  //115200 baud rate
  Serial.begin(115200);
  //Use Pin 6 to output PWM
  pinMode(6, OUTPUT);
}

void loop() {
  //Will use microsecond timeing for finer control
  currTick = micros(); 

  //Using pin A0 for the Pot input
  potIn = analogRead(A0);
  /*Map input to usable duty cycle
  The duty cycle should vary such that the ON pulse is 1ms for no throttle
  The ON pulse shold also be 2ms for full throttle
  Map 0 from ADC to 1ms long pulse in 50Hz
  Map 700 from ADC(max resolution) to 2ms long pulse at 50Hz
  50Hz is 0.02 seconds, 20ms
  50% duty is 10ms; 1ms is 5% duty cycle and 2ms is 10% duty cycle
  */
  onTime = (potIn/700)*1000 + 1000;

  /*
  Serial.print(potIn); //Gives Pot Value
  Serial.print(" Throttle Percent ");
  Serial.print(potIn*100/700); //Will give throttle percent
  Serial.print(" ");
  Serial.print(currTick - prevTick);
  Serial.print(" ");
  Serial.println(onTime); //Verify onTime equation
*/
  //If in the part of the PWM cycle where it is "ON" (less than on time)
  if((currTick - prevTick) < onTime) {
    //Pull the output high
    digitalWrite(6, HIGH);
    Serial.println("in high");
  }
  //If in the part of the PWM cycle where it is "OFF" (greater than on time and less than carrier frequency)
  if((currTick - prevTick >= onTime) && (currTick - prevTick < (1000000/freq))){
    //Pull the output low
    digitalWrite(6, LOW);
    Serial.println("in low");
  }
  //If at the end of the carrier frequency, reset the cycle
  if(currTick - prevTick >= (1000000/freq)){
    prevTick = currTick;
    Serial.println("In reset");
  }
}
