#include <TimerOne.h>
#define PULSES_PER_ROTATION 12
#define REDUCTION 34
#define GPIO_INTERRUPT_PIN 2
unsigned int pulses = 0;

void setup() {
  Serial.begin(9600);
  pinMode(GPIO_INTERRUPT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(GPIO_INTERRUPT_PIN), interrupt, FALLING);

  Timer1.initialize(1000000); //set timer period at 1000000 us = 1 second
  Timer1.attachInterrupt(Tim1_interrupt, 1000000);//set timer interrupt at 1000000 us = 1 second
}

void interrupt() {
  pulses++; //increase pulse number
}

void Tim1_interrupt(){
  float rps = (float) pulses/PULSES_PER_ROTATION/REDUCTION; //calculate rotations per second
  Serial.println(rps); 
  pulses = 0; //reset pulse number
}

void loop() {
  // put your main code here, to run repeatedly:
}
