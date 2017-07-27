#include <TimerOne.h>
#define PULSES_PER_ROTATION 12
#define REDUCTION 34
#define GPIO_INTERRUPT_PIN 2
unsigned int pulses = 0;

void setup() {
  Serial.begin(9600);
  pinMode(GPIO_INTERRUPT_PIN, INPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  digitalWrite(7, LOW);
  digitalWrite(8, HIGH);
  analogWrite(9, 180);
  attachInterrupt(digitalPinToInterrupt(GPIO_INTERRUPT_PIN), interrupt, FALLING);
}

void interrupt() {
  pulses++; //increase pulse number
  if (pulses == (PULSES_PER_ROTATION)/2 * REDUCTION)
    digitalWrite(8, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
}
