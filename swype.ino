#include <TimerOne.h>
#define DEGREES_PER_ROTATION 360
#define PULSES_PER_ROTATION 12
#define REDUCTION 34
#define HALL_SENSOR_PIN1 2
#define HALL_SENSOR_PIN2 3
#define DIRECTION_PIN1 7
#define DIRECTION_PIN2 8
#define MOTOR_PWM_PIN 9
int pos = 0;
int dir = 0;

long pulsesToDegrees(long pulses){
  return (pulses * DEGREES_PER_ROTATION)/(PULSES_PER_ROTATION * REDUCTION);
}

long degreesToPulses(long degrees){
  return (PULSES_PER_ROTATION * REDUCTION * degrees)/(DEGREES_PER_ROTATION);
}

void hall_sensor_interrupt1() {
  //pos++; //increase pulse number
}

void hall_sensor_interrupt2() {
  if (dir == 0)
    pos++; //increase pulse number
  else
    pos--;
}

void set_direction(){
  if (dir == 0){
    digitalWrite(DIRECTION_PIN1, LOW);
    digitalWrite(DIRECTION_PIN2, HIGH);
  }
  else{
    digitalWrite(DIRECTION_PIN1, HIGH);
    digitalWrite(DIRECTION_PIN2, LOW);
  }
}

void swype(int position, int speed){
  int overshoot;
  int pulses_limit = (int) degreesToPulses(position);
  dir = 0;
  set_direction();
  while (pos < pulses_limit){
    analogWrite(MOTOR_PWM_PIN, speed);
  }
  analogWrite(MOTOR_PWM_PIN, 0);
  overshoot = pos - pulses_limit;
  delay(600);
  Serial.println(pos);
  Serial.println(overshoot);
  dir = 1;
  set_direction();
  while (pos > overshoot){
    analogWrite(MOTOR_PWM_PIN, speed);
  }
  analogWrite(MOTOR_PWM_PIN, 0);
  Serial.println(pos);
}

void setup() {
  Serial.begin(9600);
  pinMode(HALL_SENSOR_PIN1, INPUT);
  pinMode(HALL_SENSOR_PIN2, INPUT);
  pinMode(DIRECTION_PIN1, OUTPUT);
  pinMode(DIRECTION_PIN2, OUTPUT);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  digitalWrite(DIRECTION_PIN1, LOW);
  digitalWrite(DIRECTION_PIN2, HIGH);
  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN1), hall_sensor_interrupt1, FALLING);
  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN2), hall_sensor_interrupt2, FALLING);
}

void loop() {
  // put your main code here, to run repeatedly:
  swype(90/2, 205);
  delay(1000);
}
