#define DEGREES_PER_ROTATION 360
#define PULSES_PER_ROTATION 24
#define REDUCTION 34
#define POTENTIOMETER_PIN A0
#define CURRENT_SENSOR_PIN A1
#define HALL_SENSOR_PIN1 2
#define HALL_SENSOR_PIN2 3
#define DIRECTION_PIN1 7
#define DIRECTION_PIN2 8
#define MOTOR_PWM_PIN 9
#define MASTER_SWITCH_PIN1 40
#define MASTER_SWITCH_PIN2 41
#define PUMP_BUTTON_PIN 29
#define PUMP_ENGINE_PWM_PIN 30
#define PUMP_ENGINE_DIRECTION_PIN1 35
#define PUMP_ENGINE_DIRECTION_PIN2 36
#define SWYPE_ONCE_BUTTON_PIN 31
#define DEFROST_LED 32
#define CONSTANT_SPEED_MOD1 170
#define N 3
#define ARRAY_SIZE 50
#define CURRENT_THRESHOLD 370
#define ZERO_CURRENT_VALUE 507
#define SWYPE_ANGLE 40
#define CAPACITIVE_SENSOR_PIN 5

#include "NexButton.h"
#include "NexText.h"
#include "Nextion.h"
#include "TimerOne.h"

volatile long pos = 0;
bool dir = false;
bool isFrozen = false;
bool enable = true;
int mode = 0;
bool prev_state = false;
int currentArray[50];
byte counter = 0;
float current_average = 0;
byte delay_counter = 0;

NexButton b0 = NexButton(0, 1, "b0");
NexText t1 = NexText(0, 3, "t1");

char buffer[10] = {0};


NexTouch *nex_Listen_List[] =
{
  &b0,
  &t1,

  NULL
};
/**
   Button to return the response.

   @param ptr - the parameter was transmitted to pop event function pointer.

*/
void b0PushCallback(void *ptr)
{
  digitalWrite(6, HIGH);
}

void b0PopCallback(void *ptr)
{
  digitalWrite(6, LOW);
}

long pulsesToDegrees(long pulses) {
  return (long)(pulses * DEGREES_PER_ROTATION) / (PULSES_PER_ROTATION * REDUCTION);
}

long degreesToPulses(long degrees) {
  return (long)(PULSES_PER_ROTATION * REDUCTION * degrees) / (DEGREES_PER_ROTATION);
}

void hall_sensor_interrupt() {
  if (dir == false)
    pos++;
  else
    pos--;
}

void set_direction() {
  if (dir == false) {
    digitalWrite(DIRECTION_PIN1, LOW);
    digitalWrite(DIRECTION_PIN2, HIGH);
  }
  else {
    digitalWrite(DIRECTION_PIN1, HIGH);
    digitalWrite(DIRECTION_PIN2, LOW);
  }
}

void swype(long position, int speed) {
  int duty;
  long prev_pos = 0;
  long pulses_limit = degreesToPulses(position);//90

  dir = false;
  set_direction();

  while ((pos < pulses_limit) && (isFrozen == false)) {
    if (N*pos < pulses_limit) {
      analogWrite(MOTOR_PWM_PIN, speed);
    }
    else {
      duty = (int)((double)N * speed / (N - 1) * (1 - (double)pos / pulses_limit));
      if (duty > speed)
        duty = speed;
      if (duty < 0)
        duty = 0;
      analogWrite(MOTOR_PWM_PIN, duty);
      if ((prev_pos == pos) && (isFrozen == false)) {
        if (pos < pulses_limit - 60) {
          //duty = duty + 5;
          analogWrite(MOTOR_PWM_PIN, duty);
        }
        else {
          analogWrite(MOTOR_PWM_PIN, 0);
          break;
        }
      }
      prev_pos = pos;
    }
  }
  // so far so good
  delay(300);

  dir = true;
  set_direction();

  while ((pos > 0) && (isFrozen == false)) {
    if ((N * pos) > ((N - 1) * pulses_limit)) {
      analogWrite(MOTOR_PWM_PIN, speed);
    }
    else {
      duty = (int)((double)(N / (N - 1) * speed) * (double)pos / pulses_limit);
      if (duty > speed)
        duty = speed;
      if (duty < 0)
        duty = 0;
      analogWrite(MOTOR_PWM_PIN, duty);
      if ((prev_pos == pos) && (isFrozen == false)) {
        if (pos > 60) {
          //duty = duty + 5;
          analogWrite(MOTOR_PWM_PIN, duty);
        }
        else {
          analogWrite(MOTOR_PWM_PIN, 0);
          break;
        }
      }
      prev_pos = pos;
    }
  }
  delay(300);
}

void checkMasterSwitch() {
  if ((digitalRead(MASTER_SWITCH_PIN1) == HIGH) && (digitalRead(MASTER_SWITCH_PIN2) == HIGH))
    mode = 1;
  else if (digitalRead(MASTER_SWITCH_PIN1) == LOW)
    mode = 0;
  else if (digitalRead(MASTER_SWITCH_PIN2) == LOW)
    mode = 2;
}

int convertPotentiometerValueToSpeed(int potentiometerValue) {
  return (int)potentiometerValue / 10 + 160;
}

void chooseMode() {
  switch (mode) {
    case 0:
      checkSwypeButton();
      checkPumpButton();
      break;
    case 1:
      if (enable == true) {
        swype(SWYPE_ANGLE, CONSTANT_SPEED_MOD1);
      }
      break;
    case 2:
      if (enable == true) {
        int x;
        x = convertPotentiometerValueToSpeed(analogRead(POTENTIOMETER_PIN));
        if (x > 255)
          x = 255;
        if (x < CONSTANT_SPEED_MOD1)
          x = CONSTANT_SPEED_MOD1;
        swype(SWYPE_ANGLE, x);
      }
      break;
  }
}

void checkSwypeButton() {
  if ((digitalRead(MASTER_SWITCH_PIN1) == LOW) && (digitalRead(PUMP_BUTTON_PIN) == LOW))
  {
    if (digitalRead(SWYPE_ONCE_BUTTON_PIN) == LOW) {
      if (enable == true) {
        swype(SWYPE_ANGLE, CONSTANT_SPEED_MOD1);
      }
    }
  }
}

void checkCapacitiveSensor(){
  if (digitalRead(CAPACITIVE_SENSOR_PIN) == HIGH){
      if (enable == true) {
        swype(SWYPE_ANGLE, CONSTANT_SPEED_MOD1);
      }
  }
}

void checkPumpButton() {
  if (digitalRead(PUMP_BUTTON_PIN) == HIGH) {
    if (enable == true) {
      swype(SWYPE_ANGLE, CONSTANT_SPEED_MOD1);
    }
    prev_state = true;
  }
  else {
    if (prev_state == true) {
      swype(SWYPE_ANGLE, CONSTANT_SPEED_MOD1);
    }
    prev_state = false;
  }
}

float calculateAverage(int array[], int size) {
  int i;
  float avg = 0;
  for (i = 0; i < size; i++) {
    avg += (float)(array[i] / size);
  }
  return avg;
}

void addValue(int value, int array[], int position) {
  array[position] = value;
}

void timer1_interrupt() {
  int current = 10 * abs(analogRead(CURRENT_SENSOR_PIN) - ZERO_CURRENT_VALUE);
  if (counter < ARRAY_SIZE) {
    addValue(current, currentArray, counter);
    counter++;
  }
  if (counter >= ARRAY_SIZE) {
    counter = 0;
    current_average = calculateAverage(currentArray, ARRAY_SIZE);
    if (current_average > CURRENT_THRESHOLD) {
      isFrozen = true;
      enable = false;
      analogWrite(MOTOR_PWM_PIN, 0);
      digitalWrite(DEFROST_LED, HIGH);
    }
    else {
      isFrozen = false;
      delay_counter++;
      if (delay_counter == 10) {
        digitalWrite(DEFROST_LED, LOW);
        delay_counter = 0;
        enable = true;
      }
    }
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(HALL_SENSOR_PIN1, INPUT);
  pinMode(HALL_SENSOR_PIN2, INPUT);
  pinMode(DIRECTION_PIN1, OUTPUT);
  pinMode(DIRECTION_PIN2, OUTPUT);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(POTENTIOMETER_PIN, INPUT);
  pinMode(DEFROST_LED, OUTPUT);
  pinMode(PUMP_ENGINE_PWM_PIN , OUTPUT);
  pinMode(MASTER_SWITCH_PIN1, INPUT_PULLUP);
  pinMode(MASTER_SWITCH_PIN2, INPUT_PULLUP);
  pinMode(PUMP_BUTTON_PIN, INPUT);
  pinMode(SWYPE_ONCE_BUTTON_PIN , INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN1), hall_sensor_interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN2), hall_sensor_interrupt, RISING);
  pinMode(PUMP_ENGINE_PWM_PIN, OUTPUT);
  pinMode(PUMP_ENGINE_DIRECTION_PIN1, OUTPUT);
  pinMode(PUMP_ENGINE_DIRECTION_PIN2, OUTPUT);
  digitalWrite(PUMP_ENGINE_DIRECTION_PIN1, HIGH);
  digitalWrite(PUMP_ENGINE_DIRECTION_PIN2, LOW);
  Timer1.initialize(10000);
  Timer1.attachInterrupt(timer1_interrupt);
  nexInit();
  b0.attachPop(b0PopCallback, &b0);
  b0.attachPush(b0PushCallback, &b0);
  pinMode(6, OUTPUT);
  digitalWrite(6, LOW);

  pinMode(CAPACITIVE_SENSOR_PIN, INPUT);
}

void loop() {
  nexLoop(nex_Listen_List);
  int number = analogRead(POTENTIOMETER_PIN);
  memset(buffer, 0, sizeof(buffer)); // clear buffer
  itoa(number, buffer, 10);
  t1.setText(buffer);
  checkMasterSwitch();
  checkCapacitiveSensor();
  chooseMode();
}
