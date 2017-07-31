#define ADC_PIN A0
#define VOLTAGE_HIGH 5
#define ADC_RESOLUTION 1023
/* 10-bit adc means adc conversion values are in the range 0 : 2^10 - 1 = 1023 */

unsigned int adc_value = 0;
float voltage = 0;

void setup() {
  Serial.begin(9600);
  /*configure desired pin as adc input */
  pinMode(ADC_PIN, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  /* get an adc sample between 0 and 1023 */
  adc_value = analogRead(ADC_PIN);
  /* calculate corresponding voltage */
  voltage = (float) adc_value * VOLTAGE_HIGH/ADC_RESOLUTION;
  /* print the two values on the serial monitor*/
  Serial.println(adc_value);
  Serial.println(voltage);
  delay(1000);
}
