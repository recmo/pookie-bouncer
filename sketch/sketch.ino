#include <heltec.h>
#include <driver/adc.h>
#include <AccelStepper.h>

#define PIN_POT1 36
#define PIN_POT2 37
#define PIN_POT3 38
#define PIN_POT4 39
#define PIN_DIR  12
#define PIN_STEP 13

AccelStepper stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIR);

void setup() {
  pinMode(LED,OUTPUT);
  digitalWrite(LED,HIGH);

  Heltec.begin(true /*DisplayEnable Enable*/, false /*LoRa Enable*/, true /*Serial Enable*/);

  delay(300);
  Heltec.display->clear();
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  Heltec.display->drawString(0, 0, "Start Measuring....");
  Heltec.display->display();

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_MAX);
  adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_MAX);
  adc1_config_channel_atten(ADC1_CHANNEL_2, ADC_ATTEN_MAX);
  adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_MAX);

  // set the maximum speed, acceleration factor,
  // initial speed and the target position
  stepper.setMaxSpeed(32000); // 600 rpm = 10 hz * 200 * 16
  stepper.setAcceleration(32000 / 0.2); // 100% speed in 0.2 sec
  stepper.setSpeed(32000); // 100%
  stepper.moveTo(200 * 16 / 2);
}

uint32_t pot_index = 0;
uint16_t pot_samples[4][64];
int last_time = 0;

void loop() {
  // Update POTS
  pot_samples[0][pot_index] = adc1_get_raw(ADC1_CHANNEL_0);
  pot_samples[1][pot_index] = adc1_get_raw(ADC1_CHANNEL_1);
  pot_samples[2][pot_index] = adc1_get_raw(ADC1_CHANNEL_2);
  pot_samples[3][pot_index] = adc1_get_raw(ADC1_CHANNEL_3);
  pot_index += 1;
  pot_index &= 63;

  // Compute moving average
  uint32_t val0 = 0;
  uint32_t val1 = 0;
  uint32_t val2 = 0;
  uint32_t val3 = 0;
  for (uint32_t i = 0; i < 64; i++) {
    val0 += pot_samples[0][i];
    val1 += pot_samples[1][i];
    val2 += pot_samples[2][i];
    val3 += pot_samples[3][i];
  }
  float pot0 = val0 / (64.0 * 4096.0);
  float pot1 = val1 / (64.0 * 4096.0);
  float pot2 = val2 / (64.0 * 4096.0);
  float pot3 = val3 / (64.0 * 4096.0);

  // Compute loop time
  int time_now = millis();
  int loop_time = time_now - last_time;
  last_time = time_now;
  Serial.println(loop_time);

  /*
  char buffer[100];
  sprintf(buffer, "%.4f %.4f\n%.4f %.4f\n%d", pot0, pot1, pot2, pot3, loop_time);
  Heltec.display->clear();
  Heltec.display->drawString(0, 0, buffer);
  Heltec.display->display();
  */
  
  // Spin motor
  if (stepper.distanceToGo() == 0) 
    stepper.moveTo(-stepper.currentPosition());
    
  // Move the motor one step
  stepper.run();
}

double read_pot(byte pin){
  analogRead(pin); // Read twice, take second reading
  double reading = analogRead(pin); // Reference voltage is 3v3 so maximum reading is 3v3 = 4095 in range 0 to 4095
  double result = -0.000000000000016;
  result *= reading; result += 0.000000000118171;
  result *= reading; result -= 0.000000301211691;
  result *= reading; result += 0.001109019271794;
  result *= reading; result += 0.034143524634089;
  return result * 100.0 / 3.3;
}
