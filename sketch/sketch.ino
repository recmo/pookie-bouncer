#include <heltec.h>
#include <driver/adc.h>
#include <driver/rmt.h>
#include <soc/sens_reg.h>
#include <soc/sens_struct.h>
#include <SimpleKalmanFilter.h>

#define PIN_POT1 36
#define PIN_POT2 37
#define PIN_POT3 38
#define PIN_POT4 39
#define PIN_DIR  12
#define PIN_STEP GPIO_NUM_13

rmt_config_t step_rmt_config;

rmt_item32_t step_rmt_items[] = {
  // duration, level, duration, level (duration 15 bit in 1 μs ticks)
  {{{200, 1, 200, 0}}}, // 200 μs HIGH, 200 μs LOW
};

// SimpleKalmanFilter(e_mea, e_est, q);
// e_mea: Measurement Uncertainty 
// e_est: Estimation Uncertainty 
// q: Process Noise

uint32_t pot_index = 0;
uint16_t pot_samples[4][64];
int last_time = 0;

const float pot_scale = 1.0 / 4096.0;

struct kalman_filter_t {
  float estimate;
  float err_estimate;
};
const float kalman_measurement_error = 0.05; // Sensor noise
const float kalman_q = 0.02; // Process noise
kalman_filter_t kf[] = {
  {0.5, kalman_measurement_error},
  {0.5, kalman_measurement_error},
  {0.5, kalman_measurement_error},
  {0.5, kalman_measurement_error},
};

hw_timer_t* adc_timer = NULL;
portMUX_TYPE DRAM_ATTR timer_mutex = portMUX_INITIALIZER_UNLOCKED; 


#define ADC_SAMPLES_COUNT 1024 // Must be power of two
uint16_t adc_buffer[ADC_SAMPLES_COUNT][4];
uint16_t adc_write = 0;
uint16_t adc_read = 0;

void kalman_update(kalman_filter_t* filter, float value) {
  float kalman_gain = filter->err_estimate / (filter->err_estimate + kalman_measurement_error);
  float delta_est = kalman_gain * (value - filter->estimate);
  filter->estimate += delta_est;
  
  if (delta_est < 0.0) {
    delta_est = -delta_est;
  }
  filter->err_estimate *= 1.0 - kalman_gain;
  filter->err_estimate += delta_est * kalman_q;
}

// Simplified version of `adc1_get_raw` for IRAM use
// Requires adc1_get_raw to be called at least once to do the setup
int IRAM_ATTR iram_adc1_get_raw(int channel) {
    uint16_t adc_value;
    SENS.sar_meas_start1.sar1_en_pad = (1 << channel); // only one channel is selected
    while (SENS.sar_slave_addr1.meas_status != 0);
    SENS.sar_meas_start1.meas1_start_sar = 0;
    SENS.sar_meas_start1.meas1_start_sar = 1;
    while (SENS.sar_meas_start1.meas1_done_sar == 0);
    adc_value = SENS.sar_meas_start1.meas1_data_sar;
    return adc_value;
}

void IRAM_ATTR on_timer() {
   portENTER_CRITICAL_ISR(&timer_mutex);
   
   // Read pots to ring buffer
   adc_buffer[adc_write][0] = iram_adc1_get_raw(ADC1_CHANNEL_0);
   adc_buffer[adc_write][1] = iram_adc1_get_raw(ADC1_CHANNEL_1);
   adc_buffer[adc_write][2] = iram_adc1_get_raw(ADC1_CHANNEL_2);
   adc_buffer[adc_write][3] = iram_adc1_get_raw(ADC1_CHANNEL_3);
   adc_write += 1;
   adc_write &= ADC_SAMPLES_COUNT - 1;
   
   portEXIT_CRITICAL_ISR(&timer_mutex);
}

void setup() {

  Heltec.begin(true /*DisplayEnable Enable*/, false /*LoRa Enable*/, true /*Serial Enable*/);
  delay(300);

  // Conf
  Heltec.display->clear();
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  Heltec.display->drawString(0, 0, "Start Measuring....");
  Heltec.display->display();

  // Configure ADC for reading pots
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_MAX);
  adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_MAX);
  adc1_config_channel_atten(ADC1_CHANNEL_2, ADC_ATTEN_MAX);
  adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_MAX);
  // Fetch now so any setup is done and we can use iram_adc1_get_raw
  adc1_get_raw(ADC1_CHANNEL_0);
  adc1_get_raw(ADC1_CHANNEL_1);
  adc1_get_raw(ADC1_CHANNEL_2);
  adc1_get_raw(ADC1_CHANNEL_3);

  // Configure RMT for driving STEP pulses
  step_rmt_config.rmt_mode = RMT_MODE_TX;
  step_rmt_config.channel = RMT_CHANNEL_0;
  step_rmt_config.gpio_num = PIN_STEP;
  step_rmt_config.mem_block_num = 1;
  step_rmt_config.tx_config.loop_en = 1; // Repeat
  step_rmt_config.tx_config.carrier_en = 0;
  step_rmt_config.tx_config.idle_output_en = 1;
  step_rmt_config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
  step_rmt_config.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;
  step_rmt_config.clk_div = 80; // 80M Hz / 80 = 1MHz or 1 μs per tick
  rmt_config(&step_rmt_config);
  rmt_driver_install(step_rmt_config.channel, 0, 0);  //  rmt_driver_install(rmt_channel_t channel, size_t rx_buf_size, int rmt_intr_num)
  
  // Spin motor
  rmt_write_items(step_rmt_config.channel, step_rmt_items, 1, 0);

  // Configure timer interupt
  adc_timer = timerBegin(3, 80, true); // 80 MHz / 80 = 1 MHz hardware clock for easy figuring
  timerAttachInterrupt(adc_timer, &on_timer, true); // Attaches the handler function to the timer 
  timerAlarmWrite(adc_timer, 1000, true); // Interrupts at 1 Mhz / 1000 = 1 kHz
  timerAlarmEnable(adc_timer);
}


char buffer[100];

void loop() {
  // Serial.println(adc_write);

  // Read the ADC ring buffer and update Kalman filters
  while (adc_read != adc_write) {
    kalman_update(&kf[0], pot_scale *adc_buffer[adc_read][0]);
    kalman_update(&kf[1], pot_scale *adc_buffer[adc_read][1]);
    kalman_update(&kf[2], pot_scale *adc_buffer[adc_read][2]);
    kalman_update(&kf[3], pot_scale *adc_buffer[adc_read][3]);
    adc_read += 1;
    adc_read &= ADC_SAMPLES_COUNT - 1;
  }

  sprintf(buffer, "%.4f %.4f %.4f %.4f\n", kf[0].estimate, kf[1].estimate, kf[2].estimate, kf[3].estimate);
  Heltec.display->clear();
  Heltec.display->drawString(0, 0, buffer);
  Heltec.display->display();
  Serial.print(buffer);

  // Update motor speed
  // 1 rps => 200 * 16 = 32000 pps => 31.5 μspp
  step_rmt_items[0].duration0 = 20 + 400 * 0.5;
  step_rmt_items[0].duration1 = 20 + 400 * 0.5;
  rmt_write_items(step_rmt_config.channel, step_rmt_items, 1, 0);

  delay(10);
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
