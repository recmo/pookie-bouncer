// Driver for the Potentiometers
//
// Pots can be read at any time using POT(n) which returns a [0, 1] value
// for pot n as a float.
//
// Uses ADCs with 1kHz sampling and Kalman filtering.
//

#pragma once
#include <driver/adc.h>
#include <soc/sens_reg.h>
#include <soc/sens_struct.h>

#define PIN_POT1 36
#define PIN_POT2 37
#define PIN_POT3 38
#define PIN_POT4 39

#define POT(n) kf[n].estimate

#define ADC_SAMPLES_COUNT 1024 // Must be power of two

struct kalman_filter_t {
  float estimate;
  float err_estimate;
};

const float pot_scale = 1.0 / 4096.0;
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

void setup_pots() {
  // Configure ADC for reading pots
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_MAX);
  adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_MAX);
  adc1_config_channel_atten(ADC1_CHANNEL_2, ADC_ATTEN_MAX);
  adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_MAX);

  // Fetch now so any setup is done and we can safely use iram_adc1_get_raw
  adc1_get_raw(ADC1_CHANNEL_0);
  adc1_get_raw(ADC1_CHANNEL_1);
  adc1_get_raw(ADC1_CHANNEL_2);
  adc1_get_raw(ADC1_CHANNEL_3);

  // Configure timer interupt
  adc_timer = timerBegin(3, 80, true); // 80 MHz / 80 = 1 MHz hardware clock for easy figuring
  timerAttachInterrupt(adc_timer, &on_timer, true); // Attaches the handler function to the timer 
  timerAlarmWrite(adc_timer, 1000, true); // Interrupts at 1 Mhz / 1000 = 1 kHz
  timerAlarmEnable(adc_timer);
}

void update_pots() {
  // Read the ADC ring buffer and update Kalman filters
  while (adc_read != adc_write) {
    kalman_update(&kf[0], pot_scale *adc_buffer[adc_read][0]);
    kalman_update(&kf[1], pot_scale *adc_buffer[adc_read][1]);
    kalman_update(&kf[2], pot_scale *adc_buffer[adc_read][2]);
    kalman_update(&kf[3], pot_scale *adc_buffer[adc_read][3]);
    adc_read += 1;
    adc_read &= ADC_SAMPLES_COUNT - 1;
  }
}
