#include <heltec.h>
#include <driver/adc.h>
#include <driver/rmt.h>
#include <soc/sens_reg.h>
#include <soc/sens_struct.h>
#include <soc/rmt_reg.h>
#include <soc/rmt_struct.h>

#include "pots.h"

#define PIN_DIR  12
#define PIN_STEP GPIO_NUM_13


char buffer[100];

rmt_config_t step_rmt_config;

rmt_item32_t step_rmt_items[] = {
  // duration, level, duration, level (duration 15 bit in 1 μs ticks)
  {{{200, 1, 200, 0}}}, // 200 μs HIGH, 200 μs LOW
};

void IRAM_ATTR on_rmt() {
   portENTER_CRITICAL_ISR(&timer_mutex);
   
   
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

  // Configure potentiometers
  setup_pots();

  // Configure direction pin
  pinMode(PIN_DIR, OUTPUT);

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
  step_rmt_config.clk_div = 8; // 80 MHz / 8 = 10MHz or .1 μs per tick
  rmt_config(&step_rmt_config);
  rmt_driver_install(step_rmt_config.channel, 0, 0);  //  rmt_driver_install(rmt_channel_t channel, size_t rx_buf_size, int rmt_intr_num)
  
  // Spin motor
  rmt_write_items(step_rmt_config.channel, step_rmt_items, 1, 0);
  //rmt_fill_tx_items(step_rmt_config.channel, step_rmt_items, 1, 0);
  //RMT.conf_ch[step_rmt_config.channel].conf1.mem_rd_rst = 1;
  //RMT.conf_ch[step_rmt_config.channel].conf1.tx_start   = 1;

  // Interupt after 32000 steps
  // rmt_isr_register(&on_rmt, NULL, 0, NULL);
  // rmt_rmt_set_tx_thr_intr_en(step_rmt_config.channel, true, 32000);
}

void loop() {
  // Update potentiometer readings
  update_pots();

  // Display potentiometer readings
  sprintf(buffer, "%.3f %.3f %.3f %.3f", POT(0), POT(1), POT(2), POT(3));
  Heltec.display->clear();
  Heltec.display->drawString(0, 0, buffer);
  Heltec.display->display();
  // Serial.println(buffer);

  float knob = 2.0 * kf[0].estimate - 1.0;
  float speed_rps = knob * 10.0; // 0..10 revolutions per second
  float speed_pps = speed_rps * 200 * 16; // pulses per second
  float pulse_uspp = 1e7 / speed_pps; // microseconds per pulse
  uint16_t uspp_int = pulse_uspp > 0 ? pulse_uspp : -pulse_uspp;
  if (speed_rps == 0.0) {
    // TODO: Stop entirely
    pulse_uspp = 32767;
  }
  if (pulse_uspp >= 32767.0 || pulse_uspp <= -32767.0) {
    pulse_uspp = 32767;
  }
  Serial.println(uspp_int);
  
  // Update motor speed
  if (speed_rps >= 0) {
    digitalWrite(PIN_DIR, HIGH);
  } else {
    digitalWrite(PIN_DIR, LOW);
  }
  step_rmt_items[0].duration0 = uspp_int / 2;
  step_rmt_items[0].duration1 = uspp_int - step_rmt_items[0].duration0;

  //rmt_fill_tx_items(step_rmt_config.channel, step_rmt_items, 1, 0);

  // IRAM version of rmt_fill_tx_items(step_rmt_config.channel, step_rmt_items, 1, 0);
  RMTMEM.chan[step_rmt_config.channel].data32[0].val = step_rmt_items[0].val;
  
  delay(10);
}
