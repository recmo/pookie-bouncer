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
#define RMT_TX_CHANNEL (RMT_CHANNEL_0)

char buffer[100];


// A single pulse counts as two events. The maximum number of pulses we
// can set the interupt alarm for is 255. At 600 rpm this corresponds to
// 126 Hz interupts, or every 8 ms.
#define MAX_THRESHOLD 510
#define MAX_PULSES 255


#define MOTOR_BUFFER_LEN 4 // Must be power of two

struct motor_command_t {
  uint32_t count;     // Pulse count
  uint16_t duration;  // Pulse duration in 100ns increments
};

motor_command_t motor_buffer[MOTOR_BUFFER_LEN] = {
  {200 * 16, 2000},
  {200 * 16, 1000},
  {200 * 16, 500},
  {200 * 16, 1000},
};
uint16_t motor_read = 0;
uint32_t counter = 0;


void IRAM_ATTR load_next_command() {
  // Set count
  counter = motor_buffer[motor_read].count;
  if (counter > MAX_PULSES) {
    RMT.tx_lim_ch[0].limit = MAX_THRESHOLD;
  } else {
    RMT.tx_lim_ch[0].limit = counter * 2;
  }

  // Set pulse duration
  uint16_t duration = motor_buffer[motor_read].duration;
  uint16_t duration0 = duration / 2;
  uint16_t duration1 = duration - duration0;
  rmt_item32_t pulse = {{{duration0, 1, duration1, 0}}};
  RMTMEM.chan[0].data32[0].val = pulse.val;

  // TODO: Direction.
  // TODO: Stopping.
  // TODO: This is constant velocity, add constant acceleration and constant jerk modes?
  
  // Advance ring buffer
  motor_read += 1;
  motor_read &= MOTOR_BUFFER_LEN - 1;
}

void IRAM_ATTR on_rmt(void* arg) {
   // Clear event
   RMT.int_clr.ch0_tx_thr_event = 1;

   // Update counter
   if (counter > MAX_PULSES) {
     counter -= MAX_PULSES;
   } else {
     counter = 0;
   }
   if (counter == 0) {
      load_next_command();
   } else if (counter < MAX_PULSES) {
     // Continue running for remaining amount
     RMT.tx_lim_ch[0].limit = counter * 2;
   }
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
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);

  // Configure RMT for driving STEP pulses
  rmt_config_t config = {
    .rmt_mode         = RMT_MODE_TX,
    .channel          = RMT_CHANNEL_0,
    .clk_div          = 8, // 80 MHz / 8 = 10MHz or .1 μs per tick
    .gpio_num         = PIN_STEP,
    .mem_block_num    = 1,
  };
  config.tx_config = {
    .loop_en              = true, // Repeat
    .carrier_freq_hz      = 0,
    .carrier_duty_percent = 50, // %
    .carrier_level        = RMT_CARRIER_LEVEL_HIGH,
    .carrier_en           = false,
    .idle_level           = RMT_IDLE_LEVEL_LOW,
    .idle_output_en       = true,
  };
  ESP_ERROR_CHECK(rmt_config(&config));
  ESP_ERROR_CHECK(rmt_set_source_clk(RMT_TX_CHANNEL, RMT_BASECLK_APB)); // 80 Mhz.
  ESP_ERROR_CHECK(rmt_isr_register(on_rmt, NULL, ESP_INTR_FLAG_LEVEL1, 0));

  // Fill the entire block with 'end of block' markers.
  rmt_item32_t endSentinel = {{{ 0, 0, 0, 0 }}};
  for (int j = 0 ; j < 64; j++)
    RMTMEM.chan[config.channel].data32[j].val = endSentinel.val;

  load_next_command();

  // Enable treshold event
  RMT.int_clr.ch0_tx_thr_event = 1;
  RMT.int_ena.ch0_tx_thr_event = 1;

  // Start pulsing!
  RMT.conf_ch[0].conf1.tx_start = 1;
}

void loop() {
  // Update potentiometer readings
  update_pots();

  Serial.print(motor_read);
  Serial.print(" ");
  Serial.println(counter);

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
  // Serial.println(uspp_int);
  
  // Update motor speed
  if (speed_rps >= 0) {
    digitalWrite(PIN_DIR, HIGH);
  } else {
    digitalWrite(PIN_DIR, LOW);
  }

  delay(10);
}
