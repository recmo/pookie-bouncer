#include <heltec.h>
#include <driver/adc.h>
#include <driver/rmt.h>
#include <soc/gpio_struct.h>
#include <soc/sens_reg.h>
#include <soc/sens_struct.h>
#include <soc/rmt_reg.h>
#include <soc/rmt_struct.h>
#include <TMCStepper.h>

#include "pots.h"

#define PIN_DIR  GPIO_NUM_12
#define PIN_STEP GPIO_NUM_13
#define RMT_TX_CHANNEL (RMT_CHANNEL_0)

// UART unfortunately doesn't seem to run on a single
// INPUT_OUTPUT_OD pin. Instead we need to do this
// externally by connecting a resistor between TX and
// RX and connecting RX to the device.
#define PIN_UART_RX GPIO_NUM_14
#define PIN_UART_TX GPIO_NUM_27

TMC2209Stepper tmc2209(&Serial1, 0.11f, 0b00); // 0.11 Ohm Rsense, Address 00

char buffer[100];

#define MIN_DURATION 312
#define MAX_RMT_DURATION 32767
#define SWITCH_DIRECTION 65535

// TODO: Signed integers for direction
#define MOTOR_BUFFER_LEN 1024 // Must be power of two
uint32_t motor_buffer[MOTOR_BUFFER_LEN];
uint32_t motor_read = 0;
uint32_t motor_write = 0;

#define RMT_LEVEL_BIT 0x8000
uint32_t rmt_level_bit = RMT_LEVEL_BIT;

#define ACTION_NONE 0
#define ACTION_DIR_LOW 1
#define ACTION_DIR_HIGH 2
#define ACTION_STOP 3
uint8_t next_action = ACTION_NONE;

// Read next half of an rmt_item32_t from the motor buffer.
// TODO: Detect buffer over run
uint32_t next_half_item() {
  uint32_t duration = motor_buffer[motor_read];
  if (duration == 0) {
    // Do not advance motor_read, keep outputting zeros until it is handled
    
    // Set idle level to rmt_level_bit to keep the signal going.
    // TODO: We need to set this while it it transmitting, or things go wrong!
    // RMT.conf_ch[0].conf1.idle_out_lv = ~rmt_level_bit >> 15; // OPT: We only need to do this once.

    // Infinitely repeat last bit
    // return MAX_RMT_DURATION | ~rmt_level_bit;

    // Store action
    next_action = ACTION_DIR_HIGH;

    // Return zero duration to stop transmission and trigger end event
    motor_read += 1;
    motor_read &= MOTOR_BUFFER_LEN - 1;
    return 0 | (rmt_level_bit ^ RMT_LEVEL_BIT);
    
  } else if (duration <= MAX_RMT_DURATION) {
    motor_read += 1;
    motor_read &= MOTOR_BUFFER_LEN - 1;
    uint32_t result = duration | rmt_level_bit;
    rmt_level_bit ^= RMT_LEVEL_BIT; // Flip level bit
    return result;
  } else {
    motor_buffer[motor_read] = duration - MAX_RMT_DURATION;
    return MAX_RMT_DURATION | rmt_level_bit;
  }
}

uint32_t rmt_buffer_offset = 0;

void IRAM_ATTR fill_half_buffer() {
  // Fill buffer
  for (int i = 0; i < 32; i++) {
    RMTMEM.chan[0].data32[i + rmt_buffer_offset].val = next_half_item() | (next_half_item() << 16);
    if (false) {
      Serial.print(RMTMEM.chan[0].data32[i + rmt_buffer_offset].duration0);
      Serial.print(" ");
      Serial.print(RMTMEM.chan[0].data32[i + rmt_buffer_offset].level0);
      Serial.print(" ");
      Serial.print(RMTMEM.chan[0].data32[i + rmt_buffer_offset].duration1);
      Serial.print(" ");
      Serial.println(RMTMEM.chan[0].data32[i + rmt_buffer_offset].level1);
    }
  }
  rmt_buffer_offset += 32;
  rmt_buffer_offset &= 63;
}

void IRAM_ATTR on_rmt(void* arg) {
  //Serial.println("Event:");
  if (RMT.int_st.ch0_tx_end) {
    // NOTE: The RMT is currently in idle and outputting the idle level
    // (which should match the last step level). The interupt handling will
    // thus elongate the last step, so we want to do it as quick as possible.
    switch (next_action) {
    case ACTION_DIR_LOW:
      // TMC2209 requires 20ns setup and hold time on DIR.
      // This interupt handler does 1200ns hold and 300ns setup at best,
      // and that is with direct register access.
      GPIO.out_w1tc = 1 << PIN_DIR;
      RMT.conf_ch[0].conf1.tx_start = 1; // Continue immediately
      break;
    case ACTION_DIR_HIGH:
      GPIO.out_w1ts = 1 << PIN_DIR;
      RMT.conf_ch[0].conf1.tx_start = 1; // Continue immediately
      break;
    case ACTION_STOP:
      // Do noting
      break;
    default:
      RMT.conf_ch[0].conf1.tx_start = 1; // Continue immediately
      break;
    }
    // Clear event
    RMT.int_clr.ch0_tx_end = 1;

    // TODO: Pop `next_action` value. (and also idle_lv?)
  }
  if (RMT.int_st.ch0_tx_thr_event) {
    // Clear event
    RMT.int_clr.ch0_tx_thr_event = 1;
    //Serial.println("Treshold event:");
    
    fill_half_buffer();  
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

  // Configure motor parameters
  // See https://www.omc-stepperonline.com/download/17HS19-2004S1.pdf
  // See https://www.omc-stepperonline.com/download/17HS19-2004S1_Torque_Curve.pdf
  double max_rpm       = 600;  // rpm
  double max_torque    = 30;   // N cm
  double inertial_mass = 82.0; // g cm2

  // Configure UART
  Serial1.begin(115200, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);
  tmc2209.begin();
  Serial.println(tmc2209.test_connection());

  // Configure TMC2209
  Serial.print(" read:ms=");
  Serial.println(tmc2209.microsteps()); 
  tmc2209.dedge(true); // Step on both edges
  

  // Configure RMT for driving STEP pulses
  Serial.println("Configuring RMT");
  rmt_config_t config = {
    .rmt_mode         = RMT_MODE_TX,
    .channel          = RMT_CHANNEL_0,
    .clk_div          = 8, // 80 MHz / 80 = 1MHz or 1 μs per tick
    .gpio_num         = PIN_STEP,
    .mem_block_num    = 1,
  };
  config.tx_config = {
    .loop_en              = false, // Loop over ringbuffer
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

  // Clear motor buffer
  Serial.println("Initializing buffer");
  for (int j = 0; j < MOTOR_BUFFER_LEN; j++)
    motor_buffer[j] = 2;

  motor_buffer[5] = 1;
  motor_buffer[6] = 0;
  motor_buffer[7] = 1;
 
  // Load buffer
  Serial.println("Loading buffer");
  // TODO: Don't have initial commands in buffer
  fill_half_buffer();
  fill_half_buffer();

  // Enable wraparound mode
  RMT.apb_conf.mem_tx_wrap_en = 1;

  Serial.println("Registering interrupts");
  // Enable end event (zero duration)
  RMT.int_clr.ch0_tx_end = 1;
  RMT.int_ena.ch0_tx_end = 1;
  
  // Enable treshold event every 32 events (half buffer)
  RMT.int_clr.ch0_tx_thr_event = 1;
  RMT.int_ena.ch0_tx_thr_event = 1;
  RMT.tx_lim_ch[0].limit = 32;

  // Start pulsing!
  Serial.println("Starting..");
  RMT.conf_ch[0].conf1.mem_rd_rst = 1;

  // TODO: Compute actual idle level (0 or 1)

  // It is important that idle_out_lv and tx_start are set at the same time.
  // Setting tx_start first and idle_out_lv second causes the value not to be picked up
  // for very short pulse sequences.
  // Setting idle_out_lv first causes the first step to take much longer.
  // Setting them together like this causes the first step to take about 100ns longer,
  // which is acceptable.
  volatile uint32_t* conf1_reg = &RMT.conf_ch[0].conf1.val;
  uint32_t conf1 = *conf1_reg;
  conf1 |= 1 << 0; // conf1.tx_start = 1
  conf1 |= 1 << 18; // conf1.idle_out_lv = 1;
  *conf1_reg = conf1;
}

uint32_t motor_time = 0;     // Time in 100ns
int32_t motor_position = 0;  // Position in 1/32000 revolutions
bool motor_direction = true; // True for positive position direction

inline unsigned int motor_buffer_space() {
  return (motor_read - motor_write) & (MOTOR_BUFFER_LEN - 1);
}

inline bool can_write_pulse() {
  return motor_buffer_space() >= 2;
}

// Writes the pulse to the motor buffer and updates time, position and direction.
inline void write_pulse(bool step_direction, uint16_t duration) {
  // Conditionally write direction change
  if (step_direction != motor_direction) {
    Serial.println("Reverse");
    motor_buffer[motor_write] = SWITCH_DIRECTION;
    motor_write += 1;
    motor_write &= MOTOR_BUFFER_LEN - 1;
  }

  // Cap duration
  if (duration < MIN_DURATION) {
    Serial.println("Under");
    duration = MIN_DURATION;
  }

  // Write pulse
  motor_buffer[motor_write] = duration;
  motor_write += 1;
  motor_write &= MOTOR_BUFFER_LEN - 1;

  // Update state
  motor_time += duration;
  motor_position += step_direction ? 1 : -1;
  motor_direction = step_direction;
}

float amplitude = 1.0;
float omega = 1.0;

float trajectory(uint32_t t) {
  // TODO: Don't use instantenous velocity but solve over range.
  float t_sec = t * 1e-7 * omega; // Time in seconds
  float revolutions = amplitude * sin(t_sec); // Position in revolutions
  float steps = revolutions * 3200.0; // Position in steps
  return steps;
}

uint tick = 0;

void loop() {
  // Update potentiometer readings
  update_pots();

  // Serial.println((motor_read - motor_write) & (MOTOR_BUFFER_LEN - 1));

  // UART
  if (Serial2.available()) {
    Serial.print("UART: ");
    while (Serial2.available()) {
      uint8_t rbyte = Serial2.read();
      sprintf(buffer, "%02x ", rbyte);
      Serial.print(buffer);
    }
    Serial.println("");
  }

  while (can_write_pulse()) { // TODO: Don't write more than ~100 ms
    if (motor_time > M_PI * 2e7 / omega) {
      motor_time -= M_PI * 2e7 / omega;
      // This is the time to commit to the new amplitude and omega.
      amplitude = 3.0 * POT(0);
      omega = 10.0 * POT(1);
    }

    float current_position = trajectory(motor_time);
    uint16_t duration = 32767;
    float new_position = trajectory(motor_time + duration);
    bool step_direction = new_position > current_position;
    if (fabs(new_position - current_position) > 1) {
      duration = float(duration) / fabs(new_position - current_position);
      if (false && duration < 32767 / 4) {
        duration *= 2;
        float new_position = trajectory(motor_time + duration);
        duration = float(duration) / fabs(new_position - current_position);
      }
    }
    // TODO: Use float to compute more accurate position.
    if (motor_time - tick > 100000) { // 100 Hz
      // Serial.println(duration);
      tick = motor_time;
    }
    write_pulse(step_direction, duration);
    // write_pulse(true, MAX_DURATION);
  }

  // Display potentiometer readings
  sprintf(buffer, "%.3f %.3f %.3f %.3f", POT(0), POT(1), POT(2), POT(3));
  Heltec.display->clear();
  Heltec.display->drawString(0, 0, buffer);
  Heltec.display->display();
  // Serial.println(buffer);

  delay(10);
}
