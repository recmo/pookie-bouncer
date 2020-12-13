// Driver for the STEP and DIR pins.
//
// 

#include <driver/rmt.h>
#include <soc/gpio_struct.h>
#include <soc/rmt_reg.h>
#include <soc/rmt_struct.h>

#define PIN_DIR  GPIO_NUM_12
#define PIN_STEP GPIO_NUM_13
#define RMT_TX_CHANNEL (RMT_CHANNEL_0)
#define MIN_DURATION 312
#define MAX_RMT_DURATION 32767
#define SWITCH_DIRECTION 65535
#define MOTOR_BUFFER_LEN 1024 // Must be power of two

#define ACTION_NONE 0
#define ACTION_DIR_LOW 1
#define ACTION_DIR_HIGH 2
#define ACTION_STOP 3

#define RMT_LEVEL_BIT 0x8000

// TODO: Signed integers for direction
uint32_t motor_buffer[MOTOR_BUFFER_LEN];
uint32_t motor_read = 0;
uint32_t motor_write = 0;
uint32_t rmt_level_bit = RMT_LEVEL_BIT;

uint8_t next_action = ACTION_NONE;

uint32_t rmt_buffer_offset = 0;

void step_dir_install() {
  // Configure direction pin
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);

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
    .loop_en              = false,
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
    motor_buffer[j] = j;

  //motor_buffer[5] = 1;
  //motor_buffer[6] = 0;
  //motor_buffer[7] = 1;
 
  // Load buffer
  Serial.println("Loading buffer");
  // TODO: Don't have initial commands in buffer
  fill_half_buffer();
  fill_half_buffer();

  // Enable wraparound mode
  // See https://www.esp32.com/viewtopic.php?t=3900
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
  Serial.println("Starting..2");

  // TODO: Compute actual idle level (0 or 1)

  // It is important that idle_out_lv and tx_start are set at the same time.
  // Setting tx_start first and idle_out_lv second causes the value not to be picked up
  // for very short pulse sequences.
  // Setting idle_out_lv first causes the first step to take much longer.
  // Setting them together like this causes the first step to take about 100ns longer,
  // which is acceptable.
  Serial.println("Starting..3");
  uint32_t conf1 = RMT.conf_ch[0].conf1.val;
  Serial.println("Starting..4");
  conf1 |= 1 << 0; // conf1.tx_start = 1
  Serial.println("Starting..5");
  conf1 |= 1 << 18; // conf1.idle_out_lv = 1;
  Serial.println("Starting..6");
  //RMT.conf_ch[0].conf1.val = conf1;
  Serial.println("Starting..7");

  
  RMT.conf_ch[0].conf1.tx_start = 1;
  
  Serial.println("/Setup");
}

void IRAM_ATTR on_rmt(void* arg) {
  Serial.println("Event:");
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

    Serial.println("/End event");
  
    // TODO: Pop `next_action` value. (and also idle_lv?)
  }
  if (RMT.int_st.ch0_tx_thr_event) {
    // Clear event
    RMT.int_clr.ch0_tx_thr_event = 1;
    Serial.println("Treshold event");
    
    fill_half_buffer();  
    Serial.println("/Treshold event");
  }
}

void IRAM_ATTR fill_half_buffer() {
  // Fill buffer
  for (int i = 0; i < 32; i++) {
    RMTMEM.chan[0].data32[i + rmt_buffer_offset].val = next_half_item() | (next_half_item() << 16);
    if (true) {
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

// Read next half of an rmt_item32_t from the motor buffer.
// TODO: Detect buffer over run
uint32_t IRAM_ATTR next_half_item() {
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
