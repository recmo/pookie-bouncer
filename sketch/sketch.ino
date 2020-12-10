#include <heltec.h>
#include <driver/adc.h>
#include <driver/rmt.h>
#include <driver/uart.h>
#include <soc/sens_reg.h>
#include <soc/sens_struct.h>
#include <soc/rmt_reg.h>
#include <soc/rmt_struct.h>

#include "pots.h"

#define TAU 6.283185307179586
#define PIN_DIR  GPIO_NUM_12
#define PIN_STEP GPIO_NUM_13
#define PIN_UART GPIO_NUM_14
#define RMT_TX_CHANNEL (RMT_CHANNEL_0)

char buffer[100];

#define MIN_DURATION 312   // 600 RPM (physical limit)
#define MAX_DURATION 65534 // 2.86 RPM (max representable)
#define SWITCH_DIRECTION 65535

#define MOTOR_BUFFER_LEN 1024 // Must be power of two
uint16_t motor_buffer[MOTOR_BUFFER_LEN];
uint16_t motor_read = MOTOR_BUFFER_LEN - 1;
uint16_t motor_write = 0;

// max pulses per second = 10 * 200 * 16 = 32 kHz = 31.25 us
// interupts every 60 pulses = 533 Hz = 1.875 ms
void IRAM_ATTR on_rmt(void* arg) {
  // Clear event
  RMT.int_clr.ch0_tx_thr_event = 1;
  
  // Set pulse duration
  for (int i = 0; i < 60; i++) {
    // Read ring buffer
    uint16_t duration = motor_buffer[motor_read];
    motor_read += 1;
    motor_read &= MOTOR_BUFFER_LEN - 1;

    // Handle direction switches 
    if (duration == SWITCH_DIRECTION) {
      digitalWrite(PIN_DIR, digitalRead(PIN_DIR) ^ 1);
      i -= 1;
      continue;
    }
    
    // Compute pulse
    uint16_t duration0 = duration / 2;
    uint16_t duration1 = duration - duration0;
    rmt_item32_t pulse = {{{duration0, 1, duration1, 0}}};

    // Write pulse to channel
    RMTMEM.chan[0].data32[i].val = pulse.val;
  }
}


void swuart_calcCRC(uint8_t* datagram, uint8_t datagramLength) {
  int i,j;
  uint8_t* crc = datagram + (datagramLength - 1); // CRC located in last byte of message
  uint8_t currentByte;
  *crc = 0;
  for (i = 0; i < (datagramLength-1); i++) {    // Execute for all bytes of a message
    currentByte = datagram[i];                  // Retrieve a byte to be sent from Array
    for (j = 0; j < 8; j++) {
      if ((*crc >> 7) ^ (currentByte & 0x01) ) { // update CRC based result of XOR operation
        *crc = (*crc << 1) ^ 0x07;
      } else {
        *crc = (*crc << 1);
      }
      currentByte = currentByte >> 1;
    } // for CRC bit
  } // for message byte}
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
  Serial2.begin(115200, SERIAL_8N1, PIN_UART, PIN_UART);
  uint8_t datagram[] = { 0x50, 0x40, 0x00, 0x00 };
  swuart_calcCRC(datagram, 4);
  Serial2.write(datagram, 4);
  delay(100);
  Serial2.write(datagram, 4);
  delay(100);
  
  /*
  uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
    .rx_flow_ctrl_thresh = 122,
  };
  // Configure UART parameters
  ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, PIN_UART, PIN_UART, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  // Setup UART buffered IO with event queue
  const int uart_buffer_size = (1024 * 2);
  QueueHandle_t uart_queue;
  // Install UART driver using an event queue here
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, uart_buffer_size, \
                                          uart_buffer_size, 10, &uart_queue, 0));
  */

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
  for (int i = 0; i < 64; i++)
    RMTMEM.chan[0].data32[i].val = 0;

  // Write motor buffer
  for (int j = 0; j < MOTOR_BUFFER_LEN; j++)
    motor_buffer[j] = MAX_DURATION;

  // Load buffer
  on_rmt(NULL);

  // Enable treshold event
  RMT.int_clr.ch0_tx_thr_event = 1;
  RMT.int_ena.ch0_tx_thr_event = 1;
  RMT.tx_lim_ch[0].limit = 60;

  // Start pulsing!
  RMT.conf_ch[0].conf1.tx_start = 1;
}

inline int motor_buffer_left() {
  
}

uint32_t motor_time = 0;     // Time in 100ns
int32_t motor_position = 0;  // Position in 1/32000 revolutions
bool motor_direction = true; // True for positive position direction

inline unsigned int motor_buffer_space() {
  return (motor_read - motor_write) & MOTOR_BUFFER_LEN - 1;
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
  } else if (duration > MAX_DURATION) {
    Serial.println("Over");
    duration = MAX_DURATION;
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

  //Serial.println((motor_read - motor_write) & (MOTOR_BUFFER_LEN - 1));

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
    uint16_t duration = MAX_DURATION;
    float new_position = trajectory(motor_time + duration);
    bool step_direction = new_position > current_position;
    if (fabs(new_position - current_position) > 1) {
      duration = float(duration) / fabs(new_position - current_position);
      if (false && duration < MAX_DURATION / 4) {
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
