#include <heltec.h>
#include <driver/uart.h>
#include <TMCStepper.h>


char buffer[100];

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

void send(uint8_t* datagram, uint8_t len) {
  // Add CRC
  swuart_calcCRC(datagram, 4);

  // Log
  Serial.print("TX ");
  for (int i = 0; i < len; i++) {
    sprintf(buffer, "%02x ", datagram[i]);
    Serial.print(buffer);
  }
  Serial.println("");

  // Send
  Serial1.write(datagram, 4);
}

void receive() {
  if (Serial1.available()) {
    Serial.print("RX ");
    while (Serial1.available()) {
      uint8_t rbyte = Serial1.read();
      sprintf(buffer, "%02x ", rbyte);
      Serial.print(buffer);
    }
    Serial.println("");
  }
}

uint32_t read_register(uint8_t reg) {
  uint8_t datagram[] = { 0x05, 0x00, reg, 0x00 };
  send(datagram, 4);

  //
  uint8_t reply[4 + 8];
  Serial.print("Reply: ");
  //Serial1.readBytesUntil(reply, 8);
  for (int i = 0; i < 12; i++) {
    sprintf(buffer, "%02x ", reply[i]);
    Serial.print(buffer);
  }
  Serial.println("");
  return 0;
}

void setup() {
  Heltec.begin(true /*DisplayEnable Enable*/, false /*LoRa Enable*/, true /*Serial Enable*/);
  delay(300);
  Serial.println("Booting...");

  // Display
  Heltec.display->clear();
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  Heltec.display->drawString(0, 0, "UART test");
  Heltec.display->display();
  
  Serial1.begin(115200, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);
  driver.begin(); 
}

uint8_t datagram[] = { 0x05, 0x00, 0x06, 0x00 };

void loop() {
  Serial.println(driver.GCONF());
    
  delay(1000);
}
