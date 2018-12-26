/*
 * ibuttdup.ino - Duplicate DS1990 "unique" ID to a RW1990 clone.
 *
 * Author: Sean Caulfield <sean@yak.net>
 * License: GPL v2.0
 *
 */

#include <Arduino.h>

// Pin configuration
const int PIN_DATA  = 5;
const int PIN_RED   = 9;
const int PIN_GREEN = 10;

// Size of the serial number in bytes
const size_t SERIAL_LEN = 8;

// OneWire command bytes
const uint8_t CMD_SKIP_ROM     = 0xCC;
const uint8_t CMD_READ_SERIAL  = 0x33;
const uint8_t CMD_WRITE_SERIAL = 0xD5;

// Delays (in usec) for various OneWire signals
const uint16_t DELAY_SEND_ZERO_MID = 44;
const uint16_t DELAY_SEND_ZERO_END = 8;
const uint16_t DELAY_SEND_ONE_MID = 8;
const uint16_t DELAY_SEND_ONE_END = 44;
const uint16_t DELAY_RECV_START_LO = 8;
const uint16_t DELAY_RECV_START_HI = 8;
const uint16_t DELAY_RECV_END = 32;
const uint16_t DELAY_WRITE_SP_ZERO_MID = 0;
const uint16_t DELAY_WRITE_SP_ZERO_END = 10;
const uint16_t DELAY_WRITE_SP_ONE_MID = 60;
const uint16_t DELAY_WRITE_SP_ONE_END = 10;
const uint16_t DELAY_RESET_LO = 480;
const uint16_t DELAY_RESET_HI = 70;
const uint16_t DELAY_RESET_END = 410;

// Dow-CRC using polynomial X^8 + X^5 + X^4 + X^0
// Tiny 2x16 entry CRC table created by Arjen Lentz
// See http://lentz.com.au/blog/calculating-crc-with-a-tiny-32-entry-lookup-table
static const uint8_t PROGMEM dscrc2x16_table[] = {
	0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
	0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
	0x00, 0x9D, 0x23, 0xBE, 0x46, 0xDB, 0x65, 0xF8,
	0x8C, 0x11, 0xAF, 0x32, 0xCA, 0x57, 0xE9, 0x74
};

static const uint8_t target_serial[] = {
#include "serialnum.h"
};

// Globals
//uint8_t target_serial[SERIAL_LEN];
uint8_t found_serial[SERIAL_LEN];

// Set LED to RED
void set_led_red() {
  digitalWrite(PIN_GREEN, LOW);
  digitalWrite(PIN_RED, HIGH);
}

// Set LED to GREEN
void set_led_green() {
  digitalWrite(PIN_RED, LOW);
  digitalWrite(PIN_GREEN, HIGH);
}

// Clear LED
void clear_led() {
  digitalWrite(PIN_RED, LOW);
  digitalWrite(PIN_GREEN, LOW);
}

// Blink red thrice
void blink_red_thrice() {
  set_led_red();
  delay(500);
  clear_led();
  delay(500);
  set_led_red();
  delay(500);
  clear_led();
  delay(500);
  set_led_red();
  delay(500);
  clear_led();
}

// Blink green thrice
void blink_green_thrice() {
  set_led_green();
  delay(500);
  clear_led();
  delay(500);
  set_led_green();
  delay(500);
  clear_led();
  delay(500);
  set_led_green();
  delay(500);
  clear_led();
}

// Set the OneWire bus low
inline void bang_lo() {
  noInterrupts();
  pinMode(PIN_DATA, OUTPUT);
  digitalWrite(PIN_DATA, LOW);
  interrupts();
}

// Set the OneWire bus "high" (tri-state and then pulled up)
inline void bang_hi() {
  noInterrupts();
  pinMode(PIN_DATA, INPUT);
  //digitalWrite(PIN_DATA, HIGH);
  interrupts();
}

// Send a byte a bit at a time down the OneWire bus
void onewire_send(uint8_t data) {
  for (int i=0; i<8; i++) {
    if (data & 1) {
      bang_lo();
      delayMicroseconds(DELAY_SEND_ONE_MID);
      bang_hi();
      delayMicroseconds(DELAY_SEND_ONE_END);
    } else {
      bang_lo();
      delayMicroseconds(DELAY_SEND_ZERO_MID);
      bang_hi();
      delayMicroseconds(DELAY_SEND_ZERO_END);
    }
    data = data >> 1;
  }
}

// Receive a byte from the OneWire bus
uint8_t onewire_recv() {
  uint8_t data = 0;
  uint8_t incoming = 0;

  for (int i=0; i<8; i++) {
    bang_lo();
    delayMicroseconds(DELAY_RECV_START_LO);
    bang_hi();
    delayMicroseconds(DELAY_RECV_START_HI);
    incoming = digitalRead(PIN_DATA);
    data = (incoming<<7) | (data >> 1);
    delayMicroseconds(DELAY_RECV_END);
  }
  return data;
}

// "Special" write byte function for writing the serial number
void onewire_write_sp(uint8_t data) {
  for (int i=0; i<8; i++) {
    if (data & 1) {
      bang_lo();
      delayMicroseconds(DELAY_WRITE_SP_ONE_MID);
      bang_hi();
      delay(DELAY_WRITE_SP_ONE_END);
    } else {
      bang_lo();
      //delayMicroseconds(DELAY_WRITE_SP_ZERO_MID);
      bang_hi();
      delay(DELAY_WRITE_SP_ZERO_END);
    }
    data = data >> 1;
  }
}

// Reset OneWire bus
uint8_t onewire_reset() {
  uint8_t retval = 0;
  uint8_t retries = 125;

  bang_hi();
  do {
    if (--retries == 0)
      return 0;
    delayMicroseconds(2);
  } while (digitalRead(PIN_DATA) == LOW);

  bang_lo();
  delayMicroseconds(DELAY_RESET_LO);

  //bang_hi(); decomposed here to hold the lock over the whole seq
  noInterrupts();
  pinMode(PIN_DATA, INPUT);
  delayMicroseconds(DELAY_RESET_HI);
  retval = (digitalRead(PIN_DATA) == LOW) ? 1 : 0;
  interrupts();

  delayMicroseconds(DELAY_RESET_END);

  return retval;
}

// Compute OneWire CRC-8.
uint8_t onewire_crc8(const uint8_t *addr, uint8_t len) {
	uint8_t crc = 0;

	while (len--) {
		crc = *addr++ ^ crc;  // just re-using crc as intermediate
		crc = pgm_read_byte(dscrc2x16_table + (crc & 0x0f)) ^
		pgm_read_byte(dscrc2x16_table + 16 + ((crc >> 4) & 0x0f));
	}

	return crc;
}

// Spin wait for device to respond
void onewire_pause() {
  while ((digitalRead(PIN_DATA) == HIGH))
    ;
  while ((digitalRead(PIN_DATA) == LOW))
    ;
}

void print_serial(uint8_t *serial) {
  if (Serial) {
    for (int i=0; i<SERIAL_LEN; i++) {
      if (serial[i] < 0x10)
        Serial.print(F("0"));
      Serial.print(serial[i], HEX);
      if (i == SERIAL_LEN - 1) {
        Serial.println();
      } else {
        Serial.print(F(" "));
      }
    }
  }
}

void setup() {
  pinMode(PIN_DATA, INPUT);
  digitalWrite(PIN_DATA, LOW);
  pinMode(PIN_RED, OUTPUT);
  pinMode(PIN_GREEN, OUTPUT);
  digitalWrite(PIN_RED, LOW);
  digitalWrite(PIN_GREEN, HIGH);
  if (Serial) {
    Serial.begin(115200);
    Serial.println("OneWire duplicator");
    //uint8_t crc = onewire_crc8(foo, SERIAL_LEN);
    //Serial.println(crc, HEX);
  }
}

void loop() {

  set_led_green();
  onewire_reset();
  onewire_pause();
  onewire_send(CMD_SKIP_ROM);
  onewire_reset();
  onewire_pause();

  // attempt to read serial
  set_led_red();
  onewire_reset();
  onewire_pause();
  onewire_send(CMD_READ_SERIAL);
  for (int i=0; i<SERIAL_LEN; i++) {
    set_led_red();
    found_serial[i] = onewire_recv();
    clear_led();
  }

#if 0
  // If serial number isn't valid, bail (and try again)
  if (onewire_crc8(target_serial, SERIAL_LEN) != 0) {
    return;
  }
#endif

  // Otherwise, dump target serial to...serial (UART)
  print_serial(found_serial);

#if 0
  // Flash to get user to put on new key
  set_led_green();
  delay(500);
  clear_led();
  delay(500);
  set_led_green();
  delay(500);
  clear_led();
  delay(500);
  set_led_green();
  delay(500);
  clear_led();
  delay(500);
  set_led_green();
  delay(5000);
#endif

  // Enter write mode
  set_led_red();
  onewire_reset();
  onewire_pause();
  onewire_send(CMD_WRITE_SERIAL);
  clear_led();
  for (int i=0; i<SERIAL_LEN; i++) {
    set_led_red();
    onewire_write_sp(target_serial[i]);
    clear_led();
  }
  onewire_reset();
  onewire_pause();

  // Read back just-written serial
  //onewire_reset();
  //onewire_pause();
  onewire_send(CMD_READ_SERIAL);
  for (int i=0; i<SERIAL_LEN; i++) {
    set_led_red();
    found_serial[i] = onewire_recv();
    clear_led();
  }
  print_serial(found_serial);

  // Verify serials match
  for (int i=0; i<SERIAL_LEN; i++) {
    if (target_serial[i] != found_serial[i]) {
      if (Serial) {
        Serial.println(F("Verification failed!"));
      }
      blink_red_thrice();
      delay(10000);
      return;
    }
  }

  // Made it this far, serials match, set to green
  set_led_green();
  if (Serial) {
    Serial.println(F("SUCCESS!"));
  }
  blink_green_thrice();
  delay(10000);

}
