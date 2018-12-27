/*
 * ibuttdup.ino - Duplicate DS1990 "unique" ID to a RW1990 clone.
 *
 * Author: Sean Caulfield <sean@yak.net>
 * License: GPL v2.0
 *
 */

#include <Arduino.h>
#include <OneWire.h>

// Pin configuration
const int PIN_DATA  = 2;

// Size of the serial number in bytes
const size_t SERIAL_LEN = 8;

// OneWire bus library object
OneWire ow(PIN_DATA);

// OneWire command bytes
const uint8_t CMD_READ_SERIAL = 0x33;
const uint8_t CMD_WRITE_SERIAL = 0xD5;
const uint8_t CMD_MAGICAL_UNLOCK = 0xD1;

// Put you key code in a header file called dis
static const uint8_t target_serial[] = {
#include "serialnum.h"
};

// Globals
uint8_t found_serial[SERIAL_LEN];

// Because it's C and I can so fuck you
#define SLOG(msg) do { if(Serial){ Serial.println(F( (msg) )); } } while (0)

// Barf out serial number over...well, serial. >_>
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
  if (Serial) {
    Serial.begin(9600);
    Serial.println("OneWire duplicator");
  }
}

void loop() {

  // Don't try doing stuff unless we've got a device present (check a few times
  // to be sure)
  for (int i=0; i<8; i++) {
    if (!ow.reset()) {
      delayMicroseconds(1000);
      return;
    }
  }

  // attempt to read serial
  ow.reset();
  ow.write(CMD_READ_SERIAL);
  ow.read_bytes(found_serial, SERIAL_LEN);
  print_serial(found_serial);

  // Some kind of magical knock sequence to enable serial programming?
  ow.reset();
  ow.write(CMD_MAGICAL_UNLOCK);
  ow.write_bit_rw1990(1);

  // Enter write mode
  ow.reset();
  ow.write(CMD_WRITE_SERIAL, 1);
  ow.write_bytes_rw1990(target_serial, SERIAL_LEN);

  // Magical closing/commiting sequence?
  ow.reset();
  ow.write(CMD_MAGICAL_UNLOCK);
  ow.write_bit_rw1990(0);

  // Read back just-written serial
  ow.reset();
  ow.write(CMD_READ_SERIAL);
  ow.read_bytes(found_serial, SERIAL_LEN);
  print_serial(found_serial);

  // Verify serials match
  bool verified = true;
  for (int i=0; i<SERIAL_LEN; i++) {
    if (target_serial[i] != found_serial[i]) {
      verified = false;
      break;
    }
  }

  // Declare victory and try again sometime
  if (verified) {
    SLOG("SUCCESS!");
  } else {
    SLOG("FAILURE!");
  }

  delay(60000);

}
