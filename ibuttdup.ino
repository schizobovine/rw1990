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
const int PIN_DATA  = 5;
const int PIN_RED   = 13;
const int PIN_GREEN = 10;

// Size of the serial number in bytes
const size_t SERIAL_LEN = 8;

// OneWire bus library object
OneWire ow(PIN_DATA);

// OneWire command bytes
const uint8_t CMD_READ_SERIAL = 0x33;
const uint8_t CMD_WRITE_SERIAL = 0xD5;

static const uint8_t target_serial[] = {
#include "serialnum.h"
};

// Globals
//uint8_t target_serial[SERIAL_LEN];
uint8_t found_serial[SERIAL_LEN];
uint8_t address[8];

// Because it's C and I can so fuck you
#define SLOG(msg) do { if(Serial){ Serial.println(F( (msg) )); } } while (0)

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

// Blink red
void blink_red(uint8_t times, uint16_t delay_ms=500) {
  for (;times>=0; times--) {
    set_led_red();
    delay(delay_ms);
    clear_led();
    delay(delay_ms);
  }
}

// Blink green
void blink_green(uint8_t times, uint16_t delay_ms=500) {
  for (;times>=0; times--) {
    set_led_green();
    delay(delay_ms);
    clear_led();
    delay(delay_ms);
  }
}

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
  pinMode(PIN_RED, OUTPUT);
  pinMode(PIN_GREEN, OUTPUT);
  set_led_green();
  if (Serial) {
    Serial.begin(115200);
    Serial.println("OneWire duplicator");
  }
}

void loop() {

  if (!ow.reset()) {
    return;
  }

  ////set_led_green();
  //if (!ow.search(address)) {
  //  ow.reset_search();
  //  //clear_led();
  //  SLOG("Failed to read serial");
  //  blink_red(3);
  //  return;
  //}
  //ow.reset_search();

  //// Otherwise, dump target serial to...serial (UART)
  //print_serial(address);
  //if (ow.crc8(address, SERIAL_LEN-1) != address[SERIAL_LEN-1]) {
  //  SLOG("CRC8 failed");
  //} else {
  //  SLOG("CRC8 OK");
  //}
  //clear_led();

  // attempt to read serial
  if (!ow.reset()) {
    SLOG("Failed to read serial");
    //SLOG("Failed to read serial #2");
    //blink_red(2);
    return;
  }
  //ow.skip();
  //ow.select(address);
  ow.write(CMD_READ_SERIAL);
  ow.read_bytes(found_serial, SERIAL_LEN);
  print_serial(found_serial);

  //delay(1000);
  //return;
  delay(16);

  // Enter write mode
  //set_led_red();
  if (!ow.reset()) {
    SLOG("Failed to write serial");
    //blink_red(3);
    return;
  }
  //ow.select(found_serial);
  //ow.skip();
  ow.write(CMD_WRITE_SERIAL);
  ow.write_bytes_rw1990(target_serial, SERIAL_LEN);
  //ow.depower();
  //ow.write(target_serial, SERIAL_LEN);
  //clear_led();
  
  //delay(10);

  // Read back just-written serial
  //set_led_green();
  if (!ow.reset()) {
  //if (!ow.search(found_serial)) {
    //ow.reset_search();
    //clear_led();
    SLOG("Failed to re-read serial");
    blink_red(6);
    return;
  }
  ow.write(CMD_READ_SERIAL);
  ow.read_bytes(found_serial, SERIAL_LEN);

  // This should be the re-read serial, maybe different, maybe not; WHO NKOWS?
  print_serial(found_serial);
  //clear_led();

  // Verify serials match
  for (int i=0; i<SERIAL_LEN; i++) {
    if (target_serial[i] != found_serial[i]) {
      SLOG("Verification failed!");
      blink_red(10);
      return;
    }
  }

  // Made it this far, serials match, set to green
  SLOG("SUCCESS!");
  blink_red(10);
  delay(10000);

}
