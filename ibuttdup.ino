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
const int PIN_DATA   = 5;
const int PIN_RED    = 9;
const int PIN_GREEN  = 10;

// OneWire bus library object
OneWire ow(PIN_DATA);

// OneWire command bytes
const uint8_t CMD_READ_SERIAL    = 0x33;
const uint8_t CMD_WRITE_SERIAL   = 0xD5;
const uint8_t CMD_MAGICAL_UNLOCK = 0xD1;

// Size of the serial number in bytes
const size_t SERIAL_LEN = 8;

// Put you key code in a header file called dis
static const uint8_t target_serial[] = {
#include "serialnum.h"
};

// Globals
uint8_t found_serial[SERIAL_LEN];

// Macros: Because it's C and I can so fuck you

#define SLOG(msg) do { if(Serial){ Serial.println(F( (msg) )); } } while (0)

#define SET_LED_RED() do { \
  digitalWrite(PIN_RED, HIGH); \
  digitalWrite(PIN_GREEN, LOW); \
} while(0)

#define SET_LED_GREEN() do { \
  digitalWrite(PIN_RED, LOW); \
  digitalWrite(PIN_GREEN, HIGH); \
} while(0)

#define CLEAR_LED() do { \
  digitalWrite(PIN_RED, LOW); \
  digitalWrite(PIN_GREEN, LOW); \
} while(0)

#define RED 1
#define GREEN 2
#define BLINK_INTERVAL 500

#define BLINK(color, times, interval) do { \
  for (int i=0; i<(times); i++) { \
    if ((color)==RED) { \
      SET_LED_RED(); \
    } else { \
      SET_LED_GREEN(); \
    } \
    delay((interval)); \
    CLEAR_LED(); \
    delay((interval)); \
  } \
} while(0)

#define BLINK_RED(times) BLINK(RED, (times), BLINK_INTERVAL)
#define BLINK_GREEN(times) BLINK(GREEN, (times), BLINK_INTERVAL)

#define PRINT(...) do { if (Serial) { Serial.print(__VA_ARGS__); } } while(0)

// If no response, reset and restart from the begining (return from // main())
#define RESET_OR_RETURN() do { \
  if (!ow.reset()) { \
    delayMicroseconds(100); \
    return; \
  } \
} while(0)

// Compare two serials
bool compare(const uint8_t *a, const uint8_t *b) {
  for (int i=0; i<SERIAL_LEN; i++) {
    if (a[i] != b[i]) {
      return false;
    }
  }
  return true;
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
  CLEAR_LED();
  if (Serial) {
    Serial.begin(9600);
    Serial.println("OneWire duplicator");
  }
}

void loop() {

  // Signal ready
  SET_LED_GREEN();

  // Don't try doing stuff unless we've got a device present (check a few times
  // to be sure)
  for (int i=0; i<8; i++) {
    RESET_OR_RETURN();
  }

  // attempt to read target key's serial
  ow.reset();
  ow.write(CMD_READ_SERIAL);
  ow.read_bytes(found_serial, SERIAL_LEN);
  PRINT(F("Before\t"));
  print_serial(found_serial);

  // Signal key read successfully
  BLINK_GREEN(2);
  SET_LED_RED();
  delay(10);

  // Some kind of magical knock sequence to enable serial programming?
  ow.reset();
  ow.write(CMD_MAGICAL_UNLOCK);
  ow.write_bit_rw1990(1);

  // Enter write mode
  PRINT(F("Writing\t"));
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
  PRINT(F("After\t"));
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
  if (compare(target_serial, found_serial)) {
    SLOG("SUCCESS!");
    BLINK_GREEN(5);
  } else {
    SLOG("FAILURE!");
    BLINK_RED(5);
  }

  // Wait a minute and start over
  delay(5000);

}
