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
uint8_t serial_to_clone[SERIAL_LEN];
uint8_t serial_before[SERIAL_LEN];
uint8_t serial_after[SERIAL_LEN];

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

// If no response, reset and restart from the begining (return from // main())
#define RESET_OR_RETURN() do { \
  if (!ow.reset()) { \
    /*SLOG("Communication failure");*/ \
    delayMicroseconds(100); \
    return; \
  } \
} while(0)

// Barf out serial number over...well, serial. >_>
#define PRINT(...) do { if(Serial) { Serial.print(__VA_ARGS__); } } while(0)
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

bool compare(const uint8_t *a, const uint8_t *b) {
  for (int i=0; i<SERIAL_LEN; i++) {
    if (a[i] != b[i]) {
      return false;
    }
  }
  return true;
}

void loop() {

  // Don't try doing stuff unless we've got a device present (check a few times
  // to be sure)
  for (int i=0; i<8; i++) {
    RESET_OR_RETURN();
  }

  // Signal ready to read key
  SET_LED_GREEN();

  // attempt to read first (key to clone) serial
  RESET_OR_RETURN();
  ow.write(CMD_READ_SERIAL);
  ow.read_bytes(serial_to_clone, SERIAL_LEN);
  PRINT(F("Original\t"));
  print_serial(serial_to_clone);

  do {

    // Signal key read successfully
    BLINK_GREEN(3);
    delay(10);

    // Loop until user replaces with new key
    while (ow.reset())
      ;

    // attempt to read target key's (original) serial
    ow.reset();
    ow.write(CMD_READ_SERIAL);
    ow.read_bytes(serial_before, SERIAL_LEN);
    PRINT(F("Before\t"));
    print_serial(serial_before);

    // If serials already match, user probably hasn't put on new key, so blink
    // and try again

  } while (compare(serial_to_clone, serial_before));

  // Make sure connection is good again
  for (int i=0; i<8; i++) {
    RESET_OR_RETURN();
  }
  SET_LED_RED();

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
  ow.read_bytes(serial_after, SERIAL_LEN);
  PRINT(F("After\t"));
  print_serial(serial_after);

  // Signal write complete
  BLINK_RED(1);
  CLEAR_LED();

  // Verify serials match

  // Declare victory and try again sometime
  if (compare(serial_to_clone, serial_after)) {
    SLOG("SUCCESS!");
    BLINK_GREEN(5);
  } else {
    SLOG("FAILURE!");
    BLINK_RED(5);
  }

  delay(60000);

}
