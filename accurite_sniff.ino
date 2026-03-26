#define RADIOLIB_LOW_LEVEL 1
#include <RadioLib.h>

#include <Wire.h>
//#include <LiquidCrystal_I2C.h>
//#include <avr/eeprom.h>

#define EEMEM

#include "M5Unified.h"
#include "M5GFX.h"


// === Receiver Code Constants and Variables ===

#define PIN           2  // 433MHz data pin
#define MAXBITS      65

// Wind directions lookup
const float winddirections[] = { 315.0, 247.5, 292.5, 270.0, 
                                 337.5, 225.0, 0.0, 202.5,
                                 67.5, 135.0, 90.0, 112.5,
                                 45.0, 157.5, 22.5, 180.0 };

// Message types
#define MT_WS_WD_RF  49
#define MT_WS_T_RH   56

// Variables for decoding
volatile unsigned int pulsecnt = 0;
volatile unsigned long risets = 0;
volatile unsigned int syncpulses = 0;
volatile byte state = 0;
volatile byte buf[8] = {0};
volatile bool reading = false;

#define RESET 0
#define INSYNC 1
#define SYNCDONE 2

// EEPROM persistence (unused in this example, can keep if you want)
unsigned int raincounter = 0;
unsigned int EEMEM raincounter_persist;
#define MARKER 0x5AA5
unsigned int EEMEM eeprom_marker = MARKER;

// === Lcd ===

//LiquidCrystal_I2C Lcd(0x27, 16, 2);

// === Variables to hold latest decoded data ===
float latestWindspeed = -1;      // km/h
float latestWindDirection = -1;  // degrees
float latestTemperature = -1000; // sentinel invalid temp (C)

// Timing for LCD updates
unsigned long lastLcdUpdate = 0;
const unsigned long LcdUpdateInterval = 1000; // ms

// === Helper Functions ===

String degreesToCompass(float degrees) {
  // 8 point compass
  const char* directions[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  int index = (int)((degrees + 22.5) / 45) % 8;
  return String(directions[index]);
}

bool acurite_crc(volatile byte row[], int cols) {
  cols -= 1; // last byte is CRC
  int sum = 0;
  for (int i = 0; i < cols; i++) sum += row[i];
  return (sum != 0 && sum % 256 == row[cols]);
}

float getTempF(byte hibyte, byte lobyte) {
  int highbits = (hibyte & 0x0F) << 7;
  int lowbits = lobyte & 0x7F;
  int rawtemp = highbits | lowbits;
  float temp = (rawtemp - 400) / 10.0;
  return temp;
}

float getWindSpeed(byte hibyte, byte lobyte) {
  int highbits = (hibyte & 0x7F) << 3;
  int lowbits = (lobyte & 0x7F) >> 4;
  float speed = highbits | lowbits;
  if (speed > 0) speed = speed * 0.23 + 0.28;
  float kph = speed * 60 * 60 / 1000;
  return kph;
}

float getWindDirection(byte b) {
  int direction = b & 0x0F;
  return winddirections[direction];
}

float kphToKnots(float kph) {
  return kph * 0.539957;
}

// === ISR prototype ===
void My_ISR();

// === Setup ===

void setup() {
  Serial.begin(155200);
  M5.begin();
  M5.Lcd.init();
  ///M5.Lcd.backlight();
  M5.Lcd.clear();
  M5.Lcd.setCursor(3, 0);
  M5.Lcd.print("WeatherSys");
  M5.Lcd.setCursor(5, 1);
  M5.Lcd.print("Starting");
  delay(1500);
  M5.Lcd.clear();

  pinMode(PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN), My_ISR, CHANGE);
}

// === Main Loop ===

void loop() {
  if (reading) {
    noInterrupts();
    bool valid = acurite_crc(buf, sizeof(buf));
    interrupts();

    if (valid) {
      int msgtype = (buf[2] & 0x3F);

      // Decode wind speed
      float ws = getWindSpeed(buf[3], buf[4]);
      if (ws > 0) latestWindspeed = ws;

      if (msgtype == MT_WS_WD_RF) {
        float wd = getWindDirection(buf[4]);
        latestWindDirection = wd;
      }
      else if (msgtype == MT_WS_T_RH) {
        float tf = getTempF(buf[4], buf[5]);
        float tc = (tf - 32) / 1.8;  // convert F to C
        latestTemperature = tc;
      }

      reading = false;
    }
  }

  unsigned long now = millis();
  if (now - lastLcdUpdate > LcdUpdateInterval) {
    lastLcdUpdate = now;

    M5.Lcd.clear();

    // Line 1: wind speed km/h and knots (e.g. "15.9km/h  8.6knt")
    M5.Lcd.setCursor(0, 0);
    if (latestWindspeed >= 0) {
      M5.Lcd.print(latestWindspeed, 1);
      M5.Lcd.print("km/h ");

      float knots = kphToKnots(latestWindspeed);
      M5.Lcd.print(knots, 1);
      M5.Lcd.print("knt");
    } else {
      M5.Lcd.print("--.-km/h --.-knt");
    }

    // Line 2: wind direction degrees + cardinal + temperature if available
    M5.Lcd.setCursor(0, 1);
    if (latestWindDirection >= 0) {
      M5.Lcd.print((int)latestWindDirection);
      M5.Lcd.write(223); // degree symbol
      M5.Lcd.print(" ");
      M5.Lcd.print(degreesToCompass(latestWindDirection));
    } else {
      M5.Lcd.print("No Wind Dir");
    }

    if (latestTemperature > -100) {
      M5.Lcd.print(" T ");
      M5.Lcd.print((int)latestTemperature);
      M5.Lcd.write(223);
      M5.Lcd.print("C");
    }
  }
}

// === ISR ===

void My_ISR() {
  unsigned long timestamp = micros();

  if (digitalRead(PIN) == HIGH) {
    if (timestamp - risets > 10000) {
      state = RESET;
      syncpulses = 0;
      pulsecnt = 0;
    }
    risets = timestamp;
    return;
  }

  unsigned long duration = timestamp - risets;

  if (state == RESET || state == INSYNC) {
    if (duration > 575 && duration < 675) {
      state = INSYNC;
      syncpulses++;
      if (syncpulses > 3) {
        state = SYNCDONE;
        syncpulses = 0;
        pulsecnt = 0;
      }
      return;
    } else {
      syncpulses = 0;
      pulsecnt = 0;
      state = RESET;
      return;
    }
  } else {
    if (pulsecnt > MAXBITS) {
      state = RESET;
      pulsecnt = 0;
      reading = true;
      return;
    }

    byte bytepos = pulsecnt / 8;
    byte bitpos = 7 - (pulsecnt % 8);

    if (duration > 375 && duration < 450) {
      bitSet(buf[bytepos], bitpos);
      pulsecnt++;
    }
    else if (duration > 175 && duration < 250) {
      bitClear(buf[bytepos], bitpos);
      pulsecnt++;
    }
  }
}
