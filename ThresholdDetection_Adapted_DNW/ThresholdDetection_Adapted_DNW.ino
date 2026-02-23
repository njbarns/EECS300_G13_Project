/**
 ******************************************************************************
 * @file    VL53L7CX_ThresholdDetection.ino
 * @author  STMicroelectronics
 * @version V1.0.0
 * @date    16 January 2023
 * @brief   Arduino test application for the STMicrolectronics VL53L7CX
 *          proximity sensor satellite based on FlightSense.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2021 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/*
 * To use these examples you need to connect the VL53L7CX satellite sensor directly to the Nucleo board with wires as explained below:
 * pin 1 (GND) of the VL53L7CX satellite connected to GND of the Nucleo board
 * pin 2 (IOVDD) of the VL53L7CX satellite connected to 3V3 pin of the Nucleo board
 * pin 3 (AVDD) of the VL53L7CX satellite connected to 5V pin of the Nucleo board
 * pin 4 (PWREN) of the VL53L7CX satellite connected to pin A5 of the Nucleo board
 * pin 5 (LPn) of the VL53L7CX satellite connected to pin A3 of the Nucleo board
 * pin 6 (SCL) of the VL53L7CX satellite connected to pin D15 (SCL) of the Nucleo board
 * pin 7 (SDA) of the VL53L7CX satellite connected to pin D14 (SDA) of the Nucleo board
 * pin 8 (I2C_RST) of the VL53L7CX satellite connected to pin A1 of the Nucleo board
 * pin 9 (INT) of the VL53L7CX satellite connected to pin A2 of the Nucleo board
 */

/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>
#include <Wire.h>
#include <vl53l7cx_class.h>

#define SDA_PIN 21
#define SCL_PIN 22

#define SerialPort Serial
#define DEV_I2C Wire

#define LPN_PIN 25
#define I2C_RST_PIN 32
#define PWREN_PIN 33
#define INT_PIN 26 

// Threshold window (mm)
static const uint16_t LOW_MM  = 0;
static const uint16_t HIGH_MM = 200;

// Resolution
static const uint8_t RES = VL53L7CX_RESOLUTION_4X4; // 4x4=16 zones, 8x8=64 zones

// Debounce in frames (increase if you see flicker/noise)
static const uint8_t ENTRY_CONFIRM_FRAMES = 2;
static const uint8_t LEAVE_CONFIRM_FRAMES = 3;

// Option: require at least this many zones to be "in-window" to declare presence
static const uint8_t MIN_ZONES_HIT = 2;

// ------------------------------------------------------------

VL53L7CX sensor(&DEV_I2C, LPN_PIN, I2C_RST_PIN);

static volatile bool g_intFlag = false;
static uint32_t g_count = 0;

// Simple ISR: set a flag only.
void IRAM_ATTR onVl53Int() { g_intFlag = true; }

// Heuristic validity check.
// VL53 target_status encoding varies by driver; this keeps it robust:
// - accept anything non-zero, but you can tighten this if you want.
static inline bool statusLooksValid(uint8_t s) { return (s != 0); }

bool computeHitNow(const VL53L7CX_ResultsData &R) {
  uint8_t zonesHit = 0;
  const uint8_t zones = RES; // 16 or 64

  // Use first target per zone (index l=0) for “presence”
  for (uint8_t z = 0; z < zones; z++) {
    if (R.nb_target_detected[z] == 0) continue;

    const uint16_t d = (uint16_t)R.distance_mm[(VL53L7CX_NB_TARGET_PER_ZONE * z) + 0];
    const uint8_t  s = (uint8_t) R.target_status[(VL53L7CX_NB_TARGET_PER_ZONE * z) + 0];

    if (!statusLooksValid(s)) continue;
    if (d >= LOW_MM && d <= HIGH_MM) {
      zonesHit++;
      if (zonesHit >= MIN_ZONES_HIT) return true;
    }
  }
  return false;
}

void setupThresholds() {
  VL53L7CX_DetectionThresholds thresholds[VL53L7CX_NB_THRESHOLDS];
  memset(thresholds, 0, sizeof(thresholds));

  // Disable before programming
  sensor.vl53l7cx_set_detection_thresholds_enable(0U);

  uint8_t i = 0;
  for (; i < RES; i++) {
    thresholds[i].zone_num = i;
    thresholds[i].measurement = VL53L7CX_DISTANCE_MM;
    thresholds[i].type = VL53L7CX_IN_WINDOW;
    thresholds[i].mathematic_operation = VL53L7CX_OPERATION_NONE;
    thresholds[i].param_low_thresh = LOW_MM;
    thresholds[i].param_high_thresh = HIGH_MM;
  }
  thresholds[i].zone_num |= VL53L7CX_LAST_THRESHOLD;

  sensor.vl53l7cx_set_detection_thresholds(thresholds);
  sensor.vl53l7cx_set_detection_thresholds_enable(1U);
}

void setup() {
  SerialPort.begin(115200);
  delay(50);

  if (PWREN_PIN >= 0) {
    pinMode(PWREN_PIN, OUTPUT);
    digitalWrite(PWREN_PIN, HIGH);
    delay(10);
  }

  DEV_I2C.begin(SDA_PIN, SCL_PIN, 400000);

  pinMode(INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INT_PIN), onVl53Int, FALLING);

  sensor.begin();
  sensor.init_sensor();

  // Choose resolution
  sensor.vl53l7cx_set_resolution(RES);

  // Configure thresholds to generate INT when something enters window
  setupThresholds();

  // Start ranging
  sensor.vl53l7cx_start_ranging();

  SerialPort.println("VL53L7CX counter ready.");
}
void loop() {
  static bool armed = true;        // ready to count next entry
  static uint8_t entryStreak = 0;  // consecutive hit frames
  static uint8_t leaveStreak = 0;  // consecutive miss frames

  VL53L7CX_ResultsData R;
  uint8_t ready = 0;
  uint8_t status = 0;

  // Poll for a new frame
  status = sensor.vl53l7cx_check_data_ready(&ready);
  if (status || !ready) {
    delay(2);        // small yield; do not busy-wait
    return;
  }

  // Read the frame
  status = sensor.vl53l7cx_get_ranging_data(&R);
  if (status) return;

  // Compute whether an object is "present" in the distance window
  const bool hitNow = computeHitNow(R);   // uses LOW_MM/HIGH_MM/MIN_ZONES_HIT

  // State machine with frame-debounce
  if (armed) {
    if (hitNow) {
      if (entryStreak < 255) entryStreak++;
      if (entryStreak >= ENTRY_CONFIRM_FRAMES) {
        g_count++;
        SerialPort.print("Count = ");
        SerialPort.println(g_count);

        armed = false;          // now wait for the object to leave
        leaveStreak = 0;
      }
    } else {
      entryStreak = 0;
    }
  } else {
    if (!hitNow) {
      if (leaveStreak < 255) leaveStreak++;
      if (leaveStreak >= LEAVE_CONFIRM_FRAMES) {
        armed = true;           // re-arm for next object
        entryStreak = 0;
      }
    } else {
      leaveStreak = 0;
    }
  }
  // Optional: limit print/CPU rate; set to 0 if you want max rate
  // delay(5);
}