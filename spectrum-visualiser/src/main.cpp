// Session 1 smoke test: confirms the ESP32 and the MAX7219 matrix are wired correctly.
// It sweeps a single lit column across the display, then fills each column to a different height.
// Set the pin numbers and module count to match your wiring before uploading.

#include <Arduino.h>
#include <MD_MAX72xx.h>
#include <SPI.h>

#define HARDWARE_TYPE MD_MAX72XX::FC16_HW  // Try GENERIC_HW or PAROLA_HW if the image is mirrored or rotated
#define MAX_DEVICES   4                    // Number of 8x8 modules chained together
#define DATA_PIN      23                   // DIN
#define CLK_PIN       18                   // CLK
#define CS_PIN        5                    // CS / LOAD

MD_MAX72XX mx(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);

void setup() {
  Serial.begin(115200);
  mx.begin();
  mx.control(MD_MAX72XX::INTENSITY, 2);
  mx.clear();
  Serial.println("Matrix smoke test started");
}

void loop() {
  const uint16_t cols = mx.getColumnCount();

  for (uint16_t c = 0; c < cols; c++) {
    mx.clear();
    mx.setColumn(c, 0xFF);
    delay(40);
  }

  mx.clear();
  for (uint16_t c = 0; c < cols; c++) {
    uint8_t height = (c % 8) + 1;
    mx.setColumn(c, (uint8_t)(0xFF << (8 - height)));
  }
  Serial.printf("Columns: %u\n", cols);
  delay(2000);
}
