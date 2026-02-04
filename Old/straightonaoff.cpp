#include <Arduino.h>

const int speaker = 4;

void setup() {
  pinMode(speaker, OUTPUT);
}

void loop() {
  digitalWrite(speaker, HIGH);
  delayMicroseconds(500);   // 500us HIGH
  digitalWrite(speaker, LOW);
  delayMicroseconds(500);   // 500us LOW → 1kHz full cycle
}