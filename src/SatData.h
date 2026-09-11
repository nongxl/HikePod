#pragma once
#include <Arduino.h>

struct SatData {
  String system;   // "GPS", "GLONASS", "Galileo", "BeiDou".
  int id;
  int elevation;   // 0-90°.
  int azimuth;     // 0-359°.
  int snr;         // 0-99.
  bool used;       // used in the fix.
  bool visible;    // visible in the last cycle.
};
