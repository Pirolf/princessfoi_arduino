#ifndef ROBOT_ENCODERS_H
#define ROBOT_ENCODERS_H

#include <stdint.h>

class Encoders {

public:
  Encoders();

  void update();

  void reset();

  int16_t readLeft();
  int16_t readRight();

  int16_t readLeftForward();
  int16_t readLeftBackward();

  int16_t readRightForward();
  int16_t readRightBackward();

private:
  static const uint8_t ITR_PULSE_COUNT = 4;
  static const uint8_t PIN_COUNT = 4;

  int16_t leftForward;
  int16_t leftBackward;
  int16_t rightForward;
  int16_t rightBackward;

  bool prevLeftForwardPin;
  bool prevLeftBackwardPin;
  bool prevRightForwardPin;
  bool prevRightBackwardPin;

  bool pulses[ITR_PULSE_COUNT][PIN_COUNT];
  uint8_t pulse_index = 0;
};

#endif
