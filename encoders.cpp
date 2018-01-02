#include "encoders.h"
#include <AStar32U4.h>

Encoders::Encoders()
{
  reset();
}

void Encoders::reset()
{
  this -> leftForward = 0;
  this -> leftBackward = 0;
  this -> rightForward = 0;
  this -> rightBackward = 0;

  this -> prevLeftForwardPin = false;
  this -> prevLeftBackwardPin = false;
  this -> prevRightForwardPin = false;
  this -> prevRightBackwardPin = false;

  this -> pulse_index = 0;
}

void Encoders::update()
{
  bool leftForwardPin = PINB & (1<<2);
  bool leftBackwardPin = PINB & (1<<1);

  bool rightForwardPin = PINB & (1<<7);
  bool rightBackwardPin = PINB & (1<<4);

  this -> pulses[this -> pulse_index][0] = leftForwardPin;
  this -> pulses[this -> pulse_index][1] = leftBackwardPin;

  this -> pulses[this -> pulse_index][2] = rightForwardPin;
  this -> pulses[this -> pulse_index][3] = rightBackwardPin;

  if (pulse_index == 2) {
    // left forward: 10 -> 00 -> (01 -> 11)
    // left backward: 01 -> 00 -> (10 -> 11)
    if (leftForwardPin > (this -> prevLeftForwardPin) && leftBackwardPin) {
      this -> leftForward ++;
    } else if (leftBackwardPin > (this -> prevLeftBackwardPin) && leftForwardPin) {
      this -> leftBackward ++;
    }

    // right forward: 01 -> 00 -> (10 -> 11)
    // right backward: 10 -> 00 -> (01 -> 11)
    if (rightBackwardPin > (this -> prevRightBackwardPin) && rightForwardPin) {
      this -> rightForward ++;
    } else if (rightForwardPin > (this -> prevRightForwardPin) && rightBackwardPin) {
      this -> rightBackward ++;
    }

    this -> pulse_index = 0;
  } else {
    this -> pulse_index ++;
  }

  this -> prevLeftForwardPin = leftForwardPin;
  this -> prevLeftBackwardPin = leftBackwardPin;
  this -> prevRightForwardPin = rightForwardPin;
  this -> prevRightBackwardPin = rightBackwardPin;
}

int16_t Encoders::readLeft()
{
  return (leftForward - leftBackward);
}

int16_t Encoders::readRight()
{
  return (rightForward - rightBackward);
}

int16_t Encoders::readLeftForward()
{
  return leftForward;
}

int16_t Encoders::readLeftBackward()
{
  return leftBackward;
}

int16_t Encoders::readRightForward()
{
  return rightForward;
}

int16_t Encoders::readRightBackward()
{
  return rightBackward;
}
