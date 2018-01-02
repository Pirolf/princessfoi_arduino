#include <Servo.h>
#include <AStar32U4.h>
#include <PololuRPiSlave.h>
#include "encoders.h"

/* This example program shows how to make the A-Star 32U4 Robot
 * Controller into a Raspberry Pi I2C slave.  The RPi and A-Star can
 * exchange data bidirectionally, allowing each device to do what it
 * does best: high-level programming can be handled in a language such
 * as Python on the RPi, while the A-Star takes charge of motor
 * control, analog inputs, and other low-level I/O.
 *
 * The example and libraries are available for download at:
 *
 * https://github.com/pololu/pololu-rpi-slave-arduino-library
 *
 * You will need the corresponding Raspberry Pi code, which is
 * available in that repository under the pi/ subfolder.  The Pi code
 * sets up a simple Python-based web application as a control panel
 * for your Raspberry Pi robot.
 */

// Custom data structure that we will use for interpreting the buffer.
// We recommend keeping this under 64 bytes total.  If you change the
// data format, make sure to update the corresponding code in
// a_star.py on the Raspberry Pi.

struct Data
{
  bool yellow, green, red;
  bool buttonA, buttonB, buttonC;

  int16_t leftMotor, rightMotor;
  uint16_t batteryMillivolts;
  uint16_t analog[6];

  bool playNotes;
  char notes[14];

  uint16_t leftEncoderForward, leftEncoderBackward, rightEncoderForward, rightEncoderBackward;
  bool resetEncoders;
};

// important!!! Delay 20 micro seconds instead of 5 to fix i2c data glitching!
PololuRPiSlave<struct Data,20> slave;
PololuBuzzer buzzer;
AStar32U4Motors motors;
AStar32U4ButtonA buttonA;
AStar32U4ButtonB buttonB;
AStar32U4ButtonC buttonC;
Encoders *encoders;

void setup()
{
  // Set up the slave at I2C address 20.
  slave.init(20);

  // encoder pin change interrupts
  PCICR  = (1 << PCIE0);
  PCMSK0 = (1 << PCINT1) | (1 << PCINT2) | (1 << PCINT4) | (1 << PCINT7);
  PCIFR  = (1 << PCIF0);

  pinMode(  8, INPUT_PULLUP );
  pinMode( 11, INPUT_PULLUP );
  pinMode( 15, INPUT_PULLUP );
  pinMode( 16, INPUT_PULLUP );

  encoders = new Encoders();
  // Play startup sound.
  buzzer.play("v10>>g16>>>c16");
}

void loop()
{
  // Call updateBuffer() before using the buffer, to get the latest
  // data including recent master writes.
  slave.updateBuffer();

  // Write various values into the data structure.
  slave.buffer.buttonA = buttonA.isPressed();
  slave.buffer.buttonB = buttonB.isPressed();
  slave.buffer.buttonC = buttonC.isPressed();

  slave.buffer.batteryMillivolts = readBatteryMillivoltsLV();

  for(uint8_t i=0; i<6; i++)
  {
    slave.buffer.analog[i] = analogRead(i);
  }

  // READING the buffer is allowed before or after finalizeWrites().
  ledYellow(slave.buffer.yellow);
  ledGreen(slave.buffer.green);
  ledRed(slave.buffer.red);

  // Playing music involves both reading and writing, since we only
  // want to do it once.
  static bool startedPlaying = false;
  
  if(slave.buffer.playNotes && !startedPlaying)
  {
    buzzer.play(slave.buffer.notes);
    startedPlaying = true;
  }
  else if (startedPlaying && !buzzer.isPlaying())
  {
    slave.buffer.playNotes = false;
    startedPlaying = false;
  }

  // encoders
  if (slave.buffer.resetEncoders)
  {
    encoders -> reset();
  }
  slave.buffer.leftEncoderForward = encoders -> readLeftForward();
  slave.buffer.leftEncoderBackward = encoders -> readLeftBackward();
  slave.buffer.rightEncoderForward = encoders -> readRightForward();
  slave.buffer.rightEncoderBackward = encoders -> readRightBackward();

  motors.setSpeeds(slave.buffer.leftMotor, slave.buffer.rightMotor);
  // When you are done WRITING, call finalizeWrites() to make modified
  // data available to I2C master.
  slave.finalizeWrites();
  Serial.print("Left Forward = ");
  Serial.println(slave.buffer.leftEncoderForward);

  //Serial.print("; Left Backward = ");
  //Serial.println(slave.buffer.leftEncoderBackward);

  Serial.print("Right Forward = ");
  Serial.println(slave.buffer.rightEncoderForward);

  //Serial.print("; Right Backward = ");
  //Serial.println(slave.buffer.rightEncoderBackward);
}

ISR(PCINT0_vect)
{
  encoders -> update();
}
