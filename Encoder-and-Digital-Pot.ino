// -----
// Programa de exemplo baseado em InterruptRotator.ino da biblioteca RotaryEncoder.
// Também utiliza a biblioteca X9C10X para o potênciômetro digital X9C10X .
// Esta classe é implementada para uso no ambiente do Arduino.
// -----
// 25/01/2023 - Início
// -----

// This example checks the state of the rotary encoder using interrupts.
// The current position and direction is printed on output when changed.

// Hardware setup:
// Attach a rotary encoder with output pins to
// * 2 and 3 on Arduino UNO. (supported by attachInterrupt)
// * A2 and A3 can be used when directly using the ISR interrupts, see comments below.
// Swap the pins when direction is detected wrong.
// The common contact should be attached to ground.
//
// Hints for using attachinterrupt see https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/

#include <Arduino.h>
#include <RotaryEncoder.h>
#include "X9C10X.h"

X9C10X pot(10000);  // Initializes the resistance to 10 KΩ
#define PIN_IN1 3
#define PIN_IN2 2

// A pointer to the dynamic created rotary encoder instance.
// This will be done in setup()
RotaryEncoder *encoder = nullptr;

// This interrupt routine will be called on any change of one of the input signals
void checkPosition()
{
  encoder->tick(); // just call tick() to check the state.
}

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("InterruptRotator example for the RotaryEncoder library.");

  // Digital Pot configuration
  pot.begin(5, 6, 7);  // (pulse, direction, select)
  pot.setPosition(0, true); // The wiper will be moved to the closest "end" position and from there moved to the 0 position.

  // Setup the rotary encoder functionality
  // Use TWO03 mode when PIN_IN1, PIN_IN2 signals are both LOW or HIGH in latch position.
  encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::TWO03);

  // Register interrupt routine
  attachInterrupt(digitalPinToInterrupt(PIN_IN1), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_IN2), checkPosition, CHANGE);
} // setup()


// Read the current position of the encoder and print out when changed.
void loop()
{
  static int pos = 0;

  int newPos = encoder->getPosition();
  if (pos != newPos) {
    if ((int)(encoder->getDirection()) > 0) {
      pot.incr();
    }
    else {
      pot.decr();    
    }
    Serial.print("pos:");
    Serial.print(newPos);
    Serial.print(" dir:");
    Serial.println((int)(encoder->getDirection()));
    pos = newPos;
  } // if
} // loop ()
