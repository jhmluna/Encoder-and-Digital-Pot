// -----
// Programa de exemplo baseado em InterruptRotator.ino da biblioteca RotaryEncoder.
// Também utiliza a biblioteca X9C10X para o voltage_coarseênciômetro digital X9C10X .
// Esta classe é implementada para uso no ambiente do Arduino.
// -----
// 25/01/2023 - Início
// -----

// This example checks the state of the rotary encoder using interrupts.
// The current position and direction is printed on output when changed.

// Hardware setup:
// Attach a rotary encoder with output pins to D6 and D7 on Arduino Nano. Atach encoder switch to D5.
// Swap the pins when direction is detected wrong.
// The common contact should be attached to ground.
//
// Hints for using attachinterrupt see https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/

#include <Arduino.h>
#include <RotaryEncoder.h>  // https://github.com/mathertel/RotaryEncoder
#include "X9C10X.h"         // https://github.com/RobTillaart/X9C10X
#include <JC_Button.h>      // https://github.com/JChristensen/JC_Button

// Encoder button pin assignments and setup
const byte ENCODER_BUTTON_PIN(5);        // Connect a button switch from this pin to ground
Button encoder_Btn(ENCODER_BUTTON_PIN);  // Define encoder button
const byte SWEEP_BUTTON_PIN(9);        // Connect a button switch from this pin to ground
Button sweep_Btn(SWEEP_BUTTON_PIN);  // Define sweep button

// A pointer to the dynamic created rotary encoder instance. This will be done in setup()
RotaryEncoder *encoder = nullptr;
// Encoder input pins
#define PIN_IN1 6
#define PIN_IN2 7

// Digital pot setup
X9C104 voltage_coarse(100000);  // Initializes the resistance to 100 KΩ
X9C103 voltage_fine(10000);     // Initializes the resistance to 10 KΩ

#define LED_PIN LED_BUILTIN
volatile bool val_debug_pin = 0;

// The Interrupt Service Routine for Pin Change Interrupt 2. This routine will only be called on any signal change on D6 and D7.
ISR(PCINT2_vect) {
  encoder->tick();  // Just call tick() to check the state.
}

// Timer1 Interrupt setup
// https://www.best-microcontroller-projects.com/arduino-timer-interrupt.html
void setup_T1(void) {

  cli(); // Stop interrupts

  // Set Timer1 interrupt at 1 Hz
  TCCR1A = 0; // Set entire TCCR1A register to 0
  TCCR1B = 0; // Same for TCCR1B
  TCNT1  = 0; // Initialize counter value to 0
  OCR1A = 0;  // Set compare match register OCR1A to 0
  OCR1B = 0;  // Same for OCR1B
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12);  
  // Enable timer compare interrupt
  TIMSK1 |= (1 << TOIE1);

  sei(); // Allow interrupts
}

// Interrupt on Timer 1 overflow.
ISR(TIMER1_OVF_vect) {
  // Toggle Led pin. 
  val_debug_pin = !val_debug_pin;
  if (val_debug_pin) digitalWrite(LED_PIN, HIGH); else digitalWrite(LED_PIN, LOW);
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("InterruptRotator example for the RotaryEncoder library.");

  // Digital pot configuration
  voltage_coarse.begin(2, 3, 4);        // (pulse, direction, select)
  voltage_coarse.setPosition(0);  // The wiper will be moved to the closest "end" position and from there moved to the 0 position.
  voltage_fine.begin(2, 3, 8);
  voltage_fine.setPosition(0);

  // Setup the rotary encoder functionality
  // Use TWO03 mode when PIN_IN1, PIN_IN2 signals are both LOW or HIGH in latch position.
  encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::TWO03);

  // To use other pins with Arduino UNO you can also use the ISR directly.
  // Here is some code for D6 and D7 using ATMega328P specific registers.

  // Setup flags to activate the ISR PCINT2 for Rotary Encoder.
  PCICR |= (1 << PCIE2);                      // This enables Pin Change Interrupt 2 that covers the Digital input pins or Port D.
  PCMSK2 |= (1 << PCINT22) | (1 << PCINT23);  // This enables the interrupt for pin 6 and 7 of Port D.

  // Initialize button objects
  encoder_Btn.begin();
  sweep_Btn.begin();

  // Interrupt test
  pinMode(LED_PIN, OUTPUT);
  setup_T1();
}  // setup()

// Read the current position of the encoder and print out when changed.
void loop() {
  static int pos = 0;

  static bool encoder_fine_step = false;  // a variable that keeps the current step status for encoder
  encoder_Btn.read();                     // Read encoder button
  sweep_Btn.read();                       // Read sweep button

  int newPos = encoder->getPosition();
  if (pos != newPos) {
    int direction = (int)(encoder->getDirection());
    if (direction > 0) {
      if (encoder_fine_step)
        voltage_fine.incr();
      else
        voltage_coarse.incr();
    } else {
      if (encoder_fine_step)
        voltage_fine.decr();
      else
        voltage_coarse.decr();
    }
    pos = newPos;
  }  // if

  if (encoder_Btn.wasReleased())  // if the encoder button was released, change fine step status
  {
    encoder_fine_step = !encoder_fine_step;
  }
  if (sweep_Btn.wasReleased())  // if the sweep button was released, print message
  {
     Serial.println("Sweep button pressed.");
  }
}  // loop()