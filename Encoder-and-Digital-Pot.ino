// -----
// Programa de exemplo baseado em InterruptRotator.ino da biblioteca RotaryEncoder.
// Também utiliza a biblioteca X9C10X para o voltage_coarseênciômetro digital X9C10X .
// Esta classe é implementada para uso no ambiente do Arduino.
// -----

// Este exemplo verifica o estado do rotary encoder usando interrupções de mudança de estado do pino.
// A posição e direção atuais são impressas na saída quando alteradas.

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

// Digital pot definition
X9C104 voltage_coarse(100000);  // Initializes the resistance to 100 KΩ
X9C103 voltage_fine(10000);     // Initializes the resistance to 10 KΩ

#define LED_PIN LED_BUILTIN
volatile bool val_debug_pin = false;
volatile int voltage_ramp_up_counter = 0;

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
  TCNT1  = 0; // Initialize tIMER1 counter value to 0
  OCR1A = 0;  // Set compare match register OCR1A to 0
  OCR1B = 0;  // Same for OCR1B
  // Set CS11 bit for 256 prescaler
  TCCR1B |= (1 << CS11);  
  // Enable Timer1 Overflow Interrupt
  TIMSK1 |= (1 << TOIE1);

  sei(); // Allow interrupts
}

// Interrupt on Timer 1 overflow.
ISR(TIMER1_OVF_vect) {
  // Toggle Led pin status.
  val_debug_pin = !val_debug_pin;
  if (voltage_ramp_up_counter > 99) {
    TCCR1B &= ~(1 << CS11); // Stop Timer1/Counter
    TIMSK1 |= (1 << TOIE1); // Disable Timer1 Overflow Interrupt
    TCNT1 = 0;
    voltage_ramp_up_counter = 0;
    val_debug_pin = false;
  } else {
    voltage_coarse.incr();
    voltage_ramp_up_counter += 1;
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("InterruptRotator example for the RotaryEncoder library.");

  // Digital pot configuration
  voltage_coarse.begin(2, 3, 4);   // (pulse, direction, select)
  voltage_coarse.setPosition(0, true);  // The wiper will be moved to the closest "end" position and from there moved to the 0 position.
  voltage_fine.begin(2, 3, 8);
  voltage_fine.setPosition(0, true);

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
/*     Serial.print("Pos = ");
    Serial.print(pos);
    Serial.print("  |  New Pos = ");
    Serial.println(newPos); */
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
  if (sweep_Btn.wasReleased())  // if the sweep button was released, set digital pots and start ramp up;
  {
    Serial.println("Sweep button pressed.");
    voltage_coarse.setPosition(0, true);
    voltage_fine.setPosition(49, true);
    encoder_fine_step = false;
    delay(1000);
    setup_T1();
  }

  if (val_debug_pin) {    
    digitalWrite(LED_PIN, HIGH);
  }
  else digitalWrite(LED_PIN, LOW);
}  // loop()