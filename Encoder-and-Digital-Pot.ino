// -----
// Programa de exemplo baseado em InterruptRotator.ino da biblioteca RotaryEncoder.
// Também utiliza a biblioteca X9C10X para o voltage_coarseênciômetro digital X9C10X .
// Esta classe é implementada para uso no ambiente do Arduino.
// -----

// Este exemplo verifica o estado do rotary encoder usando interrupções de mudança de estado do pino.
// A posição e direção atuais são impressas na saída quando alteradas.

// Configuração de hardware:
// Conecte o rotary encoder com pinos de saída para os pinos 6 e 7 no Arduino UNO.
// Troque os pinos quando a direção detectada estiver errada.
// O contato comum deve ser conectado ao terra.

#include <Arduino.h>
#include <RotaryEncoder.h>      // https://github.com/mathertel/RotaryEncoder
#include "X9C10X.h"             // https://github.com/RobTillaart/X9C10X
#include <JC_Button.h>          // https://github.com/JChristensen/JC_Button

// Encoder pin assignments and setup
const byte ENCODER_BUTTON_PIN(5);              // connect a button switch from this pin to ground
Button encoder_Btn(ENCODER_BUTTON_PIN);       // define the button
// A pointer to the dynamic created rotary encoder instance done in setup().
RotaryEncoder *encoder = nullptr;
// Encoder input pins
#define PIN_IN1 6
#define PIN_IN2 7

// The Interrupt Service Routine for Pin Change Interrupt 2
// This routine will only be called on any signal change on D6 and D7.
ISR(PCINT2_vect) {
  encoder->tick(); // just call tick() to check the state.
}

// Digital pot setup
X9C104 voltage_coarse(100000);  // Initializes the resistance to 100 KΩ
X9C103 voltage_fine(10000);     // Initializes the resistance to 10 KΩ

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("InterruptRotator example for the RotaryEncoder library.");

  // Digital pot configuration
  voltage_coarse.begin(2, 3, 4);        // (pulse, direction, select)
  voltage_coarse.setPosition(0, true);  // The wiper will be moved to the closest "end" position and from there moved to the 0 position.
  voltage_fine.begin(2, 3, 8);        // (pulse, direction, select)
  voltage_fine.setPosition(0, true);  // The wiper will be moved to the closest "end" position and from there moved to the 0 position.

  // Setup the rotary encoder functionality
  // Use TWO03 mode when PIN_IN1, PIN_IN2 signals are both LOW or HIGH in latch position.
  encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::TWO03);

  // To use other pins with Arduino UNO you can also use the ISR directly.
  // Here is some code for D6 and D7 using ATMega328P specific registers.

  // Setup flags to activate the ISR PCINT2.
  PCICR |= (1 << PCIE2);    // This enables Pin Change Interrupt 2 that covers the Digital input pins or Port D.
  PCMSK2 |= (1 << PCINT22) | (1 << PCINT23);  // This enables the interrupt for pin 6 and 7 of Port D.

  encoder_Btn.begin();              // initialize the button object
}  // setup()

// Read the current position of the encoder and print out when changed.
void loop() {
  static int pos = 0;

  static bool encoder_fine_step = false;         // a variable that keeps the current step status for encoder
  encoder_Btn.read();         // read the button

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
    Serial.print("pos:");
    Serial.print(newPos);
    Serial.print(" dir:");
    Serial.println(direction);
    pos = newPos;
    Serial.println(encoder_fine_step);
  }  // if

  if (encoder_Btn.wasReleased())    // if the button was released, change the LED state
  {
    encoder_fine_step = !encoder_fine_step;
  }
} // loop()