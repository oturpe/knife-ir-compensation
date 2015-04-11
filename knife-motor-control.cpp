// Knife motor control
//
// Firmware for atmega328p based dc motor controller with adjustable speed for
// a kinetic sculpture to be presented in the Slovenian Pavilion of Venice
// Biennale 2015.
//
// This is a simple ir controller that attempts to archieve slow rotational
// speed and high torque.
//
// Author: Otto Urpelainen
// Email: oturpe@iki.fi

#include "config.h"

#include <avr/io.h>
#include <util/delay.h>

// Cleaner setting of bits
#define BV(x) (1<<x)

/// Allowable prescaler values for timer 0.
enum PrescalerValue {
  PSV_1,
  PSV_8,
  PSV_64,
  PSV_256,
  PSV_1024,
};

/// Sets timer 0 prescaler to requested value
///
/// \param value
///   Requested prescaler value
void setPrescaler(PrescalerValue value) {
  switch (value) {
  case PSV_1:
    TCCR0B |= BV(CS00);
    break;
  case PSV_8:
    TCCR0B |= BV(CS01);
    break;
  case PSV_64:
    TCCR0B |= BV(CS01) | BV(CS00);
    break;
  case PSV_256:
    TCCR0B |= BV(CS02);
    break;
  case PSV_1024:
    TCCR0B |= BV(CS02) | BV(CS00);
    break;
  }
}

/// Initializes pin D6 as phase correct pwm with requested prescaling.
///
/// \param prescalerValue
///    Requested prescaler value
void initializePwm(PrescalerValue prescalerValue) {
  // Set PD6 as output
  DDRD |= BV(DDD6);
  PORTD |= BV(PORTD6);

  // Set up OC0A pwm
    TCCR0A |= BV(COM0A1);
  // Non-inverting mode
  TCCR0A |= BV(COM0A1);
  // Phase correct pwm mode
  TCCR0A |= BV(WGM00);

  setPrescaler(prescalerValue);
}

/// Sets pwm to requested duty cycle. 0 is off and 255 is 100 %.
///
/// \param value
///    Requested pwm duty cycle
void setPwm(uint8_t value) {
    OCR0A = value;
}

int main() {
  initializePwm(PSV_64);
  setPwm(64);
  while(true) {
  }
}