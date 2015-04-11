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

// TODOS:
// #  Find the right pwm frequency. At least too high frequency seems to make motor
//    to stall.

#include "config.h"

#include <avr/io.h>
#include <util/delay.h>

// Cleaner setting of bits
#define BV(x) (1<<x)

/// Possible prescaler values for timer 0.
enum PrescalerValue {
  PSV_1,
  PSV_8,
  PSV_64,
  PSV_256,
  PSV_1024,
};

/// Sets timer 0 prescaler to requested value.
///
/// This function assumes that Clock Select bits have not been touched yet.
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

// Possible voltage references
enum VoltageReference {
  // Aref pin
  VREF_AREF,
  // Vcc with external capacitor
  VREF_VCC,
  // Internal 1.1 V with external capacitor
  VREF_INTERNAL_1_1V
};

// Sets analog voltage refernece to requested type.
//
// This function assumes that Reference Selection bits have not been touched
// yet.
//
// \param reference
//    Requested voltage reference
void setVoltageReference(VoltageReference reference) {
  switch(reference) {
  case VREF_AREF:
    // The default, nothing to do
    break;
  case VREF_VCC:
    ADMUX |= BV(REFS0);
    break;
  case VREF_INTERNAL_1_1V:
    ADMUX |= BV(REFS1) | BV(REFS0);
    break;
  }
}

/// Initializes current sense adc by setting the reference voltage to 1.1 V
void initializeCurrentSense() {
  //setVoltageReference(VREF_INTERNAL_1_1V)
  //Testing with Vcc
  setVoltageReference(VREF_VCC);

  // Analog input: ADC0 (the default)
  //ADMUX |= BV(MUX0);

  // Prescale: /256
  ADCSRA |= BV(ADPS2) | BV(ADPS1) | BV(ADPS0);

  // Use analog input ADC0 with digital input disabled
  //ADMUX |= 0; // ADC0: all MUXn bits 0;
  DIDR0 |= BV(ADC0D);

  // Enable adc
  ADCSRA |= BV(ADEN);
}

/// Starts current sense ADC operation.
void startCurrentSense() {
  //PORTD |= BV(PORTD5); // debug
}

/// Reads values from current sense and return value with 8 bits
/// precision.
///
/// This function waits until a value is available.
///
/// \return
///   Value from adc
uint8_t readCurrentSense() {
  // start conversion and wait until value is available
  ADCSRA |= BV(ADSC);
  while(ADCSRA & (1<<ADSC));

  return ADC;
}

int main() {
  initializePwm(PSV_64);
  initializeCurrentSense();

  //Debugging >>
  DDRD |= BV(DDD5);
  // << Debugging

  //Warmup delay just in case
  _delay_ms(50);
  while(true) {
    uint8_t current = readCurrentSense();
    setPwm(current);
    _delay_ms(500);
  }
}