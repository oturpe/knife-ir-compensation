#include"config.h"

#include<avr/io.h>       // Header file for basic avr input/output
#include<util/delay.h>   // header file for delay generation

#define BV(x) (1<<x)     // See text below

int main() {
  // Set PD6 as output
  DDRD |= BV(DDD6);
  PORTD |= BV(PORTD6);

  // Set up OC0A pwm
  // Non-inverting mode
  TCCR0A |= BV(COM0A1);
  // Phase correct pwm mode
  TCCR0A |= BV(WGM00);
  // Prescale: /8
  TCCR0B |= BV(CS01);

  OCR0A = 128;
  while(true) {
  }
}