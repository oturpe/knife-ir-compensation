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
///
/// \param pwmValue
///    Initial value for pwm
void initializePwm(PrescalerValue prescalerValue, uint8_t pwmValue) {
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
  OCR0A = pwmValue;
}

/// Sets pwm duty cycle to requested value.
///
/// \param value
///    Requested duty cycle value
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
  setVoltageReference(VREF_INTERNAL_1_1V);

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

/// Reads values from current sense and return value with 8 bits
/// precision.
///
/// This function waits until a value is available.
///
/// \return
///   Value from adc
uint16_t readCurrentSense() {
  // start conversion and wait until value is available
  ADCSRA |= BV(ADSC);
  while(ADCSRA & (1<<ADSC));

  return ADC;
}

uint16_t Store[STORE_SIZE];
uint16_t Cursor = 0;

uint32_t averageCurrentValue(uint16_t value) {
  static bool initialized = false;

  // Fill with first input to avoid statup lag
  if(!initialized) {
    for (int i = 0; i < STORE_SIZE; i++) {
      Store[i] = value;
    }
    initialized = true;
    return value;
  }

  Store[Cursor] = value;
  Cursor = (Cursor + 1) % STORE_SIZE;

  uint32_t sum = 0;
  for (int i = 0; i < STORE_SIZE; i++) {
    sum += Store[i];
  }

  return sum / STORE_SIZE;

}

void control(uint32_t averaged) {
    static int32_t integral = 0;
    static uint16_t counter = 0;

    counter = (counter +1) % CONTROL_FREQ;
    if(counter)
      return;

    int32_t delta = averaged - TARGET_CURRENT;
    if(delta > DELTA_MAX) delta = DELTA_MAX;
    if(delta < DELTA_MIN) delta = DELTA_MIN;

    integral += delta;
    if(integral > INTEGRAL_MAX) integral = INTEGRAL_MAX;
    if(integral < INTEGRAL_MIN) integral = INTEGRAL_MIN;

    int32_t pwmValue = ((delta / DELTA_COEFF) + (integral / INTEGRAL_COEFF));
    if(pwmValue > 127)
      pwmValue = 127;
    if(pwmValue < -128)
      pwmValue = -128;

    setPwm(128-pwmValue);
}

#ifdef DEBUG

  /// Initializes resources needed for debugging.
  void initializeDebug() {
    // Pin D5 as output.
    DDRD |= BV(DDD5);
  }

  /// Prints debug information
  ///
  /// This crude implementation flashes a led connected to pin D5. This is only
  /// done every DEBUG_FREQ calls to avoid spending all the time flashing the
  /// led.
  ///
  /// \param averagedCurrent
  ///     Averaged current value at this moment
  void printDebugInfo(uint16_t averagedCurrent) {
    static uint16_t counter = 0;

    counter = (counter + 1) % DEBUG_FREQ;
    if(counter)
      return;

    for(int i = 0; i < averagedCurrent / 2; i++) {
      PORTD |= BV(PORTD5);
      _delay_ms(50);
      PORTD &= ~BV(PORTD5);
      _delay_ms(50);
    }
  }
#endif

int main() {
  initializePwm(PSV_8, PWM_INITIAL);
  initializeCurrentSense();

  //Warmup delay just in case
  _delay_ms(50);
  while(true) {
    uint32_t current = readCurrentSense();
    uint32_t averaged = averageCurrentValue(current);

    control(averaged);

    #ifdef DEBUG
      printDebugInfo(averaged);
    #endif
  }
}