// Cpu frequency for util/delay.h
#define F_CPU 16000000

#define PWM_INITIAL 0

#define STORE_SIZE 200

// How often pwm adjustments are made
#define CONTROL_FREQ 50
// Target value for ir controller
#define TARGET_CURRENT 20
// How many points of difference is needed change pwm by one step
#define DELTA_COEFF 2
#define DELTA_MAX (127 * DELTA_COEFF)
#define DELTA_MIN (-127 * DELTA_COEFF)
#define INTEGRAL_COEFF 200
#define INTEGRAL_MAX (127 * INTEGRAL_COEFF)
#define INTEGRAL_MIN (-127 * INTEGRAL_COEFF)

// Debugging definitions
//#define DEBUG
#define DEBUG_FREQ 4000

