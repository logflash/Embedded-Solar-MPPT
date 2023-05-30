/*                                   ECE464 (Embedded Computing) Project                                  */
/*                                           Ian Henriques - 2023                                         */

#include <Wire.h>
#include <INA3221.h>

// Comment to disable serial communication
#define DEBUG

// Comment to disable manual control (i.e. enable MPPT)
// #define MANUAL

/* ---------------------------------- END OF SETTINGS, START OF MACROS ---------------------------------- */

// Set the PWM frequency and resolution
#define PWM_FREQ             200000
#define PWM_RES              10

// Shunt resistor value for INA configuration (in mOhms)
#define R_SHUNT              100

// Define how many microseconds each "cycle" of the algorithm should be (i.e. setting the delay)
#define CYCLE_TIME           500

// How many cycles the perturb-and-observe algorithm should wait before the low-pass filter adjusts
#define PERTURB_WAIT_CYCLES  10

// Set important pins
#define PWM_PIN              3
#define LED_PIN              13

// If manual control is active, store a duty cycle variable to be updated every iteration
#ifdef MANUAL
#define HIGH_REF   14
#define MANUAL_PIN 15
#define LOW_REF    16
#endif

// Duty cycle (from 0 to 1023)
#define INITIAL_DUTY 32
double pwm_duty = INITIAL_DUTY;

// Low-pass filtering on the power sensor values
#define LP_FILTER_SIZE 500
double lp_volt[LP_FILTER_SIZE];
double lp_curr[LP_FILTER_SIZE];
int lp_filter_idx;

/* ------------------------------------ END OF MACROS, START OF CODE ------------------------------------ */

// INA power sensor initialization
INA3221 ina_0(INA3221_ADDR40_GND);

// Storing the last sampled values
double curr_mamps;
double curr_volts;
double curr_mwatt;

void setup() {

  // Debug print setup, if necessary
  #ifdef DEBUG
  Serial.begin(115200);
  while (!Serial) ;
  #endif

  // PWM signal for the buck converter
  pinMode(PWM_PIN, OUTPUT);

  // Optional, LED on to show that the code is running
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); 

  // Manual setup
  #ifdef MANUAL
  pinMode(HIGH_REF, OUTPUT);
  pinMode(LOW_REF, OUTPUT);
  digitalWrite(HIGH_REF, HIGH);
  digitalWrite(LOW_REF,  LOW);
  #endif

  // I2C setup for INA Sensors
  ina_0.begin(&Wire);
  ina_0.reset();
  ina_0.setShuntRes(R_SHUNT, R_SHUNT, R_SHUNT);

  // PWM setup
  analogWriteFrequency(PWM_PIN, PWM_FREQ);
  analogWriteResolution(PWM_RES);
}

void readInputs() {

  // Run the low-pass filtering on the INA sensor values
  lp_curr[(lp_filter_idx + 1) % LP_FILTER_SIZE] = lp_curr[lp_filter_idx] + ina_0.getCurrent(INA3221_CH1) * 1000;
  lp_volt[(lp_filter_idx + 1) % LP_FILTER_SIZE] = lp_volt[lp_filter_idx] + ina_0.getVoltage(INA3221_CH1);
  lp_filter_idx = (lp_filter_idx + 1) % LP_FILTER_SIZE;
  curr_mamps = (lp_curr[lp_filter_idx] - lp_curr[(lp_filter_idx + 1) % LP_FILTER_SIZE]) / LP_FILTER_SIZE;
  curr_volts = (lp_volt[lp_filter_idx] - lp_volt[(lp_filter_idx + 1) % LP_FILTER_SIZE]) / LP_FILTER_SIZE;
  curr_mwatt = curr_volts * curr_mamps;

  #ifdef DEBUG
  Serial.printf("I:%.4f,V:%.4f,P:%4f,D:%f\n", curr_mamps, curr_volts, curr_mwatt, pwm_duty);
  #endif
}

#ifdef MANUAL
void manual_control() {
  pwm_duty = analogRead(MANUAL_PIN);
}
#endif

void perturb_and_observe() {

  // Set the 'cycles_waited' for this function
  // If we've waited enough (i.e. cycles_waited = PERTURB_WAIT_CYCLES), run the algorithm
  // Else, return to give the low-pass filter time to adjust
  static int cycles_waited = 0;
  if (cycles_waited != PERTURB_WAIT_CYCLES) {
    ++cycles_waited;
    return;
  }

  static int function_called = 0;
  // Each time the function is called, sweep through a new duty cycle value 
  // (pausing initially to let the duty cycle adjust)
  if (function_called < 1000) {
    ++function_called;
    return;
  }

  // Store the last voltage and power (for comparison purposes)
  static double last_volts = 0.0;
  static double last_mwatt = 0.0;

  // Observe the effect of the last change, and decide
  if (curr_mwatt > last_mwatt) {
    if (curr_volts > last_volts) pwm_duty -= 1;
    else                         pwm_duty += 1;
  }
  else if (curr_mwatt < last_mwatt) {
    if (curr_volts > last_volts) pwm_duty += 1;
    else                         pwm_duty -= 1;
  }

  // Reset the cycles waited
  cycles_waited = 0;
  last_volts = curr_volts;
  last_mwatt = curr_mwatt;
}

enum gradient_state {STAY, MOVE};
#define GRADIENT_STEP 1

void gradient_descent() {

  // Set the 'cycles_waited' for this function
  // If we've waited enough (i.e. cycles_waited = PERTURB_WAIT_CYCLES), run the algorithm
  // Else, return to give the low-pass filter time to adjust
  static int cycles_waited = 0;
  if (cycles_waited != PERTURB_WAIT_CYCLES) {
    ++cycles_waited;
    return;
  }

  static int function_called = 0;
  // Each time the function is called, sweep through a new duty cycle value 
  // (pausing initially to let the duty cycle adjust)
  if (function_called < 1000) {
    ++function_called;
    return;
  }

  // The "state" of the gradient descent algorithm
  // STAY if the algorithm plans to stay at the same duty cycle for another iteration (to calculate derivative)
  // MOVE if the algorithm plans to find a new operating point (gradient descent)
  static gradient_state state = STAY;

  // Store the last duty cycle and power (for slope-calculation purposes)
  static double last_duty  = 0.0;
  static double last_mwatt = 0.0;

  // If the state is STAY
  if (state == STAY) {
    last_duty  = pwm_duty;   // Store the current duty cycle and power for derivative-calculation purposes
    last_mwatt = curr_mwatt;
    pwm_duty += 1;          // Slightly perturb the duty cycle for derivative-calculation purposes
    cycles_waited = 0;       // Reset the cycles waited
    state = MOVE;            // Change the state to MOVE for next iteration
    return;
  }

  // Otherwise, the state if MOVE, adjust according to the slope
  double pwm_adjust = GRADIENT_STEP * (curr_mwatt - last_mwatt) / (pwm_duty - last_duty);
  pwm_duty += (pwm_adjust - 1);
  cycles_waited = 0;
  state = STAY;

  // Reset the cycles waited
  cycles_waited = 0;
}

void sweep() {
  
  // Wait a fixed number of cycles (see perturb and observe) to allow readings to adjust
  static int cycles_waited = 0;
  if (cycles_waited != PERTURB_WAIT_CYCLES) {
    ++cycles_waited;
    return;
  }

  static int function_called = 0;
  // Each time the function is called, sweep through a new duty cycle value 
  // (pausing initially to let the duty cycle adjust)
  if (function_called < 1000) pwm_duty = 32;
  else                      ++pwm_duty;

  // Update for next iteration
  ++function_called;
  cycles_waited = 0;
}

void updatePWM() {

  // Check bounds - we want to make sure the duty cycle isn't too high or too low
  if (pwm_duty > 1023)
    pwm_duty = 1023;
  else if (pwm_duty < 32)
    pwm_duty = 32;
  
  // Update the PWM duty cycle
  analogWrite(PWM_PIN, (int) pwm_duty);
}

void loop() {

  // Read the INA sensor inputs
  readInputs();

  // Manual (potentiometer control)
  #ifdef MANUAL
  manual_control();
  // Perturb-and-observe / gradient descent
  #else
  //perturb_and_observe();
  sweep();
  #endif

  // Bound-check and update the PWM
  updatePWM();

  delayMicroseconds(CYCLE_TIME);
}
