/**
  Author: Eugen Dahm
   
   **
  dependencies: TimerOne: http://www.arduino.cc/playground/code/timer1
**/

#include <math.h>
#include <TimerOne.h>

#define RPM_IN 2                                          // current rpm signal input pin
#define ROTATE_LEFT 9                                     // rotate left pin for l293d h-bridge
#define ROTATE_RIGHT 5                                    // rotate right pin for l293d h-bridge
#define ENABLE 10                                         // h-bridge enable pin (must be set to high in order to roate the motor)
#define RING_BUF_SIZE 300                                 // record for 30 s
#define SERIAL_SPEED 19200                                // serial speed


void my_isr();

static const int DEBOUNCE_DELAY = 2000;                      // debounce, motor creates ugly disturbances
volatile unsigned long lastMicros = 0;                      // old micros counter
volatile unsigned long currentMicros = 0;                   // current micros counter
volatile unsigned long rpm = 0;                             // last calculated rpms
volatile unsigned long time_difference = 0;                 // difference between two rotation impulses
static const unsigned long MINUTE_IN_MICROS = 60000000;     // needed to calculate rpms
volatile unsigned long time_buffer = 0;                      // buffer time difference

static const char GNUPLOT_HEADER[8] = "#\tX\tY\0";          // gnuplot header for plotting data file
int rpm_ring_buffer[RING_BUF_SIZE];                          // rpm buffer size set to record rpm values for 30 sec
int ringBufPos = 0;


static const int REFERENCE_RPM = 1500;                        // desired rpm
long total_error = 0;                                         // total error for integration part
double set_value = 0;                                         // pwm output value
double p_factor = 1.67;                                       // proportional factor
//double p_factor = 2.35;
double i_factor = 2.2;                                        // integral factor

double d_factor = 0.01;                                       // differential factor
const double DUTY_CYCLE_DURATION = 10000;                     // 10 ms duty cycle duration
const double SECOND_IN_MICROS = 1000000;                      // needed for pid equation
unsigned long ms_interrupt_counter = 0;                       // keep track of time for sampling
double old_error = 0;                                         // stores last error for differential quotient calculation

int store_rpm_boundary = 10;                                  // counter boundary to store only every 10th rpm value
int rpm_store_counter = 1;

boolean buffer_filled = false;                                // set to true when rpm buffer is full, rpm buffer is written to serial interface when true

void setup() {
        lastMicros = 0;
	Serial.begin(SERIAL_SPEED);                          // initialize serial wire
	pinMode(RPM_IN, INPUT);                              // rotation impulse pin as input
        pinMode(ENABLE,OUTPUT);                             // pwm output signal - controls 293 d motordriver
        pinMode(ROTATE_LEFT,OUTPUT);                         // rotate left signal pin as output
        pinMode(ROTATE_RIGHT,OUTPUT);                        // rot
        Timer1.initialize(DUTY_CYCLE_DURATION);              // initialize timer sampling rate
        Timer1.pwm(ROTATE_LEFT,0);                           // initialize pwm signal
        Timer1.attachInterrupt(my_isr);                      // attach pwm isr
        digitalWrite(ENABLE,HIGH);                      //   enable pin to high   
        attachInterrupt(0,rotationCounterIncrease,FALLING);  // attach interrupt for rotation input signal
        currentMicros = lastMicros = micros();               // initialize current and last rpm impulse variable
        Serial.println(GNUPLOT_HEADER);                      // write gnuplot data header to datafile

}

void loop() {
 
    rpm = time_difference > 0 ? MINUTE_IN_MICROS / time_difference : 0;                                                                                    // store current rpm     
  char output[50];
  char float_buf[10];
  if (buffer_filled) {
     for (int i = 0; i < RING_BUF_SIZE; i++) {                                                              // write sampled data via serial line after buffer is full
       dtostrf((float)rpm_ring_buffer[i] / (float)REFERENCE_RPM,3,2,float_buf);                            // convert float to string, sprintf is broken :(
       sprintf(output,"\t%lu\t%s",++ms_interrupt_counter * 100, float_buf);                                // convert rest to string - outputs current sampled rpm in gnuplot data file format
       Serial.println(output);                                                                             // send current line
     }
   buffer_filled = false;                                                                                  // reset buffer pos counter                                                            
    Timer1.attachInterrupt(my_isr);                                                                        // reattach interrupt
  }
  
}

void my_isr() {
  noInterrupts();                                                                                                                                            // disable interrupts
//  p_boundary_counter++;
  long difference_rpm = REFERENCE_RPM - rpm;                                                                                                                // calculate current rpm difference
  if ((set_value > 0) && (set_value < 1023))                                                                                                                // filter overshoot
    total_error += difference_rpm;                                                                                                                           // total error for integration part
  double duty_cycle_seconds = (DUTY_CYCLE_DURATION / SECOND_IN_MICROS);                                                                                     // calculate duty cycle in secs
  set_value = min(1023,                                                                                                                                     // cap from above at 1023
                max(0,                                                                                                                                      // cap from below at 0
                  round(                                                                                                                                    // round, possible values between 0 and 1023
                  p_factor * difference_rpm +                                                                                                               // proportional part
                  i_factor * duty_cycle_seconds * total_error +                                                                                             // integration part - area
                  d_factor * (difference_rpm - old_error) / duty_cycle_seconds)));                                                                          // differential part - differential quotien
   
  if(rpm_store_counter++ == store_rpm_boundary) {                                                                                                            // tenth value sampled, store in buffer
    rpm_ring_buffer[ringBufPos] = rpm;
    ringBufPos++;                                                                                                                                            // increase buffer position
    rpm_store_counter = 1;                                                                                                                                   // reset sample counter
  }
  //store rpm in ringbuf
  if (ringBufPos == RING_BUF_SIZE - 1) {                                                                                                                      // ring buffer full
   ringBufPos = -1;                                                                                                                                           // reset buffer offset
   Timer1.detachInterrupt();                                                                                                                                  // disable pwm interrupt
   //detachInterrupt(0);
   buffer_filled = true;                                                                                                                                      // set buffer filled boolean flag
  }

   Timer1.pwm(ROTATE_LEFT,set_value);                                                                                                                          // update pwm output value
   old_error = difference_rpm;                                                                                                                                 // save now old rpm difference  
interrupts();
}

void rotationCounterIncrease() {
  noInterrupts();                                                                                                                                              // disable interrupts
  currentMicros = micros();                                                                                                                                    // get actual microseconds
  unsigned long difference = currentMicros - lastMicros;                                                                                                       // calculate difference between two impulses
  if (difference > DEBOUNCE_DELAY) {                                                                                                                           // debounce
    time_difference = difference;                                                                                                                              // save difference
//    rpm = time_difference > 0 ? MINUTE_IN_MICROS /  time_difference : 0;                                                                                     // calculate rpms (consider division by zero)
    lastMicros = currentMicros;                                                                                                                                // actualize lastMicros
  }
  interrupts();                                                                                                                                                // enable interrupts
}

