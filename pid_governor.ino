/**
  Author: Eugen Dahm
   
   **
  dependencies: TimerOne
**/

#include <math.h>
#include <TimerOne.h>

#define RPM_IN 2
#define ROTATE_LEFT 9
#define ROTATE_RIGHT 5
#define ENABLE 10


void my_isr();

static const int DEBOUNCE_DELAY = 2000;
volatile unsigned long lastMicros = 0;                      // old micros counter
volatile unsigned long currentMicros = 0;                   // current micros counter
volatile unsigned long rpm = 0;                             // last calculated rpms
volatile unsigned long time_difference = 0;                 // difference between two rotation impulses
static const unsigned long MINUTE_IN_MICROS = 60000000;     // needed to calculate rpms
volatile unsigned long time_buffer = 0;                      // buffer time difference
volatile unsigned long count = 0;

volatile unsigned int * rpm_ring_buffer = (unsigned int *) malloc(255 * sizeof(int));
int ringBufPos = 0;


static const int REFERENCE_RPM = 1500;
long total_error = 0;
double set_value = 0;
double p_factor = 1.6;
//double p_factor = 2.35;
double i_factor = 2;

double d_factor = 0.4; 
const double DUTY_CYCLE_DURATION = 10000; // 5 ms duty cycle duration
const double SECOND_IN_MICROS = 1000000;
double old_error = 0;
int every_thousand = 0;

int p_boundary = 100;
int p_boundary_counter = 0;
int i_boundary = 100;
int i_boundary_counter = 0;
int d_boundary = 100;
int d_boundary_counter = 0;

static const float P_INC_VALUE = 0.05;
static const float I_INC_VALUE = 0.01;
static const float D_INC_VALUE = 0.01;

int pwm_speed = 0;
int step = 5;
void setup() {
        lastMicros = 0;
	Serial.begin(9600);
	pinMode(RPM_IN, INPUT);                              // rotation impulse pin as input
        pinMode(ENABLE,OUTPUT);                             // pwm output signal - controls 293 d motordriver
        pinMode(ROTATE_LEFT,OUTPUT);                         // rotate left signal pin as output
        pinMode(ROTATE_RIGHT,OUTPUT);                        // rot
        Timer1.initialize(DUTY_CYCLE_DURATION);                           
        Timer1.pwm(ROTATE_LEFT,0);
        Timer1.attachInterrupt(my_isr);
        digitalWrite(ENABLE,HIGH);                      //   enable pin to high   
        attachInterrupt(0,rotationCounterIncrease,FALLING);  // attach interrupt for rotation input signal
        currentMicros = lastMicros = micros();               // initialize current and last rpm impulse variable
}

void loop() {
 
    rpm = time_difference > 0 ? MINUTE_IN_MICROS / time_difference : 0;                                                                                          // store current rpm

/*   if (ringBufPos == 255) { 
     for(int i = 0; i < 255; i++) {
       Serial.println(rpm_ring_buffer[i]);
    }    
   }*/
   
  if (d_boundary_counter == d_boundary) {
  //  p_factor += P_INC_VALUE;
    d_boundary_counter = 0;
    Serial.println(set_value);
    Serial.println(rpm);
    
  }
}

void my_isr() {
  noInterrupts();
//  p_boundary_counter++;
  long difference_rpm = REFERENCE_RPM - rpm;                                                                                                                // calculate current rpm difference
  if ((set_value > 0) && (set_value < 1023))
    total_error += difference_rpm;                                                                                                                           // total error for integration part
  double duty_cycle_seconds = (DUTY_CYCLE_DURATION / SECOND_IN_MICROS);                                                                                     // calculate duty cycle in secs
  set_value = min(1023,max(0,round(p_factor * difference_rpm + i_factor * duty_cycle_seconds * total_error + (d_factor * (difference_rpm - old_error) / duty_cycle_seconds))));      // calculate overall pid set value
 
   rpm_ring_buffer[ringBufPos] = rpm;
  //store rpm in ringbuf
  if (ringBufPos == 255) {
   ringBufPos = -1;
  }
  ringBufPos++;  
  
   Timer1.pwm(ROTATE_LEFT,set_value);
   old_error = difference_rpm;        // save now old rpm difference  

  d_boundary_counter++;

  interrupts();
}

void rotationCounterIncrease() {
  noInterrupts();                                                                  // disable interrupts
  currentMicros = micros();                                                        // get actual microseconds
  unsigned long difference = currentMicros - lastMicros;                          // calculate difference between two impulses
  if (difference > DEBOUNCE_DELAY) {                                                         // debounce
    time_difference = difference;                                                  // save difference
//    rpm = time_difference > 0 ? MINUTE_IN_MICROS /  time_difference : 0;           // calculate rpms (consider division by zero)
    lastMicros = currentMicros;                                                    // actualize lastMicros
  }
  interrupts();                                                                    // enable interrupts
}






