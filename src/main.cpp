#include <Arduino.h>

unsigned long now;                        // timing variables to update data at a regular interval                  
unsigned long rc_update;
const int channels = 2;                   // specify the number of receiver channels
float RC_in[channels];                    // an array to store the calibrated input from receiver 

const int pwmPIN[]={18,19,21}; // an array to identify the PWM input pins (the array can be any length) 
const int num_ch = sizeof(pwmPIN)/sizeof(int);  // calculate the number of input pins (or channels)
unsigned long int pwm_reg_old = 0;                        // intermediate address for pwm_reg_new
unsigned long int pwm_reg_new = 0;                        // intermediate address for REG_READ(GPIO_IN_REG) ^ pwm_reg_old

volatile int PW[num_ch];                        // an array to store pulsewidth measurements
volatile boolean prev_pinState[num_ch];         // an array used to determine whether a pin has gone low-high or high-low
volatile unsigned long pciTime;                 // the time of the current pin change interrupt
volatile unsigned long pwmTimer[num_ch];        // an array to store the start time of each PWM pulse

volatile boolean pwmFlag[num_ch];               // flag whenever new data is available on each pin
volatile boolean RC_data_rdy;                   // flag when all RC receiver channels have received a new pulse
unsigned long int pwmPeriod[num_ch];                 // period, mirco sec, between two pulses on each pin
int RC_inputs = 3;

/*
 * GENERIC PWM FUNCTIONS
 */

unsigned long pin_time;
float pin_pwm;
float pin_period;

boolean PWM_read(int CH){
  if(CH < 1 && CH > num_ch) return false;
  int i = CH-1;
  boolean avail = pwmFlag[i];
  if (avail == HIGH){
    pwmFlag[i] = LOW;
    noInterrupts();
    pin_time = pwmTimer[i];
    pin_pwm = PW[i];
    pin_period = pwmPeriod[i];
    interrupts();
  }
  return avail;
}

unsigned long PWM_time(){return pin_time;}
float PWM_period(){return pin_period;}
float PWM(){return pin_pwm;}
boolean reg_state;
float PWM_freq(){
  float freq;
  return freq = 1000000 / pin_period;  // frequency Hz
}

// InteruptServiceRoutine 


void IRAM_ATTR ISR() {
  
  pciTime = micros();                                             // Record the time of the PIN change in microseconds
  pwm_reg_new = REG_READ(GPIO_IN_REG) ^ pwm_reg_old;
  for (int i = 0; i < num_ch; i++){
    gpio_num_t pin = (gpio_num_t) (pwmPIN[i] & 0x1F);
    reg_state = (pwm_reg_new  >> pin) & 1U;;
    if ((prev_pinState[i] == 0) && reg_state)  {          // and the pin state has changed from LOW to HIGH (start of pulse)
      prev_pinState[i] = 1;                                     // record pin state
      pwmPeriod[i] = pciTime - pwmTimer[i];                     // calculate the time period, micro sec, between the current and previous pulse
      pwmTimer[i] = pciTime;                                    // record the start time of the current pulse
    }
    else if ((prev_pinState[i] == 1) && !reg_state){ // or the pin state has changed from HIGH to LOW (end of pulse)
      prev_pinState[i] = 0;                                     // record pin state
      PW[i] = pciTime - pwmTimer[i];                            // calculate the duration of the current pulse
      pwmFlag[i] = HIGH;                                        // flag that new data is available
      if(i+1 == RC_inputs) RC_data_rdy = HIGH;                  
    }
  }
}



void setup_pwmRead(){
  for(int i = 0; i < num_ch; i++){              // run through each input pin
    pinMode(pwmPIN[i],INPUT);              // run through each input pin
    attachInterrupt(pwmPIN[i], ISR, CHANGE);                        // enable pinchange interrupt for pin
  }
} 

boolean RC_avail(){
  boolean avail = RC_data_rdy;
  RC_data_rdy = LOW;                          // reset the flag
  return avail;
  }

void print_RCpwm(){                             // display the raw RC Channel PWM Inputs
  for (int i = 0; i < RC_inputs; i++){
    Serial.print(" ch");Serial.print(pwmPIN[i]);
    Serial.print("  ");
    if(PW[i] < 1000) Serial.print(" ");
    Serial.print(PW[i]);
  }
  Serial.println("");
}


void setup() {
    Serial.begin(115200);
    setup_pwmRead();
}

void loop() {
    now = millis();
      if(RC_avail() || now - rc_update > 25){   // if RC data is available or 25ms has passed since last update (adjust to suit frame rate of receiver)
      rc_update = now;                           
      print_RCpwm();                        // uncommment to print raw data from receiver to serial
      }
}