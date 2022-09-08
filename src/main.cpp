#include <Arduino.h>

unsigned long now;                        // timing variables to update data at a regular interval                  
unsigned long rc_update;
const int channels = 2;                   // specify the number of receiver channels
float RC_in[channels];                    // an array to store the calibrated input from receiver 

const int pwmPIN[]={18,19}; // an array to identify the PWM input pins (the array can be any length) 
const int pwmPIN18 = 18;
const int pwmPIN19 = 19;
const int num_ch = sizeof(pwmPIN)/sizeof(int);  // calculate the number of input pins (or channels)
unsigned long int pwmPIN_reg[32];                        // each of the input pins expressed as a position on it's associated port register

volatile int PW[num_ch], PW18, PW19;                        // an array to store pulsewidth measurements
volatile boolean prev_pinState[num_ch], prev_pinState18, prev_pinState19;         // an array used to determine whether a pin has gone low-high or high-low
volatile unsigned long pciTime, pciTime18, pciTime19;                 // the time of the current pin change interrupt
volatile unsigned long pwmTimer[num_ch], pwmTimer18, pwmTimer19;        // an array to store the start time of each PWM pulse

volatile boolean pwmFlag[num_ch], pwmFlag18, pwmFlag19;               // flag whenever new data is available on each pin
volatile boolean RC_data_rdy, RC_data_rdy18;                   // flag when all RC receiver channels have received a new pulse
unsigned long pwmPeriod[num_ch], pwmPeriod18, pwmPeriod19;                 // period, mirco sec, between two pulses on each pin
int RC_inputs = 2;

void pwmPin_reg_init() {
  for (int i = 0; i < 32; i++){
    pwmPIN_reg[i]= exp2(31-i);
    // String str_pwmPin_reg = String(pwmPIN_reg[i]);
    // Serial.print(i);
    // Serial.print("   ");
    // Serial.println(pwmPIN_reg[i],BIN);
  }
}
/*
 * GENERIC PWM FUNCTIONS
 */

unsigned long pin_time;
float pin_pwm;
float pin_period;

boolean PWM_read18(){
  boolean avail = pwmFlag18;
  if (avail == HIGH){
    pwmFlag18 = LOW;
    noInterrupts();
    pin_time = pwmTimer18;
    pin_pwm = PW18;
    pin_period = pwmPeriod18;
    interrupts();
  }
  return avail;
}
boolean PWM_read19(){
  boolean avail = pwmFlag19;
  if (avail == HIGH){
    pwmFlag19 = LOW;
    noInterrupts();
    pin_time = pwmTimer19;
    pin_pwm = PW19;
    pin_period = pwmPeriod19;
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
void IRAM_ATTR ISR18() {
  
  pciTime18 = micros();                                             // Record the time of the PIN change in microseconds

    if (prev_pinState18 == 0)   {          // and the pin state has changed from LOW to HIGH (start of pulse)
      prev_pinState18 = 1;                                     // record pin state
      pwmPeriod18 = pciTime18 - pwmTimer18;                     // calculate the time period, micro sec, between the current and previous pulse
      pwmTimer18 = pciTime18;                                    // record the start time of the current pulse
    }
    else if (prev_pinState18 == 1){ // or the pin state has changed from HIGH to LOW (end of pulse)
      prev_pinState18 = 0;                                     // record pin state
      PW18 = pciTime18 - pwmTimer18;                            // calculate the duration of the current pulse
      pwmFlag18 = HIGH;                                        // flag that new data is available
      RC_data_rdy = HIGH;                  
    }
  
}

void IRAM_ATTR ISR19() {
  
  pciTime19 = micros();                                             // Record the time of the PIN change in microseconds

    if (prev_pinState19 == 0)   {          // and the pin state has changed from LOW to HIGH (start of pulse)
      prev_pinState19 = 1;                                     // record pin state
      pwmPeriod19 = pciTime19 - pwmTimer19;                     // calculate the time period, micro sec, between the current and previous pulse
      pwmTimer19 = pciTime19;                                    // record the start time of the current pulse
    }
    else if (prev_pinState19 == 1){ // or the pin state has changed from HIGH to LOW (end of pulse)
      prev_pinState19 = 0;                                     // record pin state
      PW19 = pciTime19 - pwmTimer19;                            // calculate the duration of the current pulse
      pwmFlag19 = HIGH;                                        // flag that new data is available
      RC_data_rdy = HIGH;                  
    }
  
}

/*
void IRAM_ATTR ISR() {
  
  pciTime = micros();                                             // Record the time of the PIN change in microseconds

  for (int i = 0; i < num_ch; i++){
    reg_state = REG_GET_BIT(GPIO_IN_REG,pwmPIN_reg[pwmPIN[i]]);
    // if ((prev_pinState[i] == 0) && (REG_READ(GPIO_IN_REG) & pwmPIN_reg[pwmPIN[i]])){          // and the pin state has changed from LOW to HIGH (start of pulse)
    if ((prev_pinState[i] == 0) && !reg_state)  {          // and the pin state has changed from LOW to HIGH (start of pulse)
      prev_pinState[i] = 1;                                     // record pin state
      pwmPeriod[i] = pciTime - pwmTimer[i];                     // calculate the time period, micro sec, between the current and previous pulse
      pwmTimer[i] = pciTime;                                    // record the start time of the current pulse
    }
    else if ((prev_pinState[i] == 1) && reg_state){ // or the pin state has changed from HIGH to LOW (end of pulse)
      prev_pinState[i] = 0;                                     // record pin state
      PW[i] = pciTime - pwmTimer[i];                            // calculate the duration of the current pulse
      pwmFlag[i] = HIGH;                                        // flag that new data is available
      if(i+1 == RC_inputs) RC_data_rdy = HIGH;                  
    }
  }
}
*/


void setup_pwmRead(){
    pinMode(pwmPIN18,INPUT);              // run through each input pin
    pinMode(pwmPIN19,INPUT);              // run through each input pin
    attachInterrupt(pwmPIN18, ISR18, CHANGE);                        // enable pinchange interrupt for pin
    attachInterrupt(pwmPIN19, ISR19, CHANGE);                        // enable pinchange interrupt for pin
} 

boolean RC_avail(){
  boolean avail = RC_data_rdy;
  RC_data_rdy = LOW;                          // reset the flag
  return avail;
  }

void print_RCpwm(){                             // display the raw RC Channel PWM Inputs
    Serial.print(" ch");
    Serial.print(18);
    PWM_read18();
    Serial.print("  ");
    if(PW18 < 1000) Serial.print(" ");
    Serial.print(PW18);
    Serial.print(" ch");
    Serial.print(19);
    PWM_read19();
    Serial.print("  ");
    if(PW19 < 1000) Serial.print(" ");
    Serial.println(PW19);
    // Serial.print("  ");
    // String str_PWM = String(PWM_period(),3);
    // Serial.print(str_PWM);
  
  // Serial.println("");
}


void setup() {
    Serial.begin(115200);
    pwmPin_reg_init();                      
    setup_pwmRead();
}

void loop() {
    now = millis();
      if(RC_avail() || now - rc_update > 25){   // if RC data is available or 25ms has passed since last update (adjust to suit frame rate of receiver)
      rc_update = now;                           
      print_RCpwm();                        // uncommment to print raw data from receiver to serial
      }
}