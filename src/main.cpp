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

//Graupner MX-20 mit GR-24 HOTT  
//                THR     RUD     PIT     BAL     SWITCH  SLIDER
int RC_min[6] = { 1342,   1340,   976,    960,    1100,   1116};
int RC_mid[6] = { 1500,   1500,   1424,   1398,   1500,   1460};
int RC_max[6] = { 1660,   1658,   1796,   1764,   1900,   1796};

// fail safe positions

float RC_failsafe[] = {0.00, 0.00, 1, 0.00, -0.25, 0.00};

const int size_RC_min = sizeof(RC_min) / sizeof(int);           // measure the size of the calibration and failsafe arrays
const int size_RC_mid = sizeof(RC_mid) / sizeof(int);
const int size_RC_max = sizeof(RC_max) / sizeof(int);
const int size_RC_failsafe = sizeof(RC_failsafe) / sizeof(float);


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

void print_decimal2percentage(float dec){
  int pc = dec*100;
  // the number and text will take up 6 charactors i.e ___3%_ or -100%_
  if (pc >= 0) Serial.print(" ");
  if (abs(pc) < 100) Serial.print(" ");
  if (abs(pc) < 10) Serial.print(" ");
  Serial.print(" ");Serial.print(pc);Serial.print("% ");
}

float calibrate(float Rx, int Min, int Mid, int Max){
   float calibrated;
   if (Rx >= Mid)
   {
    calibrated = map(Rx, Mid, Max, 0, 1000);  // map from 0% to 100% in one direction
   }
   else if (Rx == 0)
   {
    calibrated = 0;                           // neutral
   }
   else
   {
    calibrated = map(Rx, Min, Mid, -1000, 0); // map from 0% to -100% in the other direction
   }
  return calibrated * 0.001;
}

// Basic Receiver FAIL SAFE
// check for 500-2500us and 10-330Hz (same limits as pololu)

boolean FAILSAFE(int CH){

   int i = CH-1;
   boolean failsafe_flag = LOW;
        
       if(pwmFlag[i] == 1)                             // if a new pulse has been measured.
         {
            pwmFlag[i] = 0;                            // set flag to zero
      
            if(pwmPeriod[i] > 100000)                  // if time between pulses indicates a pulse rate of less than 10Hz   
            {
              failsafe_flag = HIGH;                       
            }
            else if(pwmPeriod[i] < 3000)               // or if time between pulses indicates a pulse rate greater than 330Hz   
            {
              failsafe_flag = HIGH;                             
            }

            if(PW[i] < 500 || PW[i] > 2500)           // if pulswidth is outside of the range 500-2500ms
            {
              failsafe_flag = HIGH;                        
            }   
         }
        else if (micros() - pwmTimer[i] > 100000)     // if there is no new pulswidth measurement within 100ms (10hz)
        {
          failsafe_flag = HIGH;                      
        }

    return failsafe_flag;   
}

float RC_decode(int CH){
  if(CH < 1 || CH > RC_inputs) return 0;     // if channel number is out of bounds return zero.
  int i = CH - 1;                     

  // determine the pulse width calibration for the RC channel. The default is 1000, 1500 and 2000us.
  
  int Min;
  if(CH <= size_RC_min) Min = RC_min[CH-1]; else Min = 1000;
  
  int Mid;
  if(CH <= size_RC_mid) Mid = RC_mid[CH-1]; else Mid = 1500;
  
  int Max;
  if(CH <= size_RC_max) Max = RC_max[CH-1]; else Max = 2000;

  float CH_output;
      
  if(FAILSAFE(CH) == HIGH){                         // If the RC channel is outside of failsafe tolerances (10-330hz and 500-2500uS)
      if(CH > size_RC_failsafe) CH_output = 0;      // and if no failsafe position has been defined, set output to neutral
      else CH_output = RC_failsafe[i];              // or if defined set the failsafe position 
  }
  else{                                             // If the RC signal is valid
    CH_output = calibrate(PW[i],Min,Mid,Max);       // calibrate the pulse width to the range -1 to 1.
  }
  return CH_output;                                 

  // The signal is mapped from a pulsewidth into the range of -1 to +1, using the user defined calibrate() function in this code. 

  // 0 represents neutral or center stick on the transmitter
  // 1 is full displacement of a control input is one direction (i.e full left rudder)
  // -1 is full displacement of the control input in the other direction (i.e. full right rudder)
}



void setup() {
    Serial.begin(115200);
    setup_pwmRead();
}

void loop()  {
    now = millis();
    
    if(RC_avail() || now - rc_update > 25){   // if RC data is available or 25ms has passed since last update (adjust to suit frame rate of receiver)
      
      rc_update = now;                           
      
      // print_RCpwm();                        // uncommment to print raw data from receiver to serial
      
      for (int i = 0; i<channels; i++){       // run through each RC channel
        int CH = i+1;
        RC_in[i] = RC_decode(CH);             // decode receiver channel and apply failsafe
        print_decimal2percentage(RC_in[i]);   // uncomment to print calibrated receiver input (+-100%) to serial       
      }
      Serial.println();                       // uncomment when printing calibrated receiver input to serial.
    }
}
