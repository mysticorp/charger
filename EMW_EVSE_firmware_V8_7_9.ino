/*
EMotorWerks JuiceBox - an open-source 15kW EVSE

Micro-Controller: Arduino Pro Mini 5V, (based on a ATmega328P-PU microcontroller)

this version is matching V8.0-8.7 boards

Basic code structure:
Startup:
* initialize pins
* set output power level based on trimpot
* set duty cycle to 0

In endless loop:
* check for J1772 state
* check for EV & diode presence
* check for EV requesting power (state C)
* close relay to provide power (this is optional and code will work if no relay is present)
* run loop with power until non-C state detected or a button pressed
*     measure current and increment energy meter
*     display major params on the screen (this is optional and code will work if no LCD is present)

Created Jan 2013 by Electric Motor Werks, Inc. / Valery Miftakhov, Copyright 2013+

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, version 3 of the License

In a nutshell, it says if you wish to modify and distribute any derivation of this code, 
you must also distribute your modifications, and you MUST make the complete source code available.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details: http://www.gnu.org/licenses/
*/

//------------------------------ MAIN SWITCHES -----------------------------------

#define AC1075
const int R_C=120; // this is the value of the shunting resistor. see datasheet for the right value. 
const int V_AC_threshold=164; // normally 164 (midpoint between 120V and 208V
const int V_AC_sensitivity=180; // normally 180 (empirical)
#define PCB_83 // 8.3+ version of the PCB, includes 8.6, 8.7 versions
#define VerStr "V8.7.9 ejw 4 (9/14/2015)" // detailed exact version of firmware (thanks Gregg!)
#define GFI // need to be uncommented for GFI functionality
//------------------------------- END MAIN SWITCHES ------------------------------

#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

// Additions for use of Adafruit display and DS3223RTC
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <DS3232RTC.h>        //http://github.com/JChristensen/DS3232RTC
#include <Streaming.h>        //http://arduiniana.org/libraries/streaming/
#include <Time.h>             //http://playground.arduino.cc/Code/Time

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

// need this to remap PWM frequency
#include <TimerOne.h>

//------------------ current sensor calibration - only for AC1075 for now -----------------------------------------------
// the current sensing is done using a simple diode rectifier. As such, there is natural non-linearity to readings
// this lookup table takes care of that. Alternative approach is a precision rectifier using an op amp but 
// that adds a bit in parts and cost so taking a software approach here
// one entry per 0.1V in observed voltage on A1 pin. Since we have a 3.3v zener on that pin, need only 32-element array
// array contains current value in 0.1A units
// if the sensing was purely linear, we'd expect ~18A/volt but the forward drop on diode varies 
const unsigned int AC1075_calibration[32]={0,0,5,28,49,70,93,116,139,
                                           162,185,205,227,250,270,290,310,
                                           332,355,376,400,424,448,474,500,
                                           525,549,575,600,625,650,675};
//------------------ END current sensor calibration ---------------------------------------------------------------------

//---------------- savings consts for displays 
const int gascost=350; // in cents per gallon
const int mpg=25; // mpg of the gasoline car
const int ecost=12; // in cents per kWhr
const int whpermile=300; // energy efficiency of the ecar
int savingsPerKWH; // will be recalced later
//---------------- end of the energy constants

//---------------- pin-out constants ----------------
//---------------- analog inputs
const byte pin_pV=0; // pilot signal sensed through a 3-element divider 
const byte pin_V=1; // input voltage 
const byte pin_C=2; // AC current - as measured by the current transformer
const byte pin_throttle=3; // wired to a trimpot on a board
// pins A4 / A5 reserved for SPI comms to RTC chip

//---------------- digital inputs / outputs
// GFI trip pin - goes high on GFI fault, driven by the specialized circuit based on LM1851 
// has to be pin 3 as only pin 2 and 3 are available for interrupts on Pro Mini
const byte pin_GFI=3; 
const byte pin_inRelay=5; 
const byte pin_ctrlBtn_C=6; // control button 1 ("C" on the remote, receiver pin 2)
const byte pin_ctrlBtn_D=8; // control button 2 ("D" on the remote, receiver pin 3)
const byte pin_PWM=9; // J pilot PWM pin

const byte pin_StatusLight=10;

const byte pin_ctrlBtn_A=11; // control button 3 ("A" on the remote, receiver pin 0) 
const byte pin_GFItest=12; // pin wired to a GFCI-tripping relay - for the periodic testing of the GFCI circuit & stuck relay detection
//---------------- END PINOUTS -----------------------

//---------------- Status Light Levels ---------------
const byte status_Off=0;
const byte status_Low=10;
const byte status_Mid=128;
const byte status_Full=255;
//---------------- End Status Light Levels -----------------------

//==================================== calibration constants etc
const float Aref=5.; // should be close
float pV_min=-12.;
float V_J1772_pin_=0; // global pilot voltage
const float divider_pV_R=100./27.; // 100k over 27k
float V_Ard_pin_0;
//===============================================================

//========== define J1772 states ===============================
// defaults
const float def_state_A_Vmin=10.5, def_state_A_Vmax=14; 
const float def_state_B_Vmin=7.5, def_state_B_Vmax=10.5; 
const float def_state_C_Vmin=4.5, def_state_C_Vmax=7.5; 
const float def_state_D_Vmin=1.5, def_state_D_Vmax=4.5; 
const float def_state_E_Vmin=-1.5, def_state_E_Vmax=1.5; 
const float def_state_F_Vmin=-14., def_state_F_Vmax=-10.; 
// now adjusted for the actual voltages
float state_A_Vmin, state_A_Vmax; 
float state_B_Vmin, state_B_Vmax; 
float state_C_Vmin, state_C_Vmax; 
float state_D_Vmin, state_D_Vmax; 
float state_E_Vmin, state_E_Vmax; 
float state_F_Vmin, state_F_Vmax; 
#define STATE_INVALID 255
#define STATE_A 0
#define STATE_B 1 
#define STATE_C 2
#define STATE_D 3
#define STATE_E 4
#define STATE_F 5
//=========== end definition of J1772 states ===================

// these should be global vars  -----------------------------
unsigned int duty=0, set_duty=0;
const unsigned int PWM_res=1024;
const unsigned int PWM_FULLON=1024;
const unsigned int MAXDUTY=970; // <97% to stay in AC charging zone for J1772 standard

int sawTemp = 0;
float lastTemp = 0;

const float maxC=60; // max rated current
float inV_AC=0; // this will be measured
const float nominal_outC_240V=30; // 30A by default from a 240VAC line
const float nominal_outC_120V=15; // 15A by default from a 120VAC line
float outC=nominal_outC_240V; 
float power=0;
float energy=0; // how much energy went through - in kWHrs 
char tempstr[100]; // scratchpad for text operations

byte GFI_tripped=0;
byte GFI_trip_count=0;

int state=-1, prev_state=-1;
int min2nextrun;
// ------------- end global vars ---------------------------

//------------- timing parameters --------------------------
unsigned long timer=0, timer0=0, timer_sec=0;
long clock_offset=0; // this has to be a signed value
unsigned long sec_up=0; // uptime since last reboot
const byte GFIblankingtime=100; // mask GFI trips for this many milliseconds from relay's closing - anti-noise
unsigned int delta=0;

// sensor timings
const byte meas_cycle_delay=100; // in ms

// start and end times by weekday
//const char *daysStr[7]={"Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"};
// initialize the clock - assume no RTC and that we are getting turned on at the hour
//byte day=5, hour=12, mins=0; // default day is Sat, time is noon, 0 min
//------------ end timing params ---------------------------

void setup() {
  wdt_disable();
  
  Serial.begin(115200);
  Serial.println("Starting Initialization");

  // set digital input pins
  pinMode(pin_GFI, INPUT);
  pinMode(pin_ctrlBtn_A, INPUT);
  pinMode(pin_ctrlBtn_C, INPUT);
  pinMode(pin_ctrlBtn_D, INPUT_PULLUP);

  // set digital output pins
  pinMode(pin_PWM, OUTPUT);
  pinMode(pin_inRelay, OUTPUT);
  pinMode(pin_StatusLight, OUTPUT);
  pinMode(pin_GFItest, OUTPUT);

  // Indicate we are booting on status light
  analogWrite(pin_StatusLight, status_Low);

  Serial.println("After Pin Initialization");

  //---------------------------------- set up timers
  cli();//stop interrupts

  // use Timer1 library to set PWM frequency 
  // 10-bit PWM resolution
  Timer1.initialize(1000); // 1kHz for J1772
  Timer1.pwm(pin_PWM, 0); 
  
  //set timer2 interrupt at 8kHz
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 8khz increments
  OCR2A = 249;// = (16*10^6) / (8000*8) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 8 prescaler
  TCCR2B |= (1 << CS21);   
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);

  sei();
  //---------------------------------- end timer setup

  //---------------------------- calibrate state boundaries ---------------------------------------------
  // first, need to record a minimum value of the wave - needed for pilot voltage measurement later on
  // set output pin to negative rail
  setPilot(0); // this should produce a steady -12V signal on a pilot pin
  pV_min=read_pV(); // this is supposed to be -12V

  // now calibrate the pilot voltage thresholds based on the actual voltage of positive rail 
  // calibration is done at every power-up
  setPilot(PWM_FULLON); // this should produce a steady +12V signal on a pilot pin
  float pVcal=read_pV();

  // set default thresholds for pilot signal levels
  state_A_Vmin=def_state_A_Vmin; state_A_Vmax=def_state_A_Vmax; 
  state_B_Vmin=def_state_B_Vmin; state_B_Vmax=def_state_B_Vmax; 
  state_C_Vmin=def_state_C_Vmin; state_C_Vmax=def_state_C_Vmax; 
  state_D_Vmin=def_state_D_Vmin; state_D_Vmax=def_state_D_Vmax; 
  state_E_Vmin=-1.5, state_E_Vmax=1.5; 
  state_F_Vmin=-14., state_F_Vmax=-10.; 
  
  // recalibrate the pilot sensing code. helps fight any possible temperature / aging drifts
  // but only do it if it's not too far off - this will prevent recalibration in case the power 
  // cycles while the JuiceBox is plugged into the car
  // note that this will mean that the JuiceBox would not be able to recalibrate if the pilot is more than 
  // 10% off (unlikely with a precision 12V regulator used and R-R op amp)
  if(pVcal>def_state_B_Vmax) {  
    pVcal/=12.; // calibration constant
    // now adjust boundaries for top being not 12V
    state_A_Vmin=def_state_A_Vmin*pVcal;  state_A_Vmax=def_state_A_Vmax*pVcal; 
    state_B_Vmin=def_state_B_Vmin*pVcal;  state_B_Vmax=def_state_B_Vmax*pVcal; 
    state_C_Vmin=def_state_C_Vmin*pVcal;  state_C_Vmax=def_state_C_Vmax*pVcal; 
    state_D_Vmin=def_state_D_Vmin*pVcal;  state_D_Vmax=def_state_D_Vmax*pVcal; 
    state_E_Vmin=-1.5, state_E_Vmax=1.5; 
    state_F_Vmin=-14., state_F_Vmax=-10.; 
  }
  
  //-------------------- ONE-TIME: determine input / output AC voltage
  // this has to run before attaching interrupt to the GFI break pin
  // set the baseline 
  // V_Ard_pin_0=analogRead(pin_V)*Aref/1024.;
  V_Ard_pin_0=0; // DEBUG - override for now
  
  // now check for a stuck relay and measure input voltage 
#ifndef PCB_81
  GFI_tripped=0;
  // force the GFI pin
  digitalWrite(pin_GFItest, HIGH);
#ifdef GFI
  delay(100); 
  // by now, if the trip occurred, the GFI trip flag should be set
  if(GFI_tripped==1) {
    // we have a stuck relay, throw an error
    printErrorMsg(F("STUCK RELAY! \nContact us\nExiting..."), 30000);
    return; // break from loop() which will be called back a moment later
  }
#endif
  // turn on the main relay
  setRelay(HIGH);
  // wait for settling (RC on the pin is 0.1s so need to wait at least for 0.3s
  // but not too long or we will burn out the 10k resistor...
  delay(300);
#endif
  inV_AC=read_V();    
  digitalWrite(pin_GFItest, LOW);
  setRelay(LOW);  
  
  // attach interrupt on pin 3 (GFI)
#ifdef GFI
  attachInterrupt(1, GFI_break, RISING);
#endif

  // prep for calc of the savings
  getSavingsPerKWH(gascost, mpg, ecost, whpermile);
  
  // set watchdog - http://tushev.org/articles/arduino/item/46-arduino-and-watchdog-timer, http://www.nongnu.org/avr-libc/user-manual/group__avr__watchdog.html
  wdt_enable(WDTO_8S); // longest is 8S
  
  // Initialize Display
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3c);  // initialize with the I2C addr 0x3D (for the 128x64)

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  // init done
  
  // Initialize RTC
  //setSyncProvider() causes the Time library to synchronize with the
  //external RTC by calling RTC.get() every five minutes by default.
  setSyncProvider(RTC.get);
  Serial.print("RTC Sync");
  if (timeStatus() != timeSet) Serial.print(" FAIL!");
  Serial.println("");


  // initialize in state A - EVSE ready
  setPilot(PWM_FULLON);
}


//============================================= MAIN LOOP ============================================
void loop() {
  
  // reset GFI trip status so we can retry after GFI timeout
  // GFI is checked in the end of this cycle - by that time, a few hundreds ms pass
  GFI_tripped=0; 
      
  // check if the car is there and requesting power
  prev_state=state;
  state=getState(); // this is a blocking call for <100ms
  
  // manage state changes
  if(state!=prev_state) {
    timer=millis(); // start timer
    timer0=timer; // remember the start of charge
    
    if(state==STATE_C) {
      // entering charging state - check for diode
      setPilot(PWM_FULLON/2);
      if(read_pV()>-1.5) {
        state=STATE_F; // diode check failure!
      }
      energy=0; // reset energy counter for this cycle  
    }

    if(prev_state==STATE_C) { // exiting state C - charging
      analogWrite(pin_StatusLight, status_Mid);
    }
  } // end state transition check

  if(state!=STATE_C) {
    analogWrite(pin_StatusLight, status_Mid);
  }

  //-------------------------------- process states
  if(state==STATE_A) {
    setRelay(LOW); // relay off
    setPilot(PWM_FULLON);
  }
    
  if(state==STATE_B) {
    setRelay(LOW); // relay off
    setOutC();
    setPilot(duty);
  }
    
  if(state==STATE_C) {
    setOutC();
    setPilot(duty);
    setRelay(HIGH); // relay on
    
    analogWrite(pin_StatusLight, status_Full);

    // process energy metering
    float outC_meas=read_C();
    power=outC_meas*inV_AC/1000; // in KW

    delta=int(millis()-timer);
    timer=millis();
    energy+=power*delta/1000/3600; 

    // print real-time stats
    printTime();    
    sprintf(tempstr, "Power: %d.%01d KW  ", int(power), int(power*10)%10); 
        printJBstr(0, 2, 2, 0x1f, 0x3f, 0, tempstr);   
    sprintf(tempstr, "Time: %d min  ", int((timer-timer0)/1000)/60); 
        printJBstr(0, 3, 2, 0x1f, 0x3f, 0, tempstr);   
    // also show energy cost in this one
    // use US average cost per http://www.eia.gov/electricity/monthly/epm_table_grapher.cfm?t=epmt_5_6_a - $0.12/kwhr
    sprintf(tempstr, "%d.%01d KWH ($%d.%02d) ", int(energy), int(energy*10)%10, int(energy/8), int(energy/8*100)%100 ); 
        printJBstr(0, 5, 2, 0x1f, 0x3f, 0, tempstr);   
    sprintf(tempstr, "%dV, %dA (%d) ", int(inV_AC), int(outC_meas), int(outC)); 
        printJBstr(0, 7, 2, 0x1f, 0x3f, 0, tempstr);   
        
    // print button menu
    printJBstr(0, 9, 2, 0, 0, 0x1f, F("A=outC+, D=outC- \nB=WPS")); 
  } // end state_C
  
  if(state==STATE_D) {
    // printClrMsg(F("Vehicle requested\nVENTILATED power!\nExiting..."), 1000, 0x1f, 0x3f, 0);
    setRelay(LOW); // relay off
  }
  
  if(state==STATE_E || state==STATE_F || state==STATE_INVALID) {
    printClrMsg(F("Abnormal State!"), 1000, 0x1f, 0x3f, 0);
    setRelay(LOW); // relay off
  }  
          
  // display standby
  if(state==STATE_A || state==STATE_B) {  
      // set the output current - can be changed by trimpot or remote without a restart
      // need this here so we have an echo on user input
      setOutC(); 
      
       printTime();
      // no LCD
      sprintf(tempstr, "%dV, %dA", int(inV_AC), int(outC));
      Serial.println(tempstr);  
      analogWrite(pin_StatusLight, status_Mid);
    }

  delay(meas_cycle_delay); // reasonable delay for screen refresh

#ifdef GFI
  // check GFI flag (if a trip is detected, this flag would be set via the special interrupt)
  if(GFI_tripped) {
    printClrMsg(F("GFI tripped!\nRetrying in 15 min..."), 300, 0x1f, 0x3f, 0);
    GFI_trip_count++; // allowed max of 4; if more than 4, need 2 user inputs to override
    if(GFI_trip_count>4) {
      // wait for user to unplug; since the user then will have to re-plug to re-energize the system, this can be considered 2 actions
      printClrMsg(F("4th GFI trip!\nUnplug / re-plug\nto resume"), 1000, 0x1f, 0x3f, 0);
      while(getState()!=STATE_A);
    } else {
    }  
  }
#endif

} // end loop()


//=================================================== FUNCTIONS ==============================================
// interrupt - break on GFI trigger - breaks on RISING signal on pin 3 (transistor end of the relay)
void GFI_break() {
  // mask if within certain time from closing the main relay
  if(millis()>timer0 && millis()-timer0<GFIblankingtime) return; // first term protects from millis() overflow
  
  // check every 2mS for 20ms - if not a single additional trips, ignore
  for(byte i=0; i<10; i++) {
    delayMicroseconds(2000);
    if(digitalRead(pin_GFI)==HIGH) {
      GFI_tripped=1;
      // before tripping relay, kill pilot - maybe the onboard charger will drop current quickly and help save our relay contacts...
      setPilot(0); // -12V on pilot - ABNORMAL STATE. Compliant chargers should stop
      setRelay(LOW); // kill power NOW. generally, relay will take ~20-25ms to open
      break;
    }
  }
}

// operate output relay
void setRelay(byte state) {
  digitalWrite(pin_inRelay, state);
}


void setPilot(int _duty) {
  set_duty=_duty;
  Timer1.setPwmDuty(pin_PWM, _duty);
}


// set the pilot duty corresponding to the output current based on the pot or stored settings
// use default current setting if pin is grounded
void setOutC() {
  float minThrottle=0.05;
  float throttle=0;

  // different trimpot depending on voltage
  if(inV_AC==120) {
    outC=min(nominal_outC_120V, outC);
  } else {
    // 208V+ setting
      throttle=analogRead(pin_throttle)/1024.;
      if(throttle>minThrottle) { // if something is set on the throttle pot, use that instead of the default outC
        outC=throttle*maxC;
      }
  }

  // per J1772 standard:
  // 1% duty = 0.6A until 85% duty cycle
  // after that, 1% = 2.5A up to 96%
  if(outC<51) {
    duty=PWM_res*outC/60.;
  } else {
    duty=PWM_res*(0.64+outC/250.);
  }
  
  if(duty>MAXDUTY) duty=MAXDUTY;
}


// this will block for ~200ms due to read_pV()
int getState() {
  byte mode=1; // PWM is on 
  if(set_duty==PWM_FULLON) mode=0; // PWM is off
  
  float pV=read_pV();
  
  // in mode=1, the state is measured while pilot is oscillating so need to recalc
  // pV=pV_min*(1-duty)+pV_max*duty
  // so pV_max=(pV-pV_min*(1-duty))/duty
  if(mode==1) pV=((pV-pV_min)*PWM_res+pV_min*duty)/duty;
  
  if(pV>state_A_Vmin && pV<=state_A_Vmax) return STATE_A;
  if(pV>state_B_Vmin && pV<=state_B_Vmax) return STATE_B;
  if(pV>state_C_Vmin && pV<=state_C_Vmax) return STATE_C;
  if(pV>state_D_Vmin && pV<=state_D_Vmax) return STATE_D;
  if(pV>state_E_Vmin && pV<=state_E_Vmax) return STATE_E;
  if(pV>state_F_Vmin && pV<=state_F_Vmax) return STATE_F;

  return STATE_INVALID;
}


// read the average pilot voltage - this is a BLOCKING CALL (200ms)
// time constant of the RC filter: 27k/2 * 3.3uF = ~0.04s - enough to smooth 1kHz signal
float read_pV() {
  // ensure settling of the signal before measurement
  delay(100); // this is ~2.5 time constants of the RC filter on this pin - measured value should be within 2% of its settled value
  int reading=analogRead(pin_pV); // this takes 100uS
  // for anti-noise, read 180 degree off the prev reading 
  // (integer number of milliseconds + 500 uS (half-PWM-period) - ADC conversion time)
  delayMicroseconds(2500); 
  reading+=analogRead(pin_pV);
  float V_Ard_pin=reading*Aref/1024./2;

  V_J1772_pin_=(2*V_Ard_pin-5)*divider_pV_R+V_Ard_pin;

  return V_J1772_pin_;
}

// read the average input AC voltage 
// this function should ONLY BE CALLED in setup()
// time constant of the RC filter: 27k * 3.3uF = ~0.09s - enough to smooth 60Hz signal
float read_V() {
  float V_AC=240; // default is 240
  
  float V_Ard_pin=analogRead(pin_V)*Aref/1024.;
  delay(8); // measure 180 degrees away by AC phase to smooth out any remaining ripple on A1 pin
  V_Ard_pin+=analogRead(pin_V)*Aref/1024.;
  V_Ard_pin/=2;
  
#ifdef PCB_81
  // for PCB versions before 8.3, 
  // THIS FEATURE IS IN BETA AND MAY NOT WORK ON THE FIRST VERSION OF THE BASE BOARDS WITHOUT TWEAKING FIRMWARE
  // specifically, you may need to tweak the voltage threshold between 120V and 240V input voltage 
  //   (line starting with 'if(V_Ard_pin >' below). (1) connect JuiceBox to 120V, measure the voltage on pin A1 of the Arduino
  //   (2) connect JuiceBox to 240V, measure the voltage on pin A1. Set the threshold to the voltage in the middle between 
  //   these two values
  // with 200k resistor from AC rectified line, we have 0.8mA peak primary current at 120VAC
  //     and 1.6mA at 240VAC
  // according to PC817X1 opto's datasheet, CTR is 80-160% at 5mA
  // typical curve suggests 80% of that at 2.5mA, 50% at 1mA
  // therefore, we have a secondary peak current of 0.3-0.6mA at 120VAC, 1-2mA at 240VAC
  // this corresponds to a secondary voltage drop: 0.3-0.6V or 1-2V per 1k of secondary resistance
  //               (actually clipped to 5V since we are using 5v supply)
  // also, need to take into account that we see the significant current only at the positive peak of AC wave
  // generally, for ~1/4 of the period for 120VAC and 1/3rd for 240VAC
  // finally, the average drop within over the drop time is ~1/2 of the peak drop
  // putting it all together, we expect average drop of 0.04-0.08V per 1k for 120VAC and 0.16-0.32V per 1k for 240VAC 
  //               (with some clipping starting at 3-5k - really becoming visible at 5-7k)  
  // Example: 4.7k secondary - 0.2-0.4V and 0.8-1.5V drops
  // Example: 10k secondary - 0.4-0.8V and 1.2-2.2V drops
  // Using 10k secondary, place mid-point at 1V drop, or 4V threshold
  // cap at 4.9V to prevent from defaulting to 120V in case when no PC817 installed at all
  if(V_Ard_pin > 3.5 && V_Ard_pin<4.9) V_AC=120;
#endif

#ifdef PCB_83
  // in 8.3 and later, the implementation changed to measurement using the GFCI current sensor
  // ~200x division factor for voltage (total gain of test loop is ~1.2e-2)
  //     (306 from RMS voltage on sensor = 680x on the opamp, 0.5x due to half-wave rectification, 0.9x for converting to average from RMS)
  //     (3.9e-5x from 0.39V/A sensor sensitivity with 390R shunt resistor, 0.0001A/V voltage-to-current conversion on a 10k resistor)
  //

  // if no GFI installed, cannot measure voltage - default to 240V 
#ifdef GFI
  V_AC=V_AC_sensitivity*(V_Ard_pin-V_Ard_pin_0);
#else
  V_AC=240;
#endif
  
  if(V_AC < V_AC_threshold) { // midpoint between 120 and 208V
    V_AC=120;
  } else if(V_AC < 220) { // midpoint between 208V and 240V, allowing for sag of 4V
    V_AC=208;
  } else {
    V_AC=240; // default fall-back value
  }
  
#endif

  return V_AC; 
}

// read the AC current via the current transformer
// in the absense of the current transformer this will return zero
// RC constant defined by R11 C5 = 27k * 3.3uF = 90ms, or >5 line periods
float read_C() {
#ifdef AC1075
  const int Te=1000; // # of turns
#endif
#ifdef CT_8349-1500
  const int Te=1500; // # of turns
#endif
#ifdef CT_8420-1000
  // assume 8420-1000 current transformer (50A max current, 20-1000 Hz working range)
  const int Te=1018; // # of turns
#endif
#ifdef CT_3100
  // assume 3100 current transformer (75A max current, 20-1000 Hz working range)
  const int Te=3100; // # of turns
#endif

  // read the rectified voltage of the half-wave from the transformer
  // average between 2 readings 180 degree off each other
  int reading=analogRead(pin_C);
  delay(8);
  reading+=analogRead(pin_C);
  // this assumes an RC filter before Arduino pon with time constant >> line period and impedance >> R
  float V_C=reading*Aref/2/1024; 

#ifdef AC1075
  // use a table lookup
  int index=floor(V_C*10);
  float remainder=V_C*10-index;
  if(index>31) return 75.; // prevent from array overflow
  float coeff=float(AC1075_calibration[index+1]-AC1075_calibration[index]); // 0.1V step
  return float(AC1075_calibration[index]+remainder*coeff)/10.; // linear extrapolation
#else
  // use a crude linear approximation
  if(V_C>0.2) {
    V_C+=0.2+V_C/6; // assuming 0.2V diode drop
  } else V_C=0;

  // *2 for half-wave rectification, 1.11 for conversion of average into RMS
  // for AC1050-1075 this corresponds to ~18A/V
  return V_C*Te/R_C*2.22;  
#endif
}

//------------------------------ printing help functions -------------------------
void printJBstr(byte col, byte row, byte font, byte c1, byte c2, byte c3, const __FlashStringHelper *fstr) {
    Serial.print("    ");
    Serial.println(fstr);
}
void printJBstr(byte col, byte row, byte font, byte c1, byte c2, byte c3, const char *sstr) {
    Serial.print("    ");
    Serial.println(sstr);
}
void printClrMsg(const __FlashStringHelper *fstr, const int del, const byte red, const byte green, const byte blue) {
  printJBstr(0, 2, 2, red, green, blue, fstr);      
  delay(del);
}
void printClrMsg(const char *str, const int del, const byte red, const byte green, const byte blue) {
  printJBstr(0, 2, 2, red, green, blue, str);      
  delay(del);
}
void printErrorMsg(const __FlashStringHelper *fstr, const int del) {
  printClrMsg(fstr, 30000, 0x1f, 0x3f, 0);
}

// print time etc on the first line
void printTime() {
  // have to use tempstr here
  sprintf(tempstr, "%s", VerStr);
//  sprintf(tempstr, "%s %02d:%02d (%d) ", VerStr, hourOfDay(), minsOfHour(), state); 
  printJBstr(0, 0, 2, 0x1f, 0, 0x1f, tempstr);     
}
//---------------------------- end printing help functions ------------------------

//---------------------------- input control functions ----------------------------
byte limit(const byte value, const byte minimum, const byte maximum) {
  if(value<minimum) return minimum;
  if(value>maximum) return maximum;
  return value;
}

void getSavingsPerKWH(int gascost, int mpg, int ecost, int whpermile) {
  int gCostPerMile=gascost/mpg;
  int gCostPerKWH=gCostPerMile*1000/whpermile;
  
  savingsPerKWH=gCostPerKWH-ecost;
}

// long delays
void delaySecs(int secs) {
  for(int si=0; si<secs; si++) delay(1000);
}

