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

const int PROGMEM R_C=120; // this is the value of the shunting resistor. see datasheet for the right value. 
const int PROGMEM V_AC_threshold=164; // normally 164 (midpoint between 120V and 208V
const int PROGMEM V_AC_sensitivity=180; // normally 180 (empirical)
//#define VerStr "V8.7.9+" // detailed exact version of firmware (thanks Gregg!)
#define GFI // need to be uncommented for GFI functionality
//------------------------------- END MAIN SWITCHES ------------------------------
#define ADAFRUIT
#define DS3232RTC_DEFINE
//#define SERIAL_PRINTS

#ifdef SERIAL_PRINTS
#define ps(x) Serial.print(x);
#define psln(x) Serial.println(x);
#else
#define ps(x)
#define psln(x)
#endif

#define pit(x) snprintf(tempstr, TEMP_STRING_LEN, x);display.print(tempstr); ps(tempstr)
#define pitln(x) snprintf(tempstr, TEMP_STRING_LEN, x);display.println(tempstr); psln(tempstr)

#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

// Additions for use of Adafruit display and DS3223RTC
#ifdef DS3232RTC_DEFINE
#include <SPI.h>
#include <Wire.h>
#endif
#ifdef ADAFRUIT
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#endif

#ifdef DS3232RTC_DEFINE
#include <DS3232RTC.h>        //http://github.com/JChristensen/DS3232RTC
#include <Time.h>             //http://playground.arduino.cc/Code/Time
#endif

#ifdef ADAFRUIT
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);
#endif

// need this to remap PWM frequency
#include <TimerOne.h>

//------------------ current sensor calibration - only for AC1075 for now -----------------------------------------------
// the current sensing is done using a simple diode rectifier. As such, there is natural non-linearity to readings
// this lookup table takes care of that. Alternative approach is a precision rectifier using an op amp but 
// that adds a bit in parts and cost so taking a software approach here
// one entry per 0.1V in observed voltage on A1 pin. Since we have a 3.3v zener on that pin, need only 32-element array
// array contains current value in 0.1A units
// if the sensing was purely linear, we'd expect ~18A/volt but the forward drop on diode varies 
const unsigned int PROGMEM AC1075_calibration[32]={0,0,5,28,49,70,93,116,139,
                                           162,185,205,227,250,270,290,310,
                                           332,355,376,400,424,448,474,500,
                                           525,549,575,600,625,650,675};
//------------------ END current sensor calibration ---------------------------------------------------------------------

//---------------- savings consts for displays 
const int PROGMEM gascost=350; // in cents per gallon
const int PROGMEM mpg=25; // mpg of the gasoline car
const int PROGMEM ecost=12; // in cents per kWhr
const int PROGMEM whpermile=300; // energy efficiency of the ecar
int savingsPerKWH; // will be recalced later
//---------------- end of the energy constants

//---------------- pin-out constants ----------------
//---------------- analog inputs
const byte PROGMEM pin_pV=0; // pilot signal sensed through a 3-element divider 
const byte PROGMEM pin_V=1; // input voltage 
const byte PROGMEM pin_C=2; // AC current - as measured by the current transformer
const byte PROGMEM pin_throttle=3; // wired to a trimpot on a board
// pins A4 / A5 reserved for SPI comms to RTC chip

//---------------- digital inputs / outputs
// GFI trip pin - goes high on GFI fault, driven by the specialized circuit based on LM1851 
// has to be pin 3 as only pin 2 and 3 are available for interrupts on Pro Mini
const byte PROGMEM pin_GFI=3; 
const byte PROGMEM pin_inRelay=5; 
const byte PROGMEM pin_PWM=9; // J pilot PWM pin

const byte PROGMEM pin_StatusLight=10;

const byte PROGMEM pin_GFItest=12; // pin wired to a GFCI-tripping relay - for the periodic testing of the GFCI circuit & stuck relay detection
//---------------- END PINOUTS -----------------------

//---------------- Status Light Levels ---------------
const byte PROGMEM status_Off=0;
const byte PROGMEM status_Low=64;
const byte PROGMEM status_Mid=128;
const byte PROGMEM status_Full=255;
//---------------- End Status Light Levels -----------------------

//==================================== calibration constants etc
const float PROGMEM Aref=5.; // should be close
float pV_min=-12.;
float V_J1772_pin_=0; // global pilot voltage
const float PROGMEM divider_pV_R=100./27.; // 100k over 27k
float V_Ard_pin_0;
//===============================================================

//========== define J1772 states ===============================
// defaults
const float PROGMEM def_state_A_Vmin=10.5, def_state_A_Vmax=14; 
const float PROGMEM def_state_B_Vmin=7.5, def_state_B_Vmax=10.5; 
const float PROGMEM def_state_C_Vmin=4.5, def_state_C_Vmax=7.5; 
const float PROGMEM def_state_D_Vmin=1.5, def_state_D_Vmax=4.5; 
const float PROGMEM def_state_E_Vmin=-1.5, def_state_E_Vmax=1.5; 
const float PROGMEM def_state_F_Vmin=-14., def_state_F_Vmax=-10.; 
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
const unsigned int PROGMEM PWM_res=1024;
const unsigned int PROGMEM PWM_FULLON=1024;
const unsigned int PROGMEM MAXDUTY=970; // <97% to stay in AC charging zone for J1772 standard

int sawTemp = 0;
float lastTemp = 0;

const float PROGMEM maxC=60; // max rated current
float inV_AC=0; // this will be measured
const float PROGMEM nominal_outC_240V=30; // 30A by default from a 240VAC line
const float PROGMEM nominal_outC_120V=15; // 15A by default from a 120VAC line
float outC=nominal_outC_240V; 
float power=0;
float energy=0; // how much energy went through - in kWHrs 
#define TEMP_STRING_LEN 12
char tempstr[TEMP_STRING_LEN]; // scratchpad for text operations

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
//#define tracer(x) Serial.println(x);delay(200);
#define tracer(x)

void setup() {
  wdt_disable();
  
  Serial.begin(115200);

  tracer("Starting Initialization");

  // set digital input pins
  pinMode(pin_GFI, INPUT);

  // set digital output pins
  pinMode(pin_PWM, OUTPUT);
  pinMode(pin_inRelay, OUTPUT);
  pinMode(pin_StatusLight, OUTPUT);
  pinMode(pin_GFItest, OUTPUT);

  // Indicate we are booting on status light
  analogWrite(pin_StatusLight, status_Low);
  
  tracer("After Set Pins");

  //---------------------------------- set up timers
  cli();//stop interrupts

  // use Timer1 library to set PWM frequency 
  // 10-bit PWM resolution
  Timer1.initialize(1000); // 1kHz for J1772
  Timer1.pwm(pin_PWM, 0); 
  
  sei();
  tracer("After SEI");
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
  GFI_tripped=0;
  // force the GFI pin
  digitalWrite(pin_GFItest, HIGH);
#ifdef GFI
  delay(100); 
  // by now, if the trip occurred, the GFI trip flag should be set
  if(GFI_tripped==1) {
    // we have a stuck relay, throw an error
    tracer("STUCK RELAY! \nContact us\nExiting...");
    return; // break from loop() which will be called back a moment later
  }
#endif

  // turn on the main relay
  setRelay(HIGH);
  // wait for settling (RC on the pin is 0.1s so need to wait at least for 0.3s
  // but not too long or we will burn out the 10k resistor...
  delay(300);

  inV_AC=read_V();    
  digitalWrite(pin_GFItest, LOW);
  setRelay(LOW);  
  
  
  // attach interrupt on pin 3 (GFI)
#ifdef GFI
  attachInterrupt(1, GFI_break, RISING);
#endif
  
  // set watchdog - http://tushev.org/articles/arduino/item/46-arduino-and-watchdog-timer, http://www.nongnu.org/avr-libc/user-manual/group__avr__watchdog.html
//  wdt_enable(WDTO_8S); // longest is 8S
  
  tracer("Before Display");

#ifdef ADAFRUIT
  // Initialize Display
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3c);  // initialize with the I2C addr 0x3D (for the 128x64)

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  // display init done
  display.println("Init!");
  display.display();
#endif
  
  tracer("Before RTC");
  // Initialize RTC
  //setSyncProvider() causes the Time library to synchronize with the
  //external RTC by calling RTC.get() every five minutes by default.
#ifdef DS3232RTC_DEFINE
  setSyncProvider(RTC.get);
  ps("RTC Sync");
  if (timeStatus() != timeSet) {
    psln(" FAIL!");
  } else {
    psln(" Success!!");
  }
  // rtc init done
  tracer("After RTC");
#endif
  
  // initialize in state A - EVSE ready
  setPilot(PWM_FULLON);
  
  tracer("Finished Initialization");

}

//============================================= MAIN LOOP ============================================
void loop() {
  
#ifdef DS3232RTC_DEFINE
  static time_t tLast;
  time_t t;
  
  t = now();
  if (t != tLast) {
      tLast = t;
      printDateTime(t);
      displayDateTime(t);
      if (second(t) == 0 || sawTemp == 0) {
          float c = RTC.temperature() / 4.;
          float f = c * 9. / 5. + 32.;
          lastTemp = f;
          sawTemp = 1;
      }
      display.setTextSize(2);
      displayState();
      if (sawTemp) {
        display.print(lastTemp);
        display.println(" F");
      }  
      display.display();
  }

#endif  
  // reset GFI trip status so we can retry after GFI timeout
  // GFI is checked in the end of this cycle - by that time, a few hundreds ms pass
  GFI_tripped=0; 
      
  // check if the car is there and requesting power
  prev_state=state;
  state=getState(); // this is a blocking call for <100ms
  
  // manage state changes
  if(state!=prev_state) {
    ps("Change state: ");
    psln(state);
    timer=millis(); // start timer
    timer0=timer; // remember the start of charge
    
    if(state==STATE_C) {
      psln("Starting Charge State");
      // entering charging state - check for diode
      setPilot(PWM_FULLON/2);
      if(read_pV()>-1.5) {
        state=STATE_F; // diode check failure!
      }
      energy=0; // reset energy counter for this cycle  
    }
  } // end state transition check
  
  if(state!=STATE_C) {
    analogWrite(pin_StatusLight, status_Mid);
  } else {
    analogWrite(pin_StatusLight, status_Full);
  }

  //-------------------------------- process states
  if(state==STATE_A) {
    setRelay(LOW); // relay off
    setPilot(PWM_FULLON);
  }
    
  if(state==STATE_B) {
    setRelay(LOW); // relay off
#ifdef DS3232RTC_DEFINE
    int currentHour = hour(t);
    int currentMin = minute(t);
#else
    int currentHour = 1;
    int currentMin = 1;
#endif
    if (currentHour >= 17 && currentHour < 21) {
      setPilot(PWM_FULLON);
      pitln("WAIT");
      psln("Waiting till 5 pm to start charging");    
    } else {
      pitln("CHARGE");
      psln("Sending back charge okay");
      setOutC();
      setPilot(duty);
    }
  }
    
  if(state==STATE_C) {
    setOutC();
    setPilot(duty);
    setRelay(HIGH); // relay on

    // process energy metering
    float outC_meas=read_C();
    power=outC_meas*inV_AC/1000; // in KW

    delta=int(millis()-timer);
    timer=millis();
    energy+=power*delta/1000/3600; 

    snprintf(tempstr, TEMP_STRING_LEN, "%d min", int((timer-timer0)/1000)/60);
    pitln(tempstr);
#ifdef SERIAL_PRINTS
    // print real-time stats
    snprintf(tempstr, TEMP_STRING_LEN, "Power: %d.%01d KW  ", int(power), int(power*10)%10);
    psln(tempstr);
    snprintf(tempstr, TEMP_STRING_LEN, "Time: %d min  ", int((timer-timer0)/1000)/60); 
    psln(tempstr);
    // also show energy cost in this one
    // use US average cost per http://www.eia.gov/electricity/monthly/epm_table_grapher.cfm?t=epmt_5_6_a - $0.12/kwhr
    snprintf(tempstr, TEMP_STRING_LEN, "%d.%01d KWH ($%d.%02d) ", int(energy), int(energy*10)%10, int(energy/8), int(energy/8*100)%100 ); 
    psln(tempstr);
    snprintf(tempstr, TEMP_STRING_LEN, "%dV, %dA (%d) ", int(inV_AC), int(outC_meas), int(outC)); 
    psln(tempstr);
#endif
        
  } // end state_C
  
  if(state==STATE_D) {
    // printClrMsg(F("Vehicle requested\nVENTILATED power!\nExiting..."), 1000, 0x1f, 0x3f, 0);
    setRelay(LOW); // relay off
  }
  
  if(state==STATE_E || state==STATE_F || state==STATE_INVALID) {
    psln("Abnormal State!");
    setRelay(LOW); // relay off
  }  
          
  // display standby
  if(state==STATE_A || state==STATE_B) {  
      // set the output current - can be changed by trimpot or remote without a restart
      // need this here so we have an echo on user input
      setOutC(); 
      snprintf(tempstr, TEMP_STRING_LEN, "%dV, %dA", int(inV_AC), int(outC));
      psln(tempstr);  
    }

  delay(meas_cycle_delay); // reasonable delay for screen refresh

#ifdef GFI
  // check GFI flag (if a trip is detected, this flag would be set via the special interrupt)
  if(GFI_tripped) {
    psln("GFI tripped!\nRetrying in 15 min...");
    GFI_trip_count++; // allowed max of 4; if more than 4, need 2 user inputs to override
    if(GFI_trip_count>4) {
      // wait for user to unplug; since the user then will have to re-plug to re-energize the system, this can be considered 2 actions
      psln("4th GFI trip!\nUnplug / re-plug\nto resume");
      while(getState()!=STATE_A);
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

  Serial.print("-v: ");
  Serial.println(pV);
  display.print("-v: ");
  display.println(pV);
  
  if(pV>state_A_Vmin && pV<=state_A_Vmax) return STATE_A;
  if(pV>state_B_Vmin && pV<=state_B_Vmax) return STATE_B;
  if(pV>state_C_Vmin && pV<=state_C_Vmax) return STATE_C;
  if(pV>state_D_Vmin && pV<=state_D_Vmax) return STATE_D;
  if(pV>state_E_Vmin && pV<=state_E_Vmax) return STATE_E;
  if(pV>state_F_Vmin && pV<=state_F_Vmax) return STATE_F;

  ps("no valid state: ");
  psln(pV);

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
  

  return V_AC; 
}

// read the AC current via the current transformer
// in the absense of the current transformer this will return zero
// RC constant defined by R11 C5 = 27k * 3.3uF = 90ms, or >5 line periods
float read_C() {
//  const int Te=1000; // # of turns
  // read the rectified voltage of the half-wave from the transformer
  // average between 2 readings 180 degree off each other
  int reading=analogRead(pin_C);
  delay(8);
  reading+=analogRead(pin_C);
  // this assumes an RC filter before Arduino pon with time constant >> line period and impedance >> R
  float V_C=reading*Aref/2/1024; 

  // use a table lookup
  int index=floor(V_C*10);
  float remainder=V_C*10-index;
  if(index>31) return 75.; // prevent from array overflow
  float coeff=float(AC1075_calibration[index+1]-AC1075_calibration[index]); // 0.1V step
  return float(AC1075_calibration[index]+remainder*coeff)/10.; // linear extrapolation
}

//print date and time to Serial
void printDateTime(unsigned long t)
{

    printDate(t);
    ps(" ");
    printTime(t);
}

//print time to Serial
void printTime(unsigned long t)
{
#ifdef DS3232RTC_DEFINE
    printI00(hour(t), ':');
    printI00(minute(t), ':');
    printI00(second(t), ' ');
#endif
}

//print date to Serial
void printDate(unsigned long t)
{
#ifdef DS3232RTC_DEFINE
    printI00(day(t), 0);
    ps(monthShortStr(month(t)));
    ps(year(t));
#endif
}

//Print an integer in "00" format (with  eading zero),
//followed by a delimiter character to Serial.
//Input value assumed to be between 0 and 99.
void printI00(int val, char delim)
{
    if (val < 10) {
      ps("0");
    }
    ps(val);
    if (delim > 0) {
      ps(delim);
    }
    return;
}

void displayState()
{
  pit("State ");
  if (state == STATE_A) {
    pitln("A");
  } else if (state == STATE_B) {
    pitln("B");
  } else if (state == STATE_C) {
    pitln("C");
  } else if (state == STATE_D) {
    pitln("D");
  } else {
    pitln("??");
  }
}

void displayDateTime(unsigned long t)
{
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(1);
  displayI00(hour(t), ':');
  displayI00(minute(t), ':');
  displayI00(second(t), ' ');
  displayI00(day(t), ' ');
  display.print(monthShortStr(month(t)));
  display.print(" ");
  display.println(year(t));
  display.println("--------------------");
}

void displayI00(int val, char delim)
{
    if (val < 10) display.print("0");
    display.print(val,DEC);
    if (delim > 0) display.print(delim);
    return;

}
