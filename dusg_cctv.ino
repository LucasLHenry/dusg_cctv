//V1.1
//March 30 2023
//Implemented changes suggested by ryokell
//Out of order divs
//Non-1 first div

#include <FlashAsEEPROM_SAMD.h>
#include <avr/pgmspace.h>
#include <Arduino.h>
#include <string.h>

#include "hertzvals.h"
#include "exponential.h"
#include "filter.h"
#include "state.h"


#define HZPHASOR 91183 //phasor value for 1 hz.
#define M 511
#define H 255
#define DEFAULT_SHAPE 255
#define DEFAULT_LIN 255
#define UPSLOPE(x) ((M << 7) / x)
#define DOWNSLOPE(x) ((M << 7) / (M - x))

// void timerIsr();
void setupTimers();
void TCC0_Handler();

State ms; // for machine state


// +++++++++++++++++++++++++++++++++++ SETUP ++++++++++++++++++++++++++++++++++++++++
void setup() {
  pinMode(1, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(13, OUTPUT); // using pin 13 to check interupt on timer 1
  pinMode(6, INPUT_PULLUP); // pin 7 pushbutton to select waveform
  pinMode(A10, INPUT); //used for analogReading sync (cv2) 

  setupTimers(); //  **this may not be the right location
  randomSeed(analogRead(A8));
  
  Serial.begin(9600);
  

  // state initialization
  constexpr uint64_t default_upslope = UPSLOPE(DEFAULT_SHAPE);
  constexpr uint64_t default_downslope = DOWNSLOPE(DEFAULT_SHAPE);
  Module default_module = {0, 0, VCO, DEFAULT_SHAPE, DEFAULT_LIN, default_upslope, default_downslope};
  for (int i = 0; i < 4; i++) {
    ms.mods[0] = default_module;
  }
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++ MAIN LOOP +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void loop() {
  
   
   
  float tempphasor;
  int cv1Value; // to store value of cv1 (Frequency)
  static int potValue; // to store the value of the potentiometer
  
  filterPut(POT,analogRead(A0));
  potValue = filterGet(POT);
  filterPut(FREQ,analogRead(A4));
  cv1Value = filterGet(FREQ); // at this stage, a -12V CV corresponds to +3.3V (1023 as an analog read) on the XIAO (Because of the inverting op-amp)
  
  cv1Value = 1023-cv1Value; // so we want to invert it (making -12V correspond to 0V on the XIAO)
  // cv1Value = cv1Value - 565;  //at this point cv1Value contains between -512 and +511 (this line used to center values around zero)
  uint16_t new_linearity = cv1Value >> 1;  // now it's between 0 and 511
  uint16_t new_shape = potValue >> 1;

  if (ms.mods[0].lin != new_linearity) {
    ms.mods[0].lin = new_linearity;
    ms.mods[1].lin = new_linearity;
    ms.mods[2].lin = new_linearity;
    ms.mods[3].lin = new_linearity;
  }

  if (ms.mods[0].shape != new_shape) {
    uint32_t new_upslope = UPSLOPE(new_shape);
    uint32_t new_downslope = DOWNSLOPE(new_shape);
    for (int i = 0; i < 4; i++) {
      ms.mods[i].upslope = new_upslope;
      ms.mods[i].downslope = new_downslope;
      ms.mods[i].shape = new_shape;
    }
  }


  tempphasor=50*HZPHASOR;


  ms.mods[0].phasor=(unsigned long int)tempphasor;
  ms.mods[1].phasor=(unsigned long int)tempphasor; // dividing down for the slower outputs 
  ms.mods[2].phasor=(unsigned long int)tempphasor;
  ms.mods[3].phasor=(unsigned long int)tempphasor;

}

// +++++++++++++++++++++++++++++++++++++++++ FUNCTION DEFINITIONS ++++++++++++++++++++++++++++++++++

unsigned long int previous_acc[4];


// this is written by LUCAS to test the design
unsigned int generator(Module mod) {
  unsigned int shifted_acc = mod.acc>>23;

  uint32_t linval = 0;
  uint32_t expval = 0;
  uint32_t logval = 0;
  if (shifted_acc < mod.shape) {
    linval = mod.upslope * shifted_acc;
    expval = pgm_read_word_near(exptable + (linval >> 7));
    logval = (M << 7) - pgm_read_word_near(exptable + (mod.upslope * (mod.shape - shifted_acc) >> 7));
  } else {
    linval = mod.downslope * (M - shifted_acc);
    expval = pgm_read_word_near(exptable + (linval >> 7));
    logval = (M << 7) - pgm_read_word_near(exptable + (mod.downslope * (shifted_acc - mod.shape) >> 7));
  }
  return M - (asym_lin_map(mod.lin, expval, linval, logval) >> 7);
}

int asym_lin_map(uint16_t x, int low, int mid, int high) {
  if (x <= 0) {
    return low;
  }
  if (x < H) {
    return (x * (mid - low) >> 8) + low;
  }
  if (x == H) {
    return mid;
  }
  if (x > H) {
    return ((x - H) * (high - mid) >> 8) + mid;
  }
  if (x >= M) {
    return high;
  }
}

void setupTimers() // used to set up fast PWM on pins 1,9,2,3
{
 
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(2) |          // Divide the 48MHz clock source by divisor N=1: 48MHz/1=48MHz
                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
  
  //enable our 4 pins to be PWM outputs
  PORT->Group[g_APinDescription[1].ulPort].PINCFG[g_APinDescription[1].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[9].ulPort].PINCFG[g_APinDescription[9].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[2].ulPort].PINCFG[g_APinDescription[2].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[3].ulPort].PINCFG[g_APinDescription[3].ulPin].bit.PMUXEN = 1;

  //assign the 4 outputs to the PWM registers on PMUX
  PORT->Group[g_APinDescription[1].ulPort].PMUX[g_APinDescription[1].ulPin >> 1].reg = PORT_PMUX_PMUXE_E;  // D3 is on PA11 = odd      
  PORT->Group[g_APinDescription[9].ulPort].PMUX[g_APinDescription[9].ulPin >> 1].reg |= PORT_PMUX_PMUXO_E; // D11 is on PA08 = even 
  PORT->Group[g_APinDescription[2].ulPort].PMUX[g_APinDescription[2].ulPin >> 1].reg |= PORT_PMUX_PMUXE_F;  // D3 is on PA11 = odd
  PORT->Group[g_APinDescription[3].ulPort].PMUX[g_APinDescription[3].ulPin >> 1].reg |= PORT_PMUX_PMUXO_F; // D11 is on PA08 = even

 

  // Feed GCLK4 to TCC0 and TCC1
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK0 |     // Select GCLK4 //0 only works for interrup?
                     GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization


  // Dual slope PWM operation: timers countinuously count up to PER register value then down 0
  REG_TCC0_WAVE |= TCC_WAVE_POL(0xF) |           // Reverse the output polarity on all TCC0 outputs
                    TCC_WAVE_WAVEGEN_DSBOTH |
                    TCC_WAVE_WAVEGEN_NFRQ;     // Setup dual slope PWM on TCC0
  while (TCC0->SYNCBUSY.bit.WAVE);               // Wait for synchronization

  // Each timer counts up to a maximum or TOP value set by the PER register,
  // this determines the frequency of the PWM operation: Freq = 48Mhz/(2*N*PER)
  REG_TCC0_PER = 0x1FF;                           // Set the FreqTcc and period of the PWM on TCC1
  while (TCC0->SYNCBUSY.bit.PER);                // Wait for synchronization
 
  REG_TCC0_CC1 = 10;                             // TCC1 CC1 - on D3  50% pin 9
  while (TCC0->SYNCBUSY.bit.CC1);                   // Wait for synchronization
  REG_TCC0_CC0 = 50;                             // TCC1 CC0 - on D11 50% pin 1
  while (TCC0->SYNCBUSY.bit.CC0);                   // Wait for synchronization
    REG_TCC0_CC2 = 200;                             // TCC1 CC1 - on D3  50% pin 2
  while (TCC0->SYNCBUSY.bit.CC2);                   // Wait for synchronization
  REG_TCC0_CC3 = 254;                             // TCC1 CC0 - on D11 50% pin 3
  while (TCC0->SYNCBUSY.bit.CC3);                   // Wait for synchronization
 
  // Divide the GCLOCK signal by 1 giving  in this case 48MHz (20.83ns) TCC1 timer tick and enable the outputs
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |    // Divide GCLK4 by 1 ****************************************************************************
                    TCC_CTRLA_ENABLE;             // Enable the TCC0 output
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
  
  TCC0->INTENSET.reg = 0;
  TCC0->INTENSET.bit.CNT = 1;  //*****************************************************
  TCC0->INTENSET.bit.MC0 = 0;

  NVIC_EnableIRQ(TCC0_IRQn);
  TCC0->CTRLA.reg |= TCC_CTRLA_ENABLE ;
  
}

void TCC0_Handler() 
{
  if (TCC0->INTFLAG.bit.CNT == 1) {
    ms.mods[0].acc += ms.mods[0].phasor;
    ms.mods[1].acc += ms.mods[1].phasor;
    ms.mods[2].acc += ms.mods[2].phasor;
    ms.mods[3].acc += ms.mods[3].phasor;
    delayMicroseconds(6);
    REG_TCC0_CC0 = generator(ms.mods[0]); // pin 9 //#4
    REG_TCC0_CC1 = generator(ms.mods[1]); // pin 2 //#1
    REG_TCC0_CC2 = generator(ms.mods[2]); // pin 1 //#2  
    REG_TCC0_CC3 = generator(ms.mods[3]); // pin 3 //#3
    TCC0->INTFLAG.bit.CNT = 1;
  }
}