//V1.1
//March 30 2023
//Implemented changes suggested by ryokell
//Out of order divs
//Non-1 first div

#include <FlashAsEEPROM_SAMD.h>
#include <avr/pgmspace.h>
#include <Arduino.h>

#include "defines.h"
#include "hertzvals.h"
#include "exponential.h"
#include "filter.h"
#include "state.h"
#include "timers.h"


State ms; // for machine state, should be the only global variable


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
  
  constexpr uint64_t default_upslope = UPSLOPE(DEFAULT_SHAPE);
  constexpr uint64_t default_downslope = DOWNSLOPE(DEFAULT_SHAPE);
  Module default_module = {0, 0, VCO, DEFAULT_SHAPE, DEFAULT_LIN, default_upslope, default_downslope};
  for (int i = 0; i < NUM_MODULES; i++) {
    ms.mods[i] = default_module;
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



// core waveshape generator algorithm.
// uses lookup table to get exp and log values, calculates lin values
// maps between them based on the linearity control
unsigned int generator(uint8_t idx) {
  uint32_t shifted_acc = ms.mods[idx].acc>>23;

  uint32_t linval = 0;
  uint32_t expval = 0;
  uint32_t logval = 0;

  if (shifted_acc < ms.mods[idx].shape) {
    linval = ms.mods[idx].upslope * shifted_acc;
    expval = pgm_read_word_near(exptable + (linval >> 7));
    logval = (M << 7) - pgm_read_word_near(exptable + (ms.mods[idx].upslope * (ms.mods[idx].shape - shifted_acc) >> 7));
  } else {
    linval = ms.mods[idx].downslope * (M - shifted_acc);
    expval = pgm_read_word_near(exptable + (linval >> 7));
    logval = (M << 7) - pgm_read_word_near(exptable + (ms.mods[idx].downslope * (shifted_acc - ms.mods[idx].shape) >> 7));
  }
  return M - (asym_lin_map(ms.mods[idx].lin, expval, linval, logval) >> 7);
}

int asym_lin_map(uint16_t x, int low, int mid, int high) {
  if (x <= 0) return low;
  if (x < H) return (x * (mid - low) >> 8) + low;
  if (x == H) return mid;
  if (x > H) return ((x - H) * (high - mid) >> 8) + mid;
  if (x >= M) return high;
}

void TCC0_Handler() 
{
  if (TCC0->INTFLAG.bit.CNT == 1) {
    ms.mods[0].acc += ms.mods[0].phasor;
    ms.mods[1].acc += ms.mods[1].phasor;
    ms.mods[2].acc += ms.mods[2].phasor;
    ms.mods[3].acc += ms.mods[3].phasor;
    delayMicroseconds(6);
    REG_TCC0_CC0 = generator(0); // pin 9 //#4
    REG_TCC0_CC1 = generator(1); // pin 2 //#1
    REG_TCC0_CC2 = generator(2); // pin 1 //#2  
    REG_TCC0_CC3 = generator(3); // pin 3 //#3
    TCC0->INTFLAG.bit.CNT = 1;
  }
}