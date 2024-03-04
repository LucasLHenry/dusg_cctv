#include <FlashAsEEPROM_SAMD.h>
#include <avr/pgmspace.h>
#include <Arduino.h>
#include <FastLED.h>
#include <ResponsiveAnalogRead.h>

#include "pins.h"
#include "hertzvals.h"
#include "filter.h"
#include "state.h"
#include "timers.h"
#include "exponential.h"


#define HZPHASOR 91183 //phasor value for 1 hz.
#define DEFAULT_SHAPE 255
#define DEFAULT_LIN 255

#define M 511
#define H 255
#define UPSLOPE(x) ((M << 7) / x)
#define DOWNSLOPE(x) ((M << 7) / (M - x))

void TCC0_Handler();

State ms; // for machine state

void setup() {
  pinMode(1, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(13, OUTPUT); // using pin 13 to check interupt on timer 1
  pinMode(A10, INPUT); //used for analogReading sync (cv2) 

  setupTimers(); //  **this may not be the right location

  // state initialization
  constexpr uint64_t default_upslope = UPSLOPE(DEFAULT_SHAPE);
  constexpr uint64_t default_downslope = DOWNSLOPE(DEFAULT_SHAPE);
  Module default_module = {0, 0, VCO, DEFAULT_SHAPE, DEFAULT_LIN, default_upslope, default_downslope};
  for (int i = 0; i < 4; i++) {
    ms.mods[0] = default_module;
  }
  FastLED.addLeds<NEOPIXEL, LEDs>(ms.leds, NUM_LEDs);  // set up LEDs
}

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
  uint16_t new_shape = cv1Value >> 1;  // now it's between 0 and 511
  uint16_t new_slope = potValue >> 1;

  if (ms.mods[0].shape != new_shape) {
    ms.mods[0].shape = new_shape;
    ms.mods[1].shape = new_shape;
    ms.mods[2].shape = new_shape;
    ms.mods[3].shape = new_shape;
  }

  if (ms.mods[0].slope != new_slope) {
    uint32_t new_upslope = UPSLOPE(new_slope);
    uint32_t new_downslope = DOWNSLOPE(new_slope);
    for (int i = 0; i < 4; i++) {
      ms.mods[i].upslope = new_upslope;
      ms.mods[i].downslope = new_downslope;
      ms.mods[i].slope = new_slope;
    }
  }

  tempphasor=50*HZPHASOR;
  ms.mods[0].phasor=(unsigned long int)tempphasor;
  ms.mods[1].phasor=(unsigned long int)tempphasor; // dividing down for the slower outputs 
  ms.mods[2].phasor=(unsigned long int)tempphasor;
  ms.mods[3].phasor=(unsigned long int)tempphasor;
}


unsigned int generator(uint8_t idx) {
  unsigned int shifted_acc = ms.mods[idx].acc>>23;

  uint32_t linval = 0;
  uint32_t expval = 0;
  uint32_t logval = 0;
  if (shifted_acc < ms.mods[idx].slope) {
    linval = ms.mods[idx].upslope * shifted_acc;
    expval = pgm_read_word_near(exptable + (linval >> 7));
    logval = (M << 7) - pgm_read_word_near(exptable + (ms.mods[idx].upslope * (ms.mods[idx].slope - shifted_acc) >> 7));
  } else {
    linval = ms.mods[idx].downslope * (M - shifted_acc);
    expval = pgm_read_word_near(exptable + (linval >> 7));
    logval = (M << 7) - pgm_read_word_near(exptable + (ms.mods[idx].downslope * (shifted_acc - ms.mods[idx].slope) >> 7));
  }
  return M - (asym_lin_map(ms.mods[idx].shape, expval, linval, logval) >> 7);
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
    delayMicroseconds(4);  // modify to deal with weird spikes
    REG_TCC0_CC0 = generator(0); // pin 9 //#4
    REG_TCC0_CC1 = generator(1); // pin 2 //#1
    REG_TCC0_CC2 = generator(2); // pin 1 //#2  
    REG_TCC0_CC3 = generator(3); // pin 3 //#3
    TCC0->INTFLAG.bit.CNT = 1;
  }
}