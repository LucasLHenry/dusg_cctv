// written by Lucas Henry, based on CCTV coven lfo
// January 12 2024


#include <FlashAsEEPROM_SAMD.h>
#include <avr/pgmspace.h>
#include "antilog.h"
#include <Arduino.h>

#define HZPHASOR 91183 //phasor value for 1 hz.

long unsigned int accumulator1 = 0;
long unsigned int accumulator2 = 0;

long unsigned int phasor1;
long unsigned int phasor2;

FlashStorage(div_storage, int);
FlashStorage(wave_storage, int);
FlashStorage(init_storage, char);

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
