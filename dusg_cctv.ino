//V1.1
//March 30 2023
//Implemented changes suggested by ryokell
//Out of order divs
//Non-1 first div

#include <FlashAsEEPROM_SAMD.h>
#include <avr/pgmspace.h>
#include <Arduino.h>

#include "hertzvals.h"
#include "exponential.h"
#include "save_load.h"
#include "filter.h"
#include "state.h"


#define HZPHASOR 91183 //phasor value for 1 hz.


long unsigned int accumulator1 = 0;
long unsigned int accumulator2 = 0;
long unsigned int accumulator3 = 0;
long unsigned int accumulator4 = 0;
long unsigned int phasor1;
long unsigned int phasor2;
long unsigned int phasor3;
long unsigned int phasor4;

uint16_t shape = 255;
uint16_t linearity = 255;  // 0 to 511

char randNum[4];

////////////////////////////////////////////////////////////////////////////////////
//       DIVIDE DOWN ARRAYS                                                       //
//       Add more if you want!                                                    //
////////////////////////////////////////////////////////////////////////////////////

#define DIVSIZE 3 // if you add more divide down arrays, increase this number from 3 (the default number of arrays) to how many arrays you have total  
char divs[DIVSIZE][4]={{1,3,7,11},
                      {1,2,4,8},
                      {1,4,8,16}}; // You could add more divide down arrays here 

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

char debounceState = 0;
unsigned long int debounceTime = 0; 
int waveSelect = 1;
int divSelect = 1;
unsigned long lastSettingsSave = 0;

bool Mode = 0; // 0 = POT and 1 = SYNC 
float sweepValue;
long unsigned int Time1 = 0;
long unsigned int Time2 = 0; 
long unsigned int Periud = 0; // Period (arduino didn't allow use of word "period")
float syncFrequency = 0; 
void timerIsr();
void setupTimers();
void TCC0_Handler();



// +++++++++++++++++++++++++++++++++++ SETUP ++++++++++++++++++++++++++++++++++++++++
void setup() {
  pinMode(1, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(13, OUTPUT); // using pin 13 to check interupt on timer 1
  pinMode(6, INPUT_PULLUP); // pin 7 pushbutton to select waveform
  pinMode(A10, INPUT); //used for analogReading sync (cv2) 
  readSettings();
  //Serial.begin(9600);
  setupTimers(); //  **this may not be the right location
  randomSeed(analogRead(A8));
  
  Serial.begin(9600);
   
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++ MAIN LOOP +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void loop() {
  
  static int modeCounter = 0;
  bool lastButtonState = 0;
  static bool buttonState = 0;
  static int syncState = 0; // used to find find the leading edge to calculate period (static so it isn't updated to zero each loop)
  static int syncCounter = 0;
  int Sync; 

////////////////////////////////////////////////////////////////////////////////////
//       "EEPROM" Save                                                            //
//       Store the settings after 60 seconds                                      //
////////////////////////////////////////////////////////////////////////////////////
  if((millis() - (lastSettingsSave)) > (60000))
  {
    saveSettings();
    lastSettingsSave = millis();
  }
   
  float tempphasor;
  int cv1Value; // to store value of cv1 (Frequency)
  static int potValue; // to store the value of the potentiometer
  
  static int oldpotValue; 
  filterPut(POT,analogRead(A0));
  potValue = filterGet(POT);

  //This section pops us out of sync mode
  modeCounter++;
  if(modeCounter>100) //do this only every 100 samples
  {
    if((potValue - oldpotValue)> 20 || (oldpotValue - potValue) > 20){
    Mode = 0; 
    }

    oldpotValue = potValue;

    modeCounter = 0;
  }

  filterPut(FREQ,analogRead(A4));
  cv1Value = filterGet(FREQ); // at this stage, a -12V CV corresponds to +3.3V (1023 as an analog read) on the XIAO (Because of the inverting op-amp)
  
  cv1Value = 1023-cv1Value; // so we want to invert it (making -12V correspond to 0V on the XIAO)
  // cv1Value = cv1Value - 565;  //at this point cv1Value contains between -512 and +511 (this line used to center values around zero)
  linearity = cv1Value >> 1;  // now it's between 0 and 511
  shape = potValue >> 1;
  // Serial.print((int)linearity);
  // Serial.print(", ");
  // Serial.println((int)shape);


  tempphasor=50*HZPHASOR;

  phasor1=(unsigned long int)tempphasor;
  phasor2=(unsigned long int)tempphasor; // dividing down for the slower outputs 
  phasor3=(unsigned long int)tempphasor;
  phasor4=(unsigned long int)tempphasor;
}

// +++++++++++++++++++++++++++++++++++++++++ FUNCTION DEFINITIONS ++++++++++++++++++++++++++++++++++

unsigned long int previous_acc[4];


// this is written by LUCAS to test the design
unsigned int generator(unsigned long int acc, uint16_t shift, uint16_t lin, char channel) {
  #define M 511  // maxpoint
  #define H 255  // halfpoint
  unsigned int shifted_acc = acc>>23;

  
  
  // float logval = 0;
  // float expval = 0;
  // float linval = 0;

  // if (shifted_acc < shift) {
  //   float scaleval = M * 1.0 / shift;
  //   linval = scaleval * shifted_acc; //M * shifted_acc/shift
  //   //linval is from 0 to the waveshift point
  //   expval = fastExp(linval);
  //   logval = M - fastExp(scaleval * (shift - shifted_acc));
  // } else {
  //   float scaleval = M * 1.0 / (M - shift);
  //   linval = scaleval * (M - shifted_acc);
  //   expval = fastExp(linval);
  //   logval = M - fastExp(scaleval * (shifted_acc - shift));
  // }
  // return M - (unsigned int)asym_lin_map(lin / 127.0 - 1, expval, linval, logval);
  // uint32_t linval = 0;
  // uint32_t expval = 0;
  // uint32_t logval = 0;

  // if (shifted_acc < shift) {
  //   uint32_t scaleval = M << 7 / shift;
  //   linval = scaleval * shifted_acc; //M * shifted_acc/shift
  //   //linval is from 0 to the waveshift point
  //   expval = pgm_read_word_near(exptable + (linval >> 7));
  //   logval = M << 7 - pgm_read_word_near(exptable + (scaleval * (shift - shifted_acc) >> 7));
  // } else {
  //   uint32_t scaleval = M << 7 / (M - shift);
  //   linval = scaleval * (M - shifted_acc);
  //   expval = pgm_read_word_near(exptable + (linval >> 7));
  //   logval = M << 7 - pgm_read_word_near(exptable + (scaleval * (shifted_acc - shift) >> 7));
  // }
  // return M - (unsigned int)asym_lin_map(lin / 127.0 - 1, expval, linval, logval) >> 7;

  uint32_t linval = 0;
  uint32_t expval = 0;
  uint32_t logval = 0;
  if (shifted_acc < shift) {
    uint32_t scaleval = (M << 7) / shift;
    linval = scaleval * shifted_acc;
    expval = pgm_read_word_near(exptable + (linval >> 7));
    logval = (M << 7) - pgm_read_word_near(exptable + (scaleval * (shift - shifted_acc) >> 7));
  } else {
    uint32_t scaleval = (M << 7) / (M - shift);
    linval = scaleval * (M - shifted_acc);
    expval = pgm_read_word_near(exptable + (linval >> 7));
    logval = (M << 7) - pgm_read_word_near(exptable + (scaleval * (shifted_acc - shift) >> 7));
  }
  return M - (asym_lin_map(lin, expval, linval, logval) >> 7);
}

int asym_lin_map(uint16_t x, int low, int mid, int high) {
  if (x <= 0) {
    return low;
  }
  if (x < 255) {
    return (x * (mid - low) >> 8) + low;
  }
  if (x == 255) {
    return mid;
  }
  if (x > 255) {
    return ((x - 255) * (high - mid) >> 8) + mid;
  }
  if (x >= 511) {
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
  if (TCC0->INTFLAG.bit.CNT == 1) { //*************************************************
   accumulator1 = accumulator1 + phasor1;
   accumulator2 = accumulator2 + phasor2;
   accumulator3 = accumulator3 + phasor3;
   accumulator4 = accumulator4 + phasor4;
   delayMicroseconds(6);
   REG_TCC0_CC0 = generator(accumulator1, shape, linearity, 3); // pin 9 //#4
   // REG_TCC0_CC1 = generator(accumulator4, shape, linearity, 0); // pin 2 //#1
  //  REG_TCC0_CC2 = generator(accumulator2, shape, linearity, 1); // pin 1 //#2  
  //  REG_TCC0_CC3 = generator(accumulator3, shape, linearity, 2); // pin 3 //#3
   TCC0->INTFLAG.bit.CNT = 1; //*******************************************************
  }
}


#define NUMREADINGS 50
unsigned int pot[NUMREADINGS];
unsigned int freq[NUMREADINGS];

void filterPut (char input, unsigned int newreading)
{
  static unsigned char potptr=0;
  static unsigned char freqptr = 0;

  if(input == POT)
  {
    pot[potptr] = newreading;
    potptr++;
    if(potptr >= NUMREADINGS)
      potptr=0;
  }

  else if(input == FREQ)
  {
    freq[freqptr] = newreading;
    freqptr++;
    if(freqptr >= NUMREADINGS)
      freqptr = 0;
  }
  
}

unsigned int filterGet (bool input)
{
  unsigned long int x;
  float z;
  unsigned char y;

  x = 0;
  if(input == POT)
  {
    for (y=0;y<NUMREADINGS;y++)
    {
      x = x + pot[y];
    }
  }
  else if(input == FREQ)
  {
    for (y=0;y<NUMREADINGS;y++)
    {
      x = x + freq[y];
    }
  }

  z = x;
  z = z/NUMREADINGS;
  z = z + 0.5;
  return (unsigned int)z;
}
