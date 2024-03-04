#include "pins.h"

#define NUM_MODULES 4

typedef enum Mode {
    VCO,
    LFO,
    ENV,
} Mode;

typedef struct Module {
    unsigned long int acc;
    unsigned long int phasor;
    Mode mode;
    unsigned short int slope;
    unsigned short int shape;
    unsigned long int upslope;
    unsigned long int downslope;
} Module;

typedef struct State {
    Module mods[NUM_MODULES];
    CRGB leds[NUM_LEDs];
} State;