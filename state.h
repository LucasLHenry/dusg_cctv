#define NUM_MODULES 2

typedef enum Mode {
    VCO,
    LFO,
    ENV,
} Mode;

typedef struct Module {
    uint64_t accumulator;
    uint64_t phasor;
    uint16_t shape; // goes from 0 to 1023, because that's the precision of the ADCs
    uint16_t linearity; // goes from 0 to 1023 for the same reasons
    Mode mode;
} Module;

typedef struct State {
    Module modules[NUM_MODULES];
} State;