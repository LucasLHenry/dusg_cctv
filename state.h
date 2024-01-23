typedef enum Mode {
    VCO,
    LFO,
    ENV,
} Mode;

typedef struct Module {
    uint64_t acc;
    uint64_t phasor;
    Mode mode;
    uint16_t shape;
    uint16_t lin;
    uint64_t upslope;
    uint64_t downslope;
} Module;

typedef struct State {
    Module mods[NUM_MODULES];
} State;