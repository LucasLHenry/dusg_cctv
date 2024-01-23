#define HZPHASOR 91183 //phasor value for 1 hz.
#define M 511
#define H 255
#define DEFAULT_SHAPE 255
#define DEFAULT_LIN 255
#define UPSLOPE(x) ((M << 7) / x)
#define DOWNSLOPE(x) ((M << 7) / (M - x))
#define NUM_MODULES 4
#define NUMREADINGS 50
#define FREQ 0
#define POT 1