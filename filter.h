#define NUMREADINGS 50
#define FREQ 0
#define POT 1


unsigned int pot[NUMREADINGS];
unsigned int freq[NUMREADINGS];

void filterPut(char input, unsigned int newreading) {
  static unsigned char potptr=0;
  static unsigned char freqptr = 0;

  if(input == POT) {
    pot[potptr] = newreading;
    potptr++;
    if(potptr >= NUMREADINGS)
      potptr=0;
  } else if(input == FREQ) {
    freq[freqptr] = newreading;
    freqptr++;
    if(freqptr >= NUMREADINGS) freqptr = 0;
  }
}

unsigned int filterGet(bool input) {
  unsigned long int x;
  float z;
  unsigned char y;

  x = 0;
  if(input == POT) {
    for (y = 0; y < NUMREADINGS; y++) {
      x = x + pot[y];
    }
  } else if(input == FREQ) {
    for (y = 0; y < NUMREADINGS; y++) {
      x = x + freq[y];
    }
  }

  z = x;
  z = z / NUMREADINGS;
  z = z + 0.5;
  return (unsigned int)z;
}
