#define FREQ 1
#define POT 0
#define NUMREADINGS 50

uint32_t pot[NUMREADINGS];
uint32_t freq[NUMREADINGS];

void filterPut (char input, uint32_t newreading)
{
  static uint8_t potptr = 0;
  static uint8_t freqptr = 0;

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

uint32_t filterGet (bool input)
{
  uint64_t x;
  float z;
  uint8_t y;

  x = 0;
  if (input == POT) {
    for (y = 0; y < NUMREADINGS; y++) {
      x += pot[y];
    }
  } else if (input == FREQ) {
    for (y = 0; y < NUMREADINGS; y++) {
      x += freq[y];
    }
  }

  z = ((float)x) / NUMREADINGS;
  z += 0.5;
  return (uint32_t)z;
}
