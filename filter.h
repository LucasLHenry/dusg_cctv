unsigned int pot[NUMREADINGS];
unsigned int freq[NUMREADINGS];

void filterPut (char input, unsigned int newreading)
{
  static unsigned char potptr = 0;
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
  return (unsigned int)z;
}
