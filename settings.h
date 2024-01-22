FlashStorage(div_storage, int);
FlashStorage(wave_storage, int);
FlashStorage(init_storage, char);


void readSettings (void)
{
  char x;
  char c = 'S';
  int y = 1;
  init_storage.read(x);
  if(x == 'S')  //S means eeprom has been initialized.
  {
    wave_storage.read(waveSelect);
    div_storage.read(divSelect);
  }
  else
  {
    //we initialize, no 'S' found
    init_storage.write(c);  //use variables because this library hates constants
    wave_storage.write(y);
    div_storage.write(y);
    divSelect = 1;
    waveSelect = 1;
  }

}

void saveSettings (void)
{
  int x;
  wave_storage.read(x);
  
  if(x != waveSelect)
    wave_storage.write(waveSelect);  // <-- save the waveSelect 

   div_storage.read(x);
  
  if(x != divSelect)
    div_storage.write(divSelect);  // <-- save the waveSelect  
  

}