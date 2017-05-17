//**********************************************************************
//**********************************************************************
//                        ARDUINO UTILITIES CODE
//**********************************************************************
//**********************************************************************

void initArduinoEnvironment(void)
{
  // Simulator pin configuration: drive from code
  pinMode(GC3, INPUT); // gray4 MSB
  pinMode(GC2, INPUT);
  pinMode(GC1, INPUT);
  pinMode(GC0, INPUT); // gray4 LSB
  
  pinMode(ALED, OUTPUT);  digitalWrite(ALED,LOW);
  pinMode(TP0, OUTPUT);   digitalWrite(TP0,LOW);
  pinMode(PWMOD, OUTPUT); digitalWrite(PWMOD, LOW);
  pinMode(nEN, OUTPUT);   digitalWrite(nEN, HIGH);
  
  analogRead(VTACH); // read and discard ADC to prime registers
  
  Serial.begin(115200);
  // Serial.println(F("Console line ending: 'Carriage return'.")); 
}

//**********************************************************************
void initTimer1(void)
{ 
  // Use Timer 1 (16 bit) for hardware PWM generation.
  // Prescale the clock frequency by 8 for 0.5 usec resolution and 32 msec range.
  // FAST PWM mode with ICR1 defined cycle length (TOP value), (Mode 14)
  // Output pin OC1B (d10~) driven by counter hardware.

  cli(); // disable global interrupts  
  TCCR1B =  (1 << WGM13)|(1 << WGM12); // start Fast PWM mode 14 setup
  TCCR1A =  (1 << WGM11)|(0 << WGM10); // finish  setup
  TCCR1B |= (0 << CS12)|(1 << CS11)|(0 << CS10); // clock prescale = 8 
  TCCR1A |= (1 << COM1B1)|(0 << COM1B0); // OCR1B pin (d10~) noninverting PWM
  ICR1 = T1_TOP-1;
  OCR1A = 0;
  TIMSK1 = (1 << TOIE1); // enable overflow interrupt (cycle length)

  sei(); // enable global interrupts
}

//---------------------------------------------------------------- 
ISR(TIMER1_OVF_vect) // 10 msec sample flag
{  
  isSample = true;  // set flag enabling loop advance
}

//**********************************************************************
void syncSampleTime(void)
{
  while (!isSample); // spin awaiting ISR sample
  isSample = false;
  isClearStates = false;
}

//**********************************************************************
void initEncoderPinChangeInterrupts(void)
{
  // Position encoder ISR setup
  // PCINT2_vect ISR triggered for enabled bit changes on PCMSK2 
  
  cli(); // disable global interrupts 
  PCMSK2  = (1 << PCINT23); // GC3 -> d7
  PCMSK2 |= (1 << PCINT22); // GC2 -> d6
  PCMSK2 |= (1 << PCINT21); // GC1 -> d5
  PCMSK2 |= (1 << PCINT20); // GC0 -> d4

  PCICR = (1 << PCIE2); // enable pin change interrupt
  sei(); // enable global interrupts
}

//*********************************************************************
ISR(PCINT2_vect) // 4 bit gray decoder
 {
  gray4Enc16(); // PCINT 4 bits
//  quadEnc48(); // PCINT 2 bits: 12 quad cycles on outer 2 tracks
  
}

//*********************************************************************
void quadEnc48(void) // 2 bit quad decoder: 48 tick/rev
{
  // Detect pin changes: Arduino d5,d4
  // Identify direction, distance.

  const float MTR_RAD_PER_TICK = TWO_PI/48.0; 
  static byte oldGrayCode = 0, Ndiv = 0; // one time init
  
  isEncoderChange = true;
  grayCode = (PIND & 0b00110000)/16;
  digitalWrite(ALED,!digitalRead(ALED));
  
  // Gray sequence: 0,1,3,2
  switch (grayCode) 
  {
    case 0:
      if (oldGrayCode ==  2) grayPos += MTR_RAD_PER_TICK;
      if (oldGrayCode ==  1) grayPos -= MTR_RAD_PER_TICK;
      break;
    case 1:
      if (oldGrayCode ==  0) grayPos += MTR_RAD_PER_TICK;
      if (oldGrayCode ==  3) grayPos -= MTR_RAD_PER_TICK;
      break;
    case 3:
      if (oldGrayCode ==  1) grayPos += MTR_RAD_PER_TICK;
      if (oldGrayCode ==  2) grayPos -= MTR_RAD_PER_TICK;      
      break;
    case 2:
      if (oldGrayCode ==  3) grayPos += MTR_RAD_PER_TICK;
      if (oldGrayCode ==  0) grayPos -= MTR_RAD_PER_TICK;
      break;
  } 
  oldGrayCode = grayCode; // prep next event
  
} // quadEnc48( )

//*********************************************************************
void gray4Enc16(void) // 4 bit gray decoder: module default
{
  // Detect pin changes: Arduino d7,d6,d5,d4
  // Identify direction, distance.

  const float MTR_RAD_PER_TICK = TWO_PI/16.0; 
  static byte oldGrayCode = 0; // one time init
  
  isEncoderChange = true;
  grayCode = (PIND & 0b11110000)/16;
  digitalWrite(ALED,!digitalRead(ALED));
  
  //CW gray sequence: 0,1,3,2,6,7,5,4,12,13,15,14,10,11,9,8
  switch (grayCode) 
  {
    case 0:
      if (oldGrayCode ==  8) grayPos += MTR_RAD_PER_TICK;
      if (oldGrayCode ==  1) grayPos -= MTR_RAD_PER_TICK;
      break;
    case 1:
      if (oldGrayCode ==  0) grayPos += MTR_RAD_PER_TICK;
      if (oldGrayCode ==  3) grayPos -= MTR_RAD_PER_TICK;
      break;
    case 3:
      if (oldGrayCode ==  1) grayPos += MTR_RAD_PER_TICK;
      if (oldGrayCode ==  2) grayPos -= MTR_RAD_PER_TICK;      
      break;
    case 2:
      if (oldGrayCode ==  3) grayPos += MTR_RAD_PER_TICK;
      if (oldGrayCode ==  6) grayPos -= MTR_RAD_PER_TICK;
      break;
    case 6:
      if (oldGrayCode ==  2) grayPos += MTR_RAD_PER_TICK;
      if (oldGrayCode ==  7) grayPos -= MTR_RAD_PER_TICK;
      break;
    case 7:
      if (oldGrayCode ==  6) grayPos += MTR_RAD_PER_TICK;
      if (oldGrayCode ==  5) grayPos -= MTR_RAD_PER_TICK;
      break;
    case 5:
      if (oldGrayCode ==  7) grayPos += MTR_RAD_PER_TICK;
      if (oldGrayCode ==  4) grayPos -= MTR_RAD_PER_TICK;
      break;
    case 4:
      if (oldGrayCode ==  5) grayPos += MTR_RAD_PER_TICK;
      if (oldGrayCode == 12) grayPos -= MTR_RAD_PER_TICK;
      break;
    case 12:
      if (oldGrayCode ==  4) grayPos += MTR_RAD_PER_TICK;
      if (oldGrayCode == 13) grayPos -= MTR_RAD_PER_TICK;
      break;
    case 13:
      if (oldGrayCode == 12) grayPos += MTR_RAD_PER_TICK;
      if (oldGrayCode == 15) grayPos -= MTR_RAD_PER_TICK;
      break;
    case 15:
      if (oldGrayCode == 13) grayPos += MTR_RAD_PER_TICK;
      if (oldGrayCode == 14) grayPos -= MTR_RAD_PER_TICK;
      break;
    case 14:
      if (oldGrayCode == 15) grayPos += MTR_RAD_PER_TICK;
      if (oldGrayCode == 10) grayPos -= MTR_RAD_PER_TICK;      
      break;
    case 10:
      if (oldGrayCode == 14) grayPos += MTR_RAD_PER_TICK;
      if (oldGrayCode == 11) grayPos -= MTR_RAD_PER_TICK;
      break;
    case 11:
      if (oldGrayCode == 10) grayPos += MTR_RAD_PER_TICK;
      if (oldGrayCode ==  9) grayPos -= MTR_RAD_PER_TICK;
      break;
    case 9:
      if (oldGrayCode == 11) grayPos += MTR_RAD_PER_TICK;
      if (oldGrayCode ==  8) grayPos -= MTR_RAD_PER_TICK;
      break;
    case 8:
      if (oldGrayCode ==  9) grayPos += MTR_RAD_PER_TICK;
      if (oldGrayCode ==  0) grayPos -= MTR_RAD_PER_TICK;
      break;
  } 
  oldGrayCode = grayCode; // prep next event
  
} // gray4Enc16 // 4 bit gray decoder

//**********************************************************************
void displayEncoderData(void)
{
  static boolean isFirstPass = true;
  
  if (isEncoderChange) 
  {
    isEncoderChange = false;  
    if (isFirstPass)
    {
      isFirstPass = false;
      // Serial.println(F("\ntick\tgCode\tgCode\tpos"));
      Serial.println(F("\ngCode\tgCode\tpos"));
    }
    // Serial.print(tick); Serial.print('\t');
    Serial.print(grayCode); Serial.print('\t'); // show upper nibble
    (grayCode & 0x08) ? Serial.print(1) : Serial.print(0);
    (grayCode & 0x04) ? Serial.print(1) : Serial.print(0);
    (grayCode & 0x02) ? Serial.print(1) : Serial.print(0);
    (grayCode & 0x01) ? Serial.print(1) : Serial.print(0); 
    Serial.print('\t');
    Serial.println(grayPos,3);  
  }
}

