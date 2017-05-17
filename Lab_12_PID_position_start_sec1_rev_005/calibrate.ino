//**********************************************************************
//**********************************************************************
//                    MODULE PWM DECODER CALIBRATION CODE
//**********************************************************************
//**********************************************************************

boolean calibrateModule(void)
{ 
  const boolean IS_VERBOSE = false;
  enum {GO_NOGO, WIRE_CHECK, PWM_DEMOD, AMP_OFFSET, AMP_GAIN, DONE};  
  int state = GO_NOGO, prevState = -1, modState = 0;
  char chIn;
  float testVolts;
  boolean isCalActive = true, isNewState;
  
  digitalWrite(nEN, HIGH); // disable drive
  
  while (isCalActive)
  {
    syncSampleTime();
    isNewState = (state != prevState);
    prevState = state;
  
    chIn  = Serial.read();
    while (Serial.read() != -1); // returns -1 when input buffer empty
    
    switch (state)
    { 
      //----------------------------------------------------------------
      case GO_NOGO:
        if (isNewState) 
        {
          if (IS_VERBOSE) Serial.println("calibration: GO_NOGO");
          Serial.println(F("\nEnter 'c' to calibrate module."));
          Serial.println(F("Enter 'q' to quit calibration."));
        }
        if (chIn == 'c') state = WIRE_CHECK;
        if (chIn == 'q') state = DONE;        
      break;

      //----------------------------------------------------------------
      case WIRE_CHECK:
        if (isNewState) 
        {
          if (IS_VERBOSE) Serial.println("calibration: WIRE_CHECK");
          Serial.println(F("\nVerify +/-12V, +5V supplies."));
          Serial.println(F("Verify MOTOR DRIVE selector switch set to PWM"));
          Serial.println(F("\nEnter 'c' to continue calibration."));
        }
        digitalWrite(nEN, HIGH); // disable drive
        if (chIn == 'c') state = PWM_DEMOD;
        if (chIn == 'q') state = DONE;        
      break;
          
      //----------------------------------------------------------------
      case PWM_DEMOD:
        if (isNewState) 
        {
          if (IS_VERBOSE) Serial.println("calibration: PWM_DEMOD");
          Serial.println(F("\n**** PWM demodulator calibration ****"));
          Serial.println(F("Observe TP9 on 'Digital Control' board."));
          Serial.println(F("Adjust 'Digital Control' pots."));
          Serial.println(F("Trim for 5,4,2,0,-2,-4,-5 V staircase on TP9."));
          Serial.println(F(" R3  (left) symmetry: 5 to 4 step = -4 to -5 step"));
          Serial.println(F(" R9  (center) gain"));
          Serial.println(F(" R16 (right) offset"));
          Serial.println(F("\nEnter 'c' to continue calibration."));
        }

        switch (modState)
        {
          case 0: testVolts = 0.0;  modState++;   digitalWrite(TP0, HIGH);  break; // scope trigger;
          case 1: testVolts = 5.0;  modState++;   digitalWrite(TP0, LOW);   break; // scope trigger
          case 2: testVolts = 4.0;  modState++;   break; 
          case 3: testVolts = 2.0;  modState++;   break;
          case 4: testVolts = 0.0;  modState++;   break;
          case 5: testVolts = -2.0; modState++;   break;
          case 6: testVolts = -4.0; modState++;   break;
          case 7: testVolts = -5.0; modState++;   break;
          default: testVolts = 0.0; modState++;
        }
        if (modState >= 25) modState = 0; // hold off for horiontal sweep separation.  
        
        digitalWrite(nEN, HIGH); // disable drive
        pwmServoRC(testVolts);
        
        if (chIn == 'c') state = AMP_GAIN;
        if (chIn == 'q') state = DONE; 
        
      break;
      
      //----------------------------------------------------------------
      case AMP_OFFSET:
        if (isNewState) 
        {
          if (IS_VERBOSE) Serial.println("calibration: AMP_OFFSET");
          Serial.println(F("\n**** Power amplifier offset calibration ****"));
          Serial.println(F("Set Motor Drive Vin/PWM selector switch to PWM."));
          Serial.println(F("Re-adjust R16 (right on 'Digital Control' board) to"));
          Serial.println(F("Set TP3 = 0.0 V on 'Analogue Control' board."));
          Serial.println(F("\nEnter 'c' to continue calibration."));
        }
        
        digitalWrite(nEN, LOW); // enable drive
        pwmServoRC(0.0);
        
        if (chIn == 'c') state = AMP_GAIN;
        if (chIn == 'q') state = DONE;       
           
      break;
      
      //----------------------------------------------------------------
      case AMP_GAIN:     
        if (isNewState) 
        {
          if (IS_VERBOSE) Serial.println("calibration: AMP_GAIN");          
          Serial.println(F("\n**** Power amplifier gain calibration ****"));
          Serial.println(F("MOTOR SHOULD SPIN."));
          Serial.println(F("Trim 'Analogue Control' gain (R14) for TP3 = 6.0 V."));
          Serial.println(F("\nEnter 'c' to complete calibration."));
        }

        // Nominal tachometer gain = 22/47 = 0.468. Nominal amplifier gain = 1/0.468 = 2.136.
        // Drive PWM demodulator with 6.0 * 0.468 = 2.808V input.
        // Adjust R14 to obtain 6.0 volts amplifier output on TP3.
        
        digitalWrite(nEN, LOW); // enable drive
        pwmServoRC(2.808);
        
        if (chIn == 'c')
        {
          digitalWrite(nEN, HIGH); // disable drive
          pwmServoRC(0.0);
          Serial.println(F("Calibration complete."));
          state = DONE;
        }
      
      break;
      //----------------------------------------------------------------
      case DONE:
      if (isNewState) 
        {
          if (IS_VERBOSE) Serial.println("calibration: DONE");
          digitalWrite(nEN, HIGH); // disable drive
        }
        isCalActive = false;
        
      break;
    } // switch (case)
  } // while (isCalActive)
  
  return isCalActive;
}

//**********************************************************************
void pwmServoRC(float Vin)
{    
  const float CNT_PER_VOLT = 200.0; // 0.5msec/(0.5usec/cnt)/5V
  const int CNT_CENTER = 3000; // 1.5msec/0.5usec/cnt
  const int CNT_MIN = 2000; // 1msec/(0.5usec/count)
  const int CNT_MAX = 4000; // 2msec/(0.5usec/count)
 
  // map [-5V...+5V] -> [1.0mec...2.0msec]
  // map [-5V...+5V] -> [0%...100%] PWM
  pwmCount = CNT_CENTER + int(Vin*CNT_PER_VOLT+0.5);
  if (pwmCount < CNT_MIN) pwmCount = CNT_MIN;
  if (pwmCount > CNT_MAX) pwmCount = CNT_MAX;
  OCR1B = pwmCount; 
}

