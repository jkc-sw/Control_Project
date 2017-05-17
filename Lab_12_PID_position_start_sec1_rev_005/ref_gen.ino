//**********************************************************************
//**********************************************************************
//                      REFERENCE GENERATOR CODE
//**********************************************************************
//**********************************************************************

void fetchTrapezoidRef()
{
  const boolean IS_VERBOSE = false;
  enum{TRAP_STOP, TRAP_GO};
  static byte state = TRAP_STOP, oldState = TRAP_STOP;
  
  static int T1,T2,T3,T4,T5,T6,T7,T8,D1,D2;
  static float acc;
  static int accTicks, platTicks, dwellTicks;

  boolean isNewState = (state != oldState);
  oldState = state;

  //------------------------------------------------------
  if (state == TRAP_STOP)
  {
    if (isNewState) // entry condition
    {
      if (IS_VERBOSE) Serial.println(F("step: TRAP_STOP"));
      trapEnable = TRAP_STOP;
      Serial.print("\ncErrP_RMS = "); Serial.println(sqrt(cErrP_SqAccum/tick));
      Serial.print("cErrP_AbsPk = "); Serial.println(cErrP_AbsPk);
    }
    
    // state business: spin until menu enables trap
   
    if (trapEnable != TRAP_NOT_EN) // menu select -> exit condition
    {
      buildTrapProfile(acc,accTicks,platTicks);
      acc = (trapEnable == TRAP_REV_EN) ? -acc : acc;
      dwellTicks = int(dwellSec/TSAMP);
      tick = 0;
      state = TRAP_GO;
      isDisplayDataEn = true;
      cErrP_SqAccum = 0.0;
      cErrP_AbsPk = 0.0;
      
      tVel = 0.0;
      T1 = accTicks;
      T2 = T1 + platTicks;
      T3 = T2 + accTicks;
      T4 = T3 + dwellTicks;
      T5 = T4 + accTicks;
      T6 = T5 + platTicks;
      T7 = T6 + accTicks;
      T8 = T7 + dwellTicks;

      D1 = T3 + dwellTicks/4;
      D2 = T7 + dwellTicks/4;
    }
    
  } // if (state == TRAP_STOP)

  //------------------------------------------------------
  if (state == TRAP_GO)
  {
    if (isNewState) // entry condition
    {
      if (IS_VERBOSE) Serial.println(F("step: TRAP_GO"));
    }

    // state business  
    if (tick < T1) tAcc = acc;
    else if( tick < T2) tAcc = 0.0;
    else if (tick < T3) tAcc = -acc;
    else if (tick < T4) tAcc = 0.0;
    else if ((tick == T4)&&(trapEnable != TRAP_CYC_EN))
    {
      trapEnable = TRAP_NOT_EN;
    }
    else if (tick < T5) tAcc = -acc;
    else if (tick < T6) tAcc = 0.0;
    else if (tick < T7) tAcc = acc;
    else if (tick < T8) tAcc = 0.0;
    else trapEnable = TRAP_NOT_EN;

    if (tick < D1)  Vdist = 0.0;
    else if (tick > D2) Vdist = 0.0;
    else if (trapEnable == TRAP_CYC_EN) Vdist = distSpecV;


      tVel += tAcc*TSAMP;
      tPos += tVel*TSAMP;

    isDisplayDataEn = true;
    
    if (trapEnable == TRAP_NOT_EN) // exit condition
    {
      state = TRAP_STOP;
    }
    
  } // if (state == TRAP_GO)

} // fetchTrapezoidRef()

//**********************************************************************
void buildTrapProfile(float &Aramp, int &rampTicks, int &platTicks)  
{
  const boolean IS_VERBOSE = true;
  float Tramp, Dramps, Tplat, Vplat, Dplat, Dtrap, Dir;
  
//  Dtrap = abs(posSpec); // recast move as sign, magnitude 
//  Dir = (posSpec >= 0.0) ? 1.0 : -1.0;  
////  Dir = (posSpec >= 0.0) ? -1.0 : 1.0;  
//  Aramp = accSpec; // motor radians/sec
//  Vplat = velSpec; // motor radians/sec^2
//  Dramps = Vplat* Vplat/Aramp; // 2*(1/2)*base*height
//  
//  if (Dramps >= Dtrap) // 3 x 1/3 trap profile
//  {
//    Vplat = sqrt(Dtrap*Aramp/2);
//    Dramps = Vplat* Vplat/Aramp; // 2*(1/2)*base*height
//  }
//  
//  Tramp = Vplat/Aramp;
//  Dplat = Dtrap-Dramps;
//  Tplat = Dplat/Vplat;
//  
//  const float SAMPLES_PER_SEC = 1.0/TSAMP;
//  rampTicks = 1+int(SAMPLES_PER_SEC*Tramp+0.5); // one tick minimum
//  platTicks = 1+int(SAMPLES_PER_SEC*Tplat+0.5); // one tick minimum
//  Tramp = rampTicks*TSAMP;
//  Tplat = platTicks*TSAMP;
//  Vplat = Dtrap/(Tramp+Tplat);
////  Aramp = Dir*(Vplat/Tramp); // return signed version
//  Dtrap = Vplat*(Tramp+Tplat);

  Dtrap = abs(posSpec); // recast move as sign, magnitude 
  Dir = (posSpec >= 0.0) ? 1.0 : -1.0;  
//  Dir = (posSpec >= 0.0) ? -1.0 : 1.0;  
  Aramp = accSpec; // motor radians/sec
  Vplat = velSpec; // motor radians/sec^2
  Tramp = Vplat/Aramp;
  Dramps = 0.5*(Vplat/Aramp)*Vplat; // 2*(1/2)*base*height
  
  if (Dramps >= Dtrap*0.5) // 3 x 1/3 trap profile
  {
    Tramp = sqrt(Dtrap/Aramp);
    Tplat = 0.0; // 2*(1/2)*base*height
    Dplat = 0.0;
  }
  else
  {
    Dplat = Dtrap - 2*Dramps;
    Tplat = Dplat/Vplat;
  }
 
  
  const float SAMPLES_PER_SEC = 1.0/TSAMP;
  rampTicks = int(SAMPLES_PER_SEC*Tramp+0.5); // one tick minimum
  platTicks = int(SAMPLES_PER_SEC*Tplat+0.5); // one tick minimum
  Tramp = rampTicks*TSAMP;
  Tplat = platTicks*TSAMP;
  Vplat = Dtrap/(Tramp+Tplat);
  Aramp = Dir*(Vplat/Tramp); // return signed version
  Dtrap = Vplat*(Tramp+Tplat);

  
  if (false) // debug true false
  {
    Serial.println(F("\nTrapezoid trajectory parameters"));
    Serial.print(F("accSpec = "));   Serial.println(accSpec,4);
    Serial.print(F("velSpec = "));   Serial.println(velSpec,4);
    Serial.print(F("posSpec = "));   Serial.println(posSpec,4);
    Serial.print(F("Aramp = "));     Serial.println(Aramp,4);
    Serial.print(F("Vplat = "));     Serial.println(Vplat,4);
    Serial.print(F("Dtrap = "));     Serial.println(Dtrap,4);
    Serial.print(F("rampTicks = ")); Serial.println(rampTicks);
    Serial.print(F("platTicks = ")); Serial.println(platTicks);    
  }

} // buildTrapProfileC()

//**********************************************************************
//**********************************************************************
void fetchStepRef(void)
{
  const boolean IS_VERBOSE = false;
  enum{STEP_STOP, STEP_GO};
  static byte state = STEP_STOP, oldState = STEP_STOP;
  
  boolean isNewState = (state != oldState);
  oldState = state;
  
  //------------------------------------------------------
  if (state == STEP_GO)
  {
    if (isNewState)
    {
      if (IS_VERBOSE) Serial.println(F("step: STEP_GO"));
    }
    
    if (tick >= stepStartTick) uStep = stepSpec;
    isDisplayDataEn = true;
    
    if (tick >= (stepRunTicks-1)) // exit condition
    {
      isStepEn = false;
      state = STEP_STOP;
    }    
  } // if (state == STEP_GO)

  //------------------------------------------------------
  else if (state == STEP_STOP)
  {
    if (isNewState) // entry condition
    {
      if (IS_VERBOSE) Serial.println(F("step: STEP_STOP"));
    }
    
    if (isStepEn) // exit condition
    {
      tick = 0;
      isDisplayDataEn = true;
      state = STEP_GO;
    }
  } // else if (state == STEP_STOP)
  
} // fetchStepRef()

//**********************************************************************
//**********************************************************************
void fetchTriangleRef()
{
  const float TRI_T = 10.0; // period seconds
  const float TRI_MAX = 1.0;
  const float TRI_DELTA = (TRI_MAX/(TRI_T/4.0))*TSAMP;
  const int MAX_TICK = int(TRI_T/TSAMP);
  
  const boolean IS_VERBOSE = false;
  enum {TUP, TDOWN, DONE};
  static byte state = DONE, oldState = DONE;
 
  boolean isNewState = (state != oldState);
  oldState = state;
  switch (state)
  {
    //------------------------------------------------------
    case TUP:
    {
      // entry condition
      if (isNewState)
      {
        if (IS_VERBOSE) Serial.println(F("triangle: TUP"));
      }
      
      // state business
      trnglLvl += TRI_DELTA;
      if (trnglLvl > TRI_MAX) trnglLvl = TRI_MAX;
      isDisplayDataEn = true;
      
      // exit conditions
      if (tick >= MAX_TICK) state = DONE;
      else if (trnglLvl >= TRI_MAX-TRI_DELTA/2.0) state = TDOWN;
      
      break; // TUP  
    }
  
    //------------------------------------------------------
    case TDOWN:
    {
      // entry condition
      if (isNewState)
      {
        if (IS_VERBOSE) Serial.println(F("triangle: TDOWN"));
      }
      
      // state business
      trnglLvl -=  TRI_DELTA;
      if (trnglLvl <= -TRI_MAX) trnglLvl = -TRI_MAX;
      isDisplayDataEn = true;
      
      // exit conditions
      if (tick >= MAX_TICK) state = DONE;
      else if (trnglLvl <= -TRI_MAX+TRI_DELTA/2.0) state = TUP;
      
      break; // TDOWN      
    }
    
    //------------------------------------------------------
    case DONE:
    {
      // entry condition
      if (isNewState)
      {
        if (IS_VERBOSE) Serial.print(F("triangle: DONE"));
        isTriangleEn = false;
        trnglLvl = 0.0;
      }
      
      // state business: spin
      
      // exit condition
      if (isTriangleEn)
      {
        tick = 0;
        trnglLvl = 0.0;
        isDisplayDataEn = true;
        state = TUP;
      }
      break; // DONE
    }
  } // switch(state)  
}

//**********************************************************************
//**********************************************************************
void fetchVelocitySteps(void)
{
  const boolean IS_VERBOSE = false;
  const int STEP_TICKS = 300;
  const float stepVolt[] = {-2.0,-1.0,0.0,1.0,2.0,4.0,6.0};
  const int NUM_STEPS = sizeof(stepVolt)/sizeof(stepVolt[0]);

  enum{STEP_STOP, STEP_GO};
  static byte state = STEP_STOP, oldState = STEP_STOP; 
  static int idxVolts, idxStep = 0;

  boolean isNewState = (state != oldState);
  oldState = state;

  //------------------------------------------------------
  if (state == STEP_STOP)
  {
    if (isNewState) // entry condition
    {
      if (IS_VERBOSE) Serial.println(F("step: STEP_STOP"));
      stepSeriesLvl = 0.0;
      isDisplayDataEn = true;
      isStepSequenceEnable = false;
    }
    
    // state business: spin until menu enables trap
   
    if (isStepSequenceEnable == true) // exit condition
    {
      tick = 0;
      state = STEP_GO;
    }
  } // if (state == STEP_STOP)

  //------------------------------------------------------
  if (state == STEP_GO)
  {
    // entry condition
    if (isNewState) if (IS_VERBOSE) Serial.println(F("step: STEP_GO"));

    // state business
    stepSeriesLvl = stepVolt[idxVolts];
    if (idxStep++ == STEP_TICKS)
    {
      idxStep = 0;
      idxVolts++;
    }
    isDisplayDataEn = true;
    
    // exit condition
    if (idxVolts >= NUM_STEPS) state = STEP_STOP;
    
  } // if (state == STEP_GO)

} // fetchVelocitySteps()

