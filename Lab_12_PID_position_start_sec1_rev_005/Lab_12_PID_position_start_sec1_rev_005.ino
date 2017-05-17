#include "Arduino.h" // required for added .h file inclusion
#include "config.h"  // pins, constants definitions
#include "model.h"   // module parameters

// Simulate P-Cascade with and without disturbance
// Simulate PI-Cascade with and without disturbance
// Excel calculate peak error
// Excel calculate rms error
// Observe trapezoid generator hooks for error calculation/display.
// Excel calculate Varm, Iarm
// Run mech in P-Cascade: reconcile model/mech
// Run mech in PI-Cascade

// Reconcile load direction / motor direction / feedback sense
// Observe/fix PI compensation 'wind up'.
// Observe non-zero DC disturbance error in PD loop.
// Extend PD to PID.
// Exercise zero set, integrator enable/disable.
  
//**********************************************************************
void setup()
{    
  initArduinoEnvironment();
  initTimer1(); // PWM generator, sample timer
  initEncoderPinChangeInterrupts(); 

  Serial.println(F("Lab 10: Module Position Controllers Starter"));
//   calibrateModule();
  displayModelParms();
  showHelpMenu();

  stepSpec = 0.0, stepStartTick = 1, stepRunTicks = 200;
  accSpec = 1000.0, velSpec = 700.0, posSpec = 1000.0, dwellSec = 2.0;

//  accSpec = 350.0, velSpec = 250.0, posSpec = 1000.0, dwellSec = 2.0;
  distSpecV = -V_DIST; // sag load down per weight/counterweight
  Vstic = 0.25; // set to zero for step response testing  
  
  Kf = 0.25;
  Ka = 1.0/SYS_A;
  Kw = SYS_B/SYS_A;
  Kp = 0.0;
  Kv = 0.0;
  Ki = 2.9; //PI-cascade
//  Ki = 0.1*WN; // PID
 
} // setup()

//**********************************************************************
void loop()
{   
  syncSampleTime();
//   displayEncoderData(); // comment in for debug
  manageMenu();
  
//  fetchStepRef();
//  fetchTriangleRef();  
//  fetchTrapezoidRef();
//  fetchVelocitySteps();

//  //------------------------------------------------------------------
//  // Simulaltion based trapezoidal profile volts, current
//  // Assume trap in motor radian units
//  fetchStepRef();
//  fetchTriangleRef();  
//  fetchTrapezoidRef();
//  fetchVelocitySteps();
//  Iarm = (J_SYS*tAcc*LDMM_2_MRAD + D_SYS*tVel*LDMM_2_MRAD + T_DIST/N_MTR2LD)/K_TORQUE - Vdist/R_ARM ;
//  Varm = Iarm*R_ARM + K_EMF*tVel*LDMM_2_MRAD;
////  if (isDisplayDataEn) displayAmpsVolts();
//  if (isDisplayDataEn) displayData();

//  //------------------------------------------------------------------
//  // Open Loop Testing: Time constant, stiction
//  fetchStepRef();
//  fetchTriangleRef();  
//  fetchTrapezoidRef();
//  fetchVelocitySteps();
//  Vctl = uStep + trnglLvl + tVel + stepSeriesLvl; 
//  // simModuleMotor(Vctl, mVel, mPos);   
//  driveModuleMotor(Vctl + Kf*sticSign(Vctl), Vdrive, mVel, mPos);
//  cVel = mVel; 
//  cPos = mPos;
//  if (isDisplayDataEn) displayData();
 
//  //------------------------------------------------------------------
//  // Closed velocity loop
//  
//  ref = uStep + tVel;
//  errV = ref - mVel;
//  Vffwd = Kf*sticSign(tVel) + 0.0*Ka*tAcc + 0.0*Kw*tVel; 
//  // Vctl = Vffwd + 0.05*errV; // hard code P velocity loop: pole = 12
//  Vctl = Vffwd + 0.07*PI_compensate(errV,3.0); // hard code PI velocity loop: pole = 12
//  simModuleMotor((Vctl+Vdist), mVel, mPos);
////  driveModuleMotor((Vctl + 0.0*Vdist), Vdrive, mVel, mPos);  
//  cVel = mVel; 
//  cPos = mPos;
//  cErrV = tVel - cVel;
//  cErrP = tPos - cPos;
//  if (isDisplayDataEn) displayData();

//  //------------------------------------------------------------------
//  // Closed position loop: Lead (PD) compensation
//  
//  ref = uStep + tPos*LDMM_2_MRAD;
//  errP = ref - mPos;
//  fetchStepRef();
//  fetchTriangleRef();  
//  fetchTrapezoidRef();
//  fetchVelocitySteps();
//  Vffwd = Kf*sticSign(tVel) + Ka*tAcc*LDMM_2_MRAD + Kw*tVel*LDMM_2_MRAD;  
//  Vctl = 1.0*Vffwd; // CL configuration
//  Vctl += 1.0*KP_LEAD*PD_compensate(PI_compensate(errP, Ki));
//  simModuleMotor((Vctl+Vdist), mVel, mPos);
////  driveModuleMotor((Vctl+Vdist), Vdrive, mVel, mPos);  
//  cVel = mVel*MRAD_2_LDMM; 
//  cPos = mPos*MRAD_2_LDMM;
//  cErrV = tVel - cVel;
//  cErrP = tPos - cPos;
//  Varm = Vctl*K_AMP;
//  Iarm = (Varm - mVel*K_EMF)/R_ARM;
//  if (isDisplayDataEn) displayData();

  //------------------------------------------------------------------
  // Closed position loop: Cascade compensation

  ref = tPos*LDMM_2_MRAD;
  errP = ref - mPos;
  errV = 1.0*KP_CASC*errP - mVel + tVel*LDMM_2_MRAD;
  fetchStepRef();
  fetchTriangleRef();  
  fetchTrapezoidRef();
  fetchVelocitySteps();
  Vffwd = Kf*sticSign(tVel) + Ka*tAcc*LDMM_2_MRAD + Kw*tVel*LDMM_2_MRAD;  
//  Vctl = 1.0*Vffwd;
  Vctl = 0.0*Vffwd;
  Vctl += 1.0*PI_compensate(KV_CASC*errV, Ki);
//  Vctl += 0.0*PI_compensate(KV_CASC*errV, Ki);
//  simModuleMotor((Vctl + 1.0*Vdist), mVel, mPos);
  simModuleMotor((Vctl + 0.0*Vdist), mVel, mPos);
//  driveModuleMotor((Vctl + 1.0*Vdist), Vdrive, mVel, mPos);  
  cVel = mVel*MRAD_2_LDMM; 
  cPos = mPos*MRAD_2_LDMM;
  
  Varm = Vctl*K_AMP;
  Iarm = (Varm - mVel*K_EMF)/R_ARM;
  cErrV = tVel - cVel;
  cErrP = tPos - cPos;
  if (isDisplayDataEn) displayData();

  //------------------------------------------------------------------
  cErrP_SqAccum += cErrP*cErrP;
  cErrP_AbsPk = (abs(cErrP)>cErrP_AbsPk) ? abs(cErrP) : cErrP_AbsPk;

  
  tick++;

} // loop()

//**********************************************************************
float sticSign(float velCmd)
{    
  float sticSgn = 0.0;
  if (velCmd >  0.01) sticSgn =  1.0;
  if (velCmd < -0.01) sticSgn = -1.0;
  return sticSgn;
}

//**********************************************************************
void driveModuleMotor(float Vin, float &Vdrive, float &vel, float &pos)
{    
  // RC Servo Protocol
  const float MECH_DIR = -1.0;
  const float VMAX = 5.0, STIC_VOLTS = 0.0;
  const float CNT_PER_VOLT = 200.0; // 5msec/(0.5usec/cnt)/5V
  const int CNT_CENTER = 3000, CNT_MIN = 2000, CNT_MAX = 4000;
  
  digitalWrite(nEN,!isDriveEnable);
 
  // map [-5V...+5V] -> [1.0mec...2.0msec] RC Servo protocol
  // PW > 1.5 msec -> CW motor rotation
  pwmCount = CNT_CENTER + int(MECH_DIR*Vin * CNT_PER_VOLT);
  if (pwmCount < CNT_MIN) pwmCount = CNT_MIN;
  if (pwmCount > CNT_MAX) pwmCount = CNT_MAX;
  OCR1B = pwmCount; 

  mPos = MECH_DIR*grayPos; 

  const float RPS_PER_COUNT = 0.633; // calibrate velocity gain, offset
  int adcCount = analogRead(VTACH);
  vel = MECH_DIR*RPS_PER_COUNT*(adcCount - 511);
}

//**********************************************************************
void simModuleMotor(float Vin, float &vel, float &pos)
{  
  static float velM1 = 0.0, posM1 = 0.0; // state[n-1] storage

  // non-linear stiction model: resist velocity growth
  float Vdrive = Vin;
//  if (vel >  0.1) Vdrive -= Vstic;
//  if (vel < -0.1) Vdrive += Vstic;
  if (vel >  0.01) Vdrive -= Vstic;
  if (vel < -0.01) Vdrive += Vstic;

  // integrate kinematic states
  vel += (SYS_A*Vdrive - SYS_B*velM1)*TSAMP; // dy/dt integrated
  pos += vel*TSAMP; // pos = integral of vel
   
  velM1 = vel; // next iteration setup
  posM1 = pos;
  
  if (isClearStates) // restart conditions
  {
    vel = 0.0;  velM1 = 0.0;
    pos = 0.0;  posM1 = 0.0;
  }
}

//**********************************************************************
float PI_compensate(float x, float Ki) // Proportional plus Integral compensator
{
  // Cancels pole at location -Ki, replace with a pole at the origin.
  // Compensator L = (s+Ki)/s, Implement as 1 + Ki/s. 
  // Integrator output clamped to prevent 'wind up' memory.

  static float xi = 0.0;
  float y;

  xi += x*TSAMP;
  y = x + Ki * xi; // Y/X = 1 + Ki/s
  
  if (isClearStates) xi = 0.0;
  if (abs(Ki) < 0.01) xi = 0.0;

  // erode integrated error if P+I is overdriven and clipped
//  if (abs(y) > 5.0) xiM1 *= 0.96;

  return (y);
}

//**********************************************************************
float PD_compensate(float x) // Proportional plus Derivative compensator
{
  // Cancels pole at -c, replace with a pole at -d.
  // Compensator L = (s+c)/(s+d), Implement as 1 + (c-d)/(s+d). 

  const float c = C_LEAD, d = D_LEAD, e = c-d;
  static float y = 0.0, yd = 0.0, ydM1 = 0.0; // states[n-1]
  
  yd += (e*x - d*ydM1)*TSAMP;  ydM1 = yd;
  y = x + yd;
  
  if (isClearStates) ydM1 = 0.0;

  return y;
}

//**********************************************************************
void displayAmpsVolts(void)
{   
  isDisplayDataEn = false;

  // open loop steps display
  if (tick == 0)
  Serial.print(F("\nsmpl\tacc\tvel\tVarm\tIarm\n"));
  Serial.print(tick);    Serial.print('\t');
  Serial.print(tAcc,3);  Serial.print('\t');
  Serial.print(tVel,3);  Serial.print('\t');
  Serial.print(Varm,3);  Serial.print('\t');
  Serial.print(Iarm,3);  Serial.print('\n');
  
} // displayAmpsVolts()

//**********************************************************************
void displayData(void)
{   
  isDisplayDataEn = false;

  // open loop steps display
  if (tick == 0)
    Serial.print(F("\nsmpl\tVarm\tIarm\tVdist\ttAcc\ttVel\ttPos\tcVel\tcPos\tcErrP\n"));
  Serial.print(tick);    Serial.print('\t');
  Serial.print(Varm,3);  Serial.print('\t');
  Serial.print(Iarm, 3); Serial.print('\t');
  Serial.print(Vdist,3); Serial.print('\t');
  Serial.print(tAcc,3);  Serial.print('\t');
  Serial.print(tVel,3);  Serial.print('\t');
  Serial.print(tPos,3);  Serial.print('\t');
  Serial.print(cVel,3);  Serial.print('\t');
  Serial.print(cPos,3);  Serial.print('\t');
  Serial.print(cErrP,3); Serial.print('\n');
  
} // displayData()

