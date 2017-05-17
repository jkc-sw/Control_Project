// Controller connectivity
const int GC0 = 4;    // PD4 PCINT20 digital output
const int GC1 = 5;    // PD5 PCINT21 digital output
const int GC2 = 6;    // PD6 PCINT22 digital output
const int GC3 = 7;    // PD7 PCINT23 digital output
const int PWMOD = 10; // PB2 OC1B PWM digital output
const int nEN = 11;   // PB3 digital output
const int TP0 = 12;   // PB5 digital output
const int ALED = 13;  // PB5 digital output
const int VTACH = A5; // analog input

const byte  TSAMP_MSEC = 10;
const float TSAMP = 0.001*TSAMP_MSEC;
const int T1_TOP = 20000-1; // 20000*0.5usec = 10msec

volatile boolean isSample = false;  // set true by Timer1 ISR to start sample
volatile boolean isEncoderChange = false;//, isEncoderSych = false;
volatile byte grayCode;
volatile float grayPos = 0.0;

boolean isDisplayDataEn = false;
boolean isStepEn = false;
boolean isTriangleEn = false;
enum{TRAP_NOT_EN, TRAP_FWD_EN, TRAP_REV_EN, TRAP_CYC_EN};
byte trapEnable = TRAP_NOT_EN;
boolean isDriveEnable = false;
boolean isClearStates = false;
boolean isStepSequenceEnable = false;

unsigned long tick = 0;
float uStep, stepSpec;
int stepStartTick, stepRunTicks;
float trnglLvl, stepSeriesLvl;
float accSpec, velSpec, posSpec, dwellSec, distSpecV;
float tAcc, tVel, tPos;
int pwmCount = 0, adcCount;
float Vin, Vdrive, Vffwd, Vctl, Vdist, Vstic, Iarm, Varm;
float mPos = 0.0, mVel = 0.0, cPos, cVel;
float ref, errV, errP, cErrV, cErrP, cErrP_AbsPk, cErrP_SqAccum;
float Kf, Ka, Kw, Kv, Kp, Ki;

