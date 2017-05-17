//**********************************************************************
//**********************************************************************
//                                 MENU CODE
//**********************************************************************
//**********************************************************************
void manageMenu(void)
{
  static String strBuff = "";
  char chIn, ID, tag;
  float val;
    
  // ----- Build menu spec string one character per tick ---------
  if (Serial.available())
  {
    chIn = Serial.read();
    strBuff += chIn;
  }
 
  // ----- Extract menu fields after detecting menu EOL ----------
  if (chIn == '\r') // console carriage return detected
  {
    extractMenuFields(strBuff, ID, tag, val);
    
    // ----- Handle menu tasks -----------------------------------
    if (ID == 'h')       showHelpMenu();
    else if (ID == 'e')  toggleDriveEnable();
    else if (ID == 'u')  stepMenuTask(tag, val);
    else if (ID == 't')  trapMenuTask(tag, val);
    else if (ID == 'k')  gainMenuTask(tag, val);
    else if (ID == 'T')  isTriangleEn = true;
    else if (ID == 's')  isStepSequenceEnable = true;
    else if (ID == 'z')  zeroPosition();    
    else Serial.println(F("Invalid menu entry"));
    
  } // if (chIn == '\r')

} // manageStringMenu()


//**********************************************************************
void extractMenuFields(String &strBuff, char &ID, char &tag, float &val)
{
  int lenBuff = strBuff.length(); 
  ID =  (lenBuff > 1) ? strBuff[0] : 0;
  tag = (lenBuff > 2) ? strBuff[1] : 0;
  val = (lenBuff > 3) ? strBuff.substring(2).toFloat()+1.0e-6 
                      : float(NULL);    
  
  if (false) // debug: true false
  {
    Serial.print(F("\nmenu string = "));
    Serial.println(strBuff);
    Serial.print(F("ASCII: "));
    Serial.print(int(strBuff[0]));
    Serial.print(' ');
    Serial.println(int(strBuff[1]));
    Serial.print(F("lenBuff = ")); Serial.println(lenBuff);
    
    Serial.print(F("ID = "));
    if (ID != 0) Serial.println(ID);
    else Serial.println(F("NULL"));
    
    Serial.print(F("Tag = "));
    if (tag != 0) Serial.println(tag);
    else Serial.println(F("NULL"));
    
    Serial.print(F("Val = "));
    if (val != 0.0) Serial.println(val,4);
    else Serial.println(F("NULL"));
  }
  
  strBuff = ""; // prepare for next input
  
} // showMenuFields()

//**********************************************************************
void showHelpMenu()
{
  Serial.println(F("\nEnter h for help."));
  Serial.println(F("Enter e to toggle enable."));
  Serial.println(F("Enter z to zero position."));
  Serial.println(F("Enter u to set step reference."));
  Serial.println(F("Enter t to run trapezoid reference."));
  Serial.println(F("Enter k to set gains."));
  Serial.println(F("Enter T to run triangle reference."));
  // Serial.println(F("Enter s to run steps sequence.")); 
}

//**********************************************************************
void toggleDriveEnable()
{
  isDriveEnable = !isDriveEnable;
  if (isDriveEnable) Serial.println(F("drive ENABLED"));
  else Serial.println(F("drive DISABLED"));
}

//**********************************************************************
void zeroPosition(void)
{
  uStep = 0.0;
  tVel = 0.0;
  tPos = 0.0;
  mVel = 0.0;
  mPos = 0.0;
  isClearStates = true;
  
  isEncoderChange = true;
  grayPos = 0.0;
  Serial.println("\nSTATES ZEROED\n");
//  Serial.println();
//  displayEncoderData();
}

//**********************************************************************
void stepMenuTask(char tag, float val)
{
  //----- assign new parameter values ------
  if (val != float(NULL))
  {
    if (tag == 'v') stepSpec = val;
    if (tag == 's') stepStartTick = int(val);
    if (tag == 'r') stepRunTicks = int(val);
  }
  
  //----- initiate state action ------------
  if (tag == 'v') isStepEn = true;

  //--- display parm values to console -----
  Serial.println(F("\nSet xy to aa.bb: xyaa.bb"));
  Serial.println(F("--------------------------"));
  Serial.print(F("Step level:  uv ("));
  Serial.print(stepSpec); Serial.println(")");
  Serial.print(F("Start ticks: us ("));
  Serial.print(stepStartTick); Serial.println(")");
  Serial.print(F("Run ticks B: ur ("));
  Serial.print(stepRunTicks); Serial.println(")"); 

} // stepMenuTask()

//**********************************************************************
void trapMenuTask(char tag, float val)
{
  boolean isDisplayMenu = false;
 
  //----- assign new parameter values ------
  if (val != float(NULL))
  {
    if (tag == 'a') accSpec = val;
    if (tag == 'v') velSpec = val;
    if (tag == 'p') posSpec = val;
    if (tag == 'd') dwellSec = val;
    if (tag == 'D') distSpecV = val;
    isDisplayMenu = true; // display update
  }
  else
  {
    //----- initiate state action ------------
    if      (tag == 'f') trapEnable = TRAP_FWD_EN;
    else if (tag == 'r') trapEnable = TRAP_REV_EN;
    else if (tag == 'c') trapEnable = TRAP_CYC_EN;
    else isDisplayMenu = true; // valid tag reminder
  }

  //---- display parm values to console ------
  if (isDisplayMenu)
  {
    Serial.println(F("\ntf: forward."));
    Serial.println(F("tr: reverse."));
    Serial.println(F("tc: cycle fwd-rev."));
    Serial.println(F("tz: zero trap position."));
    Serial.println(F("--------------------------"));
    Serial.println(F("Set xy to aa.bb: xyaa.bb"));
    Serial.println(F("--------------------------"));
    Serial.print(F("accel (x/s/s): ta ("));
    Serial.print(accSpec); Serial.println(")"); 
    Serial.print(F("vel (x/s):     tv ("));
    Serial.print(velSpec); Serial.println(")");
    Serial.print(F("pos (x):       tp ("));
    Serial.print(posSpec); Serial.println(")");  
    Serial.print(F("dwell (sec):   td ("));
    Serial.print(dwellSec); Serial.println(")"); 
    Serial.print(F("Vdist (volts): tD ("));
    Serial.print(distSpecV); Serial.println(")");
  }  
} // trapMenuTask()

//**********************************************************************
void gainMenuTask(char tag, float val)
{
  //----- assign new parameter values ------
  if (tag == 'v') Kv = val;
  if (tag == 'p') Kp = val;
  if (tag == 'i') Ki = val;
  if (tag == 'a') Ka = val;
  if (tag == 'w') Kw = val;
//  if (tag == 'd') Kd = val;
//  if (tag == 'n') Kn = val;
  
  //---- display parm values to console ------
  Serial.println(F("\nSet xy to aa.bb: xyaa.bb"));
  Serial.println(F("--------------------------"));
//  Serial.print(F("Vel gain       kv: ("));
//  Serial.print(Kv, 3); Serial.println(")"); 
//  Serial.print(F("Pos gain       kp: ("));
//  Serial.print(Kp, 3); Serial.println(")");
  Serial.print(F("Intgrl gain    ki: ("));
  Serial.print(Ki); Serial.println(")"); 
  Serial.print(F("Ffwd acc gain  ka: ("));
  Serial.print(Ka, 4); Serial.println(")");
  Serial.print(F("Ffwd vel gain  kw: ("));
  Serial.print(Kw, 4); Serial.println(")");  
//  Serial.print(F("Deriv gain     kd: ("));
//  Serial.print(Kd, 2); Serial.println(")"); 
//  Serial.print(F("PID pole       kn: ("));
//  Serial.print(Kn, 2); Serial.println(")");   
} // gainMenuTask()

//**********************************************************************
void displayModelParms()
{ 
  Serial.print(F("\nKb (mV/rps) = ")); Serial.println(1000.0*K_EMF);
  Serial.print(F("Kt (mN-m/A = ")); Serial.println(1000.0*K_TORQUE);
  Serial.print(F("Rarm (ohms) = ")); Serial.println(R_ARM);
  Serial.print(F("Nmtr2ld = ")); Serial.println(N_MTR2LD);
  Serial.print(F("Dsys (uN-m/r/s) = ")); Serial.println(1e6*D_SYS);
  Serial.print(F("Jsys (ukg-m^2) = ")); Serial.println(1e6*J_SYS);
  Serial.print(F("Tdist (mN-m) = ")); Serial.println(1e3*T_DIST);
  Serial.print(F("Vdist (volts) = ")); Serial.println(V_DIST,3);
  
  Serial.print(F("\nSYS_A = ")); Serial.println(SYS_A);
  Serial.print(F("SYS_B = ")); Serial.println(SYS_B);
  Serial.print(F("TAU = ")); Serial.println(1.0/SYS_B);
  Serial.print(F("DCgain = ")); Serial.println(SYS_A/SYS_B); 

  Serial.print(F("\nTARGET WN = ")); Serial.println(WN); 
  Serial.print(F("TARGET ZETA = ")); Serial.println(ZETA); 
  Serial.print(F("KP_LEAD = ")); Serial.println(KP_LEAD); 
  Serial.print(F("KV_CASC = ")); Serial.println(KV_CASC); 
  Serial.print(F("KP_CASC = ")); Serial.println(KP_CASC);
}
