// Motor constants: Premotech 9904 120-16 111
const float K_EMF = (3.6/1000.0)*60.0/TWO_PI; // V/rad/sec
const float K_TORQUE = 3.5/100.0; // N-m/Amp
const float R_ARM = 6.2; // ohms
const float RPS_NO_LOAD = (3200.0/60.0)*TWO_PI; // rad/sec
const float AMPS_NO_LOAD = 0.030; // amps
const float D_MTR = (K_TORQUE*AMPS_NO_LOAD)/RPS_NO_LOAD;

// Module constants
const float K_AMP = 2.0;
const float J_ROTOR = 0.039*1e-4; // kg-m^2
const float R_DISK = 0.5*76.0e-3, L_DISK = 3.0e-3; // meters
const float J_BRK = 0.5*PI*L_DISK*2700*pow(R_DISK,4.0); // kg-m^2
const float J_ENC = 0.5*PI*L_DISK*1200*pow(R_DISK,4.0); // kg-m^2
const float R_PUL = 1.5/2.0/39.4, L_PUL = 0.5/39.4; // meters
const float J_PUL = 0.5*PI*L_PUL*2700*pow(R_PUL,4.0); // kg-m^2
const float J_MTR = 2*J_ROTOR + J_BRK + J_ENC; // kg-m^2
const float J_MOD = J_MTR + J_PUL/3/3 + J_PUL/9/9; // kg-m^2

// Load constants
const int   N_MTR2LD = 3; // motor to load speed reduction
const float R_HUB = 0.0147; // meters pulley hub
const float M_LOAD = 0.5; // kg
const float M_CWGHT = 0.5; // kg
const float J_LOAD = (M_LOAD + M_CWGHT)*R_HUB*R_HUB;
const float T_DIST = 9.8*(M_LOAD - M_CWGHT)*R_HUB; // N-m

// System rollup: reflect J, D, Tdist to motor
const float J_SYS = J_MOD + J_LOAD/(N_MTR2LD*N_MTR2LD);
const float D_SYS = 2*D_MTR; // motor + tach
const float V_DIST = (R_ARM/(K_TORQUE*K_AMP))*(9.8*(0.25)*R_HUB/N_MTR2LD); // at motor;

// Module model: Vin to motor and load
const float SYS_A = K_AMP*K_TORQUE/(J_SYS*R_ARM);
const float SYS_B = (K_TORQUE*K_EMF + D_SYS*R_ARM)/(J_SYS*R_ARM);
const float TAU = 1.0/SYS_B;
const float DC_GAIN = SYS_A/SYS_B;
const float LDMM_2_MRAD = N_MTR2LD / (1000.0 * R_HUB);
const float MRAD_2_LDMM = 1.0 / LDMM_2_MRAD;

// Position controller gains
const float WN = 10.0;
const float ZETA = 0.6;
const float SIGMA = ZETA*WN;
const float KI_PI = SYS_B; // PI velocity loop
const float KV_PI = 2*SIGMA/SYS_A;
const float C_LEAD = SYS_B;
const float D_LEAD = 2*SIGMA;
const float KP_LEAD = WN*WN/SYS_A;
//const float KV_PCASC = (2.0*SIGMA-SYS_B)/SYS_A;
//const float KP_PCASC = WN*WN/(KV_PCASC*SYS_A);
//const float KV_PICASC = 2.0*SIGMA/SYS_A;
//const float KP_PICASC = (WN*WN)/(KV_PICASC*SYS_A);

const float KV_CASC = (2.0*SIGMA-SYS_B/2.0)/SYS_A;
const float KP_CASC = (WN*WN)/(KV_CASC*SYS_A);

