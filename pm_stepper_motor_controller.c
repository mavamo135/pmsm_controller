//###########################################################################
// FILE:   pm_stepper_motor_controller.c
// TITLE:  Adaptive controller for a Permanent Magnet Stepper Motor
//###########################################################################
// $TI Release: F2837xS Support Library v190 $
// $Release Date: Mon Feb  1 16:59:09 CST 2016 $
// $Copyright: Copyright (C) 2014-2016 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

#include "F28x_Project.h"     // Device Headerfile and Examples Include File
#include <math.h>

void SelectGPIO(void);
void ConfigureADC(void);
void ConfigureEPWM(void);
void ConfigureEPWM7(void);
void ConfigureEPWM9(void);
void ConfigureEQEP1(void);
void SetupADCEpwm(Uint16 channel);
void SetPWMA(float);
void SetPWMB(float);
float CalcSpeed(float);
float CalcPosition(void);
float CalcPosDesired(float);
float CalcSpeedDesired(float);
float CalcAcelDesired(float);
float CalcDAcelDesired(float);
float CalcIntSigma2(float);
float CalcIntSigma5(float);
__interrupt void adca1_isr(void);
__interrupt void adcb1_isr(void);
__interrupt void cpu_timer0_isr(void);
//////////////////////////////////////////////////						//////////////////////////////////////////////////
//////////////////////////////////////////////////   Controller Gains	//////////////////////////////////////////////////
//////////////////////////////////////////////////						//////////////////////////////////////////////////
#define TEST 0
#define Kp 0.5	 //1
#define Kd 0.01  //0.01
#define AlphaA 9 //9
#define AlphaB 9 //9
#define Gamma2 1 //1
#define Gamma5 1 //1
#define GammakP 0.5 //0.5
#define GammakA 0.5 //0.5
#define gamma 9		//9
#define N 3
//////////////////////////////////////////////////						//////////////////////////////////////////////////
//////////////////////////////////////////////////   System Constants	//////////////////////////////////////////////////
//////////////////////////////////////////////////						//////////////////////////////////////////////////
#define R 5.0					// Phase Winding Resistance (Ohm)
#define L 0.006					// Phase Winding Inductance (H)
#define km 0.1  				// Motor Torque Constant (N*m/A) 0.15
#define J 0.0001872				// Rotor Inertia (kg*m^2)
#define b 0.002					// Rotor Damping (N*m/(rad/s))
#define Nr 50					// Number of teeth
#define np 8					// Number of poles
#define Ts 0.001
#define iTs 1/Ts
#define Vmax 12
#define tf 10
#define JkmI 1/(J*km)
#define kmI 1/km
#define VmaxI 1/(Vmax) //+0.5
//////////////////////////////////////////////////						//////////////////////////////////////////////////
//////////////////////////////////////////////////     uC Constants	    //////////////////////////////////////////////////
//////////////////////////////////////////////////						//////////////////////////////////////////////////
#define EPWM7_TIMER_TBPRD  5000	// Period Register 10kHz
#define EPWM7_CMPA     5000	    // 0 = 100% Duty Cycle; TBPRD = 0% Duty Cycle
#define EPWM7_DB   0x007F		// PWM Dead Band
#define EPWM9_TIMER_TBPRD  5000 // Period Register 10kHz
#define EPWM9_CMPA     5000		// 0 = 100% Duty Cycle; TBPRD = 0% Duty Cycle
#define EPWM9_DB   0x007F		// PWM Dead Band
#define RESULTS_BUFFER_SIZE 5000
//////////////////////////////////////////////////						//////////////////////////////////////////////////
//////////////////////////////////////////////////  System Variables    //////////////////////////////////////////////////
//////////////////////////////////////////////////						//////////////////////////////////////////////////
#pragma DATA_SECTION(ThetaArray, "SVArray")
#pragma DATA_SECTION(DThetaArray, "SVArray")
#pragma DATA_SECTION(IaArray, "SVArray")
#pragma DATA_SECTION(IbArray, "SVArray")
#pragma DATA_SECTION(VaArray, "SVArray")
#pragma DATA_SECTION(VbArray, "SVArray")
float ThetaArray[RESULTS_BUFFER_SIZE], DThetaArray[RESULTS_BUFFER_SIZE], IaArray[RESULTS_BUFFER_SIZE];
float IbArray[RESULTS_BUFFER_SIZE], VaArray[RESULTS_BUFFER_SIZE], VbArray[RESULTS_BUFFER_SIZE];
float time=0;
float Va=0, Vb=0, Ia=0, Ib=0, IaD=0, IbD=0;
float Theta=0, ThetaD=0, DTheta=0, DThetaD=0, DDThetaD=0, DDDThetaD=0, Tau=0;
float Sigma2=0, Sigma5=0;
static float gammakP[N]={0,0,0}, gammakA[N]={0,0,0};
int index=0, load=0;

void main(void){
	// Initialize System Control: PLL, WatchDog, enable Peripheral Clocks.
	// This example function is found in the F2837xS_SysCtrl.c file.
    InitSysCtrl();
    // Initialize GPIO:This example function is found in the F2837xS_Gpio.c file
    InitGpio();
    SelectGPIO();
    // Enable PWM7 and PWM9
    CpuSysRegs.PCLKCR2.bit.EPWM7 = 1;
    CpuSysRegs.PCLKCR2.bit.EPWM9 = 1;
    // Clear all interrupts and initialize PIE vector table: Disable CPU interrupts
    DINT;
    // Initialize the PIE control registers to their default state. The default state is all PIE interrupts disabled
    // and flags are cleared. This function is found in the F2837xS_PieCtrl.c file.
    InitPieCtrl();
    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;
    // Initialize the PIE vector table with pointers to the shell Interrupt Service Routines (ISR).
    // This will populate the entire table, even if the interrupt is not used in this example.
    // The shell ISR routines are found in F2837xS_DefaultIsr.c. This function is found in F2837xS_PieVect.c.
    InitPieVectTable();
    // Map ISR functions
    EALLOW;
    PieVectTable.ADCA1_INT = &adca1_isr; // Function for ADCA interrupt 1
    PieVectTable.ADCB1_INT = &adcb1_isr; // Function for ADCB interrupt 1
    PieVectTable.TIMER0_INT = &cpu_timer0_isr; // Function for Timer0 Interrupt
    EDIS;
    InitCpuTimers(); // Basic setup CPU Timer0, 1 and 2
    ConfigCpuTimer(&CpuTimer0, 200, iTs); // CPU - Timer0 at 1 milisecond
    StopCpuTimer0();
    ConfigureADC();
    ConfigureEPWM();
    ConfigureEPWM7();
    ConfigureEPWM9();
    ConfigureEQEP1();
    SetupADCEpwm(0);// Setup the ADC for ePWM triggered conversions on channel 0
    // Enable global Interrupts and higher priority real-time debug events:
    IER |= M_INT1; // Enable group 1 interrupts
    EINT;  // Enable Global interrupt INTM
    //ERTM;  // Enable Global realtime interrupt DBGM
    //Initialize results buffer
	for(index = 0; index < RESULTS_BUFFER_SIZE; index++)
	{
		ThetaArray[index] = 0;
		DThetaArray[index] = 0;
		IaArray[index] = 0;
		IbArray[index] = 0;
		VaArray[index] = 0;
		VbArray[index] = 0;
	}
	index = 0;
	// Enable PIE interrupt
	PieCtrlRegs.PIEIER1.bit.INTx1 = 1; // ADC A Interrupt
	PieCtrlRegs.PIEIER1.bit.INTx2 = 1; // ADC B Interrupt
	PieCtrlRegs.PIEIER1.bit.INTx7 = 1; // Timer 0 Interrupt
	// Sync ePWM
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    StartCpuTimer0(); // CpuTimer0Regs.TCR.bit.TSS = 0; // Start timer0
    // Start ePWM
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;  // Enable SOCA
	EPwm1Regs.TBCTL.bit.CTRMODE = 0; // Unfreeze, and enter up count mode
	EDIS;
    do{
    	asm(" NOP");
    }while(1);
}
void SelectGPIO(void){
	GPIO_SetupPinMux(10, GPIO_MUX_CPU1, 5); //EQEP1A
	GPIO_SetupPinOptions(10, GPIO_INPUT, GPIO_SYNC);
	GPIO_SetupPinMux(11, GPIO_MUX_CPU1, 5); //EQEP1B
	GPIO_SetupPinOptions(11, GPIO_INPUT, GPIO_SYNC);
	GPIO_SetupPinMux(13, GPIO_MUX_CPU1, 0); //LED
	GPIO_SetupPinOptions(13, GPIO_OUTPUT, GPIO_ASYNC);
	GPIO_SetupPinMux(12, GPIO_MUX_CPU1, 1); //PWM7A
	GPIO_SetupPinOptions(12, GPIO_OUTPUT, GPIO_ASYNC);
	GPIO_SetupPinMux(15, GPIO_MUX_CPU1, 0); //GPIO15-DirectionA
	GPIO_SetupPinOptions(15, GPIO_OUTPUT, GPIO_ASYNC);
	GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 5); //PWM9A
	GPIO_SetupPinOptions(16, GPIO_OUTPUT, GPIO_ASYNC);
	GPIO_SetupPinMux(17, GPIO_MUX_CPU1, 0); //GPIO17-DirectionB
	GPIO_SetupPinOptions(17, GPIO_OUTPUT, GPIO_ASYNC);
	//GpioDataRegs.GPATOGGLE.bit.GPIO13 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO13 = 1;
}
//Write ADC configurations and power up the ADC for both ADC A and ADC B
void ConfigureADC(){
	EALLOW;
	//write configurations
	AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
	AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
	//Set pulse positions to late
	AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
	AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
	//power up the ADC
	AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
	AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
	//delay for 1ms to allow ADC time to power up
	DELAY_US(1000);
	EDIS;
}

void ConfigureEPWM(){
	EALLOW;
	// Assumes ePWM clock is already enabled
	EPwm1Regs.ETSEL.bit.SOCAEN	= 0;	        // Disable SOC on A group
	EPwm1Regs.ETSEL.bit.SOCASEL	= 4;	        // Select SOC on up-count
	EPwm1Regs.ETPS.bit.SOCAPRD = 1;		        // Generate pulse on 1st event
	EPwm1Regs.CMPA.bit.CMPA = 0x6096;           // Set compare A value to 2048 counts 0x0800
	EPwm1Regs.TBPRD = 0xC12C;			        // Set period to 4096 counts 0x1000
	EPwm1Regs.TBCTL.bit.CTRMODE = 3;            // freeze counter
	EDIS;
}

void ConfigureEPWM7(){
	EALLOW;
	EPwm7Regs.TBPRD = EPWM7_TIMER_TBPRD;           // Set timer period
	EPwm7Regs.TBPHS.bit.TBPHS = 0x0000;            // Phase is 0
	EPwm7Regs.TBCTR = 0x0000;                      // Clear counter
	EPwm7Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
	EPwm7Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
	EPwm7Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
	EPwm7Regs.TBCTL.bit.CLKDIV = TB_DIV1;          // Slow so we can observe on the scope
	EPwm7Regs.CMPA.bit.CMPA = EPWM7_CMPA;
	EPwm7Regs.AQCTLA.bit.CAU = AQ_SET;             // Set PWM3A on Zero
	EPwm7Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	// Active high complementary PWMs - Setup the deadband
	EPwm7Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
	EPwm7Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
	EPwm7Regs.DBCTL.bit.IN_MODE = DBA_ALL;
	EPwm7Regs.DBRED.bit.DBRED = EPWM7_DB;
	EPwm7Regs.DBFED.bit.DBFED = EPWM7_DB;
	EDIS;
}

void ConfigureEPWM9(){
	EALLOW;
    EPwm9Regs.TBPRD = EPWM9_TIMER_TBPRD;           // Set timer period
    EPwm9Regs.TBPHS.bit.TBPHS = 0x0000;            // Phase is 0
    EPwm9Regs.TBCTR = 0x0000;                      // Clear counter
    EPwm9Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
    EPwm9Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm9Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm9Regs.TBCTL.bit.CLKDIV = TB_DIV1;          // Slow so we can observe on the scope
    EPwm9Regs.CMPA.bit.CMPA = EPWM9_CMPA;
    EPwm9Regs.AQCTLA.bit.CAU = AQ_SET;             // Set PWM3A on Zero
    EPwm9Regs.AQCTLA.bit.CAD = AQ_CLEAR;
    // Active high complementary PWMs - Setup the deadband
    EPwm9Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm9Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm9Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm9Regs.DBRED.bit.DBRED = EPWM9_DB;
    EPwm9Regs.DBFED.bit.DBFED = EPWM9_DB;
    EDIS;
}

void ConfigureEQEP1(){
	EALLOW;
    EQep1Regs.QUPRD=2000000;         // Unit Timer for 100Hz at 200 MHz SYSCLKOUT
    EQep1Regs.QDECCTL.bit.QSRC=00;      // QEP quadrature count mode
    EQep1Regs.QEPCTL.bit.FREE_SOFT=2;
    EQep1Regs.QEPCTL.bit.PCRM=00;       // PCRM=00 mode - QPOSCNT reset on index event
    EQep1Regs.QEPCTL.bit.UTE=1;         // Unit Timeout Enable
    EQep1Regs.QEPCTL.bit.QCLM=1;        // Latch on unit time out
    EQep1Regs.QPOSMAX=0xffffffff;
    EQep1Regs.QEPCTL.bit.QPEN=1;        // QEP enable
    EQep1Regs.QCAPCTL.bit.UPPS=0;       // 1/32 for unit position  5
    EQep1Regs.QCAPCTL.bit.CCPS=0;       // 1/64 for CAP clock   6
    EQep1Regs.QCAPCTL.bit.CEN=1;        // QEP Capture Enable
    EDIS;
}

void SetupADCEpwm(Uint16 channel){
	Uint16 acqps;
	//determine minimum acquisition window (in SYSCLKS) based on resolution
	if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION){
		acqps = 14; //75ns
	}
	else { //resolution is 16-bit
		acqps = 63; //320ns
	}
	//Select the channels to convert and end of conversion flag
	EALLOW;
	AdcaRegs.ADCSOC0CTL.bit.CHSEL = channel;  //SOC0 will convert pin A0
	AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
	AdcbRegs.ADCSOC0CTL.bit.CHSEL = channel;  //SOC0 will convert pin B0
	AdcbRegs.ADCSOC0CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
	AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5; //trigger on ePWM1 SOCA/C. 01h ADCTRIG1 - CPU1 Timer 0, TINT0n
	AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 5; //trigger on ePWM1 SOCA/C
	AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //end of SOC0 will set INT1 flag
	AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
	AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
	AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //end of SOC0 will set INT1 flag
	AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
	AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
	EDIS;
}

void SetPWMA(float V){
	if (V>=0){GpioDataRegs.GPASET.bit.GPIO15 = 1;}
	else{GpioDataRegs.GPACLEAR.bit.GPIO15 = 1;}
	V = abs(V);
	if (V>Vmax) {V=Vmax;}
	V = V*EPwm7Regs.TBPRD*VmaxI;
	EPwm7Regs.CMPA.bit.CMPA = EPwm7Regs.TBPRD-V;
}

void SetPWMB(float V){
	if (V>=0){GpioDataRegs.GPASET.bit.GPIO17 = 1;}
	else{GpioDataRegs.GPACLEAR.bit.GPIO17 = 1;}
	V = abs(V);
	if (V>Vmax) {V=Vmax;}
	V = V*EPwm9Regs.TBPRD*VmaxI;
	EPwm9Regs.CMPA.bit.CMPA = EPwm9Regs.TBPRD-V;
}

__interrupt void adca1_isr(void){
	Ia = AdcaResultRegs.ADCRESULT0*0.000791452315L; //0.002137 R=1k
	AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

__interrupt void adcb1_isr(void){
	Ib = AdcbResultRegs.ADCRESULT0*0.000791452315L;
	AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

__interrupt void cpu_timer0_isr(void){
	static float Sigma2D=0, Sigma5D=0;						// Integral terms of current loops
	static float ha=0, hb=0;								//
	static float gammakPD[N]={0,0,0}, gammakAD[N]={0,0,0}; 	// Derivated torque ripple estimators
	static float seno=0, cose=0, S[N]={0,0,0}, C[N]={0,0,0};// Sine and cosine
	static float IaT=0, IbT=0; 								// Currents errors
	static float ThetaT=0, DThetaT=0;						// Position and speed errors
	float foo=0, sum=0, sum1=0, sum2=0, aux1=0, aux2=0, aux3=0;
	int i=0;
	//////////////////////////////////////////////////						//////////////////////////////////////////////////
	//////////////////////////////////////////////////		Variables		//////////////////////////////////////////////////
	//////////////////////////////////////////////////						//////////////////////////////////////////////////
	if (Va<0) Ia = -Ia;
	if (Vb<0) Ib = -Ib;
	time += Ts;
	Theta = CalcPosition();
	ThetaD = CalcPosDesired(time);
	DTheta = CalcSpeed(Theta);
	DThetaD = CalcSpeedDesired(ThetaD);
	DDThetaD = CalcAcelDesired(DThetaD);
	DDDThetaD = CalcDAcelDesired(DDThetaD);
	seno = sin(Nr*Theta);
	cose = cos(Nr*Theta);
	for (i=0; i<N; i++){
		S[i] = sin((i+1)*np*Theta);
		C[i] = cos((i+1)*np*Theta);
	}
	//////////////////////////////////////////////////						//////////////////////////////////////////////////
	//////////////////////////////////////////////////		Controller		//////////////////////////////////////////////////
	//////////////////////////////////////////////////						//////////////////////////////////////////////////
	ThetaT = Theta-ThetaD;
	DThetaT = DTheta-DThetaD;
	sum = 0;
	for (i=0; i<N; i++){
		sum = sum+(gammakP[i]*C[i]+gammakA[i]*S[i]);
	}
	Tau = -Kp*ThetaT-Kd*DThetaT+sum+J*DDThetaD;
	IaD = -Tau*seno*kmI;
	IbD = Tau*cose*kmI;
	IaT = Ia-IaD;
	IbT = Ib-IbD;
	Sigma2D = -Gamma2*IaT*Tau*DTheta*cose;
	Sigma5D = -Gamma5*IbT*Tau*DTheta*seno;
	Sigma2 = CalcIntSigma2(Sigma2D)*Tau*DTheta;
	Sigma5 = CalcIntSigma5(Sigma5D)*Tau*DTheta;
	sum1 = 0;
	sum2 = 0;
	for (i=0; i<N; i++){
		aux1 = gammakAD[i]*S[i]+gammakA[i]*(i+1)*np*DTheta*C[i];
		aux2 = gammakPD[i]*C[i]+gammakP[i]*(i+1)*np*DTheta*S[i];
		aux3 = gammakPD[i]*cose-gammakP[i]*Nr*DTheta*seno;
		sum1 = sum1+(aux1+aux2);
		sum2 = sum2+(aux1+aux3);
	}
	ha = -L*kmI*(sum1+J*DDDThetaD)*seno;
	hb = L*kmI*(sum2+J*DDDThetaD)*cose;
	if (TEST == 1){
		Va = -sin(100*time)*Vmax; // 1500
		Vb = cos(100*time)*Vmax;
	}
	else{
		Va = -AlphaA*IaT+Sigma2*cose+R*IaD-km*DThetaD*seno+ha;
		Vb = -AlphaB*IbT+Sigma5*seno+R*IbD+km*DThetaD*cose+hb;
	}
	foo = gamma*ThetaT+DThetaT-L*Kd*JkmI*IaT*seno+L*Kd*JkmI*IbT*cose;
	for (i=0; i<N; i++){
		gammakPD[i] = -GammakP*foo*C[i];
		gammakAD[i] = -GammakA*foo*S[i];
		gammakP[i] = gammakP[i]+gammakPD[i]*Ts;
		gammakA[i] = gammakA[i]+gammakAD[i]*Ts;
	}
	//////////////////////////////////////////////////						//////////////////////////////////////////////////
	//////////////////////////////////////////////////  Controller Output   //////////////////////////////////////////////////
	//////////////////////////////////////////////////						//////////////////////////////////////////////////
	if (time>=tf){
		GpioDataRegs.GPASET.bit.GPIO15 = 1;
		GpioDataRegs.GPASET.bit.GPIO17 = 1;
		SetPWMA(0);
		SetPWMB(0);
		//GpioDataRegs.GPATOGGLE.bit.GPIO13 = 1;
		StopCpuTimer0();
		asm(" ESTOP0");
	}
	else{
		SetPWMA(Va);
		SetPWMB(Vb);
	}
	//////////////////////////////////////////////////						//////////////////////////////////////////////////
	//////////////////////////////////////////////////     Data Arrays	    //////////////////////////////////////////////////
	//////////////////////////////////////////////////						//////////////////////////////////////////////////
	if (load == 0){
		ThetaArray[index] = Theta;
		DThetaArray[index] = DTheta;
		IaArray[index] = Ia;
		IbArray[index] = Ib;
		VaArray[index] = Va;
		VbArray[index++] = Vb;
		load++;
	}
	else {load = 0;}
	GpioDataRegs.GPATOGGLE.bit.GPIO13 = 1;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

float CalcSpeed(float T){
	static float Theta_1=0, Theta_2=0, DTheta_1=0, DTheta_2=0;
	float DTheta=0;
	DTheta = 10*T-20*Theta_1+10*Theta_2+1.99*DTheta_1-0.99*DTheta_2;
	Theta_2 = Theta_1;
	Theta_1 = T;
	DTheta_2 = DTheta_1;
	DTheta_1 = DTheta;
	return DTheta;
}

float CalcPosition(void){
	long Counts=0;
	float T=0;
	Counts = -EQep1Regs.QPOSCNT; // Position in Counts 40000(counts)=2pi(rad)
	T = Counts*0.0001570796327L; // Position in Rad
	return T;
}

float CalcPosDesired(float t){
	float foo=0, trajectory=0;
	foo = t*t*t;
	//trajectory = foo*((0.00037699L*t*t)-(0.0094248L*t)+0.062832L); //Desired Position (trajectory)
	//trajectory = foo*((0.00075398223686L*t*t)-(0.018849555921539L*t)+0.125663706143592L);
	trajectory = foo*((0.001884955592154L*t*t)-(0.047123889803847L*t)+0.314159265358979L);
	return trajectory;
}

float CalcSpeedDesired(float trajectory){
	static float pastTrajectory=0;
	float DTrajectory=0;
	DTrajectory = (trajectory-pastTrajectory)*iTs; //Speed in rad/s
	pastTrajectory = trajectory;
	return DTrajectory;
}

float CalcAcelDesired(float Speed){
	static float Speed_1=0, Speed_2=0, DSpeed_1=0, DSpeed_2=0;
	float DSpeed=0;
	DSpeed = 10*Speed-20*Speed_1+10*Speed_2+1.99*DSpeed_1-0.99*DSpeed_2;
	Speed_2 = Speed_1;
	Speed_1 = Speed;
	DSpeed_2 = DSpeed_1;
	DSpeed_1 = DSpeed;
	return DSpeed;
}

float CalcDAcelDesired(float Acel){
	static float Acel_1=0, Acel_2=0, DAcel_1=0, DAcel_2=0;
	float DAcel=0;
	DAcel = 10*Acel-20*Acel_1+10*Acel_2+1.99*DAcel_1-0.99*DAcel_2;
	Acel_2 = Acel_1;
	Acel_1 = Acel;
	DAcel_2 = DAcel_1;
	DAcel_1 = DAcel;
	return DAcel;
}

float CalcIntSigma2(float SigmaD){
	static float Sigma=0; //SigmaD_1=0;
	Sigma = Sigma+(SigmaD)*Ts;//*0.5L (SigmaD+SigmaD_1)
	//SigmaD_1 = SigmaD;
	return Sigma;
}

float CalcIntSigma5(float SigmaD){
	static float Sigma=0; //SigmaD_1=0;
	Sigma = Sigma+(SigmaD)*Ts;//*0.5L
	// SigmaD_1 = SigmaD;
	return Sigma;
}
