//###########################################################################
// Description:
//
// ADC input to control sine PWM output
// Written by Michael Hatzikokolakis and Michelle Pham
//
//
// This configures ePWM3A for:
// - Up count
// - Period starts at 2 and goes up to 1000
// - Toggle output on PRD
//
// External Connections
// - eCap1 is on GPIO24
// - ePWM3A is on GPIO4
// - Connect GPIO4 to GPIO24.
//
// Watch Variables
// - ECap1IntCount - Successful captures
// - ECap1PassCount - Interrupt counts
//
//###########################################################################

#include "DSP28x_Project.h"     // Device Headerfile

// ADC start parameters
#if (CPU_FRQ_150MHZ)     // Default - 150 MHz SYSCLKOUT
  #define ADC_MODCLK 0x3 // HSPCLK = SYSCLKOUT/2*ADC_MODCLK2 = 150/(2*3)   = 25.0 MHz
#endif
#if (CPU_FRQ_100MHZ)
  #define ADC_MODCLK 0x2 // HSPCLK = SYSCLKOUT/2*ADC_MODCLK2 = 100/(2*2)   = 25.0 MHz
#endif
#define ADC_CKPS   0x1   // ADC module clock = HSPCLK/2*ADC_CKPS   = 25.0MHz/(1*2) = 12.5MHz
#define ADC_SHCLK  0xf   // S/H width in ADC module periods                        = 16 ADC clocks
#define AVG        1000  // Average sample limit
#define ZOFFSET    0x00  // Average Zero offset
#define BUF_SIZE   2048  // Sample buffer size


// Configure the start/end period for the timer
#define PWM3_TIMER_MIN     1000
#define PWM3_TIMER_MAX     8000

// Prototype statements for functions found within this file.
__interrupt void ecap1_isr(void);
void InitECapture(void);
void InitEPwmTimer(void);
void Fail(void);

// Global variables
Uint32  ECap1IntCount;
Uint32  ECap1PassCount;
Uint32  EPwm3TimerDirection;
Uint16 SampleTable[BUF_SIZE/2];
Uint16 SampleTable2[BUF_SIZE/2];

// To keep track of which way the timer value is moving
#define EPWM_TIMER_UP   1
#define EPWM_TIMER_DOWN 0

void main(void)
{
    Uint16 i;

// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
   InitSysCtrl();
   EALLOW;
   SysCtrlRegs.HISPCP.all = ADC_MODCLK; // HSPCLK = SYSCLKOUT/ADC_MODCLK
   EDIS;
   
   
// Step 2. Initialize GPIO:
   InitEPwm3Gpio();
   InitECap1Gpio();

   
// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
   DINT;
   
// Initialize the PIE control registers to their default state.
   InitPieCtrl();
   
// Disable CPU interrupts and clear all CPU interrupt flags:
   IER = 0x0000;
   IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines 
   InitPieVectTable();
   EALLOW;
   PieVectTable.ECAP1_INT = &ecap1_isr;
   EDIS;

   
// Step 4. Initialize all the Device Peripherals:

   InitEPwmTimer();
   InitECapture();
   InitAdc();
   AdcRegs.ADCTRL1.bit.ACQ_PS = ADC_SHCLK;
   AdcRegs.ADCTRL3.bit.ADCCLKPS = ADC_CKPS;
   AdcRegs.ADCTRL1.bit.SEQ_CASC = 1;        // 1 = Cascaded mode
   AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x0;
   AdcRegs.ADCTRL1.bit.CONT_RUN = 1;       // Setup continuous run

   
// Step 5. User specific code, enable interrupts:
// Initialize counters:
   ECap1IntCount = 0;
   ECap1PassCount = 0;

// Enable CPU INT4 which is connected to ECAP1-4 INT:
   IER |= M_INT4;

// Enable eCAP INTn in the PIE: Group 3 interrupt 1-6
   PieCtrlRegs.PIEIER4.bit.INTx1 = 1;

// Enable global interrupts and higher priority real-time debug events:
   EINT;   // Enable Global interrupt INTM
   ERTM;   // Enable Global realtime interrupt DBGM

   unsigned long int MaxValue;

// Clear SampleTable
   for (i=0; i<BUF_SIZE; i++)
   {
     SampleTable[i] = 0;
   }

   for (i=0; i<BUF_SIZE; i++)
   {
     SampleTable2[i] = 0;
   }

   // Start SEQ1
   AdcRegs.ADCTRL2.all = 0x2000;

// Step 6
   // Take ADC data and log in SampleTable array
      int max_value = 4000;
      int half_max = 2000;
      int ADC = 0;
      int ADC_squared = 0;
      volatile double percent;
      //int period = 45000;

      for (;;)
      {
        MaxValue = 0;

        for (i=0; i<AVG; i++)
        {
           while (AdcRegs.ADCST.bit.INT_SEQ1 == 0) {} // Wait for interrupt
           AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;
           ADC = AdcRegs.ADCRESULT0>>4;
           if (ADC < half_max)
               ADC = half_max - ADC;
           else
               ADC -= half_max;
           SampleTable[i] = ADC;
           // ADC_squared = (ADC * ADC);
           // ADC = ADC_squared / ADC

           percent = ADC / (half_max / 100);
           SampleTable2[i] = percent;
           EPwm3Regs.CMPA.half.CMPA = percent * 45000;
           //SampleTable[i] = percent * 45000;

           // SampleTable[i] =((AdcRegs.ADCRESULT0>>4) );
           //if (SampleTable[i] > MaxValue)
           //    MaxValue = SampleTable[i];
        }
      }
}

void InitEPwmTimer()
{
	EPwm3Regs.TBCTL.bit.CTRMODE = 0;//asymmetrical upcount
    EPwm3Regs.TBPRD = 45000; // Period = 45001 TBCLK counts assumes sysclkout=tbclk=90MHz
    EPwm3Regs.TBCTL.bit.PHSEN = 0; // Phase loading disabled
    EPwm3Regs.TBPHS.half.TBPHS = 0; // Set Phase register to zero
    EPwm3Regs.TBCTR=0;//clear counter
    EPwm3Regs.TBCTL.bit.HSPCLKDIV=0;//sysclk/1
    EPwm3Regs.TBCTL.bit.CLKDIV=1;//sysclk/2

    EPwm3Regs.CMPA.half.CMPA =   4500; //duty cycle
    EPwm3Regs.AQCTLA.bit.ZRO = 2; //EPWMA output high when counter equals the period
    EPwm3Regs.AQCTLA.bit.CAU = 1; //EPWMA output low when counter equals CMPA on up count
}

void InitECapture()
{
   ECap1Regs.ECEINT.all = 0x0000;             // Disable all capture interrupts
   ECap1Regs.ECCLR.all = 0xFFFF;              // Clear all CAP interrupt flags
   ECap1Regs.ECCTL1.bit.CAPLDEN = 0;          // Disable CAP1-CAP4 register loads
   ECap1Regs.ECCTL2.bit.TSCTRSTOP = 0;        // Make sure the counter is stopped

   // Configure peripheral registers
   ECap1Regs.ECCTL2.bit.CONT_ONESHT = 1;      // One-shot
   ECap1Regs.ECCTL2.bit.STOP_WRAP = 3;        // Stop at 4 events
   ECap1Regs.ECCTL1.bit.CAP1POL = 1;          // Falling edge
   ECap1Regs.ECCTL1.bit.CAP2POL = 0;          // Rising edge
   ECap1Regs.ECCTL1.bit.CAP3POL = 1;          // Falling edge
   ECap1Regs.ECCTL1.bit.CAP4POL = 0;          // Rising edge
   ECap1Regs.ECCTL1.bit.CTRRST1 = 1;          // Difference operation
   ECap1Regs.ECCTL1.bit.CTRRST2 = 1;          // Difference operation
   ECap1Regs.ECCTL1.bit.CTRRST3 = 1;          // Difference operation
   ECap1Regs.ECCTL1.bit.CTRRST4 = 1;          // Difference operation
   ECap1Regs.ECCTL2.bit.SYNCI_EN = 1;         // Enable sync in
   ECap1Regs.ECCTL2.bit.SYNCO_SEL = 0;        // Pass through
   ECap1Regs.ECCTL1.bit.CAPLDEN = 1;          // Enable capture units


   ECap1Regs.ECCTL2.bit.TSCTRSTOP = 1;        // Start Counter
   ECap1Regs.ECCTL2.bit.REARM = 1;            // arm one-shot
   ECap1Regs.ECCTL1.bit.CAPLDEN = 1;          // Enable CAP1-CAP4 register loads
   ECap1Regs.ECEINT.bit.CEVT4 = 1;            // 4 events = interrupt
}

__interrupt void ecap1_isr(void)
{
   if(ECap1Regs.CAP2 > EPwm3Regs.TBPRD*2+1 || ECap1Regs.CAP2 < EPwm3Regs.TBPRD*2-1)
   {
       Fail();
   }

   if(ECap1Regs.CAP3 > EPwm3Regs.TBPRD*2+1 || ECap1Regs.CAP3 < EPwm3Regs.TBPRD*2-1)
   {
       Fail();
   }

   if(ECap1Regs.CAP4 > EPwm3Regs.TBPRD*2+1 || ECap1Regs.CAP4 < EPwm3Regs.TBPRD*2-1)
   {
       Fail();
   }

   ECap1IntCount++;

   if(EPwm3TimerDirection == EPWM_TIMER_UP)
   {
        if(EPwm3Regs.TBPRD < PWM3_TIMER_MAX)
        {
           EPwm3Regs.TBPRD++;
        }
        else
        {
           EPwm3TimerDirection = EPWM_TIMER_DOWN;
           EPwm3Regs.TBPRD--;
        }
   }
   else
   {
        if(EPwm3Regs.TBPRD > PWM3_TIMER_MIN)
        {
           EPwm3Regs.TBPRD--;
        }
        else
        {
           EPwm3TimerDirection = EPWM_TIMER_UP;
           EPwm3Regs.TBPRD++;
        }
   }

   ECap1PassCount++;

   ECap1Regs.ECCLR.bit.CEVT4 = 1;
   ECap1Regs.ECCLR.bit.INT = 1;
   ECap1Regs.ECCTL2.bit.REARM = 1;

   PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}

void Fail()
{
    __asm("   ESTOP0");
}




//===========================================================================
// No more.
//===========================================================================
