//#############################################################################
// FILE:   labstarter_main.c
//
// TITLE:  Lab Starter
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "f28379dSerialCPU2.h"

#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define ADCB
#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398

//*****************************************************************************
// the defines for FFT
//*****************************************************************************
#define RFFT_STAGES     10
#define RFFT_SIZE       (1 << RFFT_STAGES)

//*****************************************************************************
// the globals
//*****************************************************************************
float pwrSpec[(RFFT_SIZE/2)+1];
float maxpwr = 0;
float maxpwrfreq = 0;
int16_t maxpwrindex = 0;


#ifdef __cplusplus
#pragma DATA_SECTION("FFT_buffer_2")
#else
#pragma DATA_SECTION(test_output, "FFT_buffer_2")
#endif
float test_output[RFFT_SIZE];


#ifdef __cplusplus
#pragma DATA_SECTION("FFT_buffer_1")
#else
#pragma DATA_SECTION(ping_input, "FFT_buffer_1")
#endif
float ping_input[RFFT_SIZE];

#ifdef __cplusplus
#pragma DATA_SECTION("FFT_buffer_1")
#else
#pragma DATA_SECTION(pong_input, "FFT_buffer_1")
#endif
float pong_input[RFFT_SIZE];


#ifdef __cplusplus
#pragma DATA_SECTION("FFT_buffer_2")
#else
#pragma DATA_SECTION(RFFTF32Coef,"FFT_buffer_2")
#endif //__cplusplus
//! \brief Twiddle Factors
//!
float RFFTF32Coef[RFFT_SIZE];


//! \brief Object of the structure RFFT_F32_STRUCT
//!
RFFT_F32_STRUCT rfft;

//! \brief Handle to the RFFT_F32_STRUCT object
//!
RFFT_F32_STRUCT_Handle hnd_rfft = &rfft;
//=======================FFT END===================================================//


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
//__interrupt void SWI_isr(void);
__interrupt void CPU1toCPU2IPC0(void);


void serialRXC(serial_t *s, char data);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
float *cpu2tocpu1;
float *cpu1tocpu2;
char *commandfromCPU1;
uint32_t numRXC = 0;
uint16_t UARTPrint = 0;
uint32_t i = 0;
uint32_t numCPU1COMs = 0;
float CPU2receivefloats[4] = {0,0,0,0};
float CPU2sendfloats[4] = {0,0,0,0};

// Count variables
uint32_t numRXA = 0;
int32_t ADCB_ISRCount = 0;
int32_t ping_i = 0;
int32_t pong_i = 0;


//Global variables
float volt_sound = 0.0 ;
int16_t adcbresult = 0 ;
int16_t pingpong = 0; //ping = 0; pong = 1
int16_t run_ping = 0;
int16_t run_pong = 0;
//float sound_tracker_idx_sum = 0.0;
//float sound_tracker_idx_avg = 0.0;
//float sound_tracker_idx_avg_1 = 0.0;
int16_t tracker_i = 0;
int16_t sound_timer = 0;
int16_t  silent_timer = 0;
int16_t whistle_timer = 0;
float current_idx = 0;
int16_t long_flag = 0;
int16_t whistle_flag = 0;
float samplePeriod = 0.0001;

//State Machine by sound control
int16_t current_state = 10;
int16_t state_sound_keep = 10;
int16_t state_sound_right = 20;
int16_t state_sound_left = 30;
int16_t state_sound_forward = 40;
int16_t state_sound_stop = 50;



uint32_t timecounter = 0;

char sendstring[30] = "This is a string";

#ifdef ADCB
int16_t adcb4result = 0;
float ADCb4volt = 0;
__interrupt void ADCB_ISR(void);
#endif


void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    GpioDataRegs.GPCSET.bit.GPIO67 = 1;

    commandfromCPU1 = (char*) 0x3FFFC;  // in RAM that CPU1 can R/W but CPU2 can only read.
    //location of cpu2tocpu1 ram
    cpu2tocpu1 = (float*) 0x3F800;
    cpu1tocpu2 = (float*) 0x3FC00;
    while (commandfromCPU1[0] != 'G') {
        DELAY_US(10000);
    }
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.IPC0_INT = &CPU1toCPU2IPC0;
#ifdef ADCB
    PieVectTable.ADCB1_INT = &ADCB_ISR;
#endif

    //PieVectTable.EMIF_ERROR_INT = &SWI_isr;

    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every second:
    // 200MHz CPU Freq, 1 second Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, 200, 137000); // 137ms
    ConfigCpuTimer(&CpuTimer1, 200, 1000);
    ConfigCpuTimer(&CpuTimer2, 200, 40000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    //init_serial(&SerialC,115200,serialRXC);
    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    //IER |= M_INT8;//scic
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
    //IPC0
    PieCtrlRegs.PIEIER1.bit.INTx13 = 1;

#ifdef ADCB
    // Enable ADCB1_INT PIE: Group 1 interrupt 2
    PieCtrlRegs.PIEIER1.bit.INTx2 = 1;

#endif

    IpcRegs.IPCACK.bit.IPC0 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    //============================FFT 10 TRIES==================================//
    // Clear input buffers:
    for(i=0; i < RFFT_SIZE; i++){       
        ping_input[i] = 0.0f;
    }
    for(i=0; i < RFFT_SIZE; i++){       
        pong_input[i] = 0.0f;
    }

    for (i=0;i<RFFT_SIZE;i++) {
        ping_input[i] = sin(125*2*PI*i*samplePeriod)+2*sin(2400*2*PI*i*samplePeriod);   //two combined sin wave
    }
    hnd_rfft->FFTSize   = RFFT_SIZE;
    hnd_rfft->FFTStages = RFFT_STAGES;
    hnd_rfft->InBuf     = &ping_input[0];  //Input buffer
    hnd_rfft->OutBuf    = &test_output[0];  //Output buffer
    hnd_rfft->MagBuf    = &pwrSpec[0];  //Magnitude buffer      

    hnd_rfft->CosSinBuf = &RFFTF32Coef[0];  //Twiddle factor buffer
    RFFT_f32_sincostable(hnd_rfft);         //Calculate twiddle factor

    for (i=0; i < RFFT_SIZE; i++){
        test_output[i] = 0;               //Clean up output buffer
    }

    for (i=0; i <= RFFT_SIZE/2; i++){
        pwrSpec[i] = 0;                //Clean up magnitude buffer
    }


    int16_t tries = 0;
    while(tries < 10) {
        hnd_rfft->InBuf     = &ping_input[0];  //Input buffer
        RFFT_f32(hnd_rfft);                     //Calculate real FFT

#ifdef __TMS320C28XX_TMU__ //defined when --tmu_support=tmu0 in the project, TMU-- trigonometric math unit
        // properties
        RFFT_f32_mag_TMU0(hnd_rfft);            //Calculate magnitude
#else
        RFFT_f32_mag(hnd_rfft);                 //Calculate magnitude
#endif
        maxpwr = 0;
        maxpwrindex = 0;

        for (i=0;i<(RFFT_SIZE/2);i++) {
            if (pwrSpec[i]>maxpwr) {
                maxpwr = pwrSpec[i];
                maxpwrindex = i;
            }
        }

        tries++;


        for (i=0;i<RFFT_SIZE;i++) {
            ping_input[i] = sin((125 + tries*125)*2*PI*i*samplePeriod)+2*sin((2400-tries*200)*2*PI*i*samplePeriod);
        }
    }
    //==============================FFT 10 TRIES END============================================================================//


    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM


    // IDLE loop. Just sit and loop forever (optional):
    // Calculate maxpwr and maxpwrindex from FFT
    while(1)
    {

        if(run_ping == 1){
            hnd_rfft->InBuf     = &ping_input[0];  //Input buffer
        }
        if(run_pong == 1){
            hnd_rfft->InBuf     = &pong_input[0];  //Input buffer
        }

        // ====TRUE if ping/pong buffer is full, every 102.4ms============
        if(run_ping == 1 || run_pong == 1){
            RFFT_f32(hnd_rfft);                     //Calculate real FFT

            //Make sure to indicate buffer unfull **Necessary**
            run_ping = 0;
            run_pong = 0;

#ifdef __TMS320C28XX_TMU__ //defined when --tmu_support=tmu0 in the project, TMU-- trigonometric math unit
            // properties
            RFFT_f32_mag_TMU0(hnd_rfft);            //Calculate magnitude
#else
            RFFT_f32_mag(hnd_rfft);                 //Calculate magnitude
#endif
            maxpwr = 0;
            maxpwrindex = 0;

            for (i=3;i<(RFFT_SIZE/2);i++) {
                if (pwrSpec[i]>maxpwr) {
                    maxpwr = pwrSpec[i];
                    maxpwrindex = i;
                }
            }
            maxpwrfreq = maxpwrindex*10000.0/1024.0;
            cpu2tocpu1[0] = maxpwrindex;
            cpu2tocpu1[1] = maxpwrfreq;
            IpcRegs.IPCSET.bit.IPC0 = 1;
        }




        if (UARTPrint == 1 ) {

            UARTPrint = 0;
        }
    }
}


// SWI_isr,  Using this interrupt as a Software started interrupt
//__interrupt void SWI_isr(void) {
//
//    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
//    // making it lower priority than all other Hardware interrupts.
//    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
//    asm("       NOP");                    // Wait one cycle
//    EINT;                                 // Clear INTM to enable interrupts
//
//
//
//    // Insert SWI ISR Code here.......
//
//
//    numSWIcalls++;
//
//    DINT;
//
//}


//IPC
__interrupt void CPU1toCPU2IPC0(void){
    //put data from cpu2 into cpu2tocpu1 array
    int i;
    for (i=0;i<4;i++) {
        CPU2receivefloats[i] = cpu1tocpu2[i];
    }
    numCPU1COMs++;
    UARTPrint = 1;
    IpcRegs.IPCACK.bit.IPC0 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    GpioDataRegs.GPASET.bit.GPIO7 = 1;

    CpuTimer0.InterruptCount++;

    numTimer0calls++;

    //    if ((numTimer0calls%50) == 0) {
    //        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
    //    }



    //    cpu2tocpu1[0] = sin(2*PI*0.1*(timecounter*0.137));
    //    cpu2tocpu1[1] = 2*cos(2*PI*0.1*(timecounter*0.137));
    //    cpu2tocpu1[2] = 3*sin(2*PI*0.1*(timecounter*0.137));
    //    cpu2tocpu1[3] = 4*cos(2*PI*0.1*(timecounter*0.137));
    timecounter++;

    //    IpcRegs.IPCSET.bit.IPC0 = 1;

    GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;



    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    GpioDataRegs.GPBTOGGLE.bit.GPIO61 = 1;//led12
    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
    CpuTimer1.InterruptCount++;
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
    CpuTimer2.InterruptCount++;
}

// This function is called each time a char is received over UARTC.
void serialRXC(serial_t *s, char data) {
    numRXC ++;
}

#ifdef ADCB
__interrupt void ADCB_ISR (void)
{

    adcb4result = AdcbResultRegs.ADCRESULT0;

    ADCb4volt = adcb4result*3.0/4095.0;

    //Fill out the ping_input[] and bounce to pong_input
    if(pingpong == 0){
        ping_input[ping_i] = ADCb4volt;

        if(ping_i < 1024){
            ping_i++;
        }

        if(ping_i >= 1024){
            pingpong = 1;
            run_ping = 1;
            run_pong = 0;
            ping_i = 0;
        }
    }
    //Fill out the pong_input[]
    if(pingpong == 1){
        pong_input[pong_i] = ADCb4volt;

        if(pong_i < 1024){
            pong_i++;
        }

        if(pong_i >= 1024){
            pingpong = 0;
            run_ping = 0;
            run_pong = 1;
            pong_i = 0;
        }
    }

    //    if (pingORpong == 0) {
    //        ping_input[fftsamplecount] = ADCb4volt;
    //        fftsamplecount++;
    //        if (fftsamplecount == 1024) {
    //            fftsamplecount = 0;
    //            RunPingFFT = 1;
    //            pingORpong = 1;
    //        }
    //    } else {
    //        pong_input[fftsamplecount] = ADCb4volt;
    //        fftsamplecount++;
    //        if (fftsamplecount == 1024) {
    //            fftsamplecount = 0;
    //            RunPongFFT = 1;
    //            pingORpong = 0;
    //        }
    //    }

    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}
#endif
