//#############################################################################
// FILE:   LABstarter_main.c
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
#include "F28379dSerial.h"
#include "LEDPatterns.h"
#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

extern uint16_t IPCBootCPU2(uint32_t ulBootMode);

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);
__interrupt void SPIB_isr(void); // SPI ISR predefinition
__interrupt void CPU2toCPU1IPC0(void);


float *cpu2tocpu1; //float pointer for the location CPU2 will give data to CPU1
float *cpu1tocpu2; //float pointer for the location CPU1 will give data to CPU2
char *commandtoCPU2;

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;

// Mapping variables
float OBx[228];
float OBy[228];
float OBxc[6840];
float OByc[6840];
uint16_t index = 0;

// Lidar variables
typedef struct datapts{
    float angle;
    float distance_ping;
    float distance_pong;
}datapts;

typedef struct {
    float x;        //in feet
    float y;        //in feet
} xy;

uint8_t lidar_ready = 0;
extern char G_command[];
extern uint16_t G_len;
extern uint16_t LIDARi;
extern uint16_t LIDARpingpong;
extern datapts lidar_data[228]; //distance data from lidar
extern xy lidar_pts[228]; //xy data

// Frequency control variables
float maxpwrindex = 0;
float maxpwrfreq = 0;
int32_t numCPU2COMs = 0;
int16_t statecase = 10;
int16_t statecount = 0;
int16_t statetimeout = 50;

// Wall following variables
float wall_ref = 2;
float wall_dist = 0;
float wall_error = 0;
float Kp_wall = 0.5;
uint8_t corner_state = 0;
float corner_turn = 1;
float fwd_ref = 3;
float fwd_dist = 0;

// Wheel Encoder Variables
float RightWheel = 0; // Right wheel, rotations in radians
float LeftWheel = 0; // Left wheel, rotations in radians
float PosRight_K = 0; // Right wheel, linear position in feet
float PosLeft_K = 0; // Left wheel, linear position in feet
float PosRight_K_1 = 0; // Previous value of PosRight_K
float PosLeft_K_1 = 0; // Previous value of PosLeft_K

// Velocity Variables
float VRight_K = 0; // Actual right velocity
float VLeft_K = 0; // Actual left velocity

// Setpoints
// float Vref = 0; // Velocity setpoint
float Vref = 1.5; // Initial speed, can be changed later
float turn = 0; // Turn setpoint

// Controller Gains
float Kp = 3;
float Ki = 25;
float Kp_turn = 3;

// Past and present errors
float ErrRight_K = 0;
float ErrLeft_K = 0;
float ErrRight_K_1 = 0;
float ErrLeft_K_1 = 0;
float Err_turn = 0;

// Past and present integrals
float IRight_K = 0;
float ILeft_K = 0;
float IRight_K_1 = 0;
float ILeft_K_1 = 0;

// Motor Control Effort
float uRight = 0;
float uLeft = 0;

// Pose Calculation Variables
float theta_R_K = 0;
float theta_L_K = 0;
float theta_R_K_1 = 0;
float theta_L_K_1 = 0;
float thetadot_R = 0;
float thetadot_L = 0;
float theta_avg = 0;
float thetadot_avg = 0;
float phi = 0;
float x = 0;
float y = 0;
float xdot_K = 0;
float ydot_K = 0;
float xdot_K_1 = 0;
float ydot_K_1 = 0;
// These values have been tuned experimentally:
float width = 0.585; // width between wheels in feet
float radius = 0.197; // wheel radius in feet

// Exercise 3 variables
uint16_t DANPWM = 0; // PWM command sent to DAN chip
uint16_t countdir = 0; // keeps track of count direction
uint16_t ADC1val = 0; // ADC1 return value from DAN chip
uint16_t ADC2val = 0; // ADC2 return value from DAN chip
float ADC1scaled = 0; // ADC1 scaled value in volts
float ADC2scaled = 0; // ADC2 scaled value in volts

// Accelerometer offsets
int16_t offsetX = 2905;
int16_t offsetY = -2945;
int16_t offsetZ = 4645;

int16_t offsetXmsb = 0;
int16_t offsetXlsb = 0;
int16_t offsetYmsb = 0;
int16_t offsetYlsb = 0;
int16_t offsetZmsb = 0;
int16_t offsetZlsb = 0;

// Exercise 4 variables
int16_t AccelX = 0;
int16_t AccelY = 0;
int16_t AccelZ = 0;
float AccelXscaled = 0;
float AccelYscaled = 0;
float AccelZscaled = 0;
int16_t GyroX = 0;
int16_t GyroY = 0;
int16_t GyroZ = 0;
float GyroXscaled = 0;
float GyroYscaled = 0;
float GyroZscaled = 0;
int16_t Temp = 0;

uint8_t iscloseto(float inputval, float refval, float maxerror) {
    if (fabs(inputval - refval) < maxerror) {
        return 1;
    }
    else {
        return 0;
    }
}

// eQEP setup function
void init_eQEPs(void) {

    // setup eQEP1 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pins for reduced power consumption
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1;    // Disable pull-up on GPIO20 (EQEP1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1;    // Disable pull-up on GPIO21 (EQEP1B)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 2;   // Qual every 6 samples
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 2;   // Qual every 6 samples
    EDIS;
    // This specifies which of the possible GPIO pins will be EQEP1 functional pins.
    // Comment out other unwanted lines.
    GPIO_SetupPinMux(20, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(21, GPIO_MUX_CPU1, 1);
    EQep1Regs.QEPCTL.bit.QPEN = 0;    // make sure eqep in reset
    EQep1Regs.QDECCTL.bit.QSRC = 0;   // Quadrature count mode
    EQep1Regs.QPOSCTL.all = 0x0;      // Disable eQep Position Compare
    EQep1Regs.QCAPCTL.all = 0x0;      // Disable eQep Capture
    EQep1Regs.QEINT.all = 0x0;        // Disable all eQep interrupts
    EQep1Regs.QPOSMAX = 0xFFFFFFFF;   // use full range of the 32 bit count
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2;  // EQep uneffected by emulation suspend in Code Composer
    EQep1Regs.QPOSCNT = 0;
    EQep1Regs.QEPCTL.bit.QPEN = 1;    // Enable EQep

    // setup QEP2 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pinsfor reduced power consumption
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 1;    // Disable pull-up on GPIO54 (EQEP2A)
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 1;    // Disable pull-up on GPIO55 (EQEP2B)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 2;   // Qual every 6 samples
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 2;   // Qual every 6 samples
    EDIS;
    GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2A
    GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2B
    EQep2Regs.QEPCTL.bit.QPEN = 0;   // make sure qep reset
    EQep2Regs.QDECCTL.bit.QSRC = 0;  // Quadrature count mode
    EQep2Regs.QPOSCTL.all = 0x0;     // Disable eQep Position Compare
    EQep2Regs.QCAPCTL.all = 0x0;     // Disable eQep Capture
    EQep2Regs.QEINT.all = 0x0;       // Disable all eQep interrupts
    EQep2Regs.QPOSMAX = 0xFFFFFFFF;  // use full range of the 32 bit count.
    EQep2Regs.QEPCTL.bit.FREE_SOFT = 2;  // EQep uneffected by emulation suspend
    EQep2Regs.QPOSCNT = 0;
    EQep2Regs.QEPCTL.bit.QPEN = 1;   // Enable EQep
}

// This function reads the left encoder counts
float readEncLeft(void) {

    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU;  //4294967295U

    raw = EQep1Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue;  // I don't think this is needed and never true

    // 100 slits in the encoder disk so 100 square waves per one revolution of the
    // DC motor's back shaft.  Then Quadrature Decoder mode multiplies this by 4 so 400 counts per one rev // of the DC motor's back shaft.  Then the gear motor's gear ratio is 30:1.
    return (raw*(-5.236e-4)); // conversion factor = 2*pi/(400*30) = 5.236e-4   Negative sign to flip count direction
}

// This function reads the right encoder counts
float readEncRight(void) {

    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU;  //4294967295U  -1 32bit signed int

    raw = EQep2Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue;  // I don't think this is needed and never true

    // 100 slits in the encoder disk so 100 square waves per one revolution of the
    // DC motor's back shaft.  Then Quadrature Decoder mode multiplies this by 4 so 400 counts per one rev // of the DC motor's back shaft.  Then the gear motor's gear ratio is 30:1.
    return (raw*(5.236e-4)); // conversion factor = 2*pi/(400*30) = 5.236e-4
}

// helper function for setting the value of CMPA for PWM 2 based on requested motor speed
// a saturation limit is placed on inputs that are greater than 10 are less that -10
void setEPWM2A(float controleffort) {
    // set the value of controleffort to 10 if it is greater than 10
    if (controleffort > 10) {
        controleffort = 10;
    }
    // set the value of controleffort to -10 if it is less than -10
    if (controleffort < -10) {
        controleffort = -10;
    }
    /* Scale the value of controleffort to get the correct value to load
    into the CMPA register. Note the following:
    - A control value of -10 maps to a CMPA value of 0 (0% duty cycle, full speed reverse)
    - A control value of 0 maps to a CMPA value of 1250 (50% duty cycle, motor stopped)
    - A control value of 10 maps to a CMPA value of 2500 (100% duty cycle, full speed forward)
    */
    EPwm2Regs.CMPA.bit.CMPA = ((float)1250) * (1.0 + (0.1 * controleffort)); // Scale controleffort as outlined above

}

// helper function for setting the value of CMPB for PWM 2 based on requested motor speed
// a saturation limit is placed on inputs that are greater than 10 are less that -10
void setEPWM2B(float controleffort) {
    // set the value of controleffort to 10 if it is greater than 10
    if (controleffort > 10) {
        controleffort = 10;
    }
    // set the value of controleffort to -10 if it is less than -10
    if (controleffort < -10) {
        controleffort = -10;
    }
    /* Scale the value of controleffort to get the correct value to load
    into the CMPB register. Note the following:
    - A control value of -10 maps to a CMPB value of 0 (0% duty cycle, full speed reverse)
    - A control value of 0 maps to a CMPB value of 1250 (50% duty cycle, motor stopped)
    - A control value of 10 maps to a CMPB value of 2500 (100% duty cycle, full speed forward)
    */
    EPwm2Regs.CMPB.bit.CMPB = ((float)1250) * (1.0 + (0.1 * controleffort)); // Scale controleffort as outlined above

}

// Setup function for MPU-9250
void setupSpib(void) //Call this function in main() somewhere after the DINT; line of code.
{
    int16_t temp = 0;
    // cut and paste here all the SpibRegs initializations you found for part 3.  Make sure the TXdelay in between each transfer to 0.    Also don�t forget to cut and paste the GPIO settings for GPIO9, 63, 64, 65, 66 which are also a part of the SPIB setup.
    // SpibRegs initializations moved into setup function

    // SPI setup
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);  // Set as GPIO9 and used as DAN28027 SS
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);  // Make GPIO9 an Output Pin
    GpioDataRegs.GPASET.bit.GPIO9 = 1;  //Initially Set GPIO9/SS High so DAN28027 is not selected

    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);  // Set as GPIO66 and used as MPU-9250 SS
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);  // Make GPIO66 an Output Pin
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;  //Initially Set GPIO66/SS High so MPU-9250 is not selected

    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15);  //Set GPIO63 pin to SPISIMOB (pinmux 15)
    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15);  //Set GPIO64 pin to SPISOMIB (pinmux 15)
    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15);  //Set GPIO65 pin to SPICLKB (pinmux 15)

    EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO63   = 0;  // Enable Pull-ups on SPI PINs Recommended by TI for SPI Pins
    GpioCtrlRegs.GPCPUD.bit.GPIO64   = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO65   = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3;  // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3;  // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3;  // Set I/O pin to asynchronous mode recommended for SPI
    EDIS;

    // ---------------------------------------------------------------------------
    SpibRegs.SPICCR.bit.SPISWRESET = 0;  // Put SPI in Reset (0x0)

    SpibRegs.SPICTL.bit.CLK_PHASE = 1;  //This happens to be the mode for both the DAN28027 and
    SpibRegs.SPICCR.bit.CLKPOLARITY = 0;  //The MPU-9250,  Mode 01.
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1;  // Set to SPI Master (0x1)
    SpibRegs.SPICCR.bit.SPICHAR = 15;  // Set to transmit and receive 16 bits each write to SPITXBUF (0xF)
    SpibRegs.SPICTL.bit.TALK = 1;  // Enable transmission (0x1)
    SpibRegs.SPIPRI.bit.FREE = 1;  // Free run, continue SPI operation
    SpibRegs.SPICTL.bit.SPIINTENA = 0;  // Disables the SPI interrupt (0x0)

    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 49; // Set SCLK bit rate to 1 MHz so 1us period.  SPI base clock is
    // 50MHZ.  And this setting divides that base clock to create SCLK�s period
    // Baud Rate = LSPCLK/(SPIBRR + 1)
    // To divide LSPCLK by 50, set SPIBRR to 50 - 1 = 49
    SpibRegs.SPISTS.all = 0x0000;  // Clear status flags just in case they are set for some reason

    SpibRegs.SPIFFTX.bit.SPIRST = 1;// Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive. (0x1)
    SpibRegs.SPIFFTX.bit.SPIFFENA = 1;    // Enable SPI FIFO enhancements (0x1)
    SpibRegs.SPIFFTX.bit.TXFIFO =  0;    // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1;    // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set

    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0;    // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1;    // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1;    // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFIENA = 1;   // Enable the RX FIFO Interrupt.  RXFFST >= RXFFIL (0x1)

    // Commented out when working with the MPU-9250
    // SpibRegs.SPIFFCT.bit.TXDLY = 16; //Set delay between transmits to 16 spi clocks. Needed by DAN28027 chip

    SpibRegs.SPICCR.bit.SPISWRESET = 1;    // Pull the SPI out of reset (0x1)

    SpibRegs.SPIFFTX.bit.TXFIFO = 1;    // Release transmit FIFO from reset. (0x1)
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1;    // Re-enable receive FIFO operation
    SpibRegs.SPICTL.bit.SPIINTENA = 1;    // Enables SPI interrupt.  !! I don�t think this is needed.  Need to Test

    SpibRegs.SPIFFRX.bit.RXFFIL = 16; //Interrupt Level to 16 words or more received into FIFO causes interrupt.  This is just the initial setting for the register.  Will be changed below

    SpibRegs.SPICCR.bit.SPICHAR      = 0xF;
    SpibRegs.SPIFFCT.bit.TXDLY       = 0x00; // Set TXdelay to 0
    //-----------------------------------------------------------------------------------------------------------------

    // perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x13,0x14,0x15,0x16
    // 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C 0x1D, 0x1E, 0x1F.  Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;  // Slave Select Low

    // Perform the number of needed writes to SPITXBUF to write to all 13 registers. Remember we are sending 16 bit transfers, so two registers at a time after the first 16 bit transfer.
    // Start writing at address 00x13
    // To address 00x13 write 0x00
    // Address pointer in the MPU-9250 is automatically incremented thereafter
    SpibRegs.SPITXBUF = 0x1300;  // Send register address to start writing at

    // To address 00x14 write 0x00
    // To address 00x15 write 0x00
    SpibRegs.SPITXBUF = 0x0000;

    // To address 00x16 write 0x00
    // To address 00x17 write 0x00
    SpibRegs.SPITXBUF = 0x0000;

    // To address 00x18 write 0x00
    // To address 00x19 write 0x13
    SpibRegs.SPITXBUF = 0x0013;

    // To address 00x1A write 0x02
    // To address 00x1B write 0x00
    SpibRegs.SPITXBUF = 0x0200;

    // To address 00x1C write 0x08
    // To address 00x1D write 0x06
    SpibRegs.SPITXBUF = 0x0806;

    // To address 00x1E write 0x00
    // To address 00x1F write 0x00
    SpibRegs.SPITXBUF = 0x0000;

    // wait for the correct number of 16 bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST !=7);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    // temp = SpibRegs.SPIRXBUF;
    // ???? read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    for (uint16_t i = 0; i < 7; i++) {
        temp = SpibRegs.SPIRXBUF; // read garbage values 7 times
    }
    DELAY_US(10);  // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

    // perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x23,0x24,0x25,0x26
    // 0x27, 0x28, 0x29.  Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;  // Slave Select Low

    // Perform the number of needed writes to SPITXBUF to write to all 7 registers
    SpibRegs.SPIFFRX.bit.RXFFIL = 4;  // Issue the SPIB_RX_INT when 4 values are in the RX FIFO (1 address + 7 commands)/2

    // Start writing at address 00x23
    // To address 00x23 write 0x00
    // Address pointer in the MPU-9250 is automatically incremented thereafter
    SpibRegs.SPITXBUF = 0x2300;  // Send register address to start writing at

    // To address 00x24 write 0x40
    // To address 00x25 write 0x8C
    SpibRegs.SPITXBUF = 0x408C;

    // To address 00x26 write 0x02
    // To address 00x27 write 0x88
    SpibRegs.SPITXBUF = 0x0288;

    // To address 00x28 write 0x0C
    // To address 00x29 write 0x0A
    SpibRegs.SPITXBUF = 0x0C0A;

    // wait for the correct number of 16 bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST !=4);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    // temp = SpibRegs.SPIRXBUF;
    // ???? read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    for (uint16_t i = 0; i < 4; i++) {
        temp = SpibRegs.SPIRXBUF; // read garbage values 4 times
    }
    DELAY_US(10);  // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

    // perform a single 16 bit transfer to initialize MPU-9250 register 0x2A
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    // Write to address 0x2A the value 0x81
    SpibRegs.SPITXBUF = 0x2A81;
    // wait for one byte to be received
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    // format offset values using bitwise operators
    offsetXmsb = ((offsetX << 1) & (0xFF00)) >> 8;
    offsetXlsb = (offsetX << 1) & 0x00FF;
    offsetYmsb = ((offsetY << 1) & (0xFF00)) >> 8;
    offsetYlsb = (offsetY << 1) & 0x00FF;
    offsetZmsb = ((offsetZ << 1) & (0xFF00)) >> 8;
    offsetZlsb = (offsetZ << 1) & 0x00FF;

    // The Remainder of this code is given to you and you do not need to make any changes.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3800 | 0x0001);  // 0x3800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3A00 | 0x0001);  // 0x3A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6400 | 0x0001);  // 0x6400
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6700 | 0x0003); // 0x6700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6A00 | 0x0020);  // 0x6A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6B00 | 0x0001); // 0x6B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7500 | 0x0071);  // 0x7500
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    // SpibRegs.SPITXBUF = (0x7700 | 0x00EB); // 0x7700
    SpibRegs.SPITXBUF = (0x7700 | offsetXmsb); // 0x7700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    // SpibRegs.SPITXBUF = (0x7800 | 0x0012); // 0x7800
    SpibRegs.SPITXBUF = (0x7800 | offsetXlsb); // 0x7800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    // SpibRegs.SPITXBUF = (0x7A00 | 0x0010); // 0x7A00
    SpibRegs.SPITXBUF = (0x7A00 | offsetYmsb); // 0x7A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    // SpibRegs.SPITXBUF = (0x7B00 | 0x00FA); // 0x7B00
    SpibRegs.SPITXBUF = (0x7B00 | offsetYlsb); // 0x7B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7D00 | offsetZmsb); // 0x7D00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7E00 | offsetZlsb); // 0x7E00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(50);

    // Clear SPIB interrupt source just in case it was issued due to any of the above initializations.
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1;  // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR=1;  // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}


void main(void) {

    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();


    InitIpc();


    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_INPUT, GPIO_PULLUP);

    commandtoCPU2 = (char*) 0x3FFFC;  // in RAM that CPU1 can R/W but CPU2 can only read.
    //location of cpu2tocpu1 ram
    cpu2tocpu1 = (float*) 0x3F800;
    cpu1tocpu2 = (float*) 0x3FC00;

    commandtoCPU2[0] = 'W';  // W for CPU2 wait

    //     Comment this when use CCS for debugging
//             #ifdef _FLASH
//                 // Send boot command to allow the CPU2 application to begin execution
//
    if (GpioDataRegs.GPADAT.bit.GPIO4 == 0) {
                   IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_FLASH);
    }
//
//             #else
//                 // Send boot command to allow the CPU2 application to begin execution
//                 IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_RAM);
//             #endif
 //  Or when you want to run CPU2 from its flash you free run CPU2 and just run IPCBootCPU2 from Flash command.  Actually I do not know when you need boot from RAM ???

    InitGpio();

    // Blue LED on LaunchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

    // LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;

    // LED2
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

    // LED3
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

    // LED4
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

    // LED5
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

    // LED6
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

    // LED7
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

    // LED8
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

    // LED9
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

    // LED10
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

    // LED11
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;

    // LED12
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

    // LED13
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;

    // LED14
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;

    // LED15
    GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(159, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;

    // LED16
    GPIO_SetupPinMux(160, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(160, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPFCLEAR.bit.GPIO160 = 1;

    //WIZNET Reset
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO0 = 1;

    //ESP8266 Reset
    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO1 = 1;

    //SPIRAM  CS  Chip Select
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    //DRV8874 #1 DIR  Direction
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //DRV8874 #2 DIR  Direction
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;

    //DAN28027  CS  Chip Select
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO9 = 1;

    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

    //WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;

    //PushButton 1
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_INPUT, GPIO_PULLUP);

    //Joy Stick Pushbutton
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(8, GPIO_INPUT, GPIO_PULLUP);

    // Red LED on LaunchPad, change ownership to CPU2
    GPIO_SetupPinMux(34, GPIO_MUX_CPU2, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);

    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

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
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIB_RX_INT = &RXBINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIB_TX_INT = &TXBINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;
    PieVectTable.IPC0_INT = &CPU2toCPU1IPC0;
    PieVectTable.EMIF_ERROR_INT = &SWI_isr;

    PieVectTable.SPIB_RX_INT = &SPIB_isr; // use pointer to add SPI ISR to PieVectTable

    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    // ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 20000); // Timer0 is called every 20ms for Exercise 3, originally 10ms
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 1000); // Timer0 period changed to 1ms for Exercise 4
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 4000); // Timer1 period changed to 4ms for reading wheel encoders
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 100000); // Timer2 period changed to 100ms for Lidar communication

    DELAY_US(1000000);  // Delay 1 second giving Lidar Time to power on after system power on

    for (LIDARi = 0; LIDARi < 228; LIDARi++) {
        lidar_data[LIDARi].angle = ((3*LIDARi+44)*0.3515625-135)*0.01745329; //0.017453292519943 is pi/180, multiplication is faster; 0.3515625 is 360/1024
    }

    init_serialSCIA(&SerialA,115200);
    //    init_serialSCIB(&SerialB,115200);
    init_serialSCIC(&SerialC,19200);
    //    init_serialSCID(&SerialD,115200);

    // Enable SPIB
    setupSpib();

    // Initialize eQEPs
    init_eQEPs();

    // Setup of the GPIO pin multiplexing (GPIO2 -> EPWM2A)
    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 1); // GPIO2, CPU1, Mux index of 1

    // Timer Base Control Individual Register bit configuration
    EPwm2Regs.TBCTL.bit.CTRMODE = 0;       // Setting the counter mode to "up-count"
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 2;     // Setting Free Soft Emulation mode to Free Run for TBCTL PWM register
    EPwm2Regs.TBCTL.bit.PHSEN = 0;         // Disabling phase loading
    EPwm2Regs.TBCTL.bit.CLKDIV = 0;        // Setting clock divide to the default value of 1

    // Timer Base Counter Individual Register bit configuration
    EPwm2Regs.TBCTR = 0;                   // Starting the timer at 0

    // Timer Base Period Individual Register bit configuration
    EPwm2Regs.TBPRD = 2500;                // Setting the period of the time base counter to 2500 because (50*10^6)/(20*10^3)

    // Counter Compare A Individual Register bit configuration
    EPwm2Regs.CMPA.bit.CMPA = 0;           // Starting duty cycle at 0%

    // Action Qualifier Control Individual Register bit configuration
    EPwm2Regs.AQCTLA.bit.CAU = 1;          // Forcing EPWMxA output to low when TBCTR = CMPA in an "up-count" configuration
    EPwm2Regs.AQCTLA.bit.ZRO = 2;          // Set EPWMxA output to high when TBCTR = 0

    // Timer Base Phase Individual Register bit configuration
    EPwm2Regs.TBPHS.bit.TBPHS = 0;         // Setting the phase to 0 as a safety precaution

    // Setup of the GPIO pin multiplexing (GPIO3 -> EPWM2B)
    GPIO_SetupPinMux(3, GPIO_MUX_CPU1, 1); // GPIO3, CPU1, Mux index of 1

    // Counter Compare B Individual Register bit configuration
    EPwm2Regs.CMPB.bit.CMPB = 0;           // Starting duty cycle at 0%

    // Action Qualifier Control Individual Register bit configuration
    EPwm2Regs.AQCTLB.bit.CBU = 1;          // Forcing EPWMxB output to low when TBCTR = CMPB in an "up-count" configuration
    EPwm2Regs.AQCTLB.bit.ZRO = 2;          // Set EPWMxB output to high when TBCTR = 0

    // Code to disable pull-up resistor
    EALLOW; // Below are protected registers
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1; // For EPWM2A
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1; // For EPWM2B
    EDIS;

    EALLOW;

    //write configurations for all ADCs  ADCA, ADCB, ADCC, ADCD
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcdRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);  //read calibration settings
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);  //read calibration settings
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);  //read calibration settings
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);  //read calibration settings
    //Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    //power up the ADCs
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    //delay for 1ms to allow ADC time to power up
    DELAY_US(1000);

    //Select the channels to convert and end of conversion flag
    //Many statements commented out,  To be used when using ADCA or ADCB
    //ADCA
//    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2;  //SOC0 will convert Channel you choose Does not have to be A0
//    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
//    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 0xD;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC0
//    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3;  //SOC1 will convert Channel you choose Does not have to be A1
//    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
//    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 0xD;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC1
//    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //set to last SOC that is converted and it will set INT1 flag ADCA1
//    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
//    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

    //ADCB
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 4;  //SOC0 will convert Channel you choose Does not have to be B0
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 0xD; // EPWM5 ADCSOCA or another trigger you choose will trigger SOC0
    //AdcbRegs.ADCSOC1CTL.bit.CHSEL = ???;  //SOC1 will convert Channel you choose Does not have to be B1
    //AdcbRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA or another trigger you choose will trigger SOC1
    //AdcbRegs.ADCSOC2CTL.bit.CHSEL = ???;  //SOC2 will convert Channel you choose Does not have to be B2
    //AdcbRegs.ADCSOC2CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA or another trigger you choose will trigger SOC2
    //AdcbRegs.ADCSOC3CTL.bit.CHSEL = ???;  //SOC3 will convert Channel you choose Does not have to be B3
    //AdcbRegs.ADCSOC3CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //AdcbRegs.ADCSOC3CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA or another trigger you choose will trigger SOC3
    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //set to last SOC that is converted and it will set INT1 flag ADCB1
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

    //ADCD
//    AdcdRegs.ADCSOC0CTL.bit.CHSEL = 0;  // set SOC0 to convert pin D0
//    AdcdRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
//    AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 0xD; // EPWM5 ADCSOCA will trigger SOC0
//    AdcdRegs.ADCSOC1CTL.bit.CHSEL = 1;  //set SOC1 to convert pin D1
//    AdcdRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
//    AdcdRegs.ADCSOC1CTL.bit.TRIGSEL = 0xD; // EPWM5 ADCSOCA will trigger SOC1
    //AdcdRegs.ADCSOC2CTL.bit.CHSEL = ???;  //set SOC2 to convert pin D2
    //AdcdRegs.ADCSOC2CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //AdcdRegs.ADCSOC2CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA will trigger SOC2
    //AdcdRegs.ADCSOC3CTL.bit.CHSEL = ???;  //set SOC3 to convert pin D3
    //AdcdRegs.ADCSOC3CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //AdcdRegs.ADCSOC3CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA will trigger SOC3
//    AdcdRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //set to SOC1, the last converted, and it will set INT1 flag ADCD1
//    AdcdRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
//    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    EDIS;

    EALLOW;
    EPwm5Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm5Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm5Regs.ETSEL.bit.SOCASEL = 2; // Select Event when counter equal to PRD
    EPwm5Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event (pulse is the same as trigger)
    EPwm5Regs.TBCTR = 0x0; // Clear counter
    EPwm5Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm5Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm5Regs.TBCTL.bit.CLKDIV = 0; // divide by 1  50Mhz Clock
    EPwm5Regs.TBPRD = 5000;  //10000Hz,  Input clock is 50MHz.
    //EPwm5Regs.TBPRD = 50000;  //1000Hz,  Input clock is 50MHz.
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
    EPwm5Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA

    // Unfreeze EPWM5 on CPU2
//    EPwm5Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode
    EDIS;


    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT6;  // SPIB_RX
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    //need to acknowledge IPC before use
    IpcRegs.IPCACK.bit.IPC0 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;

    // Enable SPIB_RX in the PIE: Group 6 interrupt 3
    PieCtrlRegs.PIEIER6.bit.INTx3 = 1;
    //IPC
    PieCtrlRegs.PIEIER1.bit.INTx13 = 1;

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    // Do this before giving control to CPU2
    EALLOW;
    EPwm5Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode
    EDIS;

    //for CPU2 to use SCIC and or SPIC
    EALLOW;
    DevCfgRegs.CPUSEL0.bit.EPWM5 = 1;  // EPWM5 to CPU2
    DevCfgRegs.CPUSEL11.bit.ADC_B = 1;  // ADC_B to CPU2
    MemCfgRegs.GSxMSEL.bit.MSEL_GS9 = 1;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS10 = 1;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS11 = 1;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS12 = 1;
    EDIS;

    commandtoCPU2[0] = 'G';  // G for CPU2 Go


    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    // Change the Lidar's baud rate to 115200
    char S_command[19] = "S1152000124000\n";//this change the baud rate to 115200
    uint16_t S_len = 19;
    serial_sendSCIC(&SerialC, S_command, S_len);
    DELAY_US(1000000);  // Delay letting Lidar change its Baud rate

    // Change the serial port's baud rate to 115200
    init_serialSCIC(&SerialC,115200);
    
    // Lidar is ready to start returning data
    lidar_ready = 1;

    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
                // serial_printf(&SerialA,"Num Timer2:%ld Num SerialRX: %ld\r\n",CpuTimer2.InterruptCount,numRXA);
            // serial_printf(&SerialA,"ADC1 reading: %.3f ADC2 reading: %.3f\r\n",ADC1scaled,ADC2scaled); // Print statement for Exercise 3
            // serial_printf(&SerialA,"Accelerometer (g) X: %.3f Y: %.3f Z: %.3f\r\n",AccelXscaled,AccelYscaled,AccelZscaled); // Print accelerometer values
            // serial_printf(&SerialA,"Gyroscope (deg/s) X: %.3f Y: %.3f Z: %.3f\r\n",GyroXscaled,GyroYscaled,GyroZscaled); // Print gyroscope values

            // Lab 6 print statements - some have been commented out for brevity
            // serial_printf(&SerialA,"Right Wheel (rad): %.3f Left Wheel (rad): %.3f\r\n",RightWheel,LeftWheel);
            // serial_printf(&SerialA,"Right Distance (ft): %.3f Left Distance (ft): %.3f\r\n",PosRight_K,PosLeft_K);

            serial_printf(&SerialA,"Right Velocity (ft/s): %.3f Left Velocity (ft/s): %.3f\r\n",VRight_K,VLeft_K);
            serial_printf(&SerialA,"Max Frequency (Hz): %.3f\r\n",maxpwrfreq);

            // serial_printf(&SerialA,"X Pos (ft): %.3f Y Pos (ft): %.3f Bearing (rad): %.3f\r\n",x,y,phi);
            UARTPrint = 0;
        }
    }
}


//IPC
__interrupt void CPU2toCPU1IPC0(void){

    maxpwrindex = cpu2tocpu1[0];
    maxpwrfreq = cpu2tocpu1[1];

    numCPU2COMs++;
    UARTPrint = 1;
    IpcRegs.IPCACK.bit.IPC0 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
    // making it lower priority than all other Hardware interrupts.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts



    // Insert SWI ISR Code here.......


    numSWIcalls++;
    
    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;

//    if ((numTimer0calls%50) == 0) {
//        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
//    }

    // Exercise 4
    // Enable MPU-9250 by pulling CS pin low
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Set GPIO66 low to enable DAN28027
    SpibRegs.SPIFFRX.bit.RXFFIL = 8; // Issue the SPIB_RX_INT when eight values are in the RX FIFO
    // Start command consists of the register address to start reading at (0x3A) with the MSB set high (read mode)
    // This converts to 0xBA
    SpibRegs.SPITXBUF = 0xBA00;  // Send start command (0xBA)
    // We expect 14 bytes of data to be read back
    // Must send equal number of bytes to generate clock
    // Since we are sending 16 bits at a time, 14/2 = 7 transmissions required
    for (uint16_t i = 0; i < 7; i++) {
        SpibRegs.SPITXBUF = 0;
    }

    // Disable DAN28027 for Exercise 4
    // GpioDataRegs.GPACLEAR.bit.GPIO9 = 1; // Set GPIO9 low to enable DAN28027
    // SpibRegs.SPIFFRX.bit.RXFFIL = 3;  // Issue the SPIB_RX_INT when three values are in the RX FIFO
    // SpibRegs.SPITXBUF = 0x00DA;  // Send start command (0x00DA) to DAN chip
    // SpibRegs.SPITXBUF = DANPWM;  // Send first PWM command
    // fSpibRegs.SPITXBUF = DANPWM;  // Send second PWM command

    // Exercise 3 code - commented out for Exercise 4
    /*
    // increment DANPWM by 10 if value is less than 3000 and countdir is set to upcount (1)
    if ((DANPWM < 3000) && (countdir == 1)) {
        DANPWM = DANPWM + 10;
    }
    // decrement DANPWM by 10 and change count direction if value is equal to 3000 and countdir is set to upcount (1)
    if ((DANPWM == 3000) && (countdir == 1)) {
        countdir = 0;
        DANPWM = DANPWM - 10;
    }
    // decrement DANPWM by 10 if value is greater than 0 and countdir is set to downcount (0)
    if ((DANPWM > 0) && (countdir == 0)) {
        DANPWM = DANPWM - 10;
    }
    // increment DANPWM by 10 and change count direction if value is equal to 0 and countdir is set to downcount (0)
    if ((DANPWM == 0) && (countdir == 0)) {
        countdir = 1;
        DANPWM = DANPWM + 10;
    }
    */

    // Print over UART every 200ms
    // Since Timer0 is called every 1ms, set UARTPrint high once every 200 times Timer0 is called
//    if ((numTimer0calls % 200) == 0) {
//        UARTPrint = 1;
//    }

    // Exercise 3 code - commented out for Exercise 4
    // Clear GPIO9 Low to act as a Slave Select.  Right now, just to scope.  Later to select DAN28027 chip
    // GpioDataRegs.GPACLEAR.bit.GPIO9 = 1; // Set GPIO9 low to enable DAN28027
    // SpibRegs.SPIFFRX.bit.RXFFIL = 2;  // Issue the SPIB_RX_INT when two values are in the RX FIFO
    // SpibRegs.SPITXBUF = 0x4A3B;  // 0x4A3B and 0xB517 have no special meaning.  Wanted to send
    // SpibRegs.SPITXBUF = 0xB517;  // something so you can see the pattern on the Oscilloscope


    if ((numTimer0calls%250) == 0) {
        displayLEDletter(LEDdisplaynum);
        LEDdisplaynum++;
        if (LEDdisplaynum == 0xFFFF) {  // prevent roll over exception
            LEDdisplaynum = 0;
        }
    }

    // Blink LaunchPad Red LED
    // GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{

    // Exercise 3 and 4: the following code implements the PI controller for controlling the robot's speed and turn rate
    RightWheel = readEncRight(); // Read right wheel encoder
    LeftWheel = readEncLeft(); // Read left wheel encoder
    PosRight_K = 0.2*RightWheel; // Approximately 5 radians per foot
    PosLeft_K = 0.2*LeftWheel; // Approximately 5 radians per foot

    // Calculate left and right velocities
    // Each time step is equal to 4ms
    VRight_K = (PosRight_K-PosRight_K_1)/0.004;
    VLeft_K = (PosLeft_K-PosLeft_K_1)/0.004;

    // Calculate error terms
    Err_turn = turn + VLeft_K - VRight_K;
    ErrRight_K = Vref - VRight_K + Kp_turn*Err_turn;
    ErrLeft_K = Vref - VLeft_K - Kp_turn*Err_turn;

    // Calculate integral terms using the trapezoidal rule
    // For anti-windup:
    // Make sure that control effort is not saturated before integrating
    // If saturated, do nothing
    if (fabs(uRight)<10) {
        IRight_K = IRight_K_1 + 0.004*0.5*(ErrRight_K + ErrRight_K_1);
    }

    if (fabs(uLeft)<10) {
        ILeft_K = ILeft_K_1 + 0.004*0.5*(ErrLeft_K + ErrLeft_K_1);
    }

    // Calculate and update control effort
    uRight = Kp*ErrRight_K + Ki*IRight_K;
    uLeft = Kp*ErrLeft_K + Ki*ILeft_K;

    // Right wheel controlled by EPWM 2A
    // Left wheel controlled by EPWM 2B
    setEPWM2A(uRight);
    setEPWM2B(-uLeft); // Need to reverse direction since the motors are on opposite sides of the robot

    // Exercise 5: the following code implements pose calculating for finding the robot's current position

    // Calculate angular rates
    // Once again, each time step is 4ms
    // Angular rate is the (change in angle)/timestep
    theta_R_K = RightWheel;
    theta_L_K = LeftWheel;
    thetadot_R = (theta_R_K - theta_R_K_1)/0.004;
    thetadot_L = (theta_L_K - theta_L_K_1)/0.004;

    // Calculate theta_avg and thetadot_avg
    theta_avg = 0.5*(theta_R_K + theta_L_K);
    thetadot_avg = 0.5*(thetadot_R + thetadot_L);

    // Calculate phi
    phi = radius*(theta_R_K - theta_L_K)/width;

    // Calculate xdot and ydot
    xdot_K = radius*thetadot_avg*cos(phi);
    ydot_K = radius*thetadot_avg*sin(phi);

    // Calculate x and y by integrating xdot and ydot using the trapezoidal rule
    x = x + 0.004*0.5*(xdot_K + xdot_K_1);
    y = y + 0.004*0.5*(ydot_K + ydot_K_1);

    // Update previous values for PID controller
    PosRight_K_1 = PosRight_K;
    PosLeft_K_1 = PosLeft_K;
    ErrRight_K_1 = ErrRight_K;
    ErrLeft_K_1 = ErrLeft_K;
    IRight_K_1 = IRight_K;
    ILeft_K_1 = ILeft_K;

    // Update previous values for pose calculation
    theta_R_K_1 = theta_R_K;
    theta_L_K_1 = theta_L_K;
    xdot_K_1 = xdot_K;
    ydot_K_1 = ydot_K;

    CpuTimer1.InterruptCount++;
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{


    // Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    if (lidar_ready == 1) {
        // Send G command to Lidar
        serial_sendSCIC(&SerialC, G_command, G_len);
    }

    // Read data from buffer that is not being written to
    if (LIDARpingpong == 0) {
        wall_dist = lidar_data[70].distance_pong;
        // fwd_dist = lidar_data[114].distance_pong;
        fwd_dist = 0;
        for (uint16_t i = 100; i < 130; i++) {
            fwd_dist += lidar_data[i].distance_pong;
        }
        fwd_dist /= 30;
    }
    else if (LIDARpingpong == 1) {
        wall_dist = lidar_data[70].distance_ping;
        // fwd_dist = lidar_data[114].distance_ping;
        fwd_dist = 0;
        for (uint16_t i = 100; i < 130; i++) {
            fwd_dist += lidar_data[i].distance_ping;
        }
        fwd_dist /= 30;
    }

    if (corner_state == 0) {
        // Calculate wall following error
        wall_error = wall_ref - wall_dist;
        // Update turn command
        turn = Kp_wall*wall_error;

        // Saturate turn command between -0.5 and 0.5
        if (turn > 0.5) {
            turn = 0.5;
        }
        if (turn < -0.5) {
            turn = -0.5;
        }

        // Check if we are at a corner
        if (fwd_dist < fwd_ref) {
            corner_state = 1;
        }
    }
    else if (corner_state == 1) {
        // Set a more aggressive turn to get around corner
        turn = corner_turn;

        // Check if we have left the corner
        // Use a distance three feet than fwd_ref for hysteresis
        if (fwd_dist > fwd_ref + 2) {
            corner_state = 0;
        }
    }

    // To increase speed: 600Hz -> 800Hz -> 1000Hz
    // To decrease speed: 1000Hz -> 800Hz -> 600Hz
    switch(statecase) {

    // Wait for first tone, could be 1000Hz or 600Hz
    case 10 :
        // Check for 1000Hz tone
        if (iscloseto(maxpwrfreq, 1000, 10)) {
            statecase = 20;
        }
        // Check for 600Hz tone
        else if (iscloseto(maxpwrfreq, 600, 10)) {
            statecase = 50;
        }
        break;

        // States 20 thru 40 used to decrease speed (1000Hz -> 800Hz -> 600Hz)

        // Wait for 800Hz tone after 1000Hz tone has been detected
    case 20 :
        // Check for 800Hz tone
        if (iscloseto(maxpwrfreq, 800, 10)) {
            // Reset state counter
            statecount = 0;
            // Go to state 30
            statecase = 30;
        }
        // Increment state counter if timeout period not reached
        else if (statecount < statetimeout) {
            statecount++;
        }
        else {
            // Reset state counter
            statecount = 0;
            // Return to default state upon timeout
            statecase = 10;
        }
        break;

        // Wait for 600Hz tone after 800Hz tone has been detected
    case 30 :
        // Check for 600Hz tone
        if (iscloseto(maxpwrfreq, 600, 10)) {
            // Reset state counter
            statecount = 0;
            // Go to state 40
            statecase = 40;
        }
        // Increment state counter if timeout period not reached
        else if (statecount < statetimeout) {
            statecount++;
        }
        else {
            // Reset state counter
            statecount = 0;
            // Return to default state upon timeout
            statecase = 10;
        }
        break;

        // Update Vref after 600Hz tone has been detected
    case 40 :
        // Decrease Vref
        Vref -= 0.5;
        // Make sure Vref does not go negative
        if (Vref < 0) {
            Vref = 0;
        }
        // Return to default state
        statecase = 10;
        break;

        // States 50 thru 70 used to increase speed (600Hz -> 800Hz -> 1000Hz)

        // Wait for 800Hz tone after 600Hz tone has been detected
    case 50 :
        // Check for 800Hz tone
        if (iscloseto(maxpwrfreq, 800, 10)) {
            // Reset state counter
            statecount = 0;
            // Go to state 60
            statecase = 60;
        }
        // Increment state counter if timeout period not reached
        else if (statecount < statetimeout) {
            statecount++;
        }
        else {
            // Reset state counter
            statecount = 0;
            // Return to default state upon timeout
            statecase = 10;
        }
        break;

        // Wait for 1000Hz tone after 800Hz tone has been detected
    case 60 :
        // Check for 1000Hz tone
        if (iscloseto(maxpwrfreq, 1000, 10)) {
            // Reset state counter
            statecount = 0;
            // Go to state 70
            statecase = 70;
        }
        // Increment state counter if timeout period not reached
        else if (statecount < statetimeout) {
            statecount++;
        }
        else {
            // Reset state counter
            statecount = 0;
            // Return to default state upon timeout
            statecase = 10;
        }
        break;

        // Update Vref after 1000Hz tone has been detected
    case 70 :
        // Increase Vref
        Vref += 0.5;
        // Make sure Vref does not go too high
        if (Vref > 1.5) {
            Vref = 1.5;
        }
        // Return to default state
        statecase = 10;
        break;
    }

    // Mapping code
    if ((index < 6840) && (lidar_ready == 1) && ((CpuTimer2.InterruptCount % 10) == 0)) {

        if (LIDARpingpong == 0) {
            for (int i = 0; i < 228; i++){
                OBx[i] = x + cos(PI/2.0+phi-2.0/3.0*PI+i*PI/360.0)*lidar_data[i].distance_pong;// position received by lidar at every angle
                OBy[i] = y + sin(PI/2.0+phi-2.0/3.0*PI+i*PI/360.0)*lidar_data[i].distance_pong;
            }
        }
        else if (LIDARpingpong == 1) {
            for (int i = 0; i < 228; i++){
                OBx[i] = x + cos(PI/2.0+phi-2.0/3.0*PI+i*PI/360.0)*lidar_data[i].distance_ping;// position received by lidar at every angle
                OBy[i] = y + sin(PI/2.0+phi-2.0/3.0*PI+i*PI/360.0)*lidar_data[i].distance_ping;
            }
        }

        for (int i = 0; i < 228; i++){
            OBxc[index] = OBx[i];
            index++;
        }
        for (int i = 0; i < 228; i++){
            OByc[index] = OBy[i];
            index++;
        }
    }

    CpuTimer2.InterruptCount++;

//    if ((CpuTimer2.InterruptCount % 50) == 0) {
//        UARTPrint = 1;
//    }
}

int16_t spivalue1 = 0;
int16_t spivalue2 = 0;
__interrupt void SPIB_isr(void){

    // First 16 bits in RX FIFO are just garbage values
    AccelX = SpibRegs.SPIRXBUF;

    // Start of real data
    AccelX = SpibRegs.SPIRXBUF;
    AccelY = SpibRegs.SPIRXBUF;
    AccelZ = SpibRegs.SPIRXBUF;
    Temp = SpibRegs.SPIRXBUF;
    GyroX = SpibRegs.SPIRXBUF;
    GyroY = SpibRegs.SPIRXBUF;
    GyroZ = SpibRegs.SPIRXBUF;

    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Set GPIO66 high to end Slave Select.

    // Scale accelerometer readings between -4g to 4g
    AccelXscaled = 4.0*AccelX/32767.0;
    AccelYscaled = 4.0*AccelY/32767.0;
    AccelZscaled = 4.0*AccelZ/32767.0;

    // Scaled gyroscope reading between -250deg/s to 250deg/s
    GyroXscaled = 250.0*GyroX/32767.0;
    GyroYscaled = 250.0*GyroY/32767.0;
    GyroZscaled = 250.0*GyroZ/32767.0;

    // Exercise 3 code - commented out for Exercise 4

    // spivalue1 = SpibRegs.SPIRXBUF; // Read first 16 bit value off RX FIFO. Probably is zero since no chip
    // spivalue2 = SpibRegs.SPIRXBUF; // Read second 16 bit value off RX FIFO.  Again probably zero

    /*
    ADC1val = SpibRegs.SPIRXBUF; // Read first 16 bit value off RX FIFO. (first value is meaningless, will be overwritten by next line)
    ADC1val = SpibRegs.SPIRXBUF; // Read first 16 bit value off RX FIFO. (ADC1 reading from DAN chip)
    ADC2val = SpibRegs.SPIRXBUF; // Read second 16 bit value off RX FIFO.  (ADC2 reading from DAN chip)

    GpioDataRegs.GPASET.bit.GPIO9 = 1; // Set GPIO9 high to end Slave Select.  Now Scope. Later to deselect DAN28027
    // Later when actually communicating with the DAN28027 do something with the data.  Now do nothing.

    // scale ADC values - 0 to 4095 range corresponds to 0V to 3.3V
    ADC1scaled = 3.3*(ADC1val/4095.0); // Scale ADC1 value
    ADC2scaled = 3.3*(ADC2val/4095.0); // Scale ADC2 value
     */

    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1;      // Clear Overflow flag just in case of an overflow
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1;      // Clear RX FIFO Interrupt flag so next interrupt will happen

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;  // Acknowledge INT6 PIE interrupt

}

