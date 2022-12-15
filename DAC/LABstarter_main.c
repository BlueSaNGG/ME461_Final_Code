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
__interrupt void SPIC_isr(void); // SPI ISR predefinition

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;

// Sine wave generation variables
float sinvals[100]; // Look up table for 1000Hz sin wave
uint16_t sinvals_scaled[100]; // Scaled look up table for 1000Hz sin wave
uint16_t numvals = 100; // Number of values in lookup table
uint16_t sincnt = 0; // Counter that keeps track of current position within sin wave period
extern uint16_t frequency;
extern uint16_t period;

// Setup function for DAC
void setupSpic(void) //Call this function in main() somewhere after the DINT; line of code.
{
    // int16_t temp = 0;
    // cut and paste here all the SpicRegs initializations you found for part 3.  Make sure the TXdelay in between each transfer to 0.    Also don't forget to cut and paste the GPIO settings for GPIO9, 63, 64, 65, 66 which are also a part of the Spic setup.
    // SpicRegs initializations moved into setup function

    // SPI setup
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);  // Set as GPIO125 and used as DAC SS
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);  // Make GPIO125 an Output Pin
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;  //Initially Set GPIO125/SS High so DAC is not selected

    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 0);  // Set as GPIO0 and used as DAC RST
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);  // Make GPIO0 an Output Pin
    GpioDataRegs.GPASET.bit.GPIO0 = 1;  //Initially Set GPIO0/RST High to enable DAC

    GPIO_SetupPinMux(24, GPIO_MUX_CPU1, 0);  // Set as GPIO24 and used as DAC LDAC
    GPIO_SetupPinOptions(24, GPIO_OUTPUT, GPIO_PUSHPULL);  // Make GPIO24 an Output Pin
    GpioDataRegs.GPACLEAR.bit.GPIO24 = 1;  //Initially Set GPIO024/LDAC Low to enable continuous DAC updating

    GPIO_SetupPinMux(122, GPIO_MUX_CPU1, 6);  //Set GPIO122 pin to SPISIMOC (pinmux 6)
    GPIO_SetupPinMux(123, GPIO_MUX_CPU1, 6);  //Set GPIO123 pin to SPISOMIC (pinmux 6)
    GPIO_SetupPinMux(124, GPIO_MUX_CPU1, 6);  //Set GPIO124 pin to SPICLKC (pinmux 6)

    EALLOW;
    GpioCtrlRegs.GPDPUD.bit.GPIO122  = 0;  // Enable Pull-ups on SPI PINs Recommended by TI for SPI Pins
    GpioCtrlRegs.GPDPUD.bit.GPIO123  = 0;
    GpioCtrlRegs.GPDPUD.bit.GPIO124  = 0;
    GpioCtrlRegs.GPDQSEL2.bit.GPIO122 = 3;  // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPDQSEL2.bit.GPIO123 = 3;  // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPDQSEL2.bit.GPIO124 = 3;  // Set I/O pin to asynchronous mode recommended for SPI
    EDIS;

    // ---------------------------------------------------------------------------
    SpicRegs.SPICCR.bit.SPISWRESET = 0;  // Put SPI in Reset (0x0)

    SpicRegs.SPICTL.bit.CLK_PHASE = 1;  //This happens to be the mode for both the DAN28027 and
    SpicRegs.SPICCR.bit.CLKPOLARITY = 1;  //The MPU-9250,  Mode 01.
    SpicRegs.SPICTL.bit.MASTER_SLAVE = 1;  // Set to SPI Master (0x1)
    SpicRegs.SPICCR.bit.SPICHAR = 15;  // Set to transmit and receive 16 bits each write to SPITXBUF (0xF)
    SpicRegs.SPICTL.bit.TALK = 1;  // Enable transmission (0x1)
    SpicRegs.SPIPRI.bit.FREE = 1;  // Free run, continue SPI operation
    SpicRegs.SPICTL.bit.SPIINTENA = 0;  // Disables the SPI interrupt (0x0)

    SpicRegs.SPIBRR.bit.SPI_BIT_RATE = 0;
    // SpicRegs.SPIBRR.bit.SPI_BIT_RATE = 49; // Set SCLK bit rate to 1 MHz so 1us period.  SPI base clock is
    // 50MHZ.  And this setting divides that base clock to create SCLK's period
    // Baud Rate = LSPCLK/(SPICRR + 1)
    // To divide LSPCLK by 50, set SPICRR to 50 - 1 = 49
    SpicRegs.SPISTS.all = 0x0000;  // Clear status flags just in case they are set for some reason

    SpicRegs.SPIFFTX.bit.SPIRST = 1;// Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive. (0x1)
    SpicRegs.SPIFFTX.bit.SPIFFENA = 1;    // Enable SPI FIFO enhancements (0x1)
    SpicRegs.SPIFFTX.bit.TXFIFO =  0;    // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpicRegs.SPIFFTX.bit.TXFFINTCLR = 1;    // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set

    SpicRegs.SPIFFRX.bit.RXFIFORESET = 0;    // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpicRegs.SPIFFRX.bit.RXFFOVFCLR = 1;    // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
    SpicRegs.SPIFFRX.bit.RXFFINTCLR = 1;    // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set
    SpicRegs.SPIFFRX.bit.RXFFIENA = 1;   // Enable the RX FIFO Interrupt.  RXFFST >= RXFFIL (0x1)

    SpicRegs.SPICCR.bit.SPISWRESET = 1;    // Pull the SPI out of reset (0x1)

    SpicRegs.SPIFFTX.bit.TXFIFO = 1;    // Release transmit FIFO from reset. (0x1)
    SpicRegs.SPIFFRX.bit.RXFIFORESET = 1;    // Re-enable receive FIFO operation
    SpicRegs.SPICTL.bit.SPIINTENA = 1;    // Enables SPI interrupt.  !! I don't think this is needed.  Need to Test

    SpicRegs.SPIFFRX.bit.RXFFIL = 16; //Interrupt Level to 16 words or more received into FIFO causes interrupt.  This is just the initial setting for the register.  Will be changed below

    SpicRegs.SPICCR.bit.SPICHAR      = 0xF;
    SpicRegs.SPIFFCT.bit.TXDLY       = 0x00; // Set TXdelay to 0

    // Clear Spic interrupt source just in case it was issued due to any of the above initializations.
    SpicRegs.SPIFFRX.bit.RXFFOVFCLR=1;  // Clear Overflow flag
    SpicRegs.SPIFFRX.bit.RXFFINTCLR=1;  // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}

void writeDAC(uint16_t output, uint16_t channel) {

    uint16_t command_bits = 0;
    uint16_t data_bits = 0;

    // Set up 16 bit value to write over SPI
    // Command bits to write new value is 0001b
    uint16_t command = 1;
    // Address bits to decide which channel to write to
    uint16_t address = 0;
    if (channel == 0) {
        address = 15; // write to all channels
    }
    else if (channel == 1) {
        address = 1; // write to channel A
    }
    else if (channel == 2) {
        address = 2; // write to channel B
    }
    else if (channel == 3) {
        address = 4; // write to channel C
    }
    else if (channel == 4) {
        address = 8; // write to channel D
    }

    // Format data to be sent over SPI
    command_bits = (command << 4) | (address & 0xF);
    data_bits = (output << 2) & 0xFFFF;

    GpioDataRegs.GPDCLEAR.bit.GPIO125 = 1; // Set GPIO125 low to enable DAC
    SpicRegs.SPIFFRX.bit.RXFFIL = 2;  // Issue the SPIC_RX_INT when two values are in the RX FIFO
    SpicRegs.SPITXBUF = command_bits;  // Send command and address bits
    SpicRegs.SPITXBUF = data_bits;  // Send output data bits

}

void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();

    // Blue LED on LaunchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

    // Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

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

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;

    PieVectTable.SPIC_RX_INT = &SPIC_isr; // use pointer to add SPI ISR to PieVectTable

    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 10000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 20); // Start at 20us (500Hz) but can be changed later
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 10000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serialSCIA(&SerialA,115200);
    //    init_serialSCIB(&SerialB,115200);
    //    init_serialSCIC(&SerialC,115200);
    //    init_serialSCID(&SerialD,115200);

    // Enable Spic
    setupSpic();

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT6;  // SPIC_RX
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;

    // Enable SPIC_RX in the PIE: Group 6 interrupt 9
    PieCtrlRegs.PIEIER6.bit.INTx9 = 1;

    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    // Calculate sin values
    for (uint16_t i = 0; i < numvals; i++) {
        // sinvals[i] = 0.1*sin(i*PI/50.0) + 0.1;
        sinvals[i] = sin(i*PI/50.0) + 1.5;

        // Scale values for 14-bit DAC
        sinvals_scaled[i] = 16383*(sinvals[i]/5.0);

        // Limit value to 16383
        if (sinvals_scaled[i] > 16383) {
            sinvals_scaled[i] = 16383;
        }
    }

    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
            // serial_printf(&SerialA,"Num Timer2:%ld Num SerialRX: %ld\r\n",CpuTimer2.InterruptCount,numRXA);
            serial_printf(&SerialA,"Output Frequency (Hz): %d\r\n",frequency);
            UARTPrint = 0;

        }
    }
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

    if ((numTimer0calls%250) == 0) {
        displayLEDletter(LEDdisplaynum);
        LEDdisplaynum++;
        if (LEDdisplaynum == 0xFFFF) {  // prevent roll over exception
            LEDdisplaynum = 0;
        }
    }

    // Blink LaunchPad Red LED
    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
    CpuTimer1.InterruptCount++;

    writeDAC(sinvals_scaled[sincnt], 0); // Update DAC output voltage

    // Increment sincnt if it is less than (numvals - 1), otherwise reset it
    if (sincnt < (numvals - 1)) {
        sincnt++;
    }
    else {
        sincnt = 0;
        ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, period); // Update Timer1 frequency for next period
        CpuTimer1Regs.TCR.all = 0x4000; // Re-enable Timer1 interrupt after changing frequency
    }
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{


    // Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    CpuTimer2.InterruptCount++;

    if ((CpuTimer2.InterruptCount % 50) == 0) {
        UARTPrint = 1;
    }
}


__interrupt void SPIC_isr(void)
{
    GpioDataRegs.GPDSET.bit.GPIO125 = 1; // Set GPIO125 high to end Slave Select

    SpicRegs.SPIFFRX.bit.RXFIFORESET = 0; // Reset RX FIFO pointer to zero
    SpicRegs.SPIFFRX.bit.RXFIFORESET = 1; // Needed to re-enable RX FIFO after reset

    SpicRegs.SPIFFRX.bit.RXFFOVFCLR = 1;      // Clear Overflow flag just in case of an overflow
    SpicRegs.SPIFFRX.bit.RXFFINTCLR = 1;      // Clear RX FIFO Interrupt flag so next interrupt will happen

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;  // Acknowledge INT6 PIE interrupt
}

