#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include <stdio.h>
#include <stdlib.h>
#include <hw_nvic.h>
#include <hw_types.h>

#define GREEN_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define BLUE_LED       (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define RED_LED        (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_BOARD    (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4)))
#define RED_BOARD      (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4)))
#define DE             (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4)))

#define MAX_CHARS       100
#define MAX_MSGS        25
#define MAX_DATA        100
#define MAX_RETRIES     5
#define BC_ADD          255

#define DATA_SIZE_SET        1
#define DATA_SIZE_RESET      0
#define DATA_SIZE_POLL_REQ   0
#define DATA_SIZE_POLL_RES   1
#define DATA_SIZE_SA         1
#define DATA_SIZE_RGB        3
#define DATA_SIZE_ACK        1
#define DATA_SIZE_GET_REQ    0
#define DATA_SIZE_GET_RES    1
#define DATA_SIZE_PULSE      3
#define DATA_SIZE_SQUARE     8
#define DATA_SIZE_SAWTOOTH   7
#define DATA_SIZE_TRIANGLE   8

#define CMD_SET          0
#define CMD_RESET      127
#define CMD_SA        0x7A
#define CMD_RGB       0x48
#define CMD_UART      0x50
#define CMD_ACK       0x70
#define CMD_POLL_RES  0x79
#define CMD_POLL_REQ  0x78
#define CMD_GET_REQ   0x20
#define CMD_GET_RES   0x21
#define CMD_PULSE     0x02
#define CMD_SQUARE    0x03
#define CMD_SAWTOOTH  0X04
#define CMD_TRIANGLE  0X05
#define CMD_PIECE     0x01

#define MY_NULL       0x00

uint8_t SRC_ADD = 0x04;
uint8_t OLD_SRC_ADD = 0x00;

char str[MAX_CHARS];                          //Used to store the entered character
char uart_data[MAX_CHARS];                    //Used to store the data for uart command from position[3]
char type[MAX_CHARS];                         //Used to determine the type of the character entered
uint8_t fields = 0;                           //Used to store the number of fields in th entered command
uint8_t position[MAX_CHARS];                  //Used to store the position of each field

bool cs_enable = false;                       //Used to turn on/off carrier sense
bool ran_re = false;                          //Used to turn on/off Random retransmission
bool ack = false;                             //Used to turn on/off Acknowledgment request
bool in_progress = false;                     //Used for transmission of the data
bool Reset_flag=false;                        //Used to indicate the reset command
bool pulse_flag=false;                        //Used to indicate the pulse command
bool square_flag=false;                       //Used to indicate the square command
bool amp_flag = true;                         //Flag to to switch between amplitude 1 and amplitude 2
bool red_flag = false;                        //Flag to indicate RED LED
bool green_flag = false;                      //Flag to indicate GREEN LED
bool sawtooth_flag = false;                   //Flag to indicate sawtooth wave
bool triangle_flag = false;                   //Flag to indicate triangle wave
bool up = true;                               //Flag to up count while triangle command
bool piece_flag= false;                       //Flag to indicate piecewise wave

uint8_t i = 0;                                //Don't touch i, used in all for loops in ISR
uint8_t j = 0;                                //Don't touch j, used in calculation of fields
uint8_t k = 0;                                //Don't touch k,used in calculation of fields and types
uint8_t l = 0;                                //Don't touch l,used in masking delimiters with null
uint8_t h = 0;                                //Used in some for loop inside inithw function
uint8_t r = 0;                                //Don't touch, Used in Send_packet function
uint8_t e = 0;                                //Used in for loop to nullify all the arrays every cycle
uint8_t t = 0;                                //Don't touch t, used in for loop in Send packet
uint8_t calc = 0;                             //Used to calculate sum of the data in Send packet
uint8_t check_temp = 0;                       //Used to calculate the checksum temporarily in isr
uint8_t numb = 0;                             //Used as index for random sequence array
uint8_t get_value[5] = {0};                   //Used to get the value for get response command
uint8_t po = 0;                               //Used to extract the data after position[3] for uart command
uint8_t zeros = 0;                            //Used to calculate number of zeros in the valid_entry structure
uint8_t length = 0;                           //Used to calculate length of the uart data
uint8_t address = 0;                          //Used to store the address of the given command
uint8_t channel = 0;                          //Used to store the channel of the given command
uint8_t value[MAX_CHARS] = {0};               //Used to store the data/value of the given command
uint8_t valid = 0;                            //Used to validate/invalidate the command given by the user
uint8_t new_address[2] = {0};                 //Used to store the new address
uint8_t seq = 0;                              //Used to store and increment the sequence ID
uint8_t current_index = 0;                    //Indicates the index of the structure that is transmitting
uint8_t current_phase = 0;                    //Indicates the phase of the particular index
uint8_t rxdata[MAX_CHARS] = {0};              //Used to store the received data
uint8_t rx_phase = 0;                         //Used to indicate the phase of the received data
uint8_t sequence[10] = {0,20,3,0,7,25,6,4,2,5}; //Sequence used to generate random numbers for Retranstimeout
uint8_t piece_size=0;                         //Used to store the size of the data of piecewise wave
uint8_t piece_value[100] = {0};               //Used to store the value of the piece wise wave
uint8_t piece_count=0;                        //Count to store the current index of the pice wise wave
uint8_t dummy[2]={0};                         //Dummy variable

uint8_t oldrxphase = 0;                       ///////////
uint8_t oldtxphase = 0;                       //
uint16_t rxtimeout = 0;                       //
uint16_t olddrxphase = 0;                     // Variables used for deadlocks
uint16_t drxtimeout = 0;                      //
uint16_t txtimeout = 0;                       //////////

uint16_t d;                                  //Don't even touch d as it is used in ISR to receive data
uint16_t u = 0;                              //Used to control on board LEDs
uint16_t v = 0;                              //Used to nullify the effect on LEDs by Timer1Isr while startup
uint16_t time = 0;                           //Used to calculate the time period of the pulse command
uint16_t time1 = 0;                          //Used to calculate the time for amplitude 1
uint16_t time2 = 0;                          //Used to calculate the time for amplitude 2
uint16_t amplitude = 0;                      //Used to calculate the amplitude of the pulse command
uint16_t amplitude1 = 0;                     //Used to calculate the Higher amplitude of the square wave
uint16_t amplitude2 = 0;                     //Used to calculate the lower amplitude of the square wave
uint16_t cycles = 0;                         //Used to calculate the cycle for square wave
uint16_t time_count = 0;                     //Used to calculate the time count for pulse command
uint16_t time1_count = 0;                    //Used to count the time1
uint16_t time2_count = 0;                    //Used to count the time2
uint16_t cycle_count = 0;                    //Used to count the cycle
uint16_t value_16[MAX_CHARS] = {0};          //Used to get the 2 bytes of data for pulse command
uint16_t drx_phase = 0;                      //Dummy rx_phase used for Carrier Sense
uint16_t delta = 0;                          //To store delta value for sawtooth and triangle
uint16_t dwell = 0;                          //To store the dwell in 10ms
uint16_t dwell_count = 0;                    //To count the dwell values in terms of cycles
uint16_t delta_count=0;                      //To count the delta values in terms of amplitude
uint16_t delta2to1 = 0;                      //Delta from amplitude 2 to amplitude 1
uint16_t delta1to2 = 0;                      //Delta from amplitude 1 to amplitude 2
uint16_t delta1to2_count = 0;                //To store the delta1to2 count
uint16_t delta2to1_count = 0;                //To store the delta2to1 count


// Array structure of step 6
bool valid_entry[MAX_MSGS]={0};
uint8_t dst_add[MAX_MSGS];
uint8_t seq_id[MAX_MSGS];
uint8_t comm[MAX_MSGS];
uint8_t src_add[MAX_MSGS];
uint8_t chan[MAX_MSGS];
uint8_t size[MAX_MSGS];
uint8_t checksum[MAX_MSGS];
uint8_t data[MAX_MSGS][MAX_DATA];
uint8_t retranscount[MAX_MSGS];
uint16_t retranstimeout[MAX_MSGS]={0};


//Temporary variables for sprintf
char temp1[MAX_CHARS];
char temp2[MAX_CHARS];
char temp3[MAX_CHARS];
char temp4[MAX_CHARS];
char temp5[MAX_CHARS];
char temp6[MAX_CHARS];
char temp7[MAX_CHARS];
char temp8[MAX_CHARS];
char temp9[MAX_CHARS];
char temp10[MAX_CHARS];
char temp11[MAX_CHARS];
char temp12[MAX_CHARS];
char temp13[MAX_CHARS];
char temp14[MAX_CHARS];
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO ports F, A, C peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOA |SYSCTL_RCGC2_GPIOC;

    // Configure LED pins
    GPIO_PORTF_DIR_R |= 0x0E;      // make bits 3,2,1 an output
    GPIO_PORTF_DR2R_R |= 0x0E;     // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R |= 0x0E;      // enable Green, Red, Blue LEDs on board
    GPIO_PORTF_AFSEL_R |= 0x0E;
    GPIO_PORTF_PCTL_R |= GPIO_PCTL_PF1_M1PWM5 | GPIO_PCTL_PF2_M1PWM6 | GPIO_PCTL_PF3_M1PWM7;

    //Configuring PWM 2b,3a,3b
    SYSCTL_RCGC0_R |= SYSCTL_RCGC0_PWM0;
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
    __asm(" NOP");
    __asm(" NOP");
    __asm(" NOP");
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;
    SYSCTL_SRPWM_R = 0;
    PWM1_2_CTL_R = 0;
    PWM1_3_CTL_R = 0;
    PWM1_2_GENB_R = PWM_1_GENB_ACTCMPBD_ZERO | PWM_1_GENB_ACTLOAD_ONE;
    PWM1_3_GENA_R = PWM_1_GENA_ACTCMPAD_ZERO | PWM_1_GENA_ACTLOAD_ONE;
    PWM1_3_GENB_R = PWM_1_GENB_ACTCMPBD_ZERO | PWM_1_GENB_ACTLOAD_ONE;

    PWM1_2_LOAD_R = 1024;
    PWM1_3_LOAD_R = 1024;
    PWM1_INVERT_R = PWM_INVERT_PWM5INV | PWM_INVERT_PWM6INV | PWM_INVERT_PWM7INV;

    PWM1_2_CMPB_R = 0;
    PWM1_3_CMPB_R = 0;
    PWM1_3_CMPA_R = 0;

    PWM1_2_CTL_R = PWM_1_CTL_ENABLE;
    PWM1_3_CTL_R = PWM_1_CTL_ENABLE;
    PWM1_ENABLE_R = PWM_ENABLE_PWM5EN | PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;

    // Configure the solder board LED pins
    GPIO_PORTA_DIR_R |= 0xC0;  // make bit 3 an output
    GPIO_PORTA_DR2R_R |= 0xC0; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTA_DEN_R |= 0xC0;  // enable Green LED on board

    // Configure UART0 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    // Configure DE pin
    GPIO_PORTC_DIR_R |= 0x40;                        // make bit 6 an output
    GPIO_PORTC_DR2R_R |= 0x40;                       // set drive strength to 2mA (not needed since default configuration -- for clarity)

    // Configure UART1 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;                  // turn-on UART1, leave other uarts in same status
    GPIO_PORTC_DEN_R |= 0x00000070;                           // for DE,Uarttx and Uartrx pins
    GPIO_PORTC_AFSEL_R |= 0x00000030;                         // default, added for clarity
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC5_U1TX | GPIO_PCTL_PC4_U1RX;

    // Configure UART1 to 38400 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART1_CTL_R = 0;                                   // turn-off UART1 to allow safe programming
    UART1_CC_R = UART_CC_CS_SYSCLK;                    // use system clock (40 MHz)
    UART1_IBRD_R = 65;                                 // r = 40 MHz / (Nx38.4kHz), set floor(r)=65, where N=16
    UART1_FBRD_R = 7;                                  // round(fract(r)*64)=6.66
    UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN ;  // configure for 8N1 w/ 16-level FIFO
    UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    // Configure Timer 1 as the time base
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 0x00009C40;                     // set load value to 40e3 for 1 KHz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
    TIMER1_CTL_R |= TIMER_CTL_TAEN;

}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

// Function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}

// Function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);
    return UART0_DR_R & 0xFF;
}


void send_packet(uint8_t dst_add1,uint8_t seq1, uint8_t command1, uint8_t channel1, uint8_t size1, uint8_t* data1)
{
    if((r <= 24))
        {
         sprintf(temp2,"Queuing message %u \r\n",seq1);
         putsUart0(temp2);
         dst_add[r] = dst_add1;                                                             //Storing destination address
         src_add[r] = SRC_ADD;                                                              //Storing source address(Not required)
         seq_id[r] = seq1;                                                                  //Storing sequence ID

         if (ack == true)                                                                   //Turning on/off acknowledgment and setting 7th bit of the command to notify that we need acknowledgment
             {if(command1 != 0x70)
                 comm[r] = 0x80 | command1;                                                 //Turning off 7th bit for Acknowledgment Command
             else comm[r] = command1;
             }
         else comm[r] = command1;

         chan[r] = channel1;                                                                //Storing the channel number

         size[r] = size1;                                                                   //Storing the size of the data

         for(t=0;t<size[r];t++)                                                             //Storing the data to 2 dimensional array
         {data[r][t] = *data1;
         data1++;}

         for(t=0;t<size[r];t++)
             calc=calc+data[r][t];
         checksum[r] = ~(dst_add1 + seq1 + comm[r] + SRC_ADD + channel1 + size1 + calc);    //Storing the calculated checksum value
         calc=0;

         valid_entry[r] = true;                                                             //Setting the valid array 1 for that index

         retranscount[r] = 0;                                                               //Setting re-transmit count to 0

         r++;
        }
    else if (valid_entry[0]==0)
        r=0;
}


void Acknowledgemsg()
{
    uint8_t o[MAX_MSGS] = {0};
    o[0] = rxdata[2];
    send_packet(rxdata[1], seq, CMD_ACK, MY_NULL, DATA_SIZE_ACK, o);
    seq=seq+1;
}

bool checksum_check()
{
    check_temp=0;
    for(i=0;i<rxdata[5]+6;i++)
         check_temp = check_temp+rxdata[i];
        check_temp = ~check_temp;
    if(check_temp == rxdata[6+rxdata[5]])
        {
        return true;
        }
    else return false;
}

void checksumerrorled()
{
    GREEN_BOARD = 1;
    v = 2000;
    green_flag = true;
}

void processmsg()
{
    //code to turn on/off RED LED in channel 1 through SET command(channel 1)
    if((rxdata[3]==CMD_SET || rxdata[3]==0x80) && (rxdata[4]==1) && (rxdata[5]==1))
               {
                    if(checksum_check())
                        {
                            if(rxdata[6] == 1)
                                PWM1_2_CMPB_R = 1023;
                            else PWM1_2_CMPB_R = 0;
                            if((rxdata[3] & 0x80) == 0x80)
                                Acknowledgemsg();
                        }
                    else checksumerrorled();

                }

    //code to turn on/off GREEN LED in channel 2 through SET command(channel 2)
    if((rxdata[3]==CMD_SET || rxdata[3]==0x80) && (rxdata[4]==2) && (rxdata[5]==1))
               {
                    if(checksum_check())
                        {
                            if(rxdata[6] == 1)
                                PWM1_3_CMPB_R = 1023;
                            else PWM1_3_CMPB_R = 0;
                            if((rxdata[3] & 0x80) == 0x80)
                                Acknowledgemsg();
                        }
                    else checksumerrorled();
                }

    //code to turn on/off BLUE LED in channel 3 through SET command(channel 3)
    if((rxdata[3]==CMD_SET || rxdata[3]==0x80) && (rxdata[4]==3) && (rxdata[5]==1))
               {
                    if(checksum_check())
                        {
                            if(rxdata[6] == 1)
                                PWM1_3_CMPA_R = 1023;
                            else PWM1_3_CMPA_R = 0;
                            if((rxdata[3] & 0x80) == 0x80)
                                Acknowledgemsg();
                        }
                    else checksumerrorled();
                }

    //Code to turn on/off 3 LEDs in channel 1 through RGB command(channel 4)
    if((rxdata[3]==CMD_RGB || rxdata[3]==0xC8) && (rxdata[4]==4) && (rxdata[5]==3))
               {
                   if(checksum_check())
                       {
                       PWM1_2_CMPB_R = rxdata[6]*4;
                       PWM1_3_CMPB_R = rxdata[7]*4;
                       PWM1_3_CMPA_R = rxdata[8]*4;
                           if((rxdata[3] & 0x80) == 0x80)
                               Acknowledgemsg();
                       }
                   else checksumerrorled();

               }

    //Code to display uart data received(channel 5)
    if((rxdata[3]==CMD_UART || rxdata[3]==0xD0) && (rxdata[4]==5))
                {
                     if(checksum_check())
                         {
                         for(i=0;i<rxdata[5];i++)
                             putcUart0(rxdata[6+i]);
                         putsUart0("\r\n");
                         if((rxdata[3] & 0x80) == 0x80)
                             Acknowledgemsg();
                         }
                     else checksumerrorled();
                }

    //Code to generate flag for the received pulse command in multiples of 10ms(channel 6)
    if((rxdata[3]==CMD_PULSE || rxdata[3]==0x82) && (rxdata[4]==6))
                {
                     if(checksum_check())
                         {
                         pulse_flag = 1;
                         amplitude = rxdata[6]*4;
                         time = ((rxdata[7]*256)+(rxdata[8]))*10;
                         if((rxdata[3] & 0x80) == 0x80)
                             Acknowledgemsg();
                         }
                     else checksumerrorled();
                }

    //Code to generate square (channel 7)
    if((rxdata[3]==CMD_SQUARE || rxdata[3]==0x83) && (rxdata[4]==7))
                {
                     if(checksum_check())
                         {
                         square_flag = 1;
                         amplitude1 = rxdata[6]*4;
                         amplitude2 = rxdata[7]*4;
                         time1 = ((rxdata[8]*256)+(rxdata[9]))*10;
                         time2 = ((rxdata[10]*256)+(rxdata[11]))*10;
                         cycles = ((rxdata[12]*256)+(rxdata[13]));
                         if((rxdata[3] & 0x80) == 0x80)
                             Acknowledgemsg();
                         }
                     else checksumerrorled();

                }

    //Code to generate sawtooth wave (channel 8)
    if((rxdata[3]==CMD_SAWTOOTH || rxdata[3]==0x84) && (rxdata[4]==8))
                {
                         if(checksum_check())
                             {
                             sawtooth_flag = 1;
                             amplitude1 = rxdata[6]*4;
                             amplitude2 = rxdata[7]*4;
                             delta = rxdata[8]*4;
                             dwell = ((rxdata[9]*256)+(rxdata[10]))*10;
                             cycles = (rxdata[11]*256)+(rxdata[12]);
                             if((rxdata[3] & 0x80) == 0x80)
                                 Acknowledgemsg();
                             }
                         else checksumerrorled();

                }

    //Code to generate triangle wave (channel 9)
    if((rxdata[3]==CMD_TRIANGLE || rxdata[3]==0x85) && (rxdata[4]==9))
                {
                         if(checksum_check())
                             {
                             triangle_flag = true;
                             amplitude1 = rxdata[6]*4;
                             amplitude2 = rxdata[7]*4;
                             delta1to2 = rxdata[8]*4;
                             delta2to1 = rxdata[9]*4;
                             dwell = ((rxdata[10]*256)+(rxdata[11]))*10;
                             cycles = (rxdata[12]*256)+(rxdata[13]);
                             if((rxdata[3] & 0x80) == 0x80)
                                 Acknowledgemsg();
                             }
                         else checksumerrorled();

                }

    //Code to generate piecewise (channel 10)
    if((rxdata[3]==CMD_PIECE || rxdata[3]==0x81) && (rxdata[4]==10))
                {
                         if(checksum_check())
                             {
                             piece_flag = true;
                             piece_size = rxdata[6];
                             for(i=0;i<piece_size;i++)
                             {
                                 piece_value[i] = rxdata[7+i]*4;
                             }
                             dwell = ((rxdata[7+piece_size]*256)+(rxdata[8+piece_size]))*10;
                             cycles = (rxdata[9+piece_size]*256)+(rxdata[10+piece_size]);
                             if((rxdata[3] & 0x80) == 0x80)
                                 Acknowledgemsg();
                             }
                         else checksumerrorled();

                }

    //Code to display poll response
    if((rxdata[0]==SRC_ADD) && (rxdata[3]==CMD_POLL_RES || rxdata[3]==0xF9) && (rxdata[5]==DATA_SIZE_POLL_RES))
                {
                     if(checksum_check())
                         {
                         if((rxdata[3] & 0x80) == 0x80)
                             Acknowledgemsg();
                         sprintf(temp11,"Poll response received from %u\r\n",rxdata[1]);
                         putsUart0(temp11);

                         }
                     else checksumerrorled();
                }

    //Code to respond to the poll message
    if((rxdata[0]==BC_ADD) && (rxdata[3]==CMD_POLL_REQ || rxdata[3]==0xF8) && (rxdata[5]==DATA_SIZE_POLL_REQ))
                {
                     if(checksum_check())
                         {
                         if((rxdata[3] & 0x80) == 0x80)
                             Acknowledgemsg();
                         dummy[0]=SRC_ADD;
                         send_packet(rxdata[1],seq,CMD_POLL_RES,channel,DATA_SIZE_POLL_RES,dummy);
                         seq=seq+1;
                         }
                     else checksumerrorled();
                }

    //Code to respond to the get message
    if((rxdata[3]==CMD_GET_REQ || rxdata[3]==0xA0) && (rxdata[5]==DATA_SIZE_GET_REQ))
                {
                     if(checksum_check())
                         {
                         if(rxdata[4]==1)
                         {   get_value[0]=PWM1_2_CMPB_R/1023;
                             if((rxdata[3] & 0x80) == 0x80)
                                 Acknowledgemsg();
                             send_packet(rxdata[1],seq,CMD_GET_RES,rxdata[4],DATA_SIZE_GET_RES,get_value);
                             seq=seq+1;

                         }
                         if(rxdata[4]==2)
                         {
                             get_value[0]=PWM1_3_CMPB_R/1023;
                             if((rxdata[3] & 0x80) == 0x80)
                                 Acknowledgemsg();
                             send_packet(rxdata[1],seq,CMD_GET_RES,rxdata[4],DATA_SIZE_GET_RES,get_value);
                             seq=seq+1;

                         }
                         if(rxdata[4]==3)
                         {
                             get_value[0]=PWM1_3_CMPA_R/1023;
                             if((rxdata[3] & 0x80) == 0x80)
                                 Acknowledgemsg();
                             send_packet(rxdata[1],seq,CMD_GET_RES,rxdata[4],DATA_SIZE_GET_RES,get_value);
                             seq=seq+1;

                         }
                         }
                     else checksumerrorled();
                }

    //Code to display get response
    if((rxdata[3]==CMD_GET_RES || rxdata[3]==0xA1) && (rxdata[5]==DATA_SIZE_GET_RES))
                {
                     if(checksum_check())
                         {
                         sprintf(temp12,"The status of the channel %u in the address %u is %u\r\n",rxdata[4],rxdata[1],rxdata[6]);
                         putsUart0(temp12);
                         if((rxdata[3] & 0x80) == 0x80)
                             Acknowledgemsg();
                         }
                     else checksumerrorled();
                }

    //Code to raise the flag for Reset
    if((rxdata[3]==CMD_RESET || rxdata[3]==0xFF) && (rxdata[5]==DATA_SIZE_RESET))
                {
                     if(checksum_check())
                         {
                         Reset_flag = 1;
                         if((rxdata[3] & 0x80) == 0x80)
                             Acknowledgemsg();
                         }
                     else checksumerrorled();
                }


    //Code to invalidate the message after the Acknowledgment is received
    if((rxdata[3]==CMD_ACK || rxdata[3]==0xF0) && (rxdata[5]==DATA_SIZE_ACK))
               {
                   if(checksum_check())
                       {
                           for(i=0;i<25;i++)
                           {
                               if(rxdata[6] == seq_id[i])
                                   {
                                   valid_entry[i]=0;
                                   break;
                                   }
                           }
                           sprintf(temp13,"Ack received for the ID %u\r\n",seq_id[i]);
                           putsUart0(temp13);
                       }
                   else checksumerrorled();
               }

    //Code to set new address
    if((rxdata[3]==CMD_SA || rxdata[3]==0xFA) && (rxdata[5]==DATA_SIZE_SA))
    {
        if(checksum_check())
        {
            OLD_SRC_ADD = SRC_ADD;
            SRC_ADD = rxdata[6];
            sprintf(temp14,"My address changed from %u to %u ",OLD_SRC_ADD,SRC_ADD);
            putsUart0(temp14);
            if((rxdata[3] & 0x80) == 0x80)
                Acknowledgemsg();
        }
        else checksumerrorled();
    }

}

void Transmit()
{
    DE = 1;

        if(current_phase == 0)
        {UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN | UART_LCRH_SPS | UART_LCRH_PEN ; //Transmitting with parity bit 1
        if ((UART1_FR_R & UART_FR_BUSY) == 0)
        {UART1_DR_R = dst_add[current_index];
         current_phase = current_phase+1;}}

        if(current_phase == 1)
        {if ((UART1_FR_R & UART_FR_BUSY) == 0)
         {UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN | UART_LCRH_SPS | UART_LCRH_PEN | UART_LCRH_EPS; //Transmitting with parity bit 0
          UART1_DR_R = src_add[current_index];
          current_phase = current_phase+1;}}

        if(current_phase == 2)
        {if ((UART1_FR_R & UART_FR_TXFF) == 0)
         {UART1_DR_R = seq_id[current_index];
         current_phase = current_phase+1;}}

        if(current_phase == 3)
        {if ((UART1_FR_R & UART_FR_TXFF) == 0)
         {UART1_DR_R = comm[current_index];
         current_phase = current_phase+1;}}

        if(current_phase == 4)
        {if ((UART1_FR_R & UART_FR_TXFF) == 0)
         {UART1_DR_R = chan[current_index];
         current_phase = current_phase+1;}}

        if(current_phase == 5)
        {if ((UART1_FR_R & UART_FR_TXFF) == 0)
         {UART1_DR_R = size[current_index];
         current_phase = current_phase+1;}}

        if((current_phase >= 6) && (current_phase <= (size[current_index]+5)))
        {
            for(i=current_phase;i<(size[current_index]+6);i++)
                {if((UART1_FR_R & UART_FR_TXFF) == 0)
                    {UART1_DR_R = data[current_index][current_phase-6];
                     current_phase = current_phase+1;}
                else break;
                }
        }

        if(current_phase == (6+size[current_index]))
        {if ((UART1_FR_R & UART_FR_TXFF) == 0)
         {UART1_DR_R = checksum[current_index];
         current_phase = current_phase+1;}}

        if(current_phase == (size[current_index]+7))
        {
            if((UART1_FR_R & UART_FR_BUSY) == 0)
            {
                txtimeout=0;
                DE=0;
                in_progress = false;
                current_phase = 0;
                if(ack == 0)
                   valid_entry[current_index]=false;
                else if(ack == 1)
                    {
                        if(comm[current_index] == 0x70)
                            valid_entry[current_index]=0;
                        else
                            {
                            retranscount[current_index]++;
                            if(ran_re == 0)
                            retranstimeout[current_index] = 600 + (2^retranscount[current_index])*100;  //Can be changed according to the requirement
                            else
                                {
                                retranstimeout[current_index] = 600 + (2^sequence[numb])*100;
                                numb=(numb+1)%10;
                                }
                            if(retranscount[current_index] > 1)
                                {
                                sprintf(temp3,"Transmitting message %u, retransmission attempt %u\r\n",seq_id[current_index],(retranscount[current_index]-1));
                                putsUart0(temp3);
                                if(retranscount[current_index] > MAX_RETRIES)
                                    {
                                        valid_entry[current_index]=false;
                                        RED_BOARD = 1;
                                        u = 2000;
                                        red_flag = true;
                                        sprintf(temp1,"Message %u was retransmitted %u times, but failed\r\n",seq_id[current_index],retranscount[current_index]-1);
                                        putsUart0(temp1);
                                    }
                                }

                            }
                    }
            }
        }
}

void Timer1Isr()
{

    if(!in_progress)
    {
        for(i=0;i<MAX_MSGS;i++)
        {
            if((valid_entry[i] == true)&&(retranstimeout[i] == 0))
                {
                    RED_BOARD = 1;
                    u = 50;
                    red_flag = true;
                    in_progress = true;
                    current_index = i;
                    current_phase = 0x00;
                    if(retranscount[current_index] == 0)
                        {
                        sprintf(temp7,"Transmitting message %u\r\n",seq_id[current_index]);
                        putsUart0(temp7);
                        }
                    break;
                }
        }
    }

    if(in_progress)
    {
        if(cs_enable == true)
            {
            if((rx_phase == 0) && (drx_phase == 0))
                {
                Transmit();
                }
            }
        else if(rx_phase == 0)
            {
            Transmit();
            }
    }

    for(i=0;i<MAX_MSGS;i++)
      {if(retranstimeout[i]>0)
          retranstimeout[i]--;}

       //Step 8 Receive side
        if(current_phase == 0)
            {
             UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN | UART_LCRH_SPS | UART_LCRH_PEN | UART_LCRH_EPS;
             while((UART1_FR_R & UART_FR_RXFE)==0)
             {
                 drx_phase++;
                 drxtimeout = 0;
                 d = UART1_DR_R;
                 if(d & 0x200)
                 {
                     rx_phase = 0;
                     rxdata[rx_phase] = (d & 0xFF);
                     if(((d & 0xFF) == SRC_ADD) || ((d & 0xFF) == BC_ADD))
                         {
                         GREEN_BOARD = 1;
                         v = 50;
                         green_flag = true;
                         rx_phase++;
                         }

                 }
                 else
                 {
                     if(rx_phase != 0)
                     {
                         rxdata[rx_phase] = d;
                         rx_phase++;
                     }
                     if(rx_phase == (rxdata[5]+7))
                         {
                          rx_phase = 0;
                          sprintf(temp8,"Destination address = %u, Sequence ID = %u, Command = %u, Channel = %u, Size = %u, data = %u, checksum = %u \r\n",rxdata[1],rxdata[2],rxdata[3],rxdata[4],rxdata[5],rxdata[6],rxdata[7]);
                          putsUart0(temp8);
                          processmsg();
                          break;
                         }
                 }
                 rxtimeout=0;
             }
            }

        //Receive deadlock
        if((oldrxphase == rx_phase) && (rx_phase !=0))
        {
            rxtimeout++;
            if(rxtimeout == 10)
                {rx_phase=0;
                rxtimeout=0;}
        }
        oldrxphase=rx_phase;

        //Dummy receive deadlock for carrier sense
        if((olddrxphase == drx_phase) && (drx_phase !=0))
        {
            drxtimeout++;
            if(drxtimeout == 10)
                {
                drx_phase=0;
                drxtimeout=0;
                }
        }
        olddrxphase=drx_phase;

        //Transmit deadlock
        if((oldtxphase == current_phase) && (current_phase != 0))
        {
            txtimeout++;
            if(txtimeout == 10)
                {current_phase=0;
                txtimeout=0;}
        }
        oldtxphase=current_phase;


        //Used to reset the board after sending all the messages in the structure
        if(Reset_flag == 1)
            {
            zeros=0;
            for(i=0;i<MAX_MSGS;i++)
                {
                    if(valid_entry[i]==0)
                        zeros=zeros+1;
                }
            if(zeros==25)
                HWREG(NVIC_APINT) = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
            }

        //Used for pulse command
        if(pulse_flag == 1)
            {
                time_count++;
                PWM1_2_CMPB_R = amplitude;
                if(time_count == time)
                    {
                    PWM1_2_CMPB_R = 0;
                    time_count = 0;
                    pulse_flag = false;
                    }

            }

        //Used for square command
        if(square_flag)
            {
                if(amp_flag)
                {
                    PWM1_3_CMPB_R = amplitude1;
                    time1_count++;
                    if(time1_count == time1)
                    {
                        time1_count = 0;
                        amp_flag = false;
                    }

                }
                if(!amp_flag)
                {
                    PWM1_3_CMPB_R = amplitude2;
                    time2_count++;
                    if(time2_count == time2)
                    {
                        time2_count = 0;
                        amp_flag = true;
                        cycles--;
                    }
                }
                if(cycles == 0)
                    {
                    square_flag = false;
                    PWM1_3_CMPB_R = 0;
                    }

            }

        //Used for sawtooth command
        if(sawtooth_flag)
        {
            if(dwell_count <= dwell)
            {
                if((amplitude2+delta*delta_count) <= amplitude1)
                    {
                        PWM1_3_CMPA_R = amplitude2+delta*delta_count;
                        dwell_count++;
                    }
                else
                    {
                        PWM1_3_CMPA_R = 0;
                        delta_count=0;
                        dwell_count=0;
                        cycles--;
                    }
                if(cycles == 0)
                    {
                        sawtooth_flag = false;
                        PWM1_3_CMPA_R = 0;
                        dwell_count=0;
                        delta_count=0;
                    }
            }
            else
            {
                delta_count++;
                dwell_count=0;
            }
        }

        //Used for triangle command
        if(triangle_flag)
        {
            if(dwell_count <= dwell)
                        {
                            if(up)
                            {
                                if((amplitude2+(delta2to1)*(delta2to1_count)) <= amplitude1)
                                    {
                                        PWM1_3_CMPA_R = (amplitude2+(delta2to1)*(delta2to1_count));
                                        dwell_count++;
                                    }
                                else
                                    {
                                    PWM1_3_CMPA_R = amplitude1;
                                    up = false;
                                    dwell_count=0;
                                    delta1to2_count=0;
                                    }
                            }
                            if(!up)
                            {
                                if((amplitude1-(delta1to2)*(delta1to2_count)) >= amplitude2)
                                    {
                                        PWM1_3_CMPA_R = (amplitude1-(delta1to2)*(delta1to2_count));
                                        dwell_count++;
                                    }
                                else
                                    {
                                        PWM1_3_CMPA_R = amplitude2;
                                        up = true;
                                        cycles--;
                                        dwell_count=0;
                                        delta2to1_count=0;
                                    }
                            }
                            if(cycles == 0)
                                {
                                    triangle_flag = false;
                                    PWM1_3_CMPA_R = 0;
                                }
                        }
            else
            {
                if(up) delta2to1_count++;
                if(!up) delta1to2_count++;
                dwell_count=0;
            }
        }


        //Used for piecewise command
        if(piece_flag)
        {
            if(dwell_count <= dwell)
            {
                if(piece_count < piece_size)
                {
                    PWM1_3_CMPA_R = piece_value[piece_count];
                    dwell_count++;
                }
                else
                {
                    cycles--;
                    piece_count = 0;
                }
            }
            else
            {
                dwell_count=0;
                piece_count++;
            }
            if(cycles==0)
            {
                PWM1_3_CMPA_R = 0;
                piece_flag = false;
                for(i=0;i<piece_count;i++)
                    piece_value[i]=0;
            }
        }

        //Used to turn on/off Red LED
        if(red_flag)
        {
            u--;
            if(u==0)
            {
                red_flag = false;
                RED_BOARD = 0;
            }
        }

        //Used to turn on/off Green LED
        if(green_flag)
        {
            v--;
            if(v==0)
            {
                green_flag = false;
                GREEN_BOARD = 0;
            }
        }

    TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
}

//Function used to check whether the command and parameters entered are matching
int is_command(char* strx, int fieldx)
{
    if(strcmp(strx, &str[position[0]])==0)
    {
        if(fieldx<=fields-1)
            return 1;
    }
    return 0;
}


//Function used to extract the command from the entered string
char* get_string(uint8_t field)
{
    return &str[position[field]];
}

//Function used to store the entered numbers in integer format
int get_number(uint8_t z)
{
    uint8_t a;
    a = atoi(&str[position[z]]);
    return a;
}

//Function used to get the 16 bit number
int get_16number(uint16_t z)
{
    uint16_t a;
    a = atoi(&str[position[z]]);
    return a;
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
      // Initialize hardware
      initHw();
      // Turn on Green LED for 500msec
      GREEN_BOARD = 1;
      waitMicrosecond(500000);
      GREEN_BOARD = 0;
      putsUart0("\r\nReady\r\n");

    while(1)
    {
        for(e=0;e<MAX_CHARS;e++)
        {
            str[e] = 0;
            uart_data[e] = 0;
            position[e] = 0;
            type[e]=0;
            value[e]=0;
        }

        length = 0;
        fields = 0;
        j=0;
        valid=0;

      uint8_t count = 0;
      for(count=0;count<MAX_CHARS;count++)    //This loop is to get the valid character from the user and to store it in a string
      {
          str[count]=getcUart0();
          while(str[0]==8)                        //To avoid negative count if the first character entered is backspace
              str[0]=getcUart0();

          if(str[count]==8)                       //To delete the previous character if Backspace is entered
              {str[count]=0x00;
              count=count-2;}

          while(str[count]<=31 && str[count]!=8 && str[count]!=13 && str[count]!=0x00)      //To show error if the entered character is not valid
          {
              if(str[count]<=31)
              {
                  if(str[count]!=8)
                      {putsUart0("Enter a valid character\r\n");
                      str[count]=getcUart0();}
                  else if(str[count]!=13)
                      {putsUart0("Enter a valid character\r\n");
                      str[count]=getcUart0();}
              }
          }

          if(str[count]==13)                                            //To terminate the loop if the Character Return key is pressed
              {str[count]=0x00;
              break;}
      }

      if(count>=97)                                                    //To show error message if the entered command is too lengthy
      {putsUart0("Your command is too lengthy\r\n");}
      else                                                             //To perform operations on entered string if it matches all the above conditions
          {
              uint8_t p;
              putsUart0(str);                           //Discarding step
              putsUart0("\r\n");                        //Discarding step

              for(k=0 ; k < strlen(str) ; k++)                                  //To get the positions, type and fields of the entered string
              {
                 if(str[k] >= 48 && str[k] <= 57)
                 {
                     position[j]=k;
                     type[j]='n';
                     j=j+1;
                     while(str[k+1]>=48 & str[k+1]<=57)
                     k=k+1;
                 }
                 if((str[k]>=65 && str[k]<=90) | (str[k]>=97 && str[k]<=122))
                 {
                     position[j]=k;
                     type[j]='a';
                     j=j+1;
                     while((str[k+1]>=65 && str[k+1]<=90) || (str[k+1]>=97 && str[k+1]<=122))
                     k=k+1;
                  }
               }
              fields=j;

              for(po=0;po<100;po++)                                             //To take data from position[3] for th UART command
                      uart_data[po] = str[position[3]+po];

              for(p=0; p < strlen(str); p++)                                    //To convert all data from Upper case to lower case
              {
                  if(str[p]>=65 && str[p]<=90)
                  str[p]=str[p]+32;
              }


              for(l=0;l<=k;l++)                                                  //To pad zeros for all the delimiters
              {
                     if((str[l]>=48 && str[l]<=57) || (str[l]>=65 && str[l]<=90) || (str[l]>=97 && str[l]<=122))
                         str[l]=str[l];
                     else
                         str[l]=0x00;
              }


           if(is_command("set",3))                                               //SET command loop
           {
               if( (type[1]=='n') && (type[2]=='n') && (type[3]=='n'))
               {
               valid = 1;
               address = get_number(1);
               channel = get_number(2);
               value[0] = get_number(3);
               send_packet(address,seq,CMD_SET,channel,DATA_SIZE_SET, value);
               seq = seq + 1;
               value[0] = 0;
               }
               else putsUart0("Please enter numbers as parameters for the SET command\r\n");
           }

           else if(is_command("rgb",5))                                              //RGB command loop
           {
               if( (type[1]=='n') && (type[2]=='n') && (type[3]=='n') && (type[4]=='n') && (type[5]=='n'))
               {
               valid = 1;
               address = get_number(1);
               channel = get_number(2);
               value[0] = get_number(3);
               value[1] = get_number(4);
               value[2] = get_number(5);
               send_packet(address,seq,CMD_RGB,channel,DATA_SIZE_RGB, value);
               seq = seq + 1;
               channel = 0;
               value[0] = 0;
               value[1] = 0;
               value[2] = 0;
               }
               else putsUart0("Please enter numbers as parameters for the RGB command\r\n");
           }

           else if(is_command("cs",1))                                         //Carrier Sense command loop
           {
               valid = 1;
               if(strcmp(get_string(1),"on")==0)
                       {
                       cs_enable=1;
                       sprintf(temp9,"Carrier Sense is on\r\n");
                       putsUart0(temp9);
                       }
               else if(strcmp(get_string(1),"off")==0)
                       {
                       sprintf(temp10,"Carrier Sense is off\r\n");
                       putsUart0(temp10);
                       cs_enable=0;
                       }

               else putsUart0("Please enter 'cs on' or 'cs off'\r\n ");

           }

           else if(is_command("uart",3))                                              //LCD command loop
           {
                       if( (type[1]=='n') && (type[2]=='n') )
                       {
                       valid = 1;
                       length = strlen(&uart_data[0]);
                       address = get_number(1);
                       channel = get_number(2);
                       send_packet(address,seq,CMD_UART,channel,length,(uint8_t*) &uart_data[0]);
                       seq = seq + 1;
                       }
                       else putsUart0("Please enter numbers as parameters for UART command\r\n");
           }

           else if(is_command("reset",1))                                      //RESET command loop
           {
               if(type[1]=='n')
               {
               valid = 1;
               address = get_number(1);
               send_packet(address,seq,CMD_RESET,MY_NULL,DATA_SIZE_RESET,value);
               seq = seq+1;
               address = 0;
               }
               else putsUart0("Please enter number as parameter for RESET command\r\n");
           }

           else if(is_command("random",1))                                     //RANDOM command loop
           {
               valid = 1;
               if(strcmp(get_string(1),"on")==0)
                      {
                       ran_re=1;
                       putsUart0("Random retransmission is on\r\n");
                      }
               else if(strcmp(get_string(1),"off")==0)
                      {
                       ran_re=0;
                       putsUart0("Random retransmission  is off\r\n");
                      }
               else putsUart0("Please enter 'random on' or 'random off'\r\n ");
           }

           else if(is_command("get",2))                                        //GET command loop
           {
               if( (type[1]=='n') && (type[2]=='n') )
               {
               valid=1;
               address = get_number(1);
               channel = get_number(2);
               send_packet(address,seq,CMD_GET_REQ,channel,DATA_SIZE_GET_REQ,MY_NULL);
               seq = seq + 1;
               }
               else putsUart0("Please enter numbers as parameters for GET command");
           }

           else if(is_command("poll",0))                                      //POLL command loop
           {
               valid=1;
               address = BC_ADD;
               send_packet(address,seq,CMD_POLL_REQ,MY_NULL,DATA_SIZE_POLL_REQ,MY_NULL);
               seq=seq+1;
           }

           else if(is_command("sa",2))                                        //Set address command loop
           {
               valid=1;
               address = get_number(1);
               new_address[0] = get_number(2);
               channel = 0;
               send_packet(address,seq,CMD_SA,channel,DATA_SIZE_SA,new_address);
               seq=seq+1;
           }

           else if(is_command("ack",1))                                       //Acknowledge command loop
           {
              valid = 1;
              if(strcmp(get_string(1),"on")==0)
                      {ack=true;
                      sprintf(temp4,"Acknowledgment request is on\r\n");
                      putsUart0(temp4);}
              else if(strcmp(get_string(1),"off")==0)
                      {ack=false;
                      sprintf(temp5,"Acknowledgment request is off\r\n");
                      putsUart0(temp5);}
              else putsUart0("Please enter 'ack on' or 'ack off'\r\n ");
           }

           else if(is_command("pulse",4))
           {
               if( (type[1]=='n') && (type[2]=='n') && (type[3]=='n') && (type[4]=='n'))
               {
                         valid = 1;
                         address = get_number(1);
                         channel = get_number(2);
                         value[0] = get_number(3);
                         value_16[0] = get_16number(4)/256;
                         value_16[1] = get_16number(4)%256;
                         value[1] = (uint8_t) value_16[0];
                         value[2] = (uint8_t) value_16[1];
                         send_packet(address,seq,CMD_PULSE,channel,DATA_SIZE_PULSE,value);
                         seq=seq+1;
               }
               else putsUart0("Please enter numbers as parameters for the PULSE command");
           }

           else if(is_command("square",7))
           {
               if( (type[1]=='n') && (type[2]=='n') && (type[3]=='n') && (type[4]=='n') && (type[5]=='n') && (type[6]=='n') && (type[7]=='n'))
               {
               valid = 1;
               address = get_number(1);
               channel = get_number(2);
               value[0] = get_number(3);
               value[1] = get_number(4);
               value_16[0] = get_16number(5)/256;
               value_16[1] = get_16number(5)%256;
               value_16[2] = get_16number(6)/256;
               value_16[3] = get_16number(6)%256;
               value_16[4] = get_16number(7)/256;
               value_16[5] = get_16number(7)%256;
               value[2] = (uint8_t) value_16[0];
               value[3] = (uint8_t) value_16[1];
               value[4] = (uint8_t) value_16[2];
               value[5] = (uint8_t) value_16[3];
               value[6] = (uint8_t) value_16[4];
               value[7] = (uint8_t) value_16[5];
               send_packet(address,seq,CMD_SQUARE,channel,DATA_SIZE_SQUARE,value);
               seq=seq+1;
               }
               else putsUart0("Please enter numbers as parameters for the SQUARE command");
           }

           else if(is_command("sawtooth",7))
           {
               if( (type[1]=='n') && (type[2]=='n') && (type[3]=='n') && (type[4]=='n') && (type[5]=='n') && (type[6]=='n') && (type[7]=='n') )
               {
                  valid = 1;
                  address = get_number(1);
                  channel = get_number(2);
                  value[0] = get_number(3);
                  value[1] = get_number(4);
                  value[2] = get_number(5);
                  value_16[0] = get_16number(6)/256;
                  value_16[1] = get_16number(6)%256;
                  value_16[2] = get_16number(7)/256;
                  value_16[3] = get_16number(7)%256;
                  value[3] = (uint8_t) value_16[0];
                  value[4] = (uint8_t) value_16[1];
                  value[5] = (uint8_t) value_16[2];
                  value[6] = (uint8_t) value_16[3];
                  send_packet(address,seq,CMD_SAWTOOTH,channel,DATA_SIZE_SAWTOOTH,value);
                  seq=seq+1;
               }
               else putsUart0("Please enter numbers as parameters for the SAWTOOTH command");
           }

           else if(is_command("triangle",8))
              {
                  if( (type[1]=='n') && (type[2]=='n') && (type[3]=='n') && (type[4]=='n') && (type[5]=='n') && (type[6]=='n') && (type[7]=='n') &&  (type[8]=='n'))
                  {
                     valid = 1;
                     address = get_number(1);
                     channel = get_number(2);
                     value[0] = get_number(3);
                     value[1] = get_number(4);
                     value[2] = get_number(5);
                     value[3] = get_number(6);
                     value_16[0] = get_16number(7)/256;
                     value_16[1] = get_16number(7)%256;
                     value_16[2] = get_16number(8)/256;
                     value_16[3] = get_16number(8)%256;
                     value[4] = (uint8_t) value_16[0];
                     value[5] = (uint8_t) value_16[1];
                     value[6] = (uint8_t) value_16[2];
                     value[7] = (uint8_t) value_16[3];
                     send_packet(address,seq,CMD_TRIANGLE,channel,DATA_SIZE_TRIANGLE,value);
                     seq=seq+1;
                  }
                  else putsUart0("Please enter numbers as parameters for the TRIANGLE command");
             }

           else if(is_command("piece",3))
                         {
                             if((type[1]=='n') && (type[2]=='n') && (type[3]=='n'))
                             {
                                valid = 1;
                                address = get_number(1);
                                channel = get_number(2);
                                value[0] = get_number(3);
                                for(l=1;l<(value[0]+1);l++)
                                    value[l]=get_number(3+l);
                                value_16[0] = get_16number(4+value[0])/256;
                                value_16[1] = get_16number(4+value[0])%256;
                                value_16[2] = get_16number(5+value[0])/256;
                                value_16[3] = get_16number(5+value[0])%256;
                                value[1+value[0]] = (uint8_t) value_16[0];
                                value[2+value[0]] = (uint8_t) value_16[1];
                                value[3+value[0]] = (uint8_t) value_16[2];
                                value[4+value[0]] = (uint8_t) value_16[3];
                                send_packet(address,seq,CMD_PIECE,channel,value[0]+5,value);
                                seq=seq+1;
                             }
                             else putsUart0("Please enter numbers as parameters for the PIECEWISE command");
                        }

           if(!valid)
               {
               putsUart0("Command you entered doesn't exist or the arguments required for that command is insufficient\r\n");
               valid=0;
               }
          }
    }
}
