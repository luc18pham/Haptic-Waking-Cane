// Luc Pham

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz
// Stack:           4096 bytes (needed for snprintf)
\
// Hardware configuration:
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
// Blue LED:
//   PF2 drives an NPN transistor that powers the blue LED
// Pushbutton:
//   SW1 pulls pin PF4 low (internal pull-up is used)
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1
// Frequency counter and timer input:
//   SIGNAL_IN on PC6 (WT1CCP0)

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "clock.h"
#include "wait.h"
#include "uart0.h"
#include "tm4c123gh6pm.h"
//#include "eeprom.h"

//Output: Port E 1, 2 ,3
#define TRIGGER_OUT0 (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 1*4))) //PE1
#define TRIGGER_OUT1 (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 2*4))) //PE2
#define TRIGGER_OUT2 (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 3*4))) //PE3

//Input: Port C
#define ECHO_IN0 (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 5*4))) //PC5 uses WT0B
#define ECHO_IN1 (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4))) //PC6 uses WT1A

//Input: Port D
#define ECHO_IN2 (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 0*4))) //PD0 uses WT2A

//PortD masks
#define INPUT1 32
#define INPUT2 64
#define INPUT3 1

//PortE masks
#define OUTPUT1 2
#define OUTPUT2 4
#define OUTPUT3 8

//PortF masks
#define MOTOR_MASK 2
#define RED_LED_MASK 2
#define BLUE_LED_MASK 4
#define GREEN_LED_MASK 8

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

uint32_t distance[3];
uint32_t realDistance[3];
uint32_t n = 0;

#define MAX_CHARS 80
#define MAX_FIELDS 6

typedef struct _USER_DATA
{
    char buffer[MAX_CHARS+1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
    }USER_DATA;

uint32_t strCompare(char* str1, char* str2)
{
    int i = 0;
    int flag = 0;
    while(str1[i] != '\0' && str2[i] != '\0')
    {
        if(str1[i] != str2[i])
        {
            flag = 1;
            break;
        }
        i++;
    }
    if(flag == 0 && str1[i] == '\0' && str2[i] == '\0')
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void parseFields(USER_DATA *data)
    {
        int i= 0;
        int j = 0;
        data->fieldCount = j;

        while(i < MAX_CHARS && data->fieldCount != MAX_FIELDS && data->buffer[i] != '\0')
        {
            if((data->buffer[i] >= 97 && data->buffer[i] <= 122) || (data->buffer[i] >= 65 && data->buffer[i] <= 90))
            {
                data->fieldType[data->fieldCount] = 'a';
                data->fieldPosition[data->fieldCount] = i;
                j++;
                data->fieldCount = j;

                while((data->buffer[i] >= 97 && data->buffer[i] <= 122) || (data->buffer[i] >= 65 && data->buffer[i] <= 90))
                {
                    i++;
                }
            }
            else if(data->buffer[i] >= 48 && data->buffer[i] <= 57)
            {
                data->fieldType[data->fieldCount] = 'n';
                data->fieldPosition[data->fieldCount] = i;
                j++;
                data->fieldCount = j;

                while(data->buffer[i] >= 48 && data->buffer[i] <= 57)
                {
                    i++;
                }

            }
            else
            {
               data->buffer[i] = '\0';
            }
            data->buffer[i] = '\0';
            i++;
        }
        return;
    }

char* getFieldString(USER_DATA* data, uint8_t fieldNumber)
    {
        return &data->buffer[data->fieldPosition[fieldNumber]];
    }

int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber)
    {
        return atoi(&data->buffer[data->fieldPosition[fieldNumber]]);
    }

bool isCommand(USER_DATA* data, const char strCommand[], uint8_t minArguments)
    {
        int i = 0;
        bool b = false;

        if(data->fieldCount -1 >= minArguments)
        {
            while(data->buffer[i] != '\0' && strCommand[i] != '\0' && data->buffer[i] == strCommand[i])
            {
                i++;
            }
            if(strCommand[i] == data->buffer[i])
            {
                b = true;
            }
            else
            {
                b = false;
            }
        }
        return b;

    }

void getsUart0(USER_DATA *data)
    {
        int count = 0;
        char c;

        while( count < MAX_CHARS)
        {
            c = getcUart0();

            if(c == 127 || c == 8 && count > 0)
            {
                if(count > 0)
                {
                    count--;
                }
            }
            else if(c == 13 || c == 10)
            {
                data->buffer[count++] = '\0';
                return;
            }
            else if(c >= 32)
            {
                data->buffer[count] = c;
                count= count + 1;

                if(count == MAX_CHARS)
                {
                    data->buffer[count] = '\0';
                    return;
                }
            }
        }
    }

// Initialize Hardware
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1; //enable timer 1, used as a counter
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R0 | SYSCTL_RCGCTIMER_R1 | SYSCTL_RCGCTIMER_R2; //enables wide timers 0,1,2
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2 | SYSCTL_RCGCGPIO_R3 | SYSCTL_RCGCGPIO_R4 | SYSCTL_RCGCGPIO_R5; //enable port C, D, E, F
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;  // enable motor
    _delay_cycles(3);

    //Configure Transmitter and Receiver
    GPIO_PORTE_DIR_R |= OUTPUT1 | OUTPUT2 | OUTPUT3 ;
    GPIO_PORTE_DEN_R |= OUTPUT1 | OUTPUT2 | OUTPUT3 ;

    GPIO_PORTC_DIR_R &= ~(INPUT1 | INPUT2) ;
    GPIO_PORTC_DEN_R |= INPUT1 | INPUT2 ;

    GPIO_PORTD_DIR_R &= ~INPUT3;
    GPIO_PORTD_DEN_R |= INPUT3;

    // Configure SIGNAL_IN for frequency and time measurements for WIDE TIMER 0
    GPIO_PORTC_AFSEL_R |= INPUT1;                    // select alternative functions for SIGNAL_IN pin
    GPIO_PORTC_PCTL_R &= ~GPIO_PCTL_PC5_M;           // map alt fns to SIGNAL_IN
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC5_WT0CCP1;

    // Configure SIGNAL_IN for frequency and time measurements for WIDE TIMER 1
    GPIO_PORTC_AFSEL_R |= INPUT2;                    // select alternative functions for SIGNAL_IN pin
    GPIO_PORTC_PCTL_R &= ~GPIO_PCTL_PC6_M;           // map alt fns to SIGNAL_IN
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC6_WT1CCP0;

    // Configure SIGNAL_IN for frequency and time measurements for WIDE TIMER 2
    GPIO_PORTD_AFSEL_R |= INPUT3;                    // select alternative functions for SIGNAL_IN pin
    GPIO_PORTD_PCTL_R &= ~GPIO_PCTL_PD0_M;           // map alt fns to SIGNAL_IN
    GPIO_PORTD_PCTL_R |= GPIO_PCTL_PD0_WT2CCP0;

    //FROM RGB_LED.C
    GPIO_PORTF_DEN_R |= RED_LED_MASK | GREEN_LED_MASK | BLUE_LED_MASK;
    GPIO_PORTF_AFSEL_R |= RED_LED_MASK | GREEN_LED_MASK | BLUE_LED_MASK;
    GPIO_PORTF_PCTL_R &= ~(GPIO_PCTL_PF1_M | GPIO_PCTL_PF2_M | GPIO_PCTL_PF3_M);
    GPIO_PORTF_PCTL_R |= GPIO_PCTL_PF1_M1PWM5 | GPIO_PCTL_PF2_M1PWM6 | GPIO_PCTL_PF3_M1PWM7;

    // Configure PWM module 1 to drive RGB LED
    // RED   on M1PWM5 (PF1), M1PWM2b
    // BLUE  on M1PWM6 (PF2), M1PWM3a
    // GREEN on M1PWM7 (PF3), M1PWM3b
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;                // reset PWM1 module
    SYSCTL_SRPWM_R = 0;                              // leave reset state
    PWM1_2_CTL_R = 0;                                // turn-off PWM1 generator 2 (drives outs 4 and 5)
    PWM1_3_CTL_R = 0;                                // turn-off PWM1 generator 3 (drives outs 6 and 7)
    PWM1_2_GENB_R = PWM_1_GENB_ACTCMPBD_ONE | PWM_1_GENB_ACTLOAD_ZERO;
                                                         // output 5 on PWM1, gen 2b, cmpb
    PWM1_3_GENA_R = PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO;
                                                         // output 6 on PWM1, gen 3a, cmpa
    PWM1_3_GENB_R = PWM_1_GENB_ACTCMPBD_ONE | PWM_1_GENB_ACTLOAD_ZERO;
                                                         // output 7 on PWM1, gen 3b, cmpb

    PWM1_2_LOAD_R = 1024;                            // set frequency to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz
    PWM1_3_LOAD_R = 1024;                            // (internal counter counts down from load value to zero)

    PWM1_2_CMPB_R = 0;                               // red off (0=always low, 1023=always high)
    PWM1_3_CMPB_R = 0;                               // green off
    PWM1_3_CMPA_R = 0;                               // blue off

    PWM1_2_CTL_R = PWM_1_CTL_ENABLE;                 // turn-on PWM1 generator 2
    PWM1_3_CTL_R = PWM_1_CTL_ENABLE;                 // turn-on PWM1 generator 3
    PWM1_ENABLE_R = PWM_ENABLE_PWM5EN | PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;
}

void enableTimers()
{
    //Configure wide Timer 0
    WTIMER0_CTL_R &= ~TIMER_CTL_TBEN;                // turn-off counter before reconfiguring
    WTIMER0_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER0_TBMR_R = TIMER_TBMR_TBCMR | TIMER_TBMR_TBMR_CAP | TIMER_TBMR_TBCDIR;
                                                     // configure for edge time mode, count up
    WTIMER0_CTL_R = TIMER_CTL_TBEVENT_BOTH;          // measure time from positive edge to positive edge
    WTIMER0_IMR_R = TIMER_IMR_CBEIM;                 // turn-on interrupts
    WTIMER0_TBV_R = 0;                               // zero counter for first period
    WTIMER0_CTL_R |= TIMER_CTL_TBEN;                 // turn-on counter
    NVIC_EN2_R = 1 << (INT_WTIMER0B-16-64);          // turn-on interrupt 112 (WTIMER0A)

    //Configure wide Timer 1
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER1_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER1_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR;
                                                     // configure for edge time mode, count up
    WTIMER1_CTL_R = TIMER_CTL_TAEVENT_BOTH;          // measure time from positive edge to positive edge
    WTIMER1_IMR_R = TIMER_IMR_CAEIM;                 // turn-on interrupts
    WTIMER1_TAV_R = 0;                               // zero counter for first period
    WTIMER1_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
    NVIC_EN3_R = 1 << (INT_WTIMER1A-16-96);          // turn-on interrupt 112 (WTIMER1A)

    //Configure wide Timer 2
    WTIMER2_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER2_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER2_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR;
                                                     // configure for edge time mode, count up
    WTIMER2_CTL_R = TIMER_CTL_TAEVENT_BOTH;          // measure time from positive edge to positive edge
    WTIMER2_IMR_R = TIMER_IMR_CAEIM;                 // turn-on interrupts
    WTIMER2_TAV_R = 0;                               // zero counter for first period
    WTIMER2_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
    NVIC_EN3_R = 1 << (INT_WTIMER2A-16-96);          // turn-on interrupt 112 (WTIMER2A)

    // Configure Timer 1 as the time base
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 100000;                       // set load value to 40e6 for 1 Hz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    NVIC_EN0_R = 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
}

///////////////////////////////////////////////////CHECK TIMER 0 IF OFFSET IS CORRECT/////////////////////////////////////////////////////////////////////////
void disableTimers()
{
    WTIMER0_CTL_R &= ~TIMER_CTL_TBEN;                // turn-off counter
    NVIC_DIS3_R = 1 << (INT_WTIMER0B-16-64);        // turn-off interrupt 112 (WTIMER0B)

    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter
    NVIC_DIS3_R = 1 << (INT_WTIMER1A-16-96);        // turn-off interrupt 112 (WTIMER1A)

    WTIMER2_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter
    NVIC_DIS3_R = 1 << (INT_WTIMER2A-16-96);        // turn-off interrupt 112 (WTIMER2A)
}

void wideTimer0Isr()
{
    distance[0] = WTIMER0_TBV_R;
    WTIMER0_TBV_R = 0;                           // reset counter for next period
    WTIMER0_ICR_R = TIMER_ICR_CBECINT;
}

void wideTimer1Isr()
{
    distance[1] = WTIMER1_TAV_R;
    WTIMER1_TAV_R = 0;                           // reset counter for next period
    WTIMER1_ICR_R = TIMER_ICR_CAECINT;
}

void wideTimer2Isr()
{
    distance[2] = WTIMER2_TAV_R;
    WTIMER2_TAV_R = 0;                           // reset counter for next period
    WTIMER2_ICR_R = TIMER_ICR_CAECINT;
}

void Timer1Isr()
{
    if(n > 2)
    {
        n = 0;
    }

    if(n == 0)
    {
        TRIGGER_OUT0 = 1;
        waitMicrosecond(10);
        TRIGGER_OUT0 = 0;
        realDistance[0] = distance[0] * .004287;
        n++;
    }
    else if(n == 1)
    {
        TRIGGER_OUT1 = 1;
        waitMicrosecond(10);
        TRIGGER_OUT1 = 0;
        realDistance[1] = distance[1] * .004287;
        n++;
    }
    else if(n == 2)
    {
        TRIGGER_OUT2 = 1;
        waitMicrosecond(10);
        TRIGGER_OUT2 = 0;
        realDistance[2] = distance[2] * .004287;
        n++;
    }
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;
}

//setting the motor + the extra LEDs
void setRgbColor(uint16_t red, uint16_t green, uint16_t blue)
{
    PWM1_2_CMPB_R = red;
    PWM1_3_CMPA_R = blue;
    PWM1_3_CMPB_R = green;
}

void initEeprom(void)
{
    SYSCTL_RCGCEEPROM_R = 1;
    _delay_cycles(3);
    while (EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
}

void writeEeprom(uint16_t add, uint32_t data)
{
    EEPROM_EEBLOCK_R = add >> 4;
    EEPROM_EEOFFSET_R = add & 0xF;
    EEPROM_EERDWR_R = data;
    while (EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
}

uint32_t readEeprom(uint16_t add)
{
    EEPROM_EEBLOCK_R = add >> 4;
    EEPROM_EEOFFSET_R = add & 0xF;
    return EEPROM_EERDWR_R;
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{

    //Fixes issue while on battery power
    waitMicrosecond(2000000);
    // Initialize hardware
    initHw();
    initUart0();
    initEeprom();
    // Setup UART0 baud rate
    setUart0BaudRate(115200, 40e6);

    USER_DATA data;
    char line[80];
    uint16_t j;
    uint16_t k;
    uint16_t z;

    while(true)
    {
        enableTimers();

        sprintf(line, "value of distance from sensor 0: %" PRIu32 " mm\n", realDistance[0]);
        putsUart0(line);

        sprintf(line, "value of distance from sensor 1: %" PRIu32 " mm\n", realDistance[1]);
        putsUart0(line);

        sprintf(line, "value of distance from sensor 2: %" PRIu32 " mm\n", realDistance[2]);
        putsUart0(line);
        waitMicrosecond(200000);


        if(kbhitUart0())
        {
            for(z = 0; z < MAX_CHARS; z++)
            {
                data.buffer[z] = NULL;
            }

            for(z = 0; z < MAX_FIELDS; z++)
            {
                data.fieldPosition[z] = NULL;
                data.fieldType[z] = NULL;
            }

            getsUart0(&data);
            parseFields(&data);

            if(isCommand(&data, "reset", 0))
            {
                NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
            }

            else if(isCommand(&data, "and", 3)) // command example: "and y n p" , y = compound event num to create, n & p = event nums
            {
                uint16_t h = 0;
                uint16_t l = 0;
                bool set = false;
                bool valid = false;

                if(readEeprom(0 + 8*getFieldInteger(&data, 2)) != readEeprom(0 + 8*getFieldInteger(&data, 3)) ) // makes sure not to use the same sensor
                {
                    valid = true;
                }

                for(h = 0; h < 2; h++)
                {
                    for(l = 0; l < 8; l++)
                    {
                        if(readEeprom(l + 8*getFieldInteger(&data, (h+1))) == -1)
                        {
                            set = false;
                        }
                        else
                        {
                            set = true;
                        }
                    }
                }

                if(set == true && valid == true)
                {
                    putsUart0("Both set and valid are true\n");
                    if(getFieldInteger(&data, 1) > 15 && getFieldInteger(&data, 1) < 20)
                    {
                        putsUart0("valid compound event address, creating compound event\n");
                        writeEeprom(0 + 8*getFieldInteger(&data, 1), getFieldInteger(&data, 2)); //First Event's number
                        writeEeprom(1 + 8*getFieldInteger(&data, 1), getFieldInteger(&data, 3)); //Second Event's number
                    }
                    else
                    {
                        putsUart0("Not a valid compound event address\n");
                    }
                }
                else
                {
                    if(valid == false && set == false)
                    {
                        putsUart0("cannot use the same sensor number and values are not set\n");
                    }
                    else if(valid == false)
                    {
                        putsUart0("cannot use the same sensor number\n");
                    }
                    else if(set == false)
                    {
                        putsUart0("values are not set for one of the sensors\n");
                    }
                    else
                    {
                        putsUart0("SOMETHING ELSE IS WRONG\n");
                    }
                }
            }

            else if(isCommand(&data, "erase", 1))
            {
                writeEeprom(0 + 8*getFieldInteger(&data, 1), 0xFFFFFFFF);
                writeEeprom(1 + 8*getFieldInteger(&data, 1), 0xFFFFFFFF);
                writeEeprom(2 + 8*getFieldInteger(&data, 1), 0xFFFFFFFF);
                writeEeprom(3 + 8*getFieldInteger(&data, 1), 0xFFFFFFFF);
                writeEeprom(4 + 8*getFieldInteger(&data, 1), 0xFFFFFFFF);
                writeEeprom(5 + 8*getFieldInteger(&data, 1), 0xFFFFFFFF);
                writeEeprom(6 + 8*getFieldInteger(&data, 1), 0xFFFFFFFF);
                writeEeprom(7 + 8*getFieldInteger(&data, 1), 0xFFFFFFFF);

                putsUart0("cleared that event\n");
            }

            else if(isCommand(&data, "show", 1))
            {
                if(strCompare(getFieldString(&data, 1), "events"))
                {
                    for(j = 0; j <= 15; j++)
                    {
                        sprintf(line, "EVENT: %" PRIi32 "\n", j);
                        putsUart0(line);
                        sprintf(line, "SENSOR number: %" PRIi32 "\n", readEeprom(0 + 8*j) );
                        putsUart0(line);
                        sprintf(line, "MIN_DIST number: %" PRIi32 " mm \n", readEeprom(1 + 8*j) );
                        putsUart0(line);
                        sprintf(line, "MAX_DIST: %" PRIi32 " mm \n", readEeprom(2 + 8*j) );
                        putsUart0(line);
                        sprintf(line, "HAPTIC: %" PRIi32 "\n", readEeprom(3 + 8*j) );
                        putsUart0(line);
                        sprintf(line, "Time on: %" PRIi32 " ms \n", readEeprom(4 + 8*j) );
                        putsUart0(line);
                        sprintf(line, "Time off: %" PRIi32 " ms \n", readEeprom(5 + 8*j) );
                        putsUart0(line);
                        sprintf(line, "Beat Count: %" PRIi32 "\n", readEeprom(6 + 8*j) );
                        putsUart0(line);
                        sprintf(line, "Duty cycle: %" PRIi32 " %% \n", readEeprom(7 + 8*j) );
                        putsUart0(line);
                        putsUart0("\n");
                    }

                    for(j = 16; j <= 19; j++)
                    {
                        sprintf(line, "COMPOUND EVENT: %" PRIi32 "\n", j);
                        putsUart0(line);
                        sprintf(line, "FIRST EVENT: %" PRIi32 "\n", readEeprom(0 + 8*j) );
                        putsUart0(line);
                        sprintf(line, "SECOND EVENT: %" PRIi32 "\n", readEeprom(1 + 8*j) );
                        putsUart0(line);
                        sprintf(line, "FIRST EVENT'S MIN DIST: %" PRIi32 " mm \nFIRST EVENT's MAX DIST: %" PRIi32 " mm \n"
                                "SECOND EVENT's MIN DIST: %" PRIi32 " mm \nSECOND EVENT'S MAX DIST: %" PRIi32 " mm \n",
                                readEeprom(1 + 8*(readEeprom(0 + 8*j))), readEeprom(2 + 8*(readEeprom(0 + 8*j))),
                                readEeprom(1 + 8*(readEeprom(1 + 8*j))), readEeprom(2 + 8*(readEeprom(1 + 8*j))) );

                        putsUart0(line);
                        sprintf(line, "HAPTIC: %" PRIi32 "\n", readEeprom(3 + 8*j) );
                        putsUart0(line);
                        sprintf(line, "Time on: %" PRIi32 " ms \n", readEeprom(4 + 8*j) );
                        putsUart0(line);
                        sprintf(line, "Time off: %" PRIi32 " ms \n", readEeprom(5 + 8*j) );
                        putsUart0(line);
                        sprintf(line, "Beat Count: %" PRIi32 "\n", readEeprom(6 + 8*j) );
                        putsUart0(line);
                        sprintf(line, "Duty cycle: %" PRIi32 " %% \n", readEeprom(7 + 8*j) );
                        putsUart0(line);
                        putsUart0("\n");
                    }
                }
                else if(strCompare(getFieldString(&data, 1), "patterns"))
                {
                    for(k = 0; k <= 15; k++)
                    {
                        sprintf(line, "EVENT: %" PRIi32 "\n", k);
                        putsUart0(line);
                        sprintf(line, "Time on: %" PRIi32 " ms \n", readEeprom(4 + 8*k) );
                        putsUart0(line);
                        sprintf(line, "Time off: %" PRIi32 " ms \n", readEeprom(5 + 8*k) );
                        putsUart0(line);
                        sprintf(line, "Beat Count: %" PRIi32 "\n", readEeprom(6 + 8*k) );
                        putsUart0(line);
                        sprintf(line, "Duty cycle: %" PRIi32 " %% \n", readEeprom(7 + 8*k) );
                        putsUart0(line);
                        putsUart0("\n");
                    }

                    for(k = 16; k <= 19; k++)
                    {
                        sprintf(line, "COMPOUND EVENT: %" PRIi32 "\n", k);
                        putsUart0(line);
                        sprintf(line, "Time on: %" PRIi32 " ms \n", readEeprom(4 + 8*k) );
                        putsUart0(line);
                        sprintf(line, "Time off: %" PRIi32 " ms \n", readEeprom(5 + 8*k) );
                        putsUart0(line);
                        sprintf(line, "Beat Count: %" PRIi32 "\n", readEeprom(6 + 8*k) );
                        putsUart0(line);
                        sprintf(line, "Duty cycle: %" PRIi32 " %% \n", readEeprom(7 + 8*k) );
                        putsUart0(line);
                        putsUart0("\n");
                    }

                }
                else
                {
                    putsUart0("invalid command\n");
                }
            }

            else if(isCommand(&data, "event", 4))
            {
                if(getFieldInteger(&data, 1) < 16 && getFieldInteger(&data, 1) > -1)
                {
                    writeEeprom(0 + 8*getFieldInteger(&data, 1), getFieldInteger(&data, 2));
                    writeEeprom(1 + 8*getFieldInteger(&data, 1), getFieldInteger(&data, 3));
                    writeEeprom(2 + 8*getFieldInteger(&data, 1), getFieldInteger(&data, 4));
                    putsUart0("successfully created or edited event\n");
                }
                else
                {
                    putsUart0("Event must be between 0 and 15\n");
                }
            }

            else if(isCommand(&data, "haptic", 2))
            {
                uint32_t value;
                uint32_t value2;

                value  = strCompare("on", getFieldString(&data, 2));
                value2 = strCompare(getFieldString(&data, 2), "off");

                if(value == 1)
                {
                    writeEeprom(3 + 8*getFieldInteger(&data, 1), 1);
                    putsUart0("Successfully set haptic to on\n");
                }
                else if(value2 == 1)
                {
                    writeEeprom(3 + 8*getFieldInteger(&data, 1), 0);
                    putsUart0("Successfully set haptic to off\n");
                }
                else
                {
                    putsUart0("invalid haptic parameter\n");
                    //sprintf(line, "strCompare function value: %" PRIu32 "\n", strcmp( "on",getFieldString(&data, 2)));
                    //putsUart0(line);
                    //putsUart0(getFieldString(&data, 2));
                    //putsUart0("\n");
                }
            }
            else if(isCommand(&data, "pattern", 5))
            {
                sprintf(line, "entered pattern command\n");
                putsUart0(line);
                writeEeprom(7 + 8*getFieldInteger(&data, 1), getFieldInteger(&data, 2)); // PWM
                writeEeprom(6 + 8*getFieldInteger(&data, 1), getFieldInteger(&data, 3)); // BEATS
                writeEeprom(4 + 8*getFieldInteger(&data, 1), getFieldInteger(&data, 4)); // ON_TIME
                writeEeprom(5 + 8*getFieldInteger(&data, 1), getFieldInteger(&data, 5)); // OFF_TIME
            }
            else
            {
                putsUart0("invalid command\n");
            }
        }

        int eventNum;
        int beats;
        int sensorNum;
        int minDist;
        int maxDist;
        int hapticState;
        int timeOn;
        int timeOff;
        int beatCount;
        int dutyCyc;

        int beatIndex;

        int sensorNum2;
        int minDist2;
        int maxDist2;



        for(eventNum = 19; eventNum >= 16; eventNum--)
        {
            hapticState = readEeprom(3 + 8*eventNum);
            if(hapticState != -1)
            {
                sensorNum  = readEeprom(0 + 8*(readEeprom(0 + 8*eventNum)));
                sensorNum2 = readEeprom(0 + 8*(readEeprom(1 + 8*eventNum)));
                minDist    = readEeprom(1 + 8*(readEeprom(0 + 8*eventNum)));
                minDist2   = readEeprom(1 + 8*(readEeprom(1 + 8*eventNum)));
                maxDist    = readEeprom(2 + 8*(readEeprom(0 + 8*eventNum)));
                maxDist2   = readEeprom(2 + 8*(readEeprom(1 + 8*eventNum)));

                timeOn    = readEeprom(4 + 8*eventNum) * 1000;
                timeOff   = readEeprom(5 + 8*eventNum) * 1000;
                beatCount = readEeprom(6 + 8*eventNum);
                dutyCyc   = readEeprom(7 + 8*eventNum) * 10;

                if(realDistance[sensorNum] >= minDist && realDistance[sensorNum] <= maxDist && realDistance[sensorNum2] >= minDist2 && realDistance[sensorNum2] <= maxDist2 )
                {
                    sprintf(line, "Running Compound Event %" PRIi32 "\n", eventNum );
                    putsUart0(line);
                    disableTimers();
                    for(beatIndex = 0; beatIndex < beatCount; beatIndex++)
                    {
                        setRgbColor(0,0,0);
                        setRgbColor(dutyCyc,0,0);
                        waitMicrosecond(timeOn);
                        setRgbColor(0,0,0);
                        waitMicrosecond(timeOff);
                    }
                    sprintf(line, "Finished running Compound Event %" PRIi32 " now waiting 3 seconds\n", eventNum);
                    putsUart0(line);
                    waitMicrosecond(1500000);
                    enableTimers();
                    waitMicrosecond(1500000);
                }
            }
        }

        for(eventNum = 15; eventNum >= 0; eventNum--)
        {
            hapticState = readEeprom(3 + 8*eventNum);
            if(hapticState != -1)
            {
                sensorNum = readEeprom(0 + 8*eventNum);
                minDist   = readEeprom(1 + 8*eventNum);
                maxDist   = readEeprom(2 + 8*eventNum);

                timeOn    = readEeprom(4 + 8*eventNum) * 1000;
                timeOff   = readEeprom(5 + 8*eventNum) * 1000;
                beatCount = readEeprom(6 + 8*eventNum);
                dutyCyc   = readEeprom(7 + 8*eventNum) * 10;

                if(realDistance[sensorNum] >= minDist && realDistance[sensorNum] <= maxDist)
                {
                    sprintf(line, "Running Simple Event %" PRIi32 "\n", eventNum);
                    putsUart0(line);
                    disableTimers();
                    for(beatIndex = 0; beatIndex < beatCount; beatIndex++)
                    {
                        setRgbColor(0,0,0);
                        setRgbColor(dutyCyc,0,0);
                        waitMicrosecond(timeOn);
                        setRgbColor(0,0,0);
                        waitMicrosecond(timeOff);
                    }
                    sprintf(line, "Finished running Simple Event %" PRIi32 " now waiting 3 seconds\n", eventNum);
                    putsUart0(line);
                    waitMicrosecond(1500000);
                    enableTimers();
                    waitMicrosecond(1500000);
                }
            }
        }
    }
}
