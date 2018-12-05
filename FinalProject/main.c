/*  -------------------------------------------------------------------------------------------------------------------------
*   Author(s):      Nicholas Veltema & Corbin Bremmeyr
*   Date:           November 18, 2018
*   Class:          EGR226-908
*   Assignment:     Final Project
*   Description:
*   -------------------------------------------------------------------------------------------------------------------------
*   P5.5 (A0) Analog input for screen brightness
*   P5.4 (A1) Analog input for temp
*   P6.7 Alarm Speaker PWM (TA2.4)
*   P2.7 (TA0.4) LED PWM
*
*   P2.5 LCD back light
*   P6.4 LCD rs
*   P6.5 LCD en
*   P2.0 - P2.3 LCD data
*
*   P3.5 Button down (top - far right)
*   P3.6 Button up (top - center right)
*   P4.0 Button set time (top - far left)
*   P4.1 Button set alarm (top - center left)
*   P4.2 Button (back - top)
*   P4.3 Button (back - bottom)
*
*   TODO:   - (DONE)PWM led's with wake-up functionality
*           - Analog input  --  issue: reading from 2 analog inputs
*               - temperature
*               - LCD display
*           - Serial communication
*           - Time speed buttons
*           - remove debounce from interrupts
*               - Use timer32_0 to turn off button interrupt for 'DEBOUNCE' ms
*/

#include "msp.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Pin defines for buttons

// Set 1:Back buttons and left 2 buttons on top
#define BTN_SET1 P4
#define BACK_TOP_BTN 2
#define BACK_BOTTOM_BTN 3
#define TOP_L_BTN 0
#define TOP_LC_BTN 1

// Set 2: Right 2 buttons on top
#define BTN_SET2 P3
#define TOP_R_BTN 5
#define TOP_RC_BTN 6

// LCD string locations

// Current time
#define TIME_HR_LOC  0x82
#define TIME_MIN_LOC 0x85
#define TIME_SEC_LOC 0x88
#define TIME_XM_LOC  0x8A

// Alarm time
#define ALARM_HR_LOC  0x95
#define ALARM_MIN_LOC 0x98
#define ALARM_XM_LOC  0x9A

// Temperature
#define TEMP_LOC 0xD7

#define BOUNCE 200          // debounce button press for 10ms
#define MS 3000             // 3000 clock cycles = 1ms
#define US 3                // 3 clock cycles = 1us

void init_SysTick(void);
void init_RTC(void);
void init_LEDs(void);
void init_LCD(void);
void init_Switches(void);
void init_adc(void);
void start_Menu(void);
void delay_ms(uint16_t delay);
void delay_micro(uint8_t delay);
void commandWrite(uint8_t command);
void dataWrite(uint8_t data);
void pushByte(uint8_t byte);
void pushNibble(uint8_t nibble);
void PulseEnablePin(void);
void set_time(void);
void set_alarm(void);
void update_time(void);
void write_String(char temp[]);
void debounce1(uint8_t pin, uint32_t len);
void debounce2(uint8_t pin, uint32_t len);
void init_Speaker(void);
void init_Timer32(void);
void set_lcd_brightness(void);
void update_alarm_lcd(void);
void update_temperature(void);

enum states{
    Idle,
    Set_Time,
    Set_Alarm,
    Wake_Up,
    Alarm,
    Snooze,
};

/** GLOBAL VARIABLES **/

// Raw ADC values, -1 indicates that there is no new value.
volatile int temperature_raw = -1;
volatile int lcd_raw = -1;

//Global variables for time tracking
int hr = 12;
int min = 0;
int sec = 0;

//Global variables for alarm tracking
int alarm_hr = 0;
int alarm_min = 0;
int alarm_sec = 0;

//Global strings for displaying current time
char hours[3] = "12";
char minutes[3] = "00";
char seconds[3] = "00";
char xm[4] = " AM";

//Global strings for displaying alarm time
char alarm_hours[3] = "12";
char alarm_minutes[2] = "00";
char alarm_seconds[2];
char alarm_xm[4] = " AM";

int btnup_flag = 0;
int btndown_flag = 0;
int btn_fastspeed = 0;

int set_time_flag = 0;      //flag for moving thru set time
int set_alarm_flag = 0;     //flag for moving thru set alarm
int alarm_enable = 0;
char alarm[10] = "ALARM OFF";

void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

    // Setup system
    __disable_interrupt();

    init_SysTick();
    init_Switches();
    init_Speaker();
    init_LEDs();
    init_LCD();
    init_RTC();
    init_adc();
    init_Timer32();

    __enable_interrupt();

    start_Menu();                   //sends starting layout to the LCD (******this function could potentially be combined with init_LCD()*******)
    enum states state = Idle;
    while(1)
    {
        update_time();          //update current time displayed each time through the loop
        update_temperature();
        set_lcd_brightness();
        switch(state)
            {
            case Idle:

                if(btnup_flag)
                {
                    alarm_enable ^= BIT0;
                    btnup_flag = 0;
                    if(alarm_enable)
                    {
                        strcpy(alarm," ALARM ON");
                        RTC_C->AMINHR |= BIT(15) | BIT(7);      //Enable Alarm: bit15 = enable hr alarm, bit7 = enable min alarm
                    }
                    else
                    {
                        strcpy(alarm,"ALARM OFF");
                        RTC_C->AMINHR &= ~(BIT(15) | BIT(7));   //Disable Alarm
                    }
                }

                if(set_time_flag)        //if btn_setTime
                {
                    state = Set_Time;
                }
                if(set_alarm_flag)       //if btn_setAlarm
                {
                    state = Set_Alarm;
                }
                if(alarm_enable)
                {
                    if(((((RTC_C->TIM1 & 0x00FF)<<8) | ((RTC_C->TIM0 & 0xFF00)>>8)) + 5) >= (RTC_C->AMINHR & ~(BIT(15)|BIT(7))))    //if 5 min before alarm time
                    {
                            state = Wake_Up;
                    }
                }
                break;

            case Set_Time:
                set_time();                 //stays in function set_time until current time configuration is complete
                state = Idle;
                set_time_flag = 0;          //reset flag after time configuration is complete
                break;

            case Set_Alarm:
                set_alarm();                //stays in function set_alarm until alarm time configuration is complete
                state = Idle;
                set_alarm_flag = 0;         //reset flag after alarm time configuration is complete
                break;

            case Wake_Up:
                TIMER32_1->CONTROL |= BIT7;     //enable Timer32

                if(btnup_flag)
                {
                    alarm_enable = 0;
                    strcpy(alarm,"ALARM OFF");
                    RTC_C->AMINHR &= ~(BIT(15) | BIT(7));   //disable alarm
                    TIMER32_1->CONTROL &= ~BIT7;            //disable Timer32
                    TIMER_A0->CCR[4] = 0;                   //set LED duty cycle to 0%
                    state = Idle;
                    btnup_flag = 0;
                }
                if(alarm_enable)
                {
                    if((((RTC_C->TIM1 & 0x00FF)<<8) | ((RTC_C->TIM0 & 0xFF00)>>8)) >= (RTC_C->AMINHR & ~(BIT(15)|BIT(7))))    //if alarm time
                    {
                            state = Alarm;
                    }
                }

                break;

            case Alarm:

                // LCD to Full brightness
                TIMER_A0->CCR[2] = 1000 - 1;

                TIMER_A2->CCR[4] ^= (BIT5|BIT4|BIT1);  //toggle 50% duty cycle for alarm speaker
                delay_ms(1000);

                if(btnup_flag)
                {
                    TIMER_A2->CCR[4] = 0;
                    alarm_enable = 0;
                    strcpy(alarm,"ALARM OFF");
                    RTC_C->AMINHR &= ~(BIT(15) | BIT(7));   // Disable Alarm
                    TIMER32_1->CONTROL &= ~BIT7;            // Disable Timer32
                    TIMER_A0->CCR[4] = 0;                   // Set LED duty cycle to 0%
                    set_lcd_brightness();                   // Set LCD brightness to pot value
                    btnup_flag = 0;
                    state = Idle;
                }
                if(btndown_flag)
                {
                    TIMER_A2->CCR[4] = 0;
                    strcpy(alarm,"   SNOOZE");
                    RTC_C->AMINHR += 10;                //add 10 minutes to alarm time for snooze
                    update_alarm_lcd();
                    btndown_flag = 0;
                    state = Snooze;
                }
                break;

            case Snooze:

                if((((RTC_C->TIM1 & 0x00FF)<<8) | ((RTC_C->TIM0 & 0xFF00)>>8)) >= (RTC_C->AMINHR & ~(BIT(15)|BIT(7))))    //if new alarm time
                {
                    TIMER_A2->CCR[4] ^= (BIT5|BIT4|BIT1);  //toggle 50% duty cycle for alarm speaker
                    delay_ms(1000);
                }

                // If alarm off btn was pressed
                if(btnup_flag)
                {
                    // Turn off alarm
                    TIMER_A2->CCR[4] = 0;
                    alarm_enable = 0;
                    strcpy(alarm,"ALARM OFF");
                    RTC_C->AMINHR &= ~(BIT(15) | BIT(7));   // Disable Alarm in RTC
                    TIMER32_1->CONTROL &= ~BIT7;            // Disable Timer32 (wake-up lights)
                    TIMER_A0->CCR[4] = 0;                   // Set LED duty cycle to 0% (wake-up lights)

                    // Set alarm time to set value not snooze value
                    RTC_C->AMINHR -= 10;
                    update_alarm_lcd();
                    set_lcd_brightness();
                    btnup_flag = 0;
                    state = Idle;
                }
                break;
            }
    }
}

/**
 * Converts raw input and updates temperature on LCD
 */
void update_temperature(void) {

    float temp;
    char temp_str[] = "00.0";

    // Convert raw value to *F
    temp = 0.019958 * temperature_raw;

    // Display temperature on LCD
    sprintf(temp_str, "%.1f", temp);
    commandWrite(TEMP_LOC);
    write_String(temp_str);
}

/**
 * Update Alarm time on LCD
 */
void update_alarm_lcd(void) {
//update global alarm time variables
    if(alarm_enable)
    {
        alarm_hr = (RTC_C->AMINHR & 0xFF00 & ~BIT(15)) >> 8;
        alarm_min = (RTC_C->AMINHR & 0x00FF & ~BIT7);
    }
    else
    {
        alarm_hr = (RTC_C->AMINHR & 0xFF00) >> 8;
        alarm_min = (RTC_C->AMINHR & 0x00FF);
    }


    if(((RTC_C->AMINHR & 0xFF00 & ~BIT(15)) >> 8) == 0)            //if hours equal zero time is 12am ((RTC_C->AMINHR & 0xFF00)>>8)
    {
        alarm_hr = 12;

    }
    if(alarm_hr > 12)                                 //if hours are greater than 12 convert to 12hr format
    {
        alarm_hr = alarm_hr - 12;
    }

//update global alarm time display strings
    if(((RTC_C->AMINHR & 0xFF00 & ~BIT(15)) >> 8) > 11)            //if hours are greater than 11 time is pm
    {
        strcpy(alarm_xm, " PM");
    }
    else
    {
        strcpy(alarm_xm, " AM");
    }
    if(alarm_hr < 10)                           //if current alarm hour count is only one character
    {
        sprintf(alarm_hours," %d",alarm_hr);    //put the integer alarm hour count into the alarm hours string with a leading space
    }
    else
    {
        sprintf(alarm_hours,"%d",alarm_hr);     //put the integer alarm hour count into the alarm hours string
    }
    if(alarm_min < 10)                          //if current alarm minute count is only one character
    {
        sprintf(alarm_minutes,"0%d",alarm_min); //put the integer alarm minutes count into the alarm minutes string with a leading zero
    }
    else
    {
        sprintf(alarm_minutes,"%d",alarm_min);  //put the integer alarm minutes count into the alarm minutes string
    }
// Write alarm time strings to LCD locations
    commandWrite(ALARM_HR_LOC);
    write_String(alarm_hours);
    commandWrite(ALARM_MIN_LOC);
    write_String(alarm_minutes);
    commandWrite(ALARM_XM_LOC);
    write_String(alarm_xm);
}

/**
 * Converts raw value from the LCD pot to PWM value for TIMER_A.
 */
void set_lcd_brightness(void) {

    // Convert raw ADC value to duty cycle %
    // map x in 0 to 16383 to y in 0 to 100
    // x/16383 = y/100
//    int dc = lcd_raw * 100 / 16383;

    // Set new duty cycle for LCD LED PWM
//    TIMER_A0->CCR[2] = (dc * 10) - 1;

    TIMER_A0->CCR[2] = (0.061038 * (float)lcd_raw) - 1;
}

void init_SysTick(void)             //reset and enable SysTick timer, no interrupt
{
   SysTick->CTRL = 0;
   SysTick->LOAD = (MS - 1);
   SysTick->VAL = 0;
   SysTick->CTRL = 5;
}

void init_LEDs(void)
{
    //initialize LEDs on P2.7
    //TimerA0.4
    P2->SEL0 |= BIT7;
    P2->SEL1 &= ~BIT7;
    P2->DIR |= BIT7;
    P2->OUT &= ~BIT7;

    //initialize TimerA0 for PWM at 3KHz
    //3KHz = 1000 clock cycles
    TIMER_A0->CCR[0] = 1000 - 1;
    TIMER_A0->CCR[4] = 0;               //initialize LEDs off (0% duty cycle)
    TIMER_A0->CCTL[4] = 0b11100000;     //0xE0  reset/set mode
    TIMER_A0->CTL = 0b1000010100;       //no clock divider
}

void init_RTC(void)
{
    RTC_C->CTL0 = (0xA500);         //unlock RTC clock
    RTC_C->CTL13 = 0;               //????? what does this register do
    RTC_C->TIM0 = 0;                //load zero minutes and zero seconds (00:00)
    RTC_C->TIM1 = 0;                //load zero hours, 12am (12:00:00 AM)

    RTC_C->AMINHR = 0;              //load 12am into alarm (12:00 AM)

    RTC_C->PS1CTL = 0b11010;        //1 second interrupt
    RTC_C->CTL0 = (0xA500) | BIT5;  //turn on interrupt

    RTC_C->CTL13 = 0;               //????? what does this register do
    NVIC_EnableIRQ(RTC_C_IRQn);     //enable RTC interrupt handler
}

/**
 * Set up analog input A0 & A1
 */
void init_adc(void) {

    // P5.5 - A0
    // P5.4 - A1
    // 0x30 = BIT5|BIT4
    P5->SEL0 |= 0x30;
    P5->SEL1 |= 0x30;

    ADC14->CTL0 = 0;
    ADC14->CTL0 = 0b10000100001000100000001100010000;   // 0b 1000 0100 0010 0000 0000 0011 0001 0000 from lab 8
    ADC14->CTL1 = 0b110000;         // 0b 11 0000
    ADC14->MCTL[0] = 0;
    ADC14->MCTL[1] = 0 | BIT7;         //BIT7 signifies end of sequence (so ADC will not run conversions for channels 2 thru 31)
    ADC14->IER0 |= BIT0;

    // Enable ADC
    ADC14->CTL0 |= 0b10;
    NVIC->ISER[0] |= 1 << ADC14_IRQn;
}

/**
 * Record analog input values when done converting.
 */
void ADC14_IRQHandler(void) {

    // Record and clear interrupt flags
    uint32_t irq_flags = ADC14->IFGR0;
    ADC14->CLRIFGR0 |= 0x3;

    // Temperature (A1)
    if(irq_flags & 0x2) {
        temperature_raw = ADC14->MEM[1];
    }

    // LCD pot input
    if(irq_flags & 0x1) {
        lcd_raw = ADC14->MEM[0];
    }
}

/*
 * Real Time Clock Interrupt Handler
 * Interrupts every second to increment current time
 */
void RTC_C_IRQHandler()
{

    if(RTC_C->PS1CTL & BIT0)                //if RTC interrupt
    {
        RTC_C->PS1CTL &= ~BIT0;             //reset interrupt flag
        if(btn_fastspeed)
        {
            RTC_C->TIM0 += (1<<8);
        }
        hr = RTC_C->TIM1 & 0x00FF;          //record hours (from bottom 8 bits of TIM1)
        min = (RTC_C->TIM0 & 0xFF00) >> 8;  //record minutes (from top 8 bits of TIM0)
        sec = RTC_C->TIM0 & 0x00FF;         //record seconds (from bottom 8 bits of TIM0)


    }
    ADC14->CTL0 |= 0b1;                                 //start ADC Conversion
}

/*
 * Updates current time displayed each time through while loop in main()
 */
void update_time(void)
{
    if(RTC_C->TIM1 == 0)            //if hours equal zero time is 12am
    {
        hr = 12;
    }
    if(RTC_C->TIM1 > 11)            //if hours are greater than 11 time is pm
    {
        strcpy(xm, " PM");
    }
    else
    {
        strcpy(xm, " AM");
    }
    if(hr > 12)            //if hours are greater than 12 convert to 12hr format
    {
        hr = hr - 12;
    }

    if(hr < 10)                     //if current hour count is only one character
    {
        sprintf(hours," %d",hr);    //put the integer hour count into the hours string with a leading space
    }
    else
    {
        sprintf(hours,"%d",hr);     //put the integer hour count into the hours string
    }

    if(min < 10)                    //if current minute count is only one character
    {
        sprintf(minutes,"0%d",min); //put the integer minute count into the minutes string with a leading zero
    }
    else
    {
        sprintf(minutes,"%d",min);  //put the integer minute count into the minutes string
    }

    if(sec < 10)                    //if current second count is only one character
    {
        sprintf(seconds,"0%d",sec); //put the integer second count into the seconds string with a leading zero
    }
    else
    {
        sprintf(seconds,"%d",sec);  //put the integer second count into the seconds string
    }

    commandWrite(0x82);     //send updated current time to the LCD
    write_String(hours);
    commandWrite(0x85);
    write_String(minutes);
    commandWrite(0x88);
    write_String(seconds);
    commandWrite(0x8A);
    write_String(xm);

    commandWrite(0xC4);     //starting address to display alarm status
    write_String(alarm);
}


void set_time(void)
{
    char temp[2] = "  ";                //string with two spaces for blinking effect when setting time

    while(set_time_flag == 1)           //while btn_setTime has not been pressed again flash hours
    {
        //if button up has been pressed
        if(btnup_flag)
        {
            RTC_C->TIM1 += 1;                       //add one hour to current time
            if(RTC_C->TIM1 == 24)                   //if hours need to rollover
            {
                RTC_C->TIM1 = 0;                    //reset hour count to zero
            }
            btnup_flag = 0;
        }
        //if button down has been pressed
        if(btndown_flag)
        {
            if(RTC_C->TIM1 == 0)                    //if hours need to roll back
            {
                RTC_C->TIM1 = 23;                   //set hours to 23
            }
            else
            {
                RTC_C->TIM1 -= 1;                   //subtract one hour from current time
            }
            btndown_flag = 0;
        }
        //update global current time variables
        if(RTC_C->TIM1 == 0)            //if hours equal zero time is 12am
        {
            hr = 12;
        }
        if(hr > 12)            //if hours are greater than 12 convert to 12hr format
        {
            hr = hr - 12;
        }
        //update global current time display strings
        if(RTC_C->TIM1 > 11)            //if hours are greater than 11 time is pm
        {
            strcpy(xm, " PM");
        }
        else
        {
            strcpy(xm, " AM");
        }
        if(hr < 10)                     //if current hour count is only one character
        {
            sprintf(hours," %d",hr);    //put the integer hour count into the hours string with a leading space
        }
        else
        {
            sprintf(hours,"%d",hr);     //put the integer hour count into the hours string
        }
        //update display
        commandWrite(0x82);
        write_String(temp);             //load temp string with spaces
        delay_ms(500);                  //delay half a second for blinking effect
        commandWrite(0x82);
        write_String(hours);            //update hours on LCD
        commandWrite(0x8A);
        write_String(xm);
        delay_ms(500);                  //delay half a second for blinking effect
    }
    while(set_time_flag == 2)   //while btn_setTime has not been pressed again flash minutes
    {
        //if button up has been pressed
        if(btnup_flag)
        {
            if(((RTC_C->TIM0 & 0xFF00)>>8) == 59)                           //if minute count needs to rollover
            {
                RTC_C->TIM0 = (0 | (RTC_C->TIM0 & 0x00FF));                  //restart minute count at zero, keep seconds count
            }
            else
            {
                RTC_C->TIM0 += (1<<8);                                     //add one minute to current time
            }
            btnup_flag = 0;
        }
        //if button down has been pressed
        if(btndown_flag)
        {
            if((RTC_C->TIM0 & 0xFF00) == 0)                            //if minute count needs to roll back
            {
                RTC_C->TIM0 = ((59<<8) | (RTC_C->TIM0 & 0x00FF));       //set minute count to 59, keep seconds count
            }
            else
            {
                RTC_C->TIM0 -= (1<<8);                                  //subtract one minute from current time
            }
            btndown_flag = 0;
        }
        //update global current time display strings
        if(min < 10)                    //if current minute count is only one character
        {
            sprintf(minutes,"0%d",min); //put the integer minute count into the minutes string with a leading zero
        }
        else
        {
            sprintf(minutes,"%d",min);  //put the integer minute count into the minutes string
        }
        //update display
        commandWrite(0x85);
        write_String(temp);             //load temp string with spaces
        delay_ms(500);                  //delay half second for blinking effect
        commandWrite(0x85);
        write_String(minutes);          //update minutes on LCD
        delay_ms(500);                  //delay half second for blinking effect
    }

}
void set_alarm(void)
{
    char temp[2] = "  ";            //string with two spaces for blinking effect when setting alarm

    while(set_alarm_flag == 1)      //while btn_setTime has not been pressed again flash hours
    {
        //if button up has been pressed
        if(btnup_flag)
        {
            RTC_C->AMINHR += (1<<8);                       //add one hour to alarm time
            if(((RTC_C->AMINHR & 0xFF00)>>8) == 24)
            {
                RTC_C->AMINHR = (RTC_C->AMINHR & 0x00FF);   //restart alarm hour count, keep alarm minute count
            }
            btnup_flag = 0;
        }
        //if button down has been pressed
        if(btndown_flag)
        {
            if(((RTC_C->AMINHR & 0xFF00)>>8) == 0)
            {
                RTC_C->AMINHR = ((23<<8) | (RTC_C->AMINHR & 0x00FF));   //set alarm hours to 23, keep alarm minute count
            }
            else
            {
                RTC_C->AMINHR -= (1<<8);                   //subtract one hour from alarm time
            }
            btndown_flag = 0;
        }
        //update global alarm time variables
        alarm_hr = (RTC_C->AMINHR & 0xFF00) >> 8;
        if(((RTC_C->AMINHR & 0xFF00)>>8) == 0)            //if hours equal zero time is 12am ((RTC_C->AMINHR & 0xFF00)>>8)
        {
            alarm_hr = 12;

        }
        if(alarm_hr > 12)            //if hours are greater than 12 convert to 12hr format
        {
            alarm_hr = alarm_hr - 12;
        }
        //update global alarm time display strings
        if(((RTC_C->AMINHR & 0xFF00)>>8) > 11)            //if hours are greater than 11 time is pm
        {
            strcpy(alarm_xm, " PM");
        }
        else
        {
            strcpy(alarm_xm, " AM");
        }
        if(alarm_hr < 10)                           //if current alarm hour count is only one character
        {
            sprintf(alarm_hours," %d",alarm_hr);    //put the integer alarm hour count into the alarm hours string with a leading space
        }
        else
        {
            sprintf(alarm_hours,"%d",alarm_hr);     //put the integer alarm hour count into the alarm hours string
        }
        //update display
        commandWrite(0x95);
        write_String(temp);                         //load temp string with spaces
        delay_ms(500);                              //delay half second for blinking effect
        commandWrite(0x95);
        write_String(alarm_hours);                  //update alarm hours on LCD
        commandWrite(0x9A);
        write_String(alarm_xm);                     //update AM/PM
        delay_ms(500);                              //delay half second for blinking effect
    }
    while(set_alarm_flag == 2)   //while btn_setTime has not been pressed again flash minutes
    {
        //if button up has been pressed
        if(btnup_flag)
        {
            RTC_C->AMINHR += 1;                                     //add one minute to alarm time
            if((RTC_C->AMINHR & 0x00FF) == 60)                           //if alarm minute count needs to rollover
            {
                RTC_C->AMINHR = (RTC_C->AMINHR & 0xFF00);                  //restart alarm minute count, keep alarm hour count
            }
            btnup_flag = 0;
        }
        //if button down has been pressed
        if(btndown_flag)
        {
            if((RTC_C->AMINHR & 0x00FF) == 0)                            //if alarm minute count needs to roll back
            {
                RTC_C->AMINHR = ((RTC_C->AMINHR & 0xFF00) | 59);       //set alarm minute count to 59, keep alarm hour count
            }
            else
            {
                RTC_C->AMINHR -= 1;                                  //subtract one minute from alarm time
            }
            btndown_flag = 0;
        }
        //update global alarm time display strings
        alarm_min = (RTC_C->AMINHR & 0x00FF);
        if(alarm_min < 10)                          //if current alarm minute count is only one character
        {
            sprintf(alarm_minutes,"0%d",alarm_min); //put the integer alarm minutes count into the alarm minutes string with a leading zero
        }
        else
        {
            sprintf(alarm_minutes,"%d",alarm_min);  //put the integer alarm minutes count into the alarm minutes string
        }
        //update display
        commandWrite(0x98);
        write_String(temp);                         //load temp string with spaces
        delay_ms(500);                              //delay half second for blinking effect
        commandWrite(0x98);
        write_String(alarm_minutes);                //update alarm minutes on LCD
        delay_ms(500);                              //delay half second for blinking effect
    }
}
void init_Switches(void)
{
    //initialize btn_up on P3.6 with interrupt
    P3->SEL0 &= ~BIT6;
    P3->SEL1 &= ~BIT6;
    P3->DIR &= ~BIT6;
    P3->REN |= BIT6;
    P3->OUT |= BIT6;
    P3->IES |= BIT6;
    P3->IE |= BIT6;

    //initialize btn_down on P3.5 with interrupt
    P3->SEL0 &= ~BIT5;
    P3->SEL1 &= ~BIT5;
    P3->DIR &= ~BIT5;
    P3->REN |= BIT5;
    P3->OUT |= BIT5;
    P3->IES |= BIT5;
    P3->IE |= BIT5;

    NVIC_EnableIRQ(PORT3_IRQn);     //initialize port 3 interrupt handler
    P3->IFG = 0;                    //clear port 3 interrupt flags

    //initialize btn_setTime on P4.0 with interrupt
    P4->SEL0 &= ~BIT0;
    P4->SEL1 &= ~BIT0;
    P4->DIR &= ~BIT0;
    P4->REN |= BIT0;
    P4->OUT |= BIT0;
    P4->IES |= BIT0;
    P4->IE |= BIT0;

    //initialize btn_setAlarm on P4.1 with interrupt
    P4->SEL0 &= ~BIT1;
    P4->SEL1 &= ~BIT1;
    P4->DIR &= ~BIT1;
    P4->REN |= BIT1;
    P4->OUT |= BIT1;
    P4->IES |= BIT1;
    P4->IE |= BIT1;

    //initialize btn_normspeed on P4.2 with interrupt
    P4->SEL0 &= ~BIT2;
    P4->SEL1 &= ~BIT2;
    P4->DIR &= ~BIT2;
    P4->REN |= BIT2;
    P4->OUT |= BIT2;
    P4->IES |= BIT2;
    P4->IE |= BIT2;

    //initialize btn_fastspeed on P4.3 with interrupt
    P4->SEL0 &= ~BIT3;
    P4->SEL1 &= ~BIT3;
    P4->DIR &= ~BIT3;
    P4->REN |= BIT3;
    P4->OUT |= BIT3;
    P4->IES |= BIT3;
    P4->IE |= BIT3;

    NVIC_EnableIRQ(PORT4_IRQn);     //initialize port 4 interrupt handler
    P4->IFG = 0;                    //clear port 4 interrupt flags
//    delay_ms(5);                    //allow pins to stabilize before enabling system wide interrupts
}

void PORT3_IRQHandler()
{
    delay_ms(BOUNCE);           //debounce button ******(need to find an alternative solution, delay in interrupt handler is bad practice)******
    int flag = P3->IFG;             //store the port 3 interrupt flags
    P3->IFG = 0;                    //clear port 3 interrupt flags

        if(flag & BIT6)         //if btnup pressed
        {
            btnup_flag = 1;
        }

        if(flag & BIT5)         //if btndown pressed
        {
            btndown_flag = 1;
        }


}
void PORT4_IRQHandler()
{
    delay_ms(BOUNCE);           //debounce button ******(need to find an alternative solution, delay in interrupt handler is bad practice)******
    int flag = P4->IFG;         //store port 4 interrupt flags
    P4->IFG = 0;                //clear port 4 interrupt flags

    if(flag & BIT0)             //if btn_setTime
    {
        set_time_flag++;
    }
    if(flag & BIT1)             //if btn_setAlarm
    {
        set_alarm_flag++;
    }
    if(flag & BIT2)
    {
        btn_fastspeed = 0;
    }
    if(flag & BIT3)
    {
        btn_fastspeed = 1;
    }
}

void debounce1(uint8_t pin, uint32_t len) {

    uint32_t final = 0xFFFFFFFF;
    uint32_t mask = 0xFFFFFFFF << len;
    uint32_t btn_read = 0x1;

    do {
        btn_read = (btn_read << 1) | ((BTN_SET1->IN & pin) >> pin);
    } while( (final == (mask | ~btn_read)) || (final == (mask | btn_read)) );
}

void debounce2(uint8_t pin, uint32_t len) {

    uint32_t final = 0xFFFFFFFF;
    uint32_t mask = 0xFFFFFFFF << len;
    uint32_t btn_read = 0x1;

    do {
        btn_read = (btn_read << 1) | ((BTN_SET2->IN & pin) >> pin);
    } while( (final == (mask | ~btn_read)) || (final == (mask | btn_read)) );
}

/*
 * Function initializes the LCD
 */
void init_LCD(void)
{
    P2->SEL0 |= BIT5;          //using P2.5 to power LCD backlight until ADC-to-PWM is utilized
    P2->SEL1 &= ~BIT5;
    P2->DIR |= BIT5;
    P2->OUT |= BIT5;

    TIMER_A0->CCR[2] = 1000-1;          //initialize LCD backlight to 100% duty cycle
    TIMER_A0->CCTL[2] = 0b11100000;     //0xE0  reset/set mode

    //RS on P6.4, Enable on P6.5
    P6->SEL0 &= ~(BIT4|BIT5);               //set P6.4 and P6.5 for GPIO
    P6->SEL1 &= ~(BIT4|BIT5);               //set P6.4 and P6.5 for GPIO
    P6->DIR |= (BIT4|BIT5);                 //set P6.4 and P6.5 as outputs
    P6->OUT &= ~(BIT4|BIT5);                //start with P6.4 and P6.5 outputs off

    P2->SEL0 &= ~(BIT0|BIT1|BIT2|BIT3);     //set P2.0, P2.1, P2.2, P2.3 for GPIO
    P2->SEL1 &= ~(BIT0|BIT1|BIT2|BIT3);     //set P2.0, P2.1, P2.2, P2.3 for GPIO
    P2->DIR |= (BIT0|BIT1|BIT2|BIT3);       //set P2.0, P2.1, P2.2, P2.3 as outputs
    P2->OUT &= ~(BIT0|BIT1|BIT2|BIT3);      //start with P2.0, P2.1, P2.2, P2.3 outputs off
    delay_ms(60);                           //delay 60ms for LCD to stabilize

    commandWrite(3);    //reset display
    delay_ms(15);
    commandWrite(3);
    delay_ms(1);
    commandWrite(3);
    delay_ms(15);

    commandWrite(2);    //set display to 4-bit mode
    delay_ms(1);
    commandWrite(2);
    delay_ms(1);

    commandWrite(8);    //4-line format
    delay_ms(1);
    commandWrite(0x0C); //set display on, cursor off
    delay_ms(1);
    commandWrite(1);    //clear display, move cursor to home position
    delay_ms(1);
    commandWrite(6);    //increment cursor
    delay_ms(10);

}
void delay_ms(uint16_t delay)
{
    SysTick->LOAD = (delay * MS);           //load delay time in milliseconds
    SysTick->VAL = 0;                       //start timer at zero
    while((SysTick->CTRL & BIT(16))==0){}   //wait for delay time to elapse
}
void delay_micro(uint8_t delay)
{
    SysTick->LOAD = (delay * US);           //load delay time in microseconds
    SysTick->VAL = 0;                       //start timer at zero
    while((SysTick->CTRL & BIT(16))==0){}   //wait for delay time to elapse
}
void commandWrite(uint8_t command)
{
    P6->OUT &= ~BIT4;                       //set RS=0 for sending a command
    delay_ms(1);                            //delay 1ms before sending more data
    pushByte(command);                      //send the command to pushByte
}
void dataWrite(uint8_t data)
{
    P6->OUT |= BIT4;                        //set RS=1 for sending data
    delay_ms(1);                            //delay 1ms before sending more data
    pushByte(data);                         //send the data to pushByte
}
void pushByte(uint8_t byte)
{
    int x,y;
    x = (byte >> 4);            //x = 4 MSBits
    pushNibble(x);              //send first 4 bits to pushNibble
    delay_micro(250);
    y = (byte & 0xF);           //y = 4 LSBits
    pushNibble(y);              //send second 4 bits to pushNibble
    delay_micro(250);
}
void pushNibble(uint8_t nibble)
{
    P2->OUT &= ~0xF;     //send bits to the display
    P2->OUT |= nibble;
    PulseEnablePin();
}
void PulseEnablePin(void)
{
    P6->OUT &= ~BIT5;   //set enable low
    delay_micro(10);
    P6->OUT |= BIT5;    //set enable high
    delay_micro(10);
    P6->OUT &= ~BIT5;   //set enable low
    delay_micro(10);
}
void write_String(char temp[])
{
    int i,n;
    n = strlen(temp);       //determine the length of the string
    for(i=0;i<n;i++)        //loop for the length of the string
    {
        dataWrite(temp[i]);
    }
}
void start_Menu(void)
{
    char str1[] = "12:00:00 AM";
    char str2[] = "ALARM OFF";
    char str3[] = "12:00 AM";
    char str4[] = "00.0 F";

    commandWrite(1);        //clear display
    delay_ms(1);

    commandWrite(0x82);     //starting address to display string 1 on line 1 (0x80 + LCD Address in hex)
    write_String(str1);

    commandWrite(0xC4);     //starting address to display string 2 on line 2
    write_String(str2);

    commandWrite(0x95);     //starting address to display string 3 on line 3
    write_String(str3);

    commandWrite(0xD7);     //starting address to display string 4 on line 4
    write_String(str4);
}

void init_Speaker(void)
{
    P6->SEL0 |= BIT7;
    P6->SEL1 &= ~BIT7;
    P6->DIR |= BIT7;
    P6->OUT &= ~BIT7;

    TIMER_A2->CCR[0] = 6810 - 1;        //approximately 440Hz
    TIMER_A2->CCR[4] = 0;               //initialize off (0% duty cycle)
    TIMER_A2->CCTL[4] = 0b11100000;     //0xE0  reset/set mode
    TIMER_A2->CTL = 0b1000010100;       //no clock divider
}
//test comment

void init_Timer32(void)
{
    TIMER32_1->CONTROL = 0b01100010;    // set up but leave off - toggle BIT7 to turn on/off
    TIMER32_1->LOAD = 9000000 - 1;      //interrupt every 3 seconds for increasing wake up lights

    // Timer32_0 for button debounce
    TIMER32_0->CONTROL = 0b01100010;
    TIMER32_0->LOAD = DEBOUNCE - 1;

    NVIC_EnableIRQ(T32_INT1_IRQn);      //enable Timer32 interrupts
}

void T32_INT1_IRQHandler(void)
{
    TIMER32_1->INTCLR = 1;                              //clear Timer32 interrupt flag
    if(TIMER_A0->CCR[4] > 980)          //if LEDs should be at max brightness
    {
        TIMER_A0->CCR[4] = 1000 - 1;    //set LED duty cycle to 100%
        TIMER32_1->CONTROL &= ~BIT7;    //disable Timer32
    }
    else
    {
        TIMER_A0->CCR[4] += 10;         //add 1% to LED duty cycle
    }
}
