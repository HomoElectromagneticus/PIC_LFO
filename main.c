/*
 * File:   main.c
 * Author: RM Schaub
 *
 * Created on January 15, 2016, 7:44 PM
 *                   PIC16F684
 *                    _______
 *           VDD ---|1      14|--- VSS
 *   Xtal pin 1  ---|2      13|---
 *   Xtal pin 2  ---|3      12|---
 *               ---|4      11|---
 *       PWM out ---|5      10|---
 * wave choice 2 ---|6       9|--- "speed" pot
 * wave choice 1 ---|7_______8|--- "on" light
 * 
 * This program produces a PWM'd wave from the "PWM out" pin at a frequency
 * controlled by the position of a speed pot whose wiper is connected to pin
 * 9. The speed is controllable between ~0.3Hz and ~19Hz (256 total steps)
 */

#include <stdlib.h>
#include <xc.h>

// CONFIG 1
#pragma config FOSC = HS        //Oscillator Selection bits (HS oscillator: High-speed crystal/resonator on RA4/OSC2/CLKOUT and RA5/OSC1/CLKIN)
#pragma config WDTE = ON        //Watchdog Timer Enable (WDT enabled)
#pragma config PWRTE = ON       //Power-up Timer Enable (PWRT enabled)
#pragma config MCLRE = ON       //MCLR Pin Function Select (MCLR/VPP pin function is a reset input)
#pragma config CP = OFF         //Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        //Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON       //Brown-out Reset Enable (Brown-out Reset enabbled)
#pragma config IESO = OFF       //Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      //Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG 2
#define _XTAL_FREQ 20000000     //setting processor speed variable (20MHz)
#define WDTPS0 0                //set the Watchdog Timer to reset the PIC after 
#define WDTPS1 1                //17ms (31KHz / 512)
#define WDTPS2 0
#define WDTPS3 0

// global variables
int post_scale = 0;             //this is used to get the timer2 interrupt speed down
const unsigned short sine_LUT[257] = {
    508,520,533,545,558,570,583,595,607,619,631,643,655,667,679,691,
    702,714,725,736,747,758,769,780,790,801,811,821,830,840,849,858,
    867,876,884,893,901,908,916,923,930,937,944,950,956,962,967,972,
    977,982,986,990,994,998,1001,1004,1006,1009,1011,1012,1014,1015,1015,1016,
    1016,1016,1015,1015,1014,1012,1011,1009,1006,1004,1001,998,994,990,986,982,
    977,972,967,962,956,950,944,937,930,923,916,908,901,893,884,876,
    867,858,849,840,830,821,811,801,790,780,769,758,747,736,725,714,
    702,691,679,667,655,643,631,619,607,595,583,570,558,545,533,520,
    508,496,483,471,458,446,433,421,409,397,385,373,361,349,337,325,
    314,302,291,280,269,258,247,236,226,215,205,195,186,176,167,158,
    149,140,132,123,115,108,100,93,86,79,72,66,60,54,49,44,
    39,34,30,26,22,18,15,12,10,7,5,4,2,1,1,0,
    0,0,1,1,2,4,5,7,10,12,15,18,22,26,30,34,
    39,44,49,54,60,66,72,79,86,93,100,108,115,123,132,140,
    149,158,167,176,186,195,205,215,226,236,247,258,269,280,291,302,
    314,325,337,349,361,373,385,397,409,421,433,446,458,471,483,496,500
};
int table_index = 0;            //used to get into the LUT
int interpolation_bits = 0;     //used to decide how much interpolation to do
int phase_accum = 0;            //phase accumulator for "analog" output.
                                //bottom three bits are used for interpolation
                                //on the wavetable(s))
unsigned int duty_cycle = 0;    //setting for the PWM output duty cycle
int speed = 1;                  //"speed" of wavetable scanning
int interpolated = 0;           //first-order interpolation for sine LUT
int interpolation_rise = 0;     //difference in wavetable index values: used
                                //for interpolation
int tri_direction = 1;          //direction control bit for the triangle wave
int adc_result;                 //this is where the ADC value will be stored

void Timer0_init(void){
     /* Timer0 (8-bit) interrupt frequency:
     * f = _XTAL_FREQ / (4*prescaler*Timer1 resolution*2)
     * f = 20000000 / (4*2*256*2) = ~4.883kHz */
    OPTION_REGbits.T0CS = 0;    //use system clock (external 20MHz)
    OPTION_REGbits.T0SE = 0;    //rising edge
    OPTION_REGbits.PSA = 0;     //assign the prescaler to Timer0
    OPTION_REGbits.PS = 0b000;  //prescaler set to 1:2
    INTCONbits.INTF = 0;        //clear the timer0 interrupt if it exists
    INTCONbits.GIE = 1;         //enable interrupts globally
    INTCONbits.PEIE = 1;        //enable peripheral interrupts
    INTCONbits.T0IE = 1;        //Timer0 overflow interrupt enabled
    
}

void ADC_Init(void){
    // sets up the ADC
    
    ADCON0bits.ADFM = 0;        //ADC Left justified
    ADCON0bits.VCFG = 0;        //ADC reference is set to VDD
    ADCON0bits.CHS = 0b101;     //selecting the AN5 analog channel
    ADCON1bits.ADCS = 0b100;    //ADC clock set to FOSC/4

    ADCON0bits.ADON = 1;        //turn ADC on
}

void Timer2_Init(void){
    // sets up Timer2 (used for PWM and the interrupt)
    // Timer2 uses the system clock (Fosc/4) by default
    // Timer2 overflow interrupt frequency:
    //      f = _XTAL_FREQ / 4*prescaler*Timer2 resolution
    //      f = 20000000 / (4*1*256) = 19.53kHz
    T2CONbits.TMR2ON = 0;       //turn off Timer2 during setup
    PIR1bits.TMR2IF = 0;        //reset Timer2 overflow interrupt flag
    T2CONbits.T2CKPS = 0b00;    //set the Timer2 prescaler to 1
    INTCONbits.GIE = 1;         //enable interrupts globally
    INTCONbits.PEIE = 1;        //enable peripheral interrupts
    PIE1bits.T2IE = 1;          //enable Timer2 overflow interrupts      
    T2CONbits.TMR2ON = 1;       //turn on Timer2
}

void PWM_Init(void){
    // starts the PWM output
    // PWM period = [PR2 + 1] * 4 * Tosc * (Timer2 prescale value)
    // PWM frequency = 19.53kHz
    
    TRISCbits.TRISC5 = 1;       //disabling the CCP1 output driver
    PR2 = 0xFF;                 //setting PR2 for max PWM bit depth at 20MHz clock
    CCP1CONbits.CCP1M = 0b1100; //setting CCP1 for PWM mode, P1A for active high
    CCP1CONbits.P1M0 = 0;       //configuring the PWM for single output at P1A
    CCP1CONbits.P1M1 = 0;
    CCP1CONbits.DC1B0 = 0;      //setting the two LSBs of the PWM duty cycle
    CCP1CONbits.DC1B1 = 0;
    CCPR1L = 0b00110000;        //setting the MSBs of the PWM duty cycle
    
    //enable PWM after a new cycle has started
    while (PIR1bits.TMR2IF == 0);//wait for timer 2 to overflow

    TRISCbits.TRISC5 = 0;       //enable the CCP1 output driver
}

int ADC_Convert(void){
    GO_nDONE = 1;               //start ADC
    while (GO_nDONE == 1);      //wait for ADC to finish
    return ADRESH;              //return the ADC value
}

void set_sine_pwm_output(void){
    // force the phase accumulator to "overflow"
    if(phase_accum >= 4094){
        phase_accum = phase_accum - 4094;
    }
        
    // use bits 3 through 10 of the phase accumulator to find the nearest 
    // LUT index and bits 1, 2, and 3 for the interpolation step
    table_index = phase_accum >> 4;
    interpolation_bits = phase_accum & 0b000000000001111;
    
    // calculate the slope between adjacent values in the LUT in case they
    // are needed
    interpolation_rise = sine_LUT[table_index + 1] - sine_LUT[table_index];
      
    // interpolate (y = mx +b)
    duty_cycle = ((interpolation_rise / 16) * interpolation_bits) + sine_LUT[table_index];
     
    CCP1CONbits.DCB = duty_cycle & 0b11;        // writing the PWM LSBs
    CCPR1L = (duty_cycle >> 2) & 0b11111111;    // writing the PWM MSBs
    phase_accum += speed;                       // increment the PA
}

void set_tri_pwm_output(void){
    // control the wave direction by "turning the phase accumulator around"
    // if it grows too large or too small
    if (phase_accum > 4080){
        phase_accum = 4080 - (phase_accum - 4080);
        tri_direction = -1;
    }
    else if (phase_accum < 0){
        phase_accum = -1 * phase_accum;
        tri_direction = 1;
    }
 
    duty_cycle = phase_accum >> 2;
    CCP1CONbits.DCB = duty_cycle & 0b11;        //writing the PWM LSBs
    CCPR1L = (duty_cycle >> 2) & 0b11111111;    //writing the PWM MSBs
    phase_accum += (tri_direction * 2 * speed);
}

void set_sq_pwm_output(void){
    // force the phase accumulator to "overflow"
    if(phase_accum >= 4094){
        phase_accum = phase_accum - 4094;
    }
    
    // set duty cycle to 50%
    if (phase_accum >= 2048){
        duty_cycle = 1016;      //this is the maximum value of the PWM duty
    }
    else {
        duty_cycle = 0;
    }
    
    CCP1CONbits.DCB = duty_cycle & 0b11;        //writing the PWM LSBs
    CCPR1L = (duty_cycle >> 2) & 0b11111111;    //writing the PWM MSBs
    phase_accum += speed;
}

void main(void) {
    // IO CONFIG
    TRISCbits.TRISC0 = 1;       //set RC0 (pin 10) as input
    TRISCbits.TRISC1 = 1;       //set RC1 (pin 9) as input
    ANSEL = 0b00100000;         //set AN5 (pin 9) as analog input (others as digital I/O)
    TRISCbits.TRISC2 = 0;       //set RC2 (pin 8) as a  output
    TRISCbits.TRISC3 = 1;       //set RC3 (pin 7) as a input
    TRISCbits.TRISC4 = 1;       //set RC4 (pin 6) as a input
    TRISCbits.TRISC5 = 0;       //set RC5 (pin 5) as a output (for pwm))

    // Software configuration
    Timer2_Init();              //Timer2 used to periodically update the PWM 
                                //duty cycle
    ADC_Init();                 //ADC used to read the state of the "speed" pot
    PWM_Init();                 //PWM used to create the LFO
    PORTCbits.RC2 = 1;          //just to tell the user that the program started
    
    while(1){
        adc_result = ADC_Convert();
        speed = ((adc_result) >> 2) + 1;
        
        CLRWDT();               //clear the Watchdog Timer to keep the PIC from
                                //resetting
    }
    return;
}

void interrupt ISR(void){
    // check for Timer 0 overflow interrupt
    if(PIR1bits.TMR2IF == 1){

        // state of pin 6 and 7 controls which output wave is drawn
        if ((PORTCbits.RC3 == 0) && (PORTCbits.RC4 == 0)){
            set_tri_pwm_output();
        }
        else if((PORTCbits.RC3 == 1) && (PORTCbits.RC4 == 0)){
            set_sine_pwm_output();
        }
        else{
            set_sq_pwm_output();
        }
        
        PIR1bits.TMR2IF = 0;    // reset timer0 interrupt flag
    }
}