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
 * 9. The speed is controllable between ~0.25Hz and ~16Hz (256 total steps)
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
const unsigned short sine_LUT[256] = {
    508, 511, 514, 517, 520, 524, 527, 530, 533, 536, 539, 542, 545, 548, 552,
    555, 558, 561, 564, 567, 570, 573, 576, 579, 583, 586, 589, 592, 595, 598,
    601, 604, 607, 610, 613, 616, 619, 622, 625, 628, 631, 634, 637, 640, 643,
    646, 649, 652, 655, 658, 661, 664, 667, 670, 673, 676, 679, 682, 685, 688,
    691, 694, 697, 700, 702, 705, 708, 711, 714, 717, 720, 722, 725, 728, 731,
    734, 736, 739, 742, 745, 747, 750, 753, 756, 758, 761, 764, 766, 769, 772,
    774, 777, 780, 782, 785, 788, 790, 793, 795, 798, 801, 803, 806, 808, 811,
    813, 816, 818, 821, 823, 825, 828, 830, 833, 835, 837, 840, 842, 845, 847,
    849, 851, 854, 856, 858, 861, 863, 865, 867, 869, 872, 874, 876, 878, 880,
    882, 884, 886, 889, 891, 893, 895, 897, 899, 901, 903, 905, 907, 908, 910,
    912, 914, 916, 918, 920, 922, 923, 925, 927, 929, 930, 932, 934, 936, 937,
    939, 940, 942, 944, 945, 947, 948, 950, 952, 953, 955, 956, 957, 959, 960,
    962, 963, 965, 966, 967, 969, 970, 971, 972, 974, 975, 976, 977, 979, 980,
    981, 982, 983, 984, 985, 986, 987, 988, 989, 990, 991, 992, 993, 994, 995,
    996, 997, 998, 998, 999, 1000, 1001, 1002, 1002, 1003, 1004, 1004, 1005,
    1006, 1006, 1007, 1007, 1008, 1009, 1009, 1010, 1010, 1011, 1011, 1011,
    1012, 1012, 1013, 1013, 1013, 1014, 1014, 1014, 1014, 1015, 1015, 1015,
    1015, 1015, 1016, 1016, 1016, 1016, 1016, 1016, 1016,
};
int table_index = 0;            //used to get into the LUT
int interpolation_bits = 0;     //used to decide how much interpolation to do
int phase_accum = 0;            //phase accumulator for "analog" output.
                                //bottom three bits are used for interpolation
                                //on the wavetable(s))
unsigned int duty_cycle = 0;    //setting for the PWM output duty cycle
int speed = 1;                  //"speed" of wavetable scanning
int interpolated = 0;           //first-order interpolation for sine LUT
int tri_direction = 1;          //direction control bit for the triangle wave
int adc_result;                 //this is where the ADC value will be stored

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

int get_sine_value_from_pa(int local_pa){
    // returns the PWM duty cycle value for a sine wave output from the phase
    // accumulator value
    
    local_pa = (local_pa >> 4); //remove the interpolation bits

    if((local_pa >= 0) && (local_pa < 256)){
        return sine_LUT[local_pa];
    }
    else if((local_pa >= 256) && (local_pa < 512)){
        local_pa = local_pa - 256;
        return sine_LUT[255 - local_pa];
    }
    else if((local_pa >= 512) && (local_pa < 768)){
        local_pa = local_pa - 512;
        return (1016 - sine_LUT[local_pa]);
    }
    else if((local_pa >= 768) && (local_pa < 1024)){
        local_pa = local_pa - 768;
        return (1016 - sine_LUT[255 - local_pa]);
    }
    else {
        return 508;
    }
}

void set_sine_pwm_output(void){
    // force the phase accumulator to "overflow"
    if(phase_accum >= 16384){
        phase_accum = phase_accum - 16384;
    }
        
    // use bits 4 through 14 of the phase accumulator to find the nearest 
    // LUT index and bits 0, 1, 2, and 3 for the interpolation step
    interpolation_bits = phase_accum & 0b1111;
    
    // calculate the "slope" between adjacent duty cycle values in the LUT in 
    //case they are needed
    int point_1 = get_sine_value_from_pa(phase_accum + (1<<4));
    int point_2 = get_sine_value_from_pa(phase_accum);
    int interpolation_rise = (point_1 - point_2);
      
    // interpolate (y = mx +b)
    duty_cycle = ((interpolation_rise / 16) * interpolation_bits) + point_2;
    
    CCP1CONbits.DCB = duty_cycle & 0b11;        // writing the PWM LSBs
    CCPR1L = (duty_cycle >> 2) & 0b11111111;    // writing the PWM MSBs
    phase_accum += speed;                       // increment the PA
}

void set_tri_pwm_output(void){
    // control the wave direction by "turning the phase accumulator around"
    // if it grows too large or too small
    if (phase_accum > 16384){
        phase_accum = 16384 - (phase_accum - 16384);
        tri_direction = -1;
    }
    else if (phase_accum < 0){
        phase_accum = -1 * phase_accum;
        tri_direction = 1;
    }
 
    duty_cycle = phase_accum >> 4;
    CCP1CONbits.DCB = duty_cycle & 0b11;        //writing the PWM LSBs
    CCPR1L = (duty_cycle >> 2) & 0b11111111;    //writing the PWM MSBs
    phase_accum += (tri_direction * speed);
}

void set_sq_pwm_output(void){
    // force the phase accumulator to "overflow"
    if(phase_accum >= 16384){
        phase_accum = phase_accum - 16384;
    }
    
    // set duty cycle to 50%
    if (phase_accum >= 8192){
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
        else if ((PORTCbits.RC3 == 1) && (PORTCbits.RC4 == 0)){
            set_sine_pwm_output();
        }
        else {
            set_sq_pwm_output();
        }
        
        PIR1bits.TMR2IF = 0;    // reset timer0 interrupt flag
    }
}