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
const unsigned int sine_LUT[512] = {
    508, 514, 520, 527, 533, 539, 545, 552, 558, 564, 570, 576, 583, 589, 595, 
    601, 607, 613, 619, 625, 631, 637, 643, 649, 655, 661, 667, 673, 679, 685, 
    691, 697, 702, 708, 714, 720, 725, 731, 736, 742, 747, 753, 758, 764, 769, 
    774, 780, 785, 790, 795, 801, 806, 811, 816, 821, 825, 830, 835, 840, 845, 
    849, 854, 858, 863, 867, 872, 876, 880, 884, 889, 893, 897, 901, 905, 908, 
    912, 916, 920, 923, 927, 930, 934, 937, 940, 944, 947, 950, 953, 956, 959, 
    962, 965, 967, 970, 972, 975, 977, 980, 982, 984, 986, 988, 990, 992, 994, 
    996, 998, 999, 1001, 1002, 1004, 1005, 1006, 1007, 1009, 1010, 1011, 1011, 
    1012, 1013, 1014, 1014, 1015, 1015, 1015, 1016, 1016, 1016, 1016, 1016, 
    1016, 1016, 1015, 1015, 1015, 1014, 1014, 1013, 1012, 1011, 1011, 1010, 
    1009, 1007, 1006, 1005, 1004, 1002, 1001, 999, 998, 996, 994, 992, 990, 988, 
    986, 984, 982, 980, 977, 975, 972, 970, 967, 965, 962, 959, 956, 953, 950, 
    947, 944, 940, 937, 934, 930, 927, 923, 920, 916, 912, 908, 905, 901, 897, 
    893, 889, 884, 880, 876, 872, 867, 863, 858, 854, 849, 845, 840, 835, 830, 
    825, 821, 816, 811, 806, 801, 795, 790, 785, 780, 774, 769, 764, 758, 753, 
    747, 742, 736, 731, 725, 720, 714, 708, 702, 697, 691, 685, 679, 673, 667, 
    661, 655, 649, 643, 637, 631, 625, 619, 613, 607, 601, 595, 589, 583, 576, 
    570, 564, 558, 552, 545, 539, 533, 527, 520, 514 
};
unsigned int phase_accum = 0;   //phase accumulator for "analog" output.
                                //bottom three bits are used for interpolation
                                //on the wavetable(s))
unsigned int duty_cycle = 508;  //setting for the PWM output duty cycle
unsigned int speed = 1;         //"speed" of wavetable scanning
unsigned int adc_result;        //this is where the ADC value will be stored

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

void set_PWM_duty_cycle(int duty){
    CCP1CONbits.DCB = duty & 0b11;        // writing the PWM LSBs
    CCPR1L = (duty >> 2) & 0b11111111;    // writing the PWM MSBs
}

void set_sine_pwm_output(void){
    
    int local_pa = (phase_accum >> 5); //remove the lower bits

    if(local_pa < 256){
        duty_cycle = sine_LUT[local_pa];
    }
    else if(local_pa >= 256){
        local_pa = local_pa - 256;
        duty_cycle = (1016 - sine_LUT[local_pa]);
    }
    else {
        duty_cycle = 508;
    }
    
    set_PWM_duty_cycle(duty_cycle);
}

void set_tri_pwm_output(void){

    int local_pa = phase_accum >> 3;           //no interpolation needed here
    
    // set the PWM duty for sine wave output
    if ((local_pa >= 0) && (local_pa < 512)){
        duty_cycle = local_pa + 512;
    }
    else if((local_pa >= 512) && (local_pa < 1536)){
        duty_cycle = 1024 - (local_pa - 512);
    }
    else if((local_pa >= 1536)){
        duty_cycle = local_pa - 1536;
    }
    else{
        duty_cycle = 512;
    }
    
    // flatten the top of the wave
    if (duty_cycle > 1016){
        duty_cycle = 1016;
    }
 
    set_PWM_duty_cycle(duty_cycle);
}

void set_sq_pwm_output(void){
    
    // set duty cycle to 50%
    if (phase_accum >= 8192){
        duty_cycle = 1016;      //this is the maximum value of the PWM duty
    }
    else {
        duty_cycle = 0;
    }
    
    set_PWM_duty_cycle(duty_cycle);
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
        
        // force the phase accumulator to "overflow"
        if(phase_accum >= 16384){
            phase_accum = phase_accum - 16384;
        }

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
        
        phase_accum += speed;       // increment the PA
        PIR1bits.TMR2IF = 0;        // reset timer0 interrupt flag
    }
}