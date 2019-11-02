
#include <stdio.h>
#include <timers.h>
#include <stdlib.h>
#include <delays.h>
#include <p18f45k22.h>
#include "pragmas.h"
#include <usart.h>
#include <pwm.h>


char derp=20;
char herp1=10;
char herp2=10;

void ms10Delay()
{
Delay1KTCYx(100);
Delay100TCYx(2);
Delay10TCYx(5);
}

void main()
        {
        unsigned char   dc ;

        TRISC = 0 ;                     // set PORTC as output
        PORTC = 0 ;                     // clear PORTC
//  		 TRISB = 0 ;                     // set PORTC as output
//        PORTB = 0 ; 
		        
/*
         * configure CCP module as 4000 Hz PWM output
         */
		OSCCON=0b00100011;
        PR2 = 0b10011011 ;
		T2CON = 0b00000111 ;
CCPR1L = 0b00000111 ;
CCP1CON = 0b00111100 ;
CCP2CON = 0b00111100 ;
CCPR2L = 0b00000111 ;
CCPR2L=10;
                {
                /*
                 * PWM resolution is 10 bits
                 * don't use last 2 less significant bits CCPxCON,
                 * so only CCPRxL have to be touched to change duty cycle
                 */
              /*  for(dc = 0 ; dc < derp ; dc++)
                        {
                        CCPR1L = dc ;
                        CCPR2L =  dc ;
                        ms10Delay() ;
                        }
                for(dc = derp ; dc > 0 ; dc--)
                        {
                        CCPR1L = dc ;
                        CCPR2L = dc ;
                        ms10Delay() ;
                        }*/
						CCPR1L = herp1 ;//editing test
                        CCPR2L = herp2 ;
                        ms10Delay() ;
						Delay10KTCYx(100);
                }
               
 }
