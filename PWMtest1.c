
#include <stdio.h>
#include <timers.h>
#include <stdlib.h>
#include <delays.h>
#include <p18f45k22.h>
#include "pragmas.h"
#include <usart.h>
#include <pwm.h>




void ms10Delay()
{
Delay1KTCYx(100);
Delay1KTCYx(100);
Delay1KTCYx(100);
Delay1KTCYx(100);
Delay1KTCYx(100);
Delay100TCYx(2);
Delay10TCYx(5);
}

void main()
{
	unsigned char holdd=0;
    unsigned char   dc ;
	char buthold=0;
	unsigned char derp=20;
	unsigned char herp1=10;
	unsigned char herp2=0;
	unsigned char hold=0;
	unsigned char hold2=0;
	float numCalc=0;

    TRISC = 0 ;                     // set PORTC as output
    PORTC = 0 ;

	PORTA=0;
	TRISA= 0b00000011;
	ANSELA=0b00000001;

	ADCON0=0x01;
	ADCON1=0x00;
	ADCON2=0x8D;

	ADCON0=ADCON0|0x02;                    
	OSCCON=0b00100011;
	PR2 = 0b10011011 ;
	T2CON = 0b00000111 ;
	CCPR1L = 0b00000111 ;
	CCP1CON = 0b00111100 ;
	CCP2CON = 0b00001100 ;
	CCPR2L = 0b00000111 ;//bits 4,5 are used for bits 0,1
	CCPR2L=10;//
	while(1){
	//	CCPR1L = herp1 ;
		if(herp2&0b00000010)CCP2CON=CCP2CON|0b00100000;
		else CCP2CON=CCP2CON&0b11011111;
		if(herp2&0b00000001)CCP2CON=CCP2CON|0b00010000;
		else CCP2CON=CCP2CON&0b11101111;
	    CCPR2L = (herp2/4) ;
		if(PIR1bits.ADIF==1)
		{
			holdd=ADRESH;
			numCalc=(holdd/3.0)*192.0;
			holdd=(int)numCalc;
			hold2=ADRESL/4;
			hold=holdd|hold2;
			numCalc=((hold/255.0)*70)+10;
			herp2=(int)numCalc;
			ADCON0=0x03;
		}
		if(PORTAbits.RA1==0&&buthold==0)
			{
			herp2+=10;
			buthold=1;
			}
		else if (PORTAbits.RA1==1&&buthold==1)buthold=0;
	}
}		
