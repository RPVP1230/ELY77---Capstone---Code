
#include <stdio.h>
#include <timers.h>
#include <stdlib.h>
#include <delays.h>
#include <p18f45k22.h>
#include "pragmas.h"
#include <usart.h>
#include <pwm.h>
#define HOMESA 	60
#define HOMELA	22
#define HOMEZ 	56
#define APPROACHPOS1ZAXIS 56
#define APPROACHPOS1SARM 57
#define APPROACHPOS1LARM 24
#define FINALPOS1ZAXIS 56
#define FINALPOS1SARM 63
#define FINALPOS1LARM 35
#define UL 1.0
#define LL -1.0

#define GROWLIGHTS PORTAbits.RA0
#define BUTTON1 PORTCbits.RC0
#define BUTTON2 PORTBbits.RB3
#define BUTTON3 PORTCbits.RC3
#define BUTTON4 PORTCbits.RC4
#define PUMP1 PORTDbits.RD3//rinse pump
#define PUMP2 PORTDbits.RD2//fill pump
#define FAN1 PORTDbits.RD5
#define FAN2 PORTDbits.RD6
#define WATERSENSOR1ADC 0x07
#define WATERSENSOR2ADC 0x0b
#define LIGHTSENSORADC 0x2f
#define SHELFSENSORADC 0x23
#define DOORLSADC 0x13

/*
water sensor 1 AN1 RA1
water sensor 2 AN2 RA2

light AN11 RB4
shelves AN8 RB2
doors AN4 RA5


*/

void usDelay(unsigned int value);
void LCD_PRINT(char input[20]);
void startDisplay();
void _PRINT_TIME();
char rowLocation=0;
char colLocation=0;
//in order, home,approach pos1,enter pos1,lifted pos1,above approach pos1,approach pos2,enter pos2,lifted pos2,above approach pos2,approach pos3,enter pos3,lifted pos3,above approach pos3

char operationNo=0;	//0=none 1=pick 2=drop 3=move to safe 4=move to home
char operationLocation=0;//0=none, 1=right 2=mid 3=left
char homeSafeMove=0;//0=home/1=safe/2=moving


//moved items
char operation=0;
char switchStates[3]={0,0,0};
char opID=0;//0 is no op 1is LMRLMR 2 is LMML 3 is RMLRML
//					0,1,2,3,4,5,6,7,
char LMRLMROrder[]={0,1,4,5,6,7,1,2,11,10,9,8,2,3,12,13,14,15,3,1,7,6,5,4,1,2,8,9,10,11,2,3,15,14,13,12,3,0};
//char LMMLOrder[]={0,1,4,5,6,7,1,2,11,10,9,8,8,9,10,11,2,1,7,6,5,4,1,0};//not needed due to only allowing the changing of right side
char RMLRMLOrder[]={0,3,12,13,14,15,3,2,11,10,9,8,2,1,4,5,6,7,1,3,15,14,13,12,3,2,8,9,10,11,2,1,7,6,5,4,1,0};

//servo/side             0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F  G
char locationDigitsZ[]={76,76,65,50,76,76,76,76,65,65,65,65,50,50,50,50,99};
char locationDigitsS[]={60,79,79,79,60,65,65,79,60,65,65,79,50,55,55,79,99};
char locationDigitsL[]={16,16,16,16,16,16,35,35,16,16,35,35,16,16,35,35,99};

struct ID {
	char occ;
	char filledFlag;//used to determine activity
	int timeLeft;//in mins
}shelf[3];
//eo moved items



/*
operation will go as follows, internal and external sources will choose a state
state will cause prog to set .need bits,they will be entered from a loadbank of defines
if a door is opened, need will be made to match last to keep pos'n
state will revert after user checking and finish its last operation
once all movements are done in order, robot will return home
*/
struct servo
{
unsigned char last;
unsigned char need;
}ZAxis,SArm,LArm;
//here begins lcd stuff

#define RS 	PORTEbits.RE1//register select 0=command 1=text data
//#define RW 	PORTAbits.RA1//read/write 0=write 1=read
#define E 	PORTEbits.RE2//enable, set to low,prep other lines,bring high,wait 1us,bring low
#define DB4 PORTCbits.RC7
#define DB5 PORTCbits.RC6//swap to this so db7 goes into rc7, logic is backwards for some reason
#define DB6 PORTCbits.RC5
#define DB7 PORTCbits.RC4
#define NUM 0x30//nums
#define CAP 0x41//caps
#define LWR 0x61//lwr case
#define ACPI  -65//cap letters add to ascii char to get digits for LCD values
#define ALWI  -97//lwr case 
#define N2I   -48//numbers
#define COLMAX 20//max no of cols



void LCD_PRINT(char input[20]);
void startDisplay();




void initialize(void)
{
//OSCCON=0b01110011;//16mhz
	OSCCON=0b00100011;

/*
250nsTcy
1us=4Tcy
1ms=4,000Tcy
1s=4,000,000Tcy
*/
PORTB=0;
TRISB=0;
ANSELB=0;
PORTC=0x00;
TRISC=0x00;
ANSELC=0;
PORTE=0x00;
TRISE=0x00;
ANSELE=0;


}


void ACTION_PULSE(void)
{
E=0;
usDelay(1);
E=1;
usDelay(1);
E=0;
usDelay(100);
}

void pinSet(unsigned char value)
{
if(value&0x01)DB7=1;
else DB7=0;
if(value&0x02)DB6=1;
else DB6=0;
if(value&0x04)DB5=1;
else DB5=0;
if(value&0x08)DB4=1;
else DB4=0;

ACTION_PULSE();
}


void LCD_CMD(unsigned char action)
{
RS=0;
pinSet(action>>4);
pinSet(action);
}

void LCD_WRITE(unsigned char action)
{
RS=1;
pinSet(action>>4);
pinSet(action);
}

void CLEAR(void)
{
LCD_CMD(0x01);
usDelay(3000);
}

void LCD_SETPOS(unsigned char row ,unsigned char col)
{//20 col,4row
unsigned char val0=0; //start of row 0
unsigned char val1=64;//start of row 1
unsigned char val2=20;//start of row 2
unsigned char val3=84;//tart of row 3
unsigned char pos=0;

pos=*(&val0+row);
LCD_CMD(0x80|pos);
rowLocation=row;
colLocation=col;
}



void LCD_Init(void)
{
E=0;
RS=0;
pinSet(0x03);
usDelay(4500);
pinSet(0x03);
usDelay(4500);
pinSet(0x03);
usDelay(150);
pinSet(0x02);

LCD_CMD(0x28);
LCD_CMD(0x0f);
CLEAR();
LCD_CMD(0x06);
startDisplay();
}

void secondDelay()
{
int i=0;
for(i=0;i<200;i++)
	{
	Delay10KTCYx(2);
	}
}

void LCD_PRINT(char * input)
{
char item;
char x=0;
while(colLocation<COLMAX && *(input+x)!=0)
	{
	item=*(input+x);
	if(item>=97)item=item+ALWI+LWR;
	else if(item>=65)item=item+ACPI+CAP;
	else if(item>=48)item=item+N2I+NUM;
	LCD_WRITE(item);
	x++;
	colLocation++;
	};
}


void startDisplay()
{
char word[]="LRJ Greenhouse";
LCD_SETPOS(0,0);
LCD_PRINT(word);
}

//here ends lcd stuffs

void usDelay(unsigned int value)
{
	int i;
	for(i=0; i<value ; i++)//each cycle is 1us
	{
		Delay1TCY();
		Delay1TCY();
		Delay1TCY();
		Delay1TCY();
	}
}


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

void _ccp3Set(unsigned char need,unsigned char last)
{
	unsigned char value;
	if(last<need)value=last+1;
	else if (last>need)value=last-1;
	else if(last==need)value=last;
	if(value&0b00000010)CCP3CON=CCP3CON|0b00100000;
	else CCP3CON=CCP3CON&0b11011111;
	if(value&0b00000001)CCP3CON=CCP3CON|0b00010000;
	else CCP3CON=CCP3CON&0b11101111;
    CCPR3L = (value/4) ;
	ZAxis.last=value;
}
void _ccp2Set(unsigned char need,unsigned char last)
{
	unsigned char value;
	if(last<need)value=last+1;
	else if (last>need)value=last-1;
	else if(last==need)value=last;
	if(value&0b00000010)CCP2CON=CCP2CON|0b00100000;
	else CCP2CON=CCP2CON&0b11011111;
	if(value&0b00000001)CCP2CON=CCP2CON|0b00010000;
	else CCP2CON=CCP2CON&0b11101111;
    CCPR2L = (value/4) ;
	LArm.last=value;
}
void _ccp4Set(unsigned char need,unsigned char last)
{
	unsigned char value;
	if(last<need)value=last+1;
	else if (last>need)value=last-1;
	else if(last==need)value=last;
	if(value&0b00000010)CCP4CON=CCP4CON|0b00100000;
	else CCP4CON=CCP4CON&0b11011111;
	if(value&0b00000001)CCP4CON=CCP4CON|0b00010000;
	else CCP4CON=CCP4CON&0b11101111;
    CCPR4L = (value/4) ;
	SArm.last=value;
}

unsigned char _convert(unsigned char num)
{
char returnVal=0;
float numCalc=num/180.0;
numCalc=((numCalc*140)+10);
returnVal=(int)numCalc;
return returnVal;
}

void _PRINT_TIME()
{
char T0[]="Tray0:     mins left";
char T1[]="Tray1:     mins left";
char T2[]="Tray2:     mins left";
T0[6]=(shelf[0].timeLeft/10000%10)+48;
T0[7]=(shelf[0].timeLeft/1000%10)+48;
T0[8]=(shelf[0].timeLeft/100%10)+48;
T0[9]=(shelf[0].timeLeft/10%10)+48;
T0[10]=(shelf[0].timeLeft%10)+48;
T1[6]=(shelf[1].timeLeft/10000%10)+48;
T1[7]=(shelf[1].timeLeft/1000%10)+48;
T1[8]=(shelf[1].timeLeft/100%10)+48;
T1[9]=(shelf[1].timeLeft/10%10)+48;
T1[10]=(shelf[1].timeLeft%10)+48;
T2[6]=(shelf[2].timeLeft/10000%10)+48;
T2[7]=(shelf[2].timeLeft/1000%10)+48;
T2[8]=(shelf[2].timeLeft/100%10)+48;
T2[9]=(shelf[2].timeLeft/10%10)+48;
T2[10]=(shelf[2].timeLeft%10)+48;
LCD_SETPOS(1,0);
LCD_PRINT(T0);
LCD_SETPOS(2,0);
LCD_PRINT(T1);
LCD_SETPOS(3,0);
LCD_PRINT(T2);
}





void main()
{
	char word0[]="Not level!          ";
	char word1[]="                    ";
	char word2[]="Door open!          ";
	char word3[]="System ready.       ";	
	unsigned char holdd=0;
    unsigned char   operation=0;
	char buthold=0;
	char tilted=0;
	unsigned char derp=20;
	unsigned char herp1=10;
	unsigned char herp2=0;
	unsigned char hold=0;
	unsigned char hold2=0;
	int numInt=0;
	float numCalc=0;
	char readSel=0;
	char doorLSValue=1;
	int run=0;
	char lastOpID=0;
	char opNumber=0;
	char changedFlag=0;//user emptied right side tray
	char movingHomeStop=0;//0=home 1=moving 2=stopped
	int levelValue0,levelValue1;
	char firstRun=1;
	char combo=0;//0 is 110 1 is 011 2 is 101
	char comboLast=0;
	char promptFlag=0;
	unsigned int counter=0;
	char secondCounter=0;
	char timeChangeFlag=1;
	
	//tray0.location=0;
	//tray0.destination=0;
	//tray0.filledFlag=0;
	//tray0.timeLeft=0;

	//tray1.location=2;
	//tray1.destination=0;
	//tray1.filledFlag=0;
	//tray1.timeLeft=681;

	OSCCON=0b00100011;
	E=0;
	RS=0;
    TRISC=0b11111111;                     // set PORTC as output
    PORTC=0;
	TRISD=0;
	PORTD=0;
	TRISE=0;
	PORTE=0;
	initialize();
	LCD_Init();
	ZAxis.need=_convert(90);//convert is degrees to default resolution
	SArm.need=_convert(90);
	LArm.need=_convert(0);
	ZAxis.last=0;
	SArm.last=0;
	LArm.last=0;

	T0CON=0b10000000;//8 prescale 49911 value for .25sec
	INTCONbits.TMR0IF=0;
	WriteTimer0(65530);


	PORTA=0;
	TRISA =0b01100110;
	ANSELA=0b01000110;

	PORTB =0b00000000;
	TRISB =0b00011100;
	ANSELB=0b00010100;

	TRISC=TRISC  |0b00001001;
	ANSELC=ANSELC|0b00000000;

	TRISD=TRISD  |0b00000000;
	ANSELD=ANSELD|0b00000000;

	ADCON0=0x01;
	ADCON1=0x00;
	ADCON2=0x8D;

	ADCON0=WATERSENSOR1ADC;                    

	CCPTMRS0=0b00010000;
	CCPTMRS1=0b00000001;

	PR2 = 0b01001101	 ;
	T2CON = 0b00000111 ;
	PR4 = 0b01001101	 ;
	T4CON = 0b00000111 ;
	PR6 = 0b01001101	 ;
	T6CON = 0b00000111 ;

	CCPR3L = 0b00000000 ;
	CCPR4L = 0b00000000 ;
	CCP3CON = 0b00001100 ;
	CCP2CON = 0b00001100 ;
	CCP4CON = 0b00001100 ;
	CCPR2L = 0b00000000 ;//bits 4,5 are used for bits 0,1
	while(1){

		ZAxis.need=locationDigitsZ[opNumber];
		SArm.need=locationDigitsS[opNumber];
		LArm.need=locationDigitsL[opNumber];
		if(INTCONbits.TMR0IF==1&&doorLSValue==0&&firstRun==0)
		{
			_ccp3Set(ZAxis.need,ZAxis.last);//inc is left dec is right
			_ccp2Set(LArm.need,LArm.last);//inc is extend
			_ccp4Set(SArm.need,SArm.last);//inc is extend
			//all above functions restrict movement to 1 bit per call
		}
		if(INTCONbits.TMR0IF==1)
		{
			WriteTimer0(63000);//2ms/movement minimum
			INTCONbits.TMR0IF=0;

				counter++;
				if(counter==19)
				{
					counter=1;
					secondCounter++;
					if(secondCounter==10)
					{
						secondCounter=0;
						if(shelf[0].timeLeft>0)shelf[0].timeLeft--;
						if(shelf[1].timeLeft>0)shelf[1].timeLeft--;
						if(shelf[2].timeLeft>0)shelf[2].timeLeft--;
						timeChangeFlag=1;
					}
				}
			
		}

		if(PIR1bits.ADIF==1)
		{
			if(readSel==0)//
			{
				holdd=ADRESH;
				numCalc=(holdd/3.0)*192.0;
				holdd=(int)numCalc;
				hold2=ADRESL/4;
				hold=holdd|hold2;
				numCalc=((hold/255.0)*50);
				herp2=(int)numCalc;
				//ZAxis.need=herp2;//x1-debug code for servo locations
				ADCON0=WATERSENSOR2ADC;
				readSel++;
				levelValue0=numCalc;
			}
			else if (readSel==1)//water level sensor0
			{
				holdd=ADRESH;
				numCalc=(holdd/3.0)*192.0;
				holdd=(int)numCalc;
				hold2=ADRESL/4;
				hold=holdd|hold2;
				numCalc=((hold/255.0)*50);
				herp2=(int)numCalc;
				//SArm.need=herp2;//x1
				ADCON0=LIGHTSENSORADC;
				levelValue1=numCalc;
				readSel++;
				if (((levelValue0+1)==levelValue1)||((levelValue0-1)==levelValue1)||(levelValue0==levelValue1))tilted=0;
				else tilted=1;
			}
			else if (readSel==2)//WATER LEVEL SENSOR 1 and logic
			{
				holdd=ADRESL;
				numInt=0;
				numInt=ADRESH;
				numInt=numInt<<8;
				numInt+=holdd;
				numCalc=numInt;
				numCalc=numCalc/1024.0;
				numCalc=numCalc*50;
			//	LArm.need=herp2;//x1
				ADCON0=SHELFSENSORADC;
				readSel++;
			}
			else if (readSel==3)//NEED ANSWERS this will determine locations
			{
				holdd=ADRESL;
				numInt=0;
				numInt=ADRESH;
				numInt=numInt<<8;
				numInt+=holdd;
				numCalc=numInt;
				numCalc=numCalc/1024.0;
				numCalc=numCalc*50;
			//	LArm.need=herp2;//x1
				ADCON0=DOORLSADC;
				readSel++;
				if(firstRun==1)
				{
					if(numCalc>=(3.51+LL)&&numCalc<=(3.51+UL))//100
					{
					shelf[0].occ=1;//l
					shelf[1].occ=0;//m
					shelf[2].occ=0;//r
					}
					else if(numCalc>=(41.8+LL)&&numCalc<=(41.8+UL))//010
					{
					shelf[0].occ=0;
					shelf[1].occ=1;
					shelf[2].occ=0;
					}
					else if(numCalc>=(45.1+LL)&&numCalc<=(45.1+UL))//001
					{
					shelf[0].occ=0;
					shelf[1].occ=0;
					shelf[2].occ=1;
					}
					else if(numCalc>=(30.8+LL)&&numCalc<=(30.8+UL))//110
					{
					shelf[0].occ=1;
					shelf[1].occ=1;
					shelf[2].occ=0;
					}
					else if(numCalc>=(32.6+LL)&&numCalc<=(32.6+UL))//101
					{
					shelf[0].occ=1;
					shelf[1].occ=0;
					shelf[2].occ=1;
					}
					else if(numCalc>=(38.3+LL)&&numCalc<=(38.3+UL))//011
					{
					shelf[0].occ=0;
					shelf[1].occ=1;
					shelf[2].occ=1;
					}

	
				}
				combo=shelf[0].occ+(shelf[1].occ*2)+(shelf[2].occ*4);
				if (doorLSValue!=1)
				{
					comboLast=combo;
				}
				else 
				{
					if(combo!=comboLast)promptFlag=1;
				}
			
			}
			else if (readSel==4)//DOORLOGIC
			{
				holdd=ADRESL;
				numInt=0;
				numInt=ADRESH;
				numInt=numInt<<8;
				numInt+=holdd;
				numCalc=numInt;
				numCalc=numCalc/1024.0;
				numCalc=numCalc*50;
			//	LArm.need=herp2;//x1
				ADCON0=WATERSENSOR1ADC;
				readSel=0;
				if (numCalc<47)doorLSValue=1;
				else doorLSValue=0;
				if(firstRun||doorLSValue!=1||tilted)
					{
					LCD_SETPOS(1,0);
					if(tilted)LCD_PRINT(word0);
					else if(doorLSValue==0)LCD_PRINT(word3);
					else LCD_PRINT(word1);
					LCD_SETPOS(2,0);
					if(doorLSValue)LCD_PRINT(word2);
					else LCD_PRINT(word1);
					}
			}
		}
		if(changedFlag==1&&movingHomeStop==0&&firstRun==0)//if the user emptied, right will always need more time than left, just rinse and swap while rinsing
		{
			opID=2;
			movingHomeStop=0;
			operation=0;
		}
		else if ((movingHomeStop==0&&tray0.timeLeft>tray1.timeLeft&&tray0.location==0&&firstRun==0)||(movingHomeStop==0&&tray1.timeLeft>tray0.timeLeft&&tray1.location==0&&firstRun==0))	
		{
			opID=1;
			movingHomeStop=1;
			operation=0;
			if(tray0.location==0)tray0.destination=2;
			else tray0.destination=0;
			if(tray1.location==0)tray1.destination=2;
			else tray1.destination=0;

		}		//swapping left and right, using middle as a medium
		else if ((movingHomeStop==0&&tray0.location==1&&firstRun==0)||(movingHomeStop==0&&tray1.location==1&&firstRun==0))
		{
			if(tray0.location==1&&tray1.location==0)
			{
			tray0.destination=2;
			opID=1;
			}
			else if (tray0.location==1&&tray1.location==2)
			{
			tray0.destination=0;
			opID=2;
			}
			else if (tray1.location==1&&tray0.location==0)
			{
			tray1.destination=2;
			opID=1;
			}
			else if (tray1.location==1&&tray0.location==2)
			{
			tray1.destination=0;
			opID=2;
			}
			movingHomeStop=1;
			operation=25;
		}
	
		if (opID==0)
		{
			operation=0;
		}
		else if(opID==1)
		{
			if(ZAxis.need==ZAxis.last&&SArm.need==SArm.last&&LArm.need==LArm.last)
			{	
			operation++;
			opNumber=LMRLMROrder[operation];
			}	
			if(operation==37)
			{
				opID=0;
				operation=0;
				movingHomeStop==0;
				if(tray1.location==0)tray1.location=2;
				else tray1.location=0;
				if(tray0.location==0)tray0.location=2;
				else tray0.location=0;
			}
		}
		else if (opID==2)
		{
			if(ZAxis.need==ZAxis.last&&SArm.need==SArm.last&&LArm.need==LArm.last)
			{
				operation++;
				opNumber=RMLRMLOrder[operation];
			}	
			if(operation>=11&&tilted==0)
			{
				if(operation<25)PUMP1=1;
				else PUMP1=0;
			}
			if(operation==37)
			{
				opID=0;
				operation=0;
				movingHomeStop==0;
				changedFlag=0;
				tray0.location=tray0.destination;
				tray1.location=tray1.destination;
			}
		}
		else if (opID==3)
		{
			if(ZAxis.need==ZAxis.last&&SArm.need==SArm.last&&LArm.need==LArm.last)
			{
				operation++;
				opNumber=RMLRMLOrder[operation];
			}	
			if(operation==37)
			{
				opID=0;
				operation=0;
				movingHomeStop==0;
				changedFlag=0;
				tray0.location=tray0.destination;
				tray1.location=tray1.destination;
			}
		}
		if(movingHomeStop==0&&timeChangeFlag==1&&firstRun==0)
		{
		timeChangeFlag=0;
		_PRINT_TIME();
		}

	}
}		
