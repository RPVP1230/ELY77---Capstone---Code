
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
#define UL 0.02
#define LL -0.02

#define GROWLIGHTS PORTAbits.RA0
#define BUTTON1 PORTCbits.RC0
#define BUTTON2 PORTBbits.RB3
#define BUTTON3 PORTCbits.RC2
#define BUTTON4 PORTCbits.RC3
#define PUMP1 PORTDbits.RD3//rinse pump
#define PUMP2 PORTDbits.RD2//fill pump
#define FAN1 PORTDbits.RD5
#define FAN2 PORTDbits.RD6
#define WATERSENSOR1ADC 0x07
#define WATERSENSOR2ADC 0x0b
#define LIGHTSENSORADC 0x2f
#define SHELFSENSORADC 0x23
#define DOORLSADC 0x13
#define TEMPADC 0x33
#define HUMIDITYADC 0x0f

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


void _PRINT_TIME()
{
char T0[]   ="Tray0:     mins left";
char EMPTY[]="Tray   empty        ";
char index=0;
	while(index<3)
	{
		LCD_SETPOS(index+1,0);
		if(shelf[index].filledFlag==1)
		{
			T0[6]=(shelf[index].timeLeft/10000%10)+48;
			T0[7]=(shelf[index].timeLeft/1000%10)+48;
			T0[8]=(shelf[index].timeLeft/100%10)+48;
			T0[9]=(shelf[index].timeLeft/10%10)+48;
			T0[10]=(shelf[index].timeLeft%10)+48;
			T0[4]=index+49;	
			LCD_PRINT(T0);
		}
		else 
		{
			EMPTY[5]=index+49;	
			LCD_PRINT(EMPTY);
		}
		index++;
	}
}

char seedPrompt(char id,char initTrigger)
{
static char indexPrompt=0;
char prompt0[]="Tray:  Change?      ";
char prompt4[]="Are there seeds?	";
char prompt5[]="<: yes >: no        ";
	if(id==2)
	{
		shelf[indexPrompt].filledFlag=1;
		shelf[indexPrompt].timeLeft=10080;
	}
	else if(id==3)
	{
		shelf[indexPrompt].filledFlag=0;
		shelf[indexPrompt].timeLeft=20000;
	}
	if(id==2||id==3)indexPrompt++;
	if(indexPrompt<3)
	{
		if(shelf[indexPrompt].occ==0)
		{
		shelf[indexPrompt].filledFlag=0;
		indexPrompt++;
		}
	}
	if(indexPrompt<3)
	{
		if(shelf[indexPrompt].occ==0)
		{
		shelf[indexPrompt].filledFlag=0;
		indexPrompt++;
		}
	}
	if(indexPrompt<3)
	{
		if(shelf[indexPrompt].occ==0)
		{
		shelf[indexPrompt].filledFlag=0;
		indexPrompt++;
		}
	}

	if(indexPrompt<3)
	{
		prompt0[5]=indexPrompt+49;
		LCD_SETPOS(0,0);
		LCD_PRINT(prompt0);
		if(initTrigger==1)
		{
			LCD_SETPOS(1,0);
			LCD_PRINT(prompt4);
			LCD_SETPOS(2,0);
			LCD_PRINT(prompt5);;
		}
	}
		if(indexPrompt<3)return 0;
		else
		{
			indexPrompt=0;
			return 5;
		}
}


char _PROMPT_USER(char id,char initTrigger)
{
static char index=0;
char prompt0[]="Tray:  Change?      ";
char prompt1[]="^:new seeds         ";
char prompt2[]="<: no change        ";
char prompt3[]=">: harvested/empty  ";
if(id==1)
{
shelf[index].filledFlag=1;
shelf[index].timeLeft=10080;
}
else if(id==3)
{
shelf[index].filledFlag=0;
shelf[index].timeLeft=20000;
}

if(id!=0)index++;
if(shelf[index].occ==0&&index<3)
{
shelf[index].filledFlag=0;
shelf[index].timeLeft=20000;
index++;
}
if(shelf[index].occ==0&&index<3)
{
shelf[index].filledFlag=0;
shelf[index].timeLeft=20000;
index++;
}
if(shelf[index].occ==0&&index<3)
{
shelf[index].filledFlag=0;
shelf[index].timeLeft=20000;
index++;
}
	if(index<3)
	{
	prompt0[5]=index+49;
	LCD_SETPOS(0,0);
	LCD_PRINT(prompt0);
	if(initTrigger==1)
		{
		LCD_SETPOS(1,0);
		LCD_PRINT(prompt1);
		LCD_SETPOS(2,0);
		LCD_PRINT(prompt2);
		LCD_SETPOS(3,0);
		LCD_PRINT(prompt3);
		}
	}
	if(index<3)return 0;
	else
	{
		index=0;
		return 5;
	}

}




void main()
{
	char word0[]="Not level!          ";
	char word1[]="                    ";
	char word2[]="Door open!          ";
	char word3[]="System ready.       ";	
	unsigned char holdd=0;
    unsigned char operation=0;
	char tilted=0;
	unsigned char hold=0;
	int numInt=0;
	float numCalc=0;
	char readSel=0;
	char doorLSValue=1;
	char lastOpID=0;
	char opNumber=0;
	char changedFlag=0;//user emptied right side tray
	char movingHomeStop=0;//0=home 1=moving 2=stopped
	int levelValue0,levelValue1;
	char firstRun=1;
	char combo=0;//0 is 110 1 is 011 2 is 101
	char comboLast=0;
	char promptFlag=0;
	unsigned char counter=0;
	char secondCounter=0;
	char timeChangeFlag=1;
	char startCMD=0;
	int tempHold=0;
	char butID=0;
	char butTrigger=0;
	char displayTrigger=1;
	char tooManyFlag=0;
	char pumpFlag=0;
	char evalFlag=0;
	OSCCON=0b00100011;
	E=0;
	RS=0;
    TRISC=0b11111111;                     // set PORTC as output
    PORTC=0;
	TRISD=0;
	TRISE=0;
	PORTE=0;
	ANSELD=0;
	initialize();
	LCD_Init();
	ZAxis.last=0;
	SArm.last=0;
	LArm.last=0;

	T0CON=0b10000000;//8 prescale 49911 value for .25sec
	INTCONbits.TMR0IF=0;
	WriteTimer0(65530);


	PORTA=0b00000001;
	TRISA =0b01100110;
	ANSELA=0b01000110;

	PORTB =0b00000000;
	TRISB =0b00011101;
	ANSELB=0b00010101;

	TRISC=TRISC  |0b00001101;
	ANSELC=ANSELC|0b00000000;

	TRISD=TRISD  |0b10000000;
	ANSELD=ANSELD|0b10000000;

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
	while(0){

		ZAxis.need=locationDigitsZ[opNumber];
		SArm.need=locationDigitsS[opNumber];
		LArm.need=locationDigitsL[opNumber];
		if(INTCONbits.TMR0IF==1&&doorLSValue==0&&firstRun==0&&promptFlag==0&&tooManyFlag==0)
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
				if(counter==19)//operations/second
				{
					counter=1;
					secondCounter++;
					if(secondCounter==20)
					{
						secondCounter=0;
						if(shelf[0].timeLeft>0&&shelf[0].filledFlag==1)shelf[0].timeLeft--;
						if(shelf[1].timeLeft>0&&shelf[1].filledFlag==1)shelf[1].timeLeft--;
						if(shelf[2].timeLeft>0&&shelf[2].filledFlag==1)shelf[2].timeLeft--;
						timeChangeFlag=1;
						pumpFlag=(pumpFlag+1)&0b00000001;
					}
					 if(secondCounter<10&&pumpFlag==1&&movingHomeStop==0)
					{
					PORTD=PORTD|0b00000100;
					}
					else 
					{
					PORTD=PORTD&0b11111011;
					}
				}
			
		}

		if(PIR1bits.ADIF==1)
		{
			if(readSel==0)//
			{
				holdd=ADRESL;
				numInt=0;
				numInt=ADRESH;
				numInt=numInt<<8;
				numInt+=holdd;
				numCalc=numInt;
				numCalc=numCalc/1024.0;
				numCalc=numCalc*50;
				//ZAxis.need=herp2;//x1-debug code for servo locations
				ADCON0=WATERSENSOR2ADC;
				readSel++;
				levelValue0=numCalc;
			}
			else if (readSel==1)//water level sensor0
			{
				holdd=ADRESL;
				numInt=0;
				numInt=ADRESH;
				numInt=numInt<<8;
				numInt+=holdd;
				numCalc=numInt;
				numCalc=numCalc/1024.0;
				numCalc=numCalc*50;
				ADCON0=LIGHTSENSORADC;
				levelValue1=numCalc;
				readSel++;
				if (((levelValue0+1)==levelValue1)||((levelValue0-1)==levelValue1)||(levelValue0==levelValue1))tilted=0;
				else tilted=1;
			}
			else if (readSel==2)//light sensr
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
			//	LArm.need=herp2;//x1
				ADCON0=DOORLSADC;
				readSel++;
				if(firstRun==1)
				{
					if(numCalc<0.6)
					{
					shelf[0].occ=1;//l
					shelf[1].occ=1;//m
					shelf[2].occ=1;//r
					tooManyFlag=1;
					}
					if(numCalc>=(0.7002+LL)&&numCalc<=(0.7002+UL))//100
					{
					shelf[0].occ=1;//l
					shelf[1].occ=0;//m
					shelf[2].occ=0;//r
					shelf[0].timeLeft=10080;//l
					shelf[1].timeLeft=20000;//m
					shelf[2].timeLeft=20000;//r
					tooManyFlag=0;
					}
					else if(numCalc>=(0.8369+LL)&&numCalc<=(0.8369+UL))//010
					{
					shelf[0].occ=0;
					shelf[1].occ=1;
					shelf[2].occ=0;
					shelf[0].timeLeft=20000;//l
					shelf[1].timeLeft=10080;//m
					shelf[2].timeLeft=20000;//r
					tooManyFlag=0;

					}
					else if(numCalc>=(0.8994+LL)&&numCalc<=(0.8994+UL))//001
					{
					shelf[0].occ=0;
					shelf[1].occ=0;
					shelf[2].occ=1;
					shelf[0].timeLeft=20000;//l
					shelf[1].timeLeft=20000;//m
					shelf[2].timeLeft=10080;//r
					tooManyFlag=0;
					}
					else if(numCalc>=(0.6152+LL)&&numCalc<=(0.6152+UL))//110
					{
					shelf[0].occ=1;
					shelf[1].occ=1;
					shelf[2].occ=0;
					shelf[0].timeLeft=10080;//l
					shelf[1].timeLeft=10080;//m
					shelf[2].timeLeft=20000;//r
					tooManyFlag=0;
					}
					else if(numCalc>=(0.6494+LL)&&numCalc<=(0.6494+UL))//101
					{
					shelf[0].occ=1;
					shelf[1].occ=0;
					shelf[2].occ=1;
					shelf[0].timeLeft=10080;//l
					shelf[1].timeLeft=20000;//m
					shelf[2].timeLeft=10080;//r
					tooManyFlag=0;
					}
					else if(numCalc>=(0.7617+LL)&&numCalc<=(0.7617+UL))//011
					{
					shelf[0].occ=0;
					shelf[1].occ=1;
					shelf[2].occ=1;
					shelf[0].timeLeft=20000;//l
					shelf[1].timeLeft=10080;//m
					shelf[2].timeLeft=10080;//r
					tooManyFlag=0;
					}
				}
				else 
				{
					if(numCalc<0.60)
					{
						shelf[0].occ=1;//l
						shelf[1].occ=1;//m
						shelf[2].occ=1;//r
						tooManyFlag=1;
					}
					else if(numCalc>=(0.7002+LL)&&numCalc<=(0.7002+UL))//100
					{
						shelf[0].occ=1;//l
						shelf[1].occ=0;//m
						shelf[2].occ=0;//r
						tooManyFlag=0;
					}
					else if(numCalc>=(0.8369+LL)&&numCalc<=(0.8369+UL))//010
					{
						shelf[0].occ=0;
						shelf[1].occ=1;
						shelf[2].occ=0;
						tooManyFlag=0;
					}
					else if(numCalc>=(0.8994+LL)&&numCalc<=(0.8994+UL))//001
					{
						shelf[0].occ=0;
						shelf[1].occ=0;
						shelf[2].occ=1;
						tooManyFlag=0;
					}
					else if(numCalc>=(0.6152+LL)&&numCalc<=(0.6152+UL))//110
					{
						shelf[0].occ=1;
						shelf[1].occ=1;
						shelf[2].occ=0;
						tooManyFlag=0;
					}
					else if(numCalc>=(0.6494+LL)&&numCalc<=(0.6494+UL))//101
					{
						shelf[0].occ=1;
						shelf[1].occ=0;
						shelf[2].occ=1;
						tooManyFlag=0;
					}
					else if(numCalc>=(0.7617+LL)&&numCalc<=(0.7617+UL))//011
					{
						shelf[0].occ=0;
						shelf[1].occ=1;
						shelf[2].occ=1;
						tooManyFlag=0;
					}
				}
				combo=shelf[0].occ+(shelf[1].occ*2)+(shelf[2].occ*4);
				if (doorLSValue==0||firstRun==1)
				{
					comboLast=combo;
				}
				else 
				{
					if(combo!=comboLast)promptFlag=1;
				}
			evalFlag=1;
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
				ADCON0=TEMPADC;
				readSel++;
				if (numCalc<47)doorLSValue=1;
				else doorLSValue=0;
				if(doorLSValue!=0||tilted)
					{
					LCD_SETPOS(1,0);
					if(tilted)LCD_PRINT(word0);
					else if(doorLSValue==0)LCD_PRINT(word3);
					else LCD_PRINT(word1);
					LCD_SETPOS(2,0);
					if(doorLSValue)LCD_PRINT(word2);
					else LCD_PRINT(word1);
					LCD_SETPOS(3,0);
					LCD_PRINT(word1);
					}
			}
			else if (readSel==5)//temp reading
			{
				holdd=ADRESL;
				numInt=0;
				numInt=ADRESH;
				numInt=numInt<<8;
				numInt+=holdd;
				numCalc=numInt;	//290 is regular temp//283 will be too hot value
				if(numCalc<=283.0)
				{
				PORTD=PORTD|0b01100000;
				}
				else 
				{
				PORTD=PORTD&0b10011111;
				}
				ADCON0=HUMIDITYADC;
				readSel++;
			}
			else if (readSel==6)//humidity sensor
			{
				holdd=ADRESL;
				numInt=0;
				numInt=ADRESH;
				numInt=numInt<<8;
				numInt+=holdd;
				numCalc=numInt;	//not sensative enough to use properly
				/*if(numCalc<=283.0)
				{
				PORTD=PORTD|0b01100000;
				}
				else 
				{
				PORTD=PORTD&0b10011111;
				}*/
				ADCON0=WATERSENSOR1ADC;
				readSel=0;
			}
		}
		
		if (movingHomeStop==0&&shelf[0].timeLeft<shelf[2].timeLeft&&firstRun==0&&promptFlag==0&&evalFlag==1)	
		{
			opID=1;
			movingHomeStop=1;
			operation=0;
			startCMD=0;
		}		//swapping left and right, using middle as a medium
		else if (movingHomeStop==0&&shelf[1].occ==1&&firstRun==0&&promptFlag==0&&evalFlag==1)
		{
			if(shelf[0].occ==1)
			{
			opID=1;
			}
			else if(shelf[2].occ==1)
			{
			opID=2;
			}
			else 
			{
			opID=1;
			}

			movingHomeStop=1;
			operation=25;
			startCMD=1;//moving mid flag
		}
		evalFlag=0;
	
		if (opID==0)
		{
			operation=0;
			movingHomeStop=0;
		}
		else if(opID==1)
		{
			if(ZAxis.need==ZAxis.last&&SArm.need==SArm.last&&LArm.need==LArm.last)
			{	
			operation++;
			opNumber=LMRLMROrder[operation];
			}	
			if(operation>=11&&tilted==0)
			{
				if(operation<25)PUMP1=1;
				else PUMP1=0;
			}
			if(startCMD==1)
			{
				if(operation<29)PUMP1=1;
				else PUMP1=0;
			}
			if(operation==37)
			{
				opID=0;
				operation=0;
				movingHomeStop=0;
				readSel=2;
				if(startCMD==1)//swapped mid and right
				{
				startCMD=0;
				tempHold=shelf[1].timeLeft;
				shelf[1].timeLeft=shelf[2].timeLeft;
				shelf[2].timeLeft=tempHold;
				tempHold=shelf[1].filledFlag;
				shelf[1].filledFlag=shelf[2].filledFlag;
				shelf[2].filledFlag=tempHold;
				}
				else//swapped right and left
				{
				tempHold=shelf[0].timeLeft;
				shelf[0].timeLeft=shelf[2].timeLeft;
				shelf[2].timeLeft=tempHold;
				tempHold=shelf[0].filledFlag;
				shelf[0].filledFlag=shelf[2].filledFlag;
				shelf[2].filledFlag=tempHold;
				}
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
			if(startCMD==1)
			{
				if(operation<29)PUMP1=1;
				else PUMP1=0;
			}
			if(operation==37)
			{
				opID=0;
				operation=0;
				movingHomeStop=0;
				changedFlag=0;
				readSel=2;
				if(startCMD==1)//swapped mid and right
				{
				startCMD=0;
				tempHold=shelf[1].timeLeft;
				shelf[1].timeLeft=shelf[0].timeLeft;
				shelf[0].timeLeft=tempHold;
				tempHold=shelf[1].filledFlag;
				shelf[1].filledFlag=shelf[0].filledFlag;
				shelf[0].filledFlag=tempHold;
				}
				else//swapped right and left
				{
				tempHold=shelf[0].timeLeft;
				shelf[0].timeLeft=shelf[2].timeLeft;
				shelf[2].timeLeft=tempHold;
				tempHold=shelf[0].filledFlag;
				shelf[0].filledFlag=shelf[2].filledFlag;
				shelf[2].filledFlag=tempHold;
				}
			}
		}
		if(promptFlag==1&&doorLSValue==0||firstRun&&readSel==5)
		{
			if(BUTTON1==1&&BUTTON2==1&&BUTTON3==1&&BUTTON4==1&&butTrigger==1)butTrigger=0;
			if(BUTTON1==0&&butTrigger==0)
			{
				butID=1;
				butTrigger=1;
			}
			else if(BUTTON2==0&&butTrigger==0)
			{
				butID=2;
				butTrigger=1;
			}
			else if(BUTTON3==0&&butTrigger==0)
			{
				butID=3;
				butTrigger=1;
			}
			else if(BUTTON4==0&&butTrigger==0)
			{
				butID=4;
				butTrigger=1;
			}
			if(firstRun==0)butID=_PROMPT_USER(butID,displayTrigger);
			else butID=seedPrompt(butID,displayTrigger);
			displayTrigger=0;
			if(butID==5)
				{
				butID=0;
				promptFlag=0;
				startDisplay();
				_PRINT_TIME();
				displayTrigger=1;
				firstRun=0;
				}
		}
		else if(movingHomeStop==0&&timeChangeFlag==1&&firstRun==0)
		{
		timeChangeFlag=0;
		_PRINT_TIME();
		}

	}
}		
