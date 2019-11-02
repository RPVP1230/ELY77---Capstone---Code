#include <p18f45k22.h>
#include "pragmas.h"
#include <stdio.h>
#include <stdlib.h>
#include <delays.h>
#include <string.h>


#define RS 	PORTAbits.RA0//register select 0=command 1=text data
#define RW 	PORTAbits.RA1//read/write 0=write 1=read
#define E 	PORTAbits.RA2//enable, set to low,prep other lines,bring high,wait 1us,bring low
#define DB4 PORTCbits.RC4
#define DB5 PORTCbits.RC5
#define DB6 PORTCbits.RC6
#define DB7 PORTCbits.RC7
#define NUM 0x30//nums
#define CAP 0x41//caps
#define LWR 0x61//lwr case
#define ACPI  -65//cap letters add to ascii char to get digits for LCD values
#define ALWI  -97//lwr case 
#define N2I   -48//numbers
#define COLMAX 20//max no of cols


void usDelay(unsigned int value);
void LCD_PRINT(char input[20]);
void button1Pressed(char val);
void button2Pressed(char val);
void startDisplay();


char rowLocation=0;
char colLocation=0;

void initialize(void)
{
OSCCON=0b01110011;//16mhz

/*
250nsTcy
1us=4Tcy
1ms=4,000Tcy
1s=4,000,000Tcy
*/

PORTA=0x00;
TRISA=0x00;
ANSELA=0;

PORTC=0x00;
TRISC=0x00;
ANSELC=0;

PORTD=0;
TRISD=0b00000011;
ANSELD=0;

PORTB=0x00;//initial state
TRISB=0x00;//output
ANSELB=0x00;//digital

}

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
unsigned char val3=84;//start of row 3
unsigned char pos=0;

pos=*(&val0+row);
pos+=col;
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
	button1Pressed(0);
	button2Pressed(0);
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

void button1Pressed(char val)
{
char word1[]="button 1 pressed    ";
char word2[]="button 1 not pressed";
LCD_SETPOS(1,0);
if (val==1)LCD_PRINT(word1);
else LCD_PRINT(word2);
}
void button2Pressed(char val)
{
char word1[]="button 2 pressed    ";
char word2[]="button 2 not pressed";
LCD_SETPOS(2,0);
if (val==1)LCD_PRINT(word1);
else LCD_PRINT(word2);
}

void startDisplay()
{
char word[]="LRJ Greenhouse";
LCD_SETPOS(0,0);
LCD_PRINT(word);
}

void main(void)
{	int cnt=0;
	char actionNo=0;
	char word[]="byte me";
	char actionOcc=1;
	E=0;
	RW=0;
	RS=0;
	initialize();
	LCD_Init();
	LCD_SETPOS(3,4);
	while(1)
	{
		//secondDelay();
		if(actionNo==1 && actionOcc==0)
		{
			button1Pressed(1);
			actionOcc=1;	
		}
		else if(actionNo==2 && actionOcc==0)
		{
			button2Pressed(1);
			actionOcc=1;	
		}
		else if (actionNo==0 && actionOcc==1)
		{
			button1Pressed(0);
			button2Pressed(0);
			actionOcc=0;	
		}
		if(PORTDbits.RD1==1)actionNo=1;
		else if(PORTDbits.RD0==0)actionNo=2;
		else actionNo=0;
	}	
}
