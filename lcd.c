#include <stm32f4xx.h>
#include <system_stm32f4xx.h>
#include "lcd.h"

unsigned char BF;
unsigned char blank[20] = {"                   "};


void LCD_Init(void){
    //Setup LCD Pins
    LCD_PORT->MODER = 0x5055;     //Set the mode to OUTPUT for PD0:PD3,PD6,PD7
    LCD_PORT->OTYPER = 0x00;      //Set the TYPE to PUSH-PULL for all
    LCD_PORT->OSPEEDR = 0xA0AA;   //50 MHz Speed on the LCD Port
    LCD_PORT->PUPDR = 0x00;       //PULL-UP/PULL-DOWN DISABLED

    BitClr(LCD_E);                //clear enable
    BitClr(LCD_RS);               //going to write command

    DelayMS(30);                //delay for LCD to initialise.
    LCD_NYB(0x30,0);              //Required for initialisation
    DelayMS(5);                 //required delay
    LCD_NYB(0x30,0);              //Required for initialisation
    DelayMS(1);                 //required delay
    LCD_DATA(0x02,0);           //set to 4 bit interface, 1 line and 5*7 font
    LCD_DATA(0x28,0);           //set to 4 bit interface, 2 line and 5*10 font
    LCD_DATA(0x0c,0);           //set to 4 bit interface, 2 line and 5*7 font
    LCD_DATA(0x01,0);           //clear display
    LCD_DATA(0x06,0);           //move cursor right after write

}
//--------------------------------------------------------------------------------//
void LCD_DATA(unsigned char data,unsigned char type){
    
    WaitLCDBusy();                  //TEST LCD FOR BUSY 

    if(type == CMD){
        BitClr(LCD_RS);             //COMMAND MODE
    } else {
        BitSet(LCD_RS);             //CHARACTER/DATA MODE
    }

    LCD_NYB(data>>4,type);               //WRITE THE UPPER NIBBLE
    LCD_NYB(data,type);                  //WRITE THE LOWER NIBBLE
}
//--------------------------------------------------------------------------------//
void WaitLCDBusy(void){
    DelayMS(2);              //DELAY 1 MilliSeconds
}
//--------------------------------------------------------------------------------//
void LCD_NYB(unsigned char nyb,unsigned char type){
    //SEND DATA LINE THE INFO 
    LCD_PORT->BSRRH |= 0x0F;
    LCD_PORT->BSRRL |= (nyb & 0x0F);

    if(type == CMD){
        BitClr(LCD_RS);             //COMMAND MODE
    } else {
        BitSet(LCD_RS);             //CHARACTER/DATA MODE
    }

    BitSet(LCD_E);         //ENABLE LCD DATA LINE
    DelayMS(1);                 //SMALL DELAY
    BitClr(LCD_E);         //DISABLE LCD DATA LINE
}
//--------------------------------------------------------------------------------//
void LCD_STR(unsigned char *text, char clear){
unsigned char x=0;
    while(*text)
	{
        LCD_DATA(*text++,1);
		x++;
    }
	if(clear == 1)
	{
		while(x<16)
		{
			LCD_DATA(0x20,1);
			x++;
		}
	}
}
//--------------------------------------------------------------------------------//
void LCD_LINE(char line){
    switch(line){
        case 0:
        case 1:
            LCD_DATA(LINE1,0);
            break;
        case 2:
            LCD_DATA(LINE2,0);
            break;
    }
}
//--------------------------------------------------------------------------------//

//--------------------------------------------------------------------------------//
void DelayMS(unsigned int ms){
    unsigned int x,i;
    for(x=0;x<ms;x++)
    {
        for(i=0;i<16400;i++);
    }
}
//--------------------------------------------------------------------------------//
void LCD_POS(unsigned char line){
unsigned char myDat;
unsigned char pos = 18;



    LCD_LINE(line);
    switch(line){
        case 0:
        case 1:
            myDat = LINE1 + pos;
            break;
        case 2:
            myDat = LINE2 + pos;
            break;

    }

    myDat |= 0x80;
    LCD_DATA(myDat,0);
}