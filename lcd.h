#ifndef __LCD_H
#define __LCD_H

#define LCD_PORT GPIOD    //LCD DATA PORT

#define LCD_RS   7        //LCD Command/Data Control
#define LCD_E    6        //LCD Enable Line

#define CMD 0
#define TXT 1
#define        LINE1    0x80        // Start address of first line
#define        LINE2    0xC0        // Start address of second line

#define BitClr(var) (LCD_PORT->BSRRH |= (1<<var))
#define BitSet(var) (LCD_PORT->BSRRL |= (1<<var))

void WaitLCDBusy(void);
void LCD_Init(void);
void LCD_DATA(unsigned char data,unsigned char type);
void LCD_NYB(unsigned char nyb,unsigned char type);
void LCD_LINE(char line);
void ShiftDisplay(unsigned char DC,unsigned char RL);
void DelayMS(unsigned int ms);
void LCD_STR(unsigned char *text, char clear);
void LCD_CLEAR(unsigned char line);

#endif
