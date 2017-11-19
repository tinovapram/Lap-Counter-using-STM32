/*
 * LCD.h
 *
 *  Created on: 9 Nov 2017
 *      Author: tinova
 */
#include "stm32f4xx_hal.h" //ganti ini
#include "main.h" //Deklarasi RW, RS, EN, DB7-DB4 disini

#ifndef LCD_H_
#define LCD_H_

void SET_EN(void);
void CLR_EN(void);
void SET_RW(void);
void CLR_RW(void);
void SET_RS(void);
void CLR_RS(void);
void enable(void);                 // enable fulse for LCD
void lcd_init(void);               // intialization LCD
void lcd_putchar(unsigned char c); // send character to LCD
void lcd_putstr(char *s);          // send string to LCD
void lcd_clear(void);              // clear display
void lcd_gotoxy(unsigned char j,unsigned char i);  //move LCD cursor position

#endif /* LCD_H_ */
