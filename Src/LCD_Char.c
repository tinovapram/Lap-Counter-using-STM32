#include "LCD_Char.h"
#include "stm32f4xx_hal.h" //ganti ini

void SET_EN(void) {
	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
}
void CLR_EN(void) {
	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
}
void SET_RW(void) {
	HAL_GPIO_WritePin(RW_GPIO_Port, RW_Pin, GPIO_PIN_SET);
}
void CLR_RW(void) {
	HAL_GPIO_WritePin(RW_GPIO_Port, RW_Pin, GPIO_PIN_RESET);
}
void SET_RS(void) {
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_SET);
}
void CLR_RS(void) {
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_RESET);
}

void enable(void) {
	SET_EN();
	HAL_Delay(3);   // minimal 2ms don't follow databook ;((450ns)<--
	CLR_EN();
}

void PORT_LCD(unsigned char count) {

	HAL_GPIO_WritePin(DB4_GPIO_Port, DB4_Pin, count & (1 << 4));
	HAL_GPIO_WritePin(DB5_GPIO_Port, DB5_Pin, count & (1 << 5));
	HAL_GPIO_WritePin(DB6_GPIO_Port, DB6_Pin, count & (1 << 6));
	HAL_GPIO_WritePin(DB7_GPIO_Port, DB7_Pin, count & (1 << 7));
	//HAL_Delay(1);
}

void lcd_init(void) {
	/* enable output for LCD
	 */
	CLR_EN();
	HAL_Delay(15);
	//step1
	PORT_LCD(0x30);
	CLR_RS();
	CLR_RW();
	enable();
	HAL_Delay(5);
	// step2
	PORT_LCD(0x30);
	CLR_RS();
	CLR_RW();
	enable();
	HAL_Delay(1);
	// step3
	PORT_LCD(0x30);
	CLR_RS();
	CLR_RW();
	enable();
	//next init
	PORT_LCD(0x20);
	CLR_RS();
	CLR_RW();
	enable();
	PORT_LCD(0x20);
	CLR_RS();
	CLR_RW();
	enable();
	PORT_LCD(0x80);
	CLR_RS();
	CLR_RW();
	enable();
	PORT_LCD(0x00);
	CLR_RS();
	CLR_RW();
	enable(); // display on
	PORT_LCD(0xE0);
	CLR_RS();
	CLR_RW();
	enable();
	PORT_LCD(0x00);
	CLR_RS();
	CLR_RW();
	enable(); // entry mode address inc and cursor shift right
	PORT_LCD(0x60);
	CLR_RS();
	CLR_RW();
	enable();
	PORT_LCD(0x00);
	CLR_RS();
	CLR_RW();
	enable(); // display clear
	PORT_LCD(0x10);
	CLR_RS();
	CLR_RW();
	enable(); // display clear
}

void lcd_putchar(unsigned char kar) {
	unsigned char kar2;
	//kar2=(kar|(PORT_LCD&0x0F))& (PORT_LCD|0xF0); //save control in bit 0-3
	//kar2= (kar & 0xF0) + (PORT_LCD &0x0F);
	//CLRBIT(kar2,EN);
	kar2 = kar & 0xF0;
	PORT_LCD(kar2);
	SET_RS();
	CLR_RW();
	enable(); //
	kar = (kar << 4 | kar >> 4);
	kar2 = kar & 0xF0;
	PORT_LCD(kar2);
	SET_RS();
	CLR_RW();
	enable(); // display clear
}

void lcd_putstr(char *s) {
	char i = 0;
	char c;
	while ((c = *(s + (i++))) != 0)
		lcd_putchar(c);
}

void lcd_clear(void) {
	PORT_LCD(0x00);
	CLR_RS();
	CLR_RW();
	enable(); // display clear
	PORT_LCD(0x10);
	CLR_RS();
	CLR_RW();
	enable(); //
}

void lcd_gotoxy(unsigned char j, unsigned char i) {
	unsigned char posisi, kar2;
	posisi = (i * 0x40) + j;
	//posisi=(i>0?0x40+j:j);
	kar2 = posisi & 0xF0;
	PORT_LCD(kar2);
	HAL_GPIO_WritePin(DB7_GPIO_Port, DB7_Pin, GPIO_PIN_SET);
	CLR_RS();
	CLR_RW();
	enable();
	posisi = (posisi << 4) | (posisi >> 4);     // swap temp
	kar2 = posisi & 0xF0;
	PORT_LCD(kar2);
	CLR_RS();
	CLR_RW();
	enable();
}

