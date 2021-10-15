#include "string.h"
#include "stdio.h"
#include "stm32f4xx_hal.h"

#define CHAR_1_5 0x01
#define CHAR_2_5 0x02
#define CHAR_3_5 0x03
#define CHAR_4_5 0x04
#define CHAR_5_5 0x05

//  String to display
char bar[16];

// Bitmaps for custom characters
uint8_t CUSTOM_1_5[] ={0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10};
uint8_t CUSTOM_2_5[] ={0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18};
uint8_t CUSTOM_3_5[] ={0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C};
uint8_t CUSTOM_4_5[] ={0x1E,0x1E,0x1E,0x1E,0x1E,0x1E,0x1E,0x1E};
uint8_t CUSTOM_5_5[] ={0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F};


// LCD Pins
#define LCD_RS GPIOB,GPIO_PIN_2
#define LCD_E GPIOB,GPIO_PIN_1
#define LCD_D4 GPIOB,GPIO_PIN_12
#define LCD_D5 GPIOB,GPIO_PIN_13
#define LCD_D6 GPIOB,GPIO_PIN_14
#define LCD_D7 GPIOB,GPIO_PIN_15
#define LCD_BL_ON GPIOA,GPIO_PIN_4


//  LCD code
void lcd_enable(){
	HAL_GPIO_WritePin(LCD_E, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(LCD_E, GPIO_PIN_SET);  //pulse needs to be some clock cycles long, we are not in hurry right now
	HAL_Delay(1);
	HAL_GPIO_WritePin(LCD_E, GPIO_PIN_RESET);
	HAL_Delay(1);
}

//  write a nibble (4 bits)
void lcd_write4(uint8_t word){
		HAL_GPIO_WritePin(LCD_D4, (word & 0x01)?GPIO_PIN_SET:GPIO_PIN_RESET); //we AND the word and the mask. If it's true, we write GPIO_PIN_SET, else _RESET
		HAL_GPIO_WritePin(LCD_D5, (word & 0x02)?GPIO_PIN_SET:GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LCD_D6, (word & 0x04)?GPIO_PIN_SET:GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LCD_D7, (word & 0x08)?GPIO_PIN_SET:GPIO_PIN_RESET);
		lcd_enable();  //pulse the E line
}

//  write a byte (8 bits)
void lcd_write(uint8_t word){	
		lcd_write4(word>>4); //we first write the upper nibble
		lcd_write4(word);    //and then the lower nibble
}

#define LCD_CLEAR_COMMAND 0x01
#define DISPLAY_COMMAND 0x08
#define BLINK_ON 0x01
#define CURSOR_ON 0x02
#define DISPLAY_ON 0x04
#define LCD_SETDRAMADD 0x80
uint8_t _display_ctrl = 0;

//  send an instruction to the LCD
void lcd_command(uint8_t byte){
	HAL_GPIO_WritePin(LCD_RS, GPIO_PIN_RESET); //write an instruction -> RS must be low
	lcd_write(byte);
}

void lcd_clear(){
	lcd_command(LCD_CLEAR_COMMAND);
}

//  send data to the LCD
void lcd_data(uint8_t byte){
	HAL_GPIO_WritePin(LCD_RS, GPIO_PIN_SET); //write data, not instruction -> RS must be high
	lcd_write(byte);
}

//  set (x, y) position of the cursor
void setCursor(uint8_t col, uint8_t row){
	if ((col+1)*(row+1)<80){
		lcd_command(LCD_SETDRAMADD|(col + 40*row)); //in the second row, address is offset by 40
	}
}

//  print a string on the display, starting from the cursor position
void lcd_print(char string[]){  //pointer to first char in the string
	
	int size = strlen(string);
	
	while (size--){
		lcd_data(*string++);
	}
}

void lcd_println(char string[], uint8_t row){
	
	char line[] = "                ";
	
	int size = strlen(string);
	
	if (size > 16)
		size = 16;

	while (size--){
		line[size] = string[size];
	}
	setCursor(0, row);
	lcd_print(line);
}

void writeCustomChar(uint8_t address, uint8_t map[]){ //fill Character Generator RAM with custom symbols
	address &= 0x7; //address must be 0 to 7
	lcd_command(0x40 | (address <<3)); //Set CGRAM address + address shifted left by 3 bits to start writing first byte
	for (int i = 0; i<8; i++){
		lcd_data(map[i]);	
	}
}

void loadCustomChars(){ //write all custom characters to the LCD module memory
	writeCustomChar(CHAR_1_5, CUSTOM_1_5);
	writeCustomChar(CHAR_2_5, CUSTOM_2_5);
	writeCustomChar(CHAR_3_5, CUSTOM_3_5);
	writeCustomChar(CHAR_4_5, CUSTOM_4_5);
	writeCustomChar(CHAR_5_5, CUSTOM_5_5);
}

void lcd_drawBar(int value){ //draws a bar using custom characters and spaces
	setCursor(0,1); //bar is placed in the bottom row

	if (value>80)
		value = 80;
	int quotient = value / 5;
	int modulo = value % 5;
	
	int i = 0;
	
	while (i<quotient){ //we write the required number of CHAR_5_5
		bar[i] = CHAR_5_5;
		i++;
	}
	
	if (modulo == 0) bar[i] = ' '; //then we either place a space
	else {
		bar[i] = CHAR_1_5 + modulo -1; //or the correct partial block
	}
	i++;
	while (i<16){ //and we fill the remainder with spaces
		bar[i] = ' ';
		i++;
	}
	lcd_print(bar); //finally we write to the LCD
}

void lcd_initialize(){  //initialize WH1602C LCD module in 4 bit mode, page 25

	HAL_Delay(50);  //wait >40 ms as per datasheet
	HAL_GPIO_WritePin(LCD_RS, GPIO_PIN_RESET);
	//LCD WritePIn is hard-wired low as per board schematic

	//Magic reset sequence
	lcd_write4(0x03);  //4-bit mode
	HAL_Delay(5);
	lcd_write4(0x03);
	HAL_Delay(5);
	lcd_write4(0x03);
	HAL_Delay(5);
	lcd_write4(0x02); //Set 4-bit mode
	lcd_write(0x28); //4bit, 2 lines, 5x8 font
	HAL_Delay(5);
	lcd_write(0x08); //display off;
	lcd_write(LCD_CLEAR_COMMAND); 			 //display clear;
	lcd_write(0x06); //entry mode set: increment
	HAL_GPIO_WritePin(LCD_BL_ON, GPIO_PIN_SET);  //enable backlight
	//_display_ctrl = DISPLAY_COMMAND|DISPLAY_ON|CURSOR_ON|BLINK_ON;
	_display_ctrl = DISPLAY_COMMAND|DISPLAY_ON;
	lcd_write(_display_ctrl); //set as above
	lcd_write(0x02); //go home
	HAL_Delay(2);
	loadCustomChars();

}

void lcd_backlight_ON(){
	HAL_GPIO_WritePin(LCD_BL_ON, GPIO_PIN_SET);
}

void lcd_backlight_OFF(){
	HAL_GPIO_WritePin(LCD_BL_ON, GPIO_PIN_RESET);
}
