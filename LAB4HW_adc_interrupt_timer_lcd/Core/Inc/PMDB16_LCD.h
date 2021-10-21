#include "string.h"
#include "stdio.h"
#include "stm32f4xx_hal.h"

#define CHAR_1_5 0x01
#define CHAR_2_5 0x02
#define CHAR_3_5 0x03
#define CHAR_4_5 0x04
#define CHAR_5_5 0x05

// LCD Pins
#define LCD_RS GPIOB,GPIO_PIN_2
#define LCD_E GPIOB,GPIO_PIN_1
#define LCD_D4 GPIOB,GPIO_PIN_12
#define LCD_D5 GPIOB,GPIO_PIN_13
#define LCD_D6 GPIOB,GPIO_PIN_14
#define LCD_D7 GPIOB,GPIO_PIN_15
#define LCD_BL_ON GPIOA,GPIO_PIN_4

// LCD commands
#define LCD_CLEAR_COMMAND 0x01
#define DISPLAY_COMMAND 0x08
#define BLINK_ON 0x01
#define CURSOR_ON 0x02
#define DISPLAY_ON 0x04
#define LCD_SETDRAMADD 0x80

//Auxiliary functions
void lcd_enable();
void lcd_write4(uint8_t);
void lcd_write(uint8_t);
void lcd_command(uint8_t);
void lcd_clear();
void lcd_data(uint8_t);

//Move LCD cursor to write character in a certain location (row, column)
void setCursor(uint8_t, uint8_t);

//write a string form the current cursor location
void lcd_print(char*);

//write a new line, on the selected row
void lcd_println(char*, uint8_t);

//Auxiliary functions to allow the drawing of a bargraph
void writeCustomChar(uint8_t, uint8_t);
void loadCustomChars();

//draw a bargraph on the bottom row, 0 to 80
void lcd_drawBar(int);

//function ot be called to initialize the LCD controller
void lcd_initialize();

//enable/disable the LCD backlight
void lcd_backlight_ON();
void lcd_backlight_OFF();
