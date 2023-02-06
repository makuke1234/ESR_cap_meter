#include "display.hpp"

LiquidCrystal disp::lcd(LCD_RS, LCD_RW, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

void disp::init()
{
	lcd.begin(LCD_WIDTH, LCD_HEIGHT);
}
