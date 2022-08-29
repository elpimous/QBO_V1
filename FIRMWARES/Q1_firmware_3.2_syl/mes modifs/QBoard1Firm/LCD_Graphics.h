/************* Qboard1 LCD graphic header ************************
 *
 * begin(col, line)   : Initialise the display of size cols * rows, clear the display and set the cursor to the top-left.
 * clear()             : Clear the display and return the cursor to the top-left.
 * setCursor(col, lin) : col (0-19),line (0-3)
 * write(data)         : print a single character
 * print(data, [base]) : print text ; base is optional (the base in which to print numbers: BIN, DEC, OCT or HEX)
 * createChar(num, data[]) : Create a custom character (Up to eight characters of 5x8 pixels are supported (numbered 0 to 7)
 * on()         : power on light
 * of()       : power off light
 * newLine()           : Move the cursor to the first column of the next row.
 * 
 * More..... https://github.com/andatche/arduino-lcd03
 *
 * \author Vincent FOUCAULT
 *****************************************************************/

#ifndef LCD_Graphics_h
#define LCD_Graphics_h

#define lcd

void Fall_alert_picture() // 5 Bytes only
{
  byte fall_1[8] = {
  0b00000,0b00000,0b00001,0b00011,0b10111,0b11110,0b11100,0b11110
  };
    byte fall_2[8] = {
  0b00000,0b00001,0b11110,0b11111,0b11111,0b01001,0b10001,0b00000
  };
    byte fall_3[8] = {
  0b00000,0b10000,0b01000,0b00100,0b10010,0b11011,0b10000,0b00001
  };
    byte fall_4[8] = {
  0b10000,0b11000,0b10100,0b10010,0b10001,0b01001,0b00101,0b00011
  };
    byte fall_5[8] = {
  0b00101,0b01001,0b10001,0b00001,0b00010,0b00100,0b01000,0b10000
  };
    lcd.backlight();
    lcd.setCursor(6,0);
    lcd.print("|");
    lcd.setCursor(6,1);
    lcd.print("|");
    lcd.setCursor(6,2);
    lcd.print("|");
    lcd.setCursor(6,3);
    lcd.print("|"); 
    lcd.setCursor(10,0);
    lcd.print("ALERT :");
    lcd.setCursor(7,1);
    lcd.print("-------------");
    lcd.setCursor(11,2);
    lcd.print("FALL");
    lcd.setCursor(1,1);
    lcd.createChar(0, fall_1);
    lcd.write(0);
    lcd.setCursor(2,1);
    lcd.createChar(1, fall_2);
    lcd.write(1);
    lcd.setCursor(3,1);
    lcd.createChar(2, fall_3);
    lcd.write(2);
    lcd.setCursor(2,2);
    lcd.createChar(3, fall_4);
    lcd.write(3);
    lcd.setCursor(3,2);
    lcd.createChar(4, fall_5);
    lcd.write(4);
};


void Wall_alert_picture()
{
  byte wall_1[8] = {
  0b00001,0b00010,0b00100,0b01001,0b11110,0b10010,0b10010,0b10010
  };
    byte wall_2[8] = {
  0b11000,0b01000,0b10000,0b01000,0b01010,0b01110,0b01111,0b11111
  };
    byte wall_3[8] = {
  0b00000,0b00000,0b00000,0b00000,0b00000,0b00000,0b11111,0b11111
  };
    byte wall_4[8] = {
  0b10010,0b10010,0b10010,0b10010,0b10010,0b10010,0b10010,0b11110
  };
    byte wall_5[8] = {
  0b11111,0b01111,0b01110,0b01010,0b01000,0b10000,0b00000,0b00000
  };
    byte wall_6[8] = {
  0b11111,0b11111,0b00000,0b00000,0b00000,0b00000,0b00000,0b00000
  };
    lcd.backlight();
    lcd.setCursor(6,0); // colonnes, lignes
    lcd.print("|");
    lcd.setCursor(6,1);
    lcd.print("|");
    lcd.setCursor(6,2);
    lcd.print("|");
    lcd.setCursor(6,3);
    lcd.print("|"); 
    lcd.setCursor(10,0);
    lcd.print("ALERT :");
    lcd.setCursor(7,1);
    lcd.print("-------------");
    lcd.setCursor(11,2);
    lcd.print("WALL");
    lcd.setCursor(1,1);
    lcd.createChar(0, wall_1);
    lcd.write(0);
    lcd.setCursor(2,1);
    lcd.createChar(1, wall_2);
    lcd.write(1);
    lcd.setCursor(3,1);
    lcd.createChar(2, wall_3);
    lcd.write(2);
    lcd.setCursor(1,2);
    lcd.createChar(3, wall_4);
    lcd.write(3);
    lcd.setCursor(2,2);
    lcd.createChar(4, wall_5);
    lcd.write(4);
    lcd.setCursor(3,2);
    lcd.createChar(5, wall_6);
    lcd.write(5);
};


void Imu_calibration_screen()
{
  lcd.clear();
  lcd.backlight();
  lcd.setCursor(2,0);
  lcd.print("IMU CALIBRATION.");
  lcd.setCursor(0,1);
  lcd.print("--------------------");
  lcd.setCursor(1,2);
  lcd.print("Do not move robot.");
};

#endif
