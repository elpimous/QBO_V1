/*********************************************************************
 * \author Vincent FOUCAULT
 *********************************************************************/

#ifndef LCD_Graphics_h
#define LCD_Graphics_h

#include "LCDi2cW.h"

void DefineChars()

{
  // Define Custom Characters
  // batterie vide
  byte empty_batt[8] =
  {
  0b00100,0b11011,0b10001,0b10001,0b10001,0b10001,0b10001,0b10001
  };

  // batterie moitie
  byte half_batt[8] =
  {
  0b00100,0b11011,0b10001,0b10001,0b10001,0b11111,0b11111,0b11111
  };

  // batterie pleine
  byte full_batt[8] =
  {
  0b00100,0b11111,0b11111,0b11111,0b11111,0b11111,0b11111,0b11111
  };
  
  // send custom characters to the display
  lcd.load_custom_character(1,empty_batt);
  lcd.load_custom_character(2,half_batt);
  lcd.load_custom_character(3,full_batt);
  
};

#endif
