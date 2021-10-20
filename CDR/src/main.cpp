
#include <Arduino.h>
#include<LiquidCrystal.h>


/* VARIABLES DE PROGRAMA */

/* LIQUID CRYSTAL (necesita LiquidCrystal.h) */
//usa los nº de pines de arduino
const int rs = 13, en = 12, d4 = 11, d5 = 10, d6 = 9, d7 = 8;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void LCD_setup() {
  lcd.begin(16, 2);
  lcd.setCursor(0,0);
  lcd.println("SISTEMA DE RIEGO");
  lcd.setCursor(0,2);
  lcd.print("UNLaM - Taller de Electronica");
  /*for (int positionCounter = 0; positionCounter < 31; positionCounter++)
    {
      lcd.scrollDisplayLeft();
      delay(150);
    }
    */
}




void setup() {
  LCD_setup();
}

void loop() {
  // put your main code here, to run repeatedly:
}