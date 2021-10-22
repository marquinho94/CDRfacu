
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
void timer2_setup(void)
[
  cli();
  //set timer2 
  TCCR2A = 0;// 
  TCCR2B = 0;// con ambos en cero no se ultilizan pines de salida (pwm)
  TCNT2 = 0;//initialize counter value to 0
  // ajuste del regstro de comparación para una frc de interrupción de 100hz
  OCR2A = 155;// = (16*10^6) / (100*1024) - 1 (must be <256)
  // turn on CTC mode >> cuando llega al valo de comparación se resetea
  TCCR2A |= (1 << WGM21);
  // Set CS11 bit for 1024 prescaler
  TCCR2B |= ((1 << CS22) | (1 << CS21) | (1 << CS20))
  // enable timer compare interrupt solo contra el regsitro A 
  TIMSK2 |= (1 << OCIE2A);
  sei();//allow interrupts
]

/*interrupcion timer 2 cada 10ms (100hz) 
Nesecita declarar variable global bit timer2_500ms=0 */

ISR(TIMER2_COMPA_vect)
{
  static uint8_t timer2_contador = 0; //mantiene el valor de la ejecución anterior 
  ++timer2_contador; 
  if(timer2_contador >= 50)
  {
    timer2_500ms ^= 1; // operación XOR con 1 para togglear el flag
    timer2_contador = 0;
  }
}


void setup() {
  LCD_setup();
}

void loop() {
  // put your main code here, to run repeatedly:
}