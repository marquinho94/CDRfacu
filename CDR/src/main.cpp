
#include <Arduino.h>
#include<LiquidCrystal.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>



/* VARIABLES DE PROGRAMA */
//VARIABLES DE PRUEBA
uint32_t interrupcion = 0 ;
// TIMER 2 

 bool timer2_500ms = 0;

/* LIQUID CRYSTAL (necesita LiquidCrystal.h) */
//usa los nº de pines de arduino
const int rs = 13, en = 12, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
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
{
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
  TCCR2B |= ((1 << CS22) | (1 << CS21) | (1 << CS20));
  // enable timer compare interrupt solo contra el regsitro A 
  TIMSK2 |= (1 << OCIE2A);
  sei();//allow interrupts
}

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


#define DHTPIN 4    


#define DHTTYPE    DHT11     // DHT 11

DHT_Unified dht(DHTPIN, DHTTYPE);

//uint32_t delayMS;

void DHT11_setup() {
  Serial.begin(9600);
  // Initialize device.
  dht.begin();
  Serial.println(F("DHTxx Unified Sensor Example"));
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
  Serial.println(F("------------------------------------"));
  // Set delay between sensor readings based on sensor details.
  //delayMS = sensor.min_delay / 1000; -> USO EL TICK DE GENERADO POR TIMER2
}

void DHT11_lectura() {
  // Delay between measurements.
  //delay(delayMS);
  // Get temperature event and print its value.
  //-> USO EL TICK DE GENERADO POR TIMER2
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
  }
}

void GPIO_setup (void) //Congif de puertos
{
  //pinMode(LED_BUILTIN,OUTPUT); //led 
  //pinMode(PD1,OUTPUT);
  //pinMode(13, OUTPUT);
  //pinMode(12, OUTPUT);
  MCUCR |= (1 << PUD ); //habilito las pull up en general port b 
  DDRB &= 0b11111100;//((0 << DDB0) | (0 << DDB1)); //coniguracion i/o pines port b con 1-> salida  
  PORTB |= 0b00000011;//((1<< PORTB0) || (1<< PORTB1)  ); //(ACTIVO LAS PULLUP EN LAS ENTRADAS PB0 Y 1 )
}


void encoder_setup (void)
{
  cli();// DESHABILITO LAS INTERRUPCIONES DURANTE LA CONFIG 
  PCICR |= 0b00000001;//(1 << PCIE0);  //HABILITO LA INT POR CAMBIO EN LOS PINES PCINT  7 - 0 
  PCMSK0 |= 0b00000011;//((1 << PCINT0) | (1 << PCINT1)); //CONFIGURO LA MÁSCARA DE INTERRUPCION PARA LOS PINES PB0 Y PB1 
  sei();
}


ISR (PCINT0_vect)
{
  digitalWrite(13, HIGH);
  delay(1000); // Wait for 1000 millisecond(s)
  digitalWrite(13, LOW);

  delay(1000); // Wait for 1000 millisecond(s

  ++interrupcion;

}

void setup() {
  GPIO_setup();
  LCD_setup();
  timer2_setup();
  DHT11_setup();
  encoder_setup();
}

void loop() {
  if(timer2_500ms)
  {
    lcd.setCursor(0,1);
    lcd.print(interrupcion);
    DHT11_lectura();
  }
  /*else 
  {
    lcd.setCursor(0,1);
    lcd.print("cero            ");
  }*/
}

