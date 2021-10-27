
#include <Arduino.h>
#include<LiquidCrystal.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <RTClib.h>
#include <Wire.h>

/* VARIABLES DE PROGRAMA */
bool timer2_flag250 = false;
typedef enum  {INICIO, Sensor_Suelo, Riego, Sensor_DHT11, Ventilacion, Sensor_LDR} estados_automatico;
estados_automatico estados_CDR_Automatico;
uint16_t Humedad_suelo = 512;
bool Riego_encender = 0; 
float temp_ref = 0;
float humedad_ref = 0;

bool Ventilacion_encender = 0;

//VARIABLES DE PRUEBA
uint32_t interrupcion = 0 ;


// TIMER 2 

 bool timer2_500ms = 0;

/* LIQUID CRYSTAL (necesita LiquidCrystal.h) */
//usa los nº de pines de arduino
const int rs = 11, en = 12, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
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
  static uint8_t timer2_contador_250 = 0;
  

  ++timer2_contador; 
  ++timer2_contador_250;
  if(timer2_contador_250 >= 25)
  {
    timer2_flag250 = true;
    timer2_contador_250 =0;
  }
  if(timer2_contador >= 50)
  {
    timer2_500ms ^= 1; // operación XOR con 1 para togglear el flag
    timer2_contador = 0;
  }
}


#define DHTPIN 10    


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
  pinMode(A3,OUTPUT);//bomba
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(2, OUTPUT);
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

  ++interrupcion;

}


// Date and time functions using a DS1307 RTC connected via I2C and Wire lib



//VARIABLES RTCC 
RTC_DS1307 rtc;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};



void rtcC_setup () 
{
  //Serial.begin(9600); Iniciado en el setup dht11

  rtc.begin(); //Inicializamos el RTC
  Serial.println("Estableciendo Hora y fecha...");
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  Serial.println("DS1307 actualizado con la hora y fecha que se compilo este programa:");
  Serial.print("Fecha = ");
  Serial.print(__DATE__);
  Serial.print("  Hora = ");
  Serial.println(__TIME__);
}
void rtcc_lectura_prueba () {
    DateTime now = rtc.now();

    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" (");
    Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    Serial.print(") ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();

    Serial.print(" since midnight 1/1/1970 = ");
    Serial.print(now.unixtime());
    Serial.print("s = ");
    Serial.print(now.unixtime() / 86400L);
    Serial.println("d");

    // calculate a date which is 7 days, 12 hours, 30 minutes, and 6 seconds into the future
    DateTime future (now + TimeSpan(7,12,30,6));

    Serial.print(" now + 7d + 12h + 30m + 6s: ");
    Serial.print(future.year(), DEC);
    Serial.print('/');
    Serial.print(future.month(), DEC);
    Serial.print('/');
    Serial.print(future.day(), DEC);
    Serial.print(' ');
    Serial.print(future.hour(), DEC);
    Serial.print(':');
    Serial.print(future.minute(), DEC);
    Serial.print(':');
    Serial.print(future.second(), DEC);
    Serial.println();

    Serial.println();
    //delay(3000);
}

void ADC_setup(void)
{
 // analogReadResolution(10);
}

uint16_t ADC_lectura(void)
{
  //if(bitRead(ADCSRA,ADIF)) //miro el flag de conversión finalizada 
  //A RETOMAR MÁS ADELANTE CON LA COFIGURACIÓN COMPLETA DE LOS REGISTROS
  

 return analogRead(A0); //sensor de humedad de suelo
  

}


void automatico_MEF (void)
{
  sensors_event_t event; //lo uso para levantar los valroes del sensor, si bien son float las funciones bla bla bla bla  
  static uint16_t suelo_ADC[11];
  static float DHT11_temp_hum [5][2]; //tomo menos muestras por el tiempo de demora
  static uint8_t muestras_suelo = 0; 
  static uint8_t muestras_DHT11 = 0;
  uint16_t x = map(suelo_ADC[10], 0, 1030, 0, 100); //MEGA PARCHE
  Serial.println (estados_CDR_Automatico);
  switch ( estados_CDR_Automatico )
  {
    case INICIO:
      lcd.clear();
      lcd.setCursor(1,0);
      lcd.print("CDR AUTOMATICO");
      lcd.setCursor(0,1);
      lcd.print("T:");
      lcd.print(DHT11_temp_hum[4][0],1);
      lcd.print("C");
      lcd.setCursor(9,1);
      lcd.print("H:");
      lcd.print(x);//MEGA PARCHE
      lcd.setCursor(14,1);
      lcd.print("%");
      estados_CDR_Automatico = Sensor_Suelo;
      // sentencias de prueba //


      break;
    case  Sensor_Suelo:
      if(muestras_suelo < 10)
      {  
        suelo_ADC[muestras_suelo] = analogRead(A0);
        ++muestras_suelo;
        estados_CDR_Automatico = Sensor_DHT11;
      }
      else 
      { 
        for(int i=0; i < 10 ; i ++ )
        {
          suelo_ADC[10] += suelo_ADC[i];
        }
        suelo_ADC[10] = (suelo_ADC[10]/10);//guardo el promedio de los 10 valores en la ultima posición del array
        estados_CDR_Automatico = Riego;
      }
      break;

    case Riego:

      if(suelo_ADC[10] < Humedad_suelo)
      {
        Riego_encender = true ;
      }
      else if (suelo_ADC[10] >= (Humedad_suelo * 1.05) )
      {
        Riego_encender = false ;
      }
    estados_CDR_Automatico = Sensor_DHT11 ;

    case Sensor_DHT11:

      if(muestras_DHT11 < 4 && timer2_flag250 == true)
      { dht.temperature().getEvent(&event);
        DHT11_temp_hum[muestras_DHT11][0] = event.temperature ;// levanto el dato de la temp del dht11
        dht.humidity().getEvent(&event);
        DHT11_temp_hum[muestras_DHT11][1] = event.relative_humidity;// levanto el dato de la HUMEDAD del dht11
        ++muestras_DHT11;
        timer2_flag250 = false; 
        estados_CDR_Automatico = Sensor_LDR; 
        
      }
      else 
      { 
        DHT11_temp_hum[4][0] = 0;//si no limpio el dato antes de hacer el promedio se sigue ncrementando 
        DHT11_temp_hum[4][1] = 0;

        for(int i=0; i < 4 ; i ++ )
        {
          DHT11_temp_hum[4][0] += DHT11_temp_hum[i][0];
          DHT11_temp_hum[4][1] += DHT11_temp_hum[i][1];
        }
        DHT11_temp_hum[4][0] = (DHT11_temp_hum[4][0] / 4 );//guardo el promedio 
        DHT11_temp_hum[4][1] = (DHT11_temp_hum[4][1] / 4);
        estados_CDR_Automatico = Ventilacion;
        Serial.print("temperatura");
        Serial.println(DHT11_temp_hum[4][0]);
        Serial.print("humedad");
        Serial.println(DHT11_temp_hum[4][1]);
      }
      break;
    case Sensor_LDR://debería leer el valor de ADC sobre el LDR pero no lo estoy usando de momento. 
      if (muestras_suelo >= 10)
      {
        muestras_suelo = 0;
      }
      if(muestras_DHT11 >= 4 )
      {
        muestras_DHT11=0;
      }
      estados_CDR_Automatico = INICIO;
      break;
    case Ventilacion:
      if (DHT11_temp_hum[4][0] >= temp_ref)
      {
        Ventilacion_encender = true;
      }
      if (DHT11_temp_hum[4][0] <= temp_ref * 0.95)
      {
        Ventilacion_encender = false;
      }
      estados_CDR_Automatico = INICIO;
      break;
    default:
      lcd.clear();
      lcd.setCursor(8,1);
      lcd.print("ERROR");
      break;
  }
}

typedef enum {RESET , ON , OFF} estados_ACTUADORES;
estados_ACTUADORES estados_bomba = RESET;

void bomba_MEF (void)
{
  switch (estados_bomba)
  {
    case RESET:
    {
      if (Riego_encender)
      {
        estados_bomba = ON;
      }
      else estados_bomba = OFF;
    
      break;
    }
    case ON:
    {
      bitSet(PORTC,3);//bomba con bit set es más eficiente 
      if (Riego_encender)
      {
        estados_bomba = ON;
      }
      else estados_bomba = OFF;
      break;
    }
    case OFF:
    {
      bitClear(PORTC,3);//bomba con bit set es más eficiente 
      if (Riego_encender)
      {
        estados_bomba = ON;
      }
      else estados_bomba = OFF;
      break;
    }
    default:
    {
      lcd.print("ERROR");
    
      break;
    }
  }
} 

void setup() 
{
  GPIO_setup();
  LCD_setup();
  timer2_setup();
  DHT11_setup();
  encoder_setup();
  rtcC_setup();
  //ADC_setup();
}

void loop() {

  estados_CDR_Automatico = INICIO;
  lcd.clear(); 
  while (true)
  {
    automatico_MEF();
    bomba_MEF();
  }
  
  
/*
  lcd.setCursor(0,1);
  lcd.flush();
    uint16_t a = 0;
    a = ADC_lectura();
    lcd.print("heyyy");
    lcd.print(a); 

  if(a <= 512) 
  {
    digitalWrite(A3,HIGH);
  }
  else digitalWrite(A3,LOW); */
  /*if(timer2_500ms)
  {
    
    Serial.println(interrupcion);
    DHT11_lectura();
    if(bitRead(interrupcion,0) == 0)
    {
      digitalWrite(2,HIGH);
      rtcc_lectura_prueba();
    }
    else digitalWrite(2,LOW);
    
  }
  */
  /*else 
  {
    lcd.setCursor(0,1);
    lcd.print("cero            ");
  }*/
}

