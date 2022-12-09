#include <Stepper.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include <RealTimeClockDS1307.h>
#include <dht.h>

//GPIO Registers
volatile unsigned char *portF  =  (unsigned char *) 0x31;
volatile unsigned char *portDDRF = (unsigned char *) 0x30; 
volatile unsigned char *portB  = (unsigned char *) 0x25;
volatile unsigned char *portDDRB = (unsigned char *) 0x24;
volatile unsigned char *portC = (unsigned char *) 0x28;
volatile unsigned char *portDDRC = (unsigned char *) 0x27;
//ADC Registers
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;
//UART Registers
volatile unsigned char *myUCSR0A  = (unsigned char *) 0xC0;
volatile unsigned char *myUCSR0B  = (unsigned char *) 0xC1;
volatile unsigned char *myUCSR0C  = (unsigned char *) 0xC2;
volatile unsigned int  *myUBRR0   = (unsigned int *) 0xC4;
volatile unsigned char *myUDR0    = (unsigned char *) 0xC6;

const int stepsPerRevolution = 90;
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);

const int rs = 22, en = 23, d4 = 24, d5 = 25, d6 = 26, d7 = 27;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup()
{
  Serial.begin(9600);
  *portDDRB |= 0x40;
  *portB &= 0xBF;
  *portDDRF |= 0x01;
  *portF &= 0xFE;
  adc_setup();
  myStepper.setSpeed(5);
  pinMode(7, OUTPUT); // button output
  lcd.begin(16, 2);
  pinMode(A1, OUTPUT) // RTC SDA
  pinMode(A2, OUTPUT) // RTC SDL
  pinMode(4, OUTPUT) // DHT signal
  
  rtc.begin();
  rtc.setDOW(FRIDAY);
  rtc.startClock();
  rtc.setDate(9, 12, 2022);
  
  //LED setup
  *portDDRC |= 0b00111100; //digital pins 32-35 output
  
  tempThreshold = 25;
}
  
void loop()
{
  //Start button
  attachInterrupt(digitalPinToInterrupt(2), start, RISING);
  // Disabled state
  *portC &= ~(1<<4); //turn off green
  *portC |= (1<<5); //turn on yellow
  //Reset button
  attachInterrupt(digitalPinToInterrupt(3), reset, RISING);
  
  if(*portC&(0<<5) == 1) // if not in disabled state
  {
    //temperature and humidity to LCD
    if(*portC&(1<<4) == 1) // in idle state
    {
      monitor_water_levels();
      if(DHT.temperature > tempThreshold) // change to running state
      {
        *portC &= ~(1<<4); // turn off green
        *portC |= (1<<2); //turn on blue
        //serial.println change in state with rtc
      }
      else // change to idle state
      {
        *portC &= ~(1<<2); //turn off blue
        *portC |= (1<<4); // turn on green
        //serial.println change in state with rtc
      }
    }
    //print DHT to LCD every min
    //every minute
    lcd.print("Temperature: ", DHT.temperature);
    lcd.setCursor(0, 1);
    lcd.print("Humidity: ", DHT.humidity);
    //stop button
  }
}

void start() //start button interrupt function
{
    *portC &= ~(1<<5); //turn off yellow
    *portC |= (1<<4); //turn on green
    //Serial.println change in state with rtc
}

void reset() //reset button interrupt function
{
  *portC &= ~(1<<3); //turn off red
  *portC |= (1<<2); //turn on blue
  lcd.clear();
  //Serial.println change in state with rtc
}

void monitor_water_levels()
{
  if(digitalRead(7) == HIGH)
  {
    myStepper.step(stepsPerRevolution);
    Serial.println("Step Motor Rotated");
  }
  *portB |= 0b01000000;
  unsigned int waterLevel = adc_read(0);
  
  if(waterLevel < 300)
  {
    lcd.clear();
    lcd.print("Error: Water Level");
    lcd.setCursor("Too Low");
    *portC &= ~(1<<4); // turn off green
    *portC |= (1<<3); // turn on red
  }
}

void adc_setup()
{
   *my_ADCSRA |= 0b10000000;
   *my_ADCSRA &= 0b11010000;
   *my_ADCSRA &= 0b11110000;
   *myADMUX |= 0b01000000;
   *myADMUX &= 0b01000000;
}

unsigned int adc_read(unsigned char channelNum)
{
  *my_ADMUX  &= 0b11100000;
  *my_ADCSRB &= 0b11011111;
  *my_ADMUX  += channelNum;
  *my_ADCSRA |= 0x40;
  while((*my_ADCSRA & 0x40) != 0);
  return *my_ADC_DATA;
}
