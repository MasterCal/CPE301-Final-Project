//GPIO Registers
volatile unsigned char *portF  =  (unsigned char *) 0x31;
volatile unsigned char *portDDRF = (unsigned char *) 0x30; 
volatile unsigned char *portB  = (unsigned char *) 0x25;
volatile unsigned char *portDDRB = (unsigned char *) 0x24;
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

void setup()
{
  Serial.begin(9600);
  *portDDRB |= 0x40;
  *portB &= 0xBF;
  *portDDRF |= 0x01;
  *portF &= 0xFE;
  adc_setup();
}
  
void loop()
{
  *portB |= 0b01000000;
  unsigned int waterLevel = adc_read(0);
  
  if(waterLevel < 300)
  {
    Serial.print("Water Level Too Low! Reading %d", waterLevel);
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
