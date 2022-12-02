volatile unsigned char *portF  =  (unsigned char *) 0x31;
volatile unsigned char *portDDRF = (unsigned char *) 0x30; 
volatile unsigned char *portB  = (unsigned char *) 0x25;
volatile unsigned char *portDDRB = (unsigned char *) 0x24;

void setup()
{
 *portDDRB |= 0x40;
 *portB &= 0xBF; 
}
  
void loop()
{
  
}
