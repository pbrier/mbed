/*
 * Software serial example
 * Peter Brier
 */

#include "mbed.h"
#include "USBSerial.h"
 
 
//Virtual serial port over USB
USBSerial usb;
DigitalOut led(P0_7);

#define BAUD 9600
#define t_bit (1000000/BAUD) // 9600 bps, 10 microseconds bit time

//DigitalInOut pin(P0_9);
DigitalInOut pin0(P0_0);
DigitalInOut pin1(P0_9);
DigitalInOut pin2(P0_18);
DigitalInOut pin3(P0_7);
DigitalInOut pin4(P0_12);
DigitalInOut pin5(P0_12);



// Software UART send. mark is LOW space is HIGH
void send(DigitalInOut  &pin, char c)
{
  pin.output();
  pin.mode(PullDown);
  pin = 0; // 1 start bit
  wait_us(t_bit);
  for(int bit=0; bit<8; bit++) // 8 bits
  { 
    pin = ( ( (unsigned char) c & (1<<bit) ) ? 1 : 0 );
    led = ( ( (unsigned char) c & (1<<bit) ) ? 1 : 0 );
    wait_us(t_bit);
  }
  pin = 1; // 1 stop bit
  wait_us(t_bit);
  pin.input();
}


// receive RS232, 9600bps, wait for some time to see if there is a start bit, or return 0
char receive(DigitalInOut &pin)
{
  char c = 0;
  pin.input();
  pin.mode(PullDown);
  for(int i=0; i<30 && pin == 1; i++); // wait for first falling edge
     wait_us(1);
  if ( pin == 1 ) return 0; // we have not seen an edge
  wait_us(1.5*t_bit); // wait for the start bit to finish and sample haveway the first bit
  for(int bit=0; bit<8; bit++) // sample 8 bits
  { 
    c |= ( pin == 1 ?  (1<<bit) : 0 );
    led = ( ( (unsigned char) c & (1<<bit) ) ? 1 : 0 );
    wait_us(t_bit);
  }
  wait_us(t_bit); // wait for the stopbit to complete
  return c;
}


/**
*** Main function
**/
int main(void) {
  static int i;
  char c;
  //  serial_init(&stdio_uart, P1_27, P1_26); // default is 9600bps
 
   // uart.baud(BAUD);
   // serial.printf("I am a virtual serial port\r\n");
  led = 1;
  while(1)
  { 
   /* while ( serial_readable( &stdio_uart ) ) 
    {
      usb.putc( serial_getc(&stdio_uart) );  
    } */ 
	  while ( usb.available () )
    {	 
      c = usb.getc();
      send(pin0, c);   
      send(pin1, c);   
      send(pin2, c);   
      send(pin3, c);   
      send(pin4, c);   
      send(pin5, c);    
      usb.putc( c );  
    }
    c = receive(pin0);
    if ( c ) usb.putc( c );  
	 
    led = ( (i++) & 256 ? 1 : 0 );
	//wait(0.001);
  }
}

