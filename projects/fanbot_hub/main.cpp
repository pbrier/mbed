/*

FanBot HUB

RS485 to 24 Serial HUB

Copyright (c) 2013 Peter Brier

This file is part of the FanBot project.

Fanbot is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "mbed.h"
#include "USBSerial.h"
#include "pinmap.h"
#include "serial_api.h"

// externals from serial_api
extern int stdio_uart_inited;
extern serial_t stdio_uart;
 
 
//Virtual serial port over USB
//USBSerial usb;
//DigitalOut led(P0_7);
DigitalIn button(P0_1);

DigitalOut led_a(P1_28);
DigitalOut led_b(P1_31);

#define BAUD 9600
#define t_bit (1000000/BAUD) // 9600 bps, 10 microseconds bit time

//DigitalInOut pin(P0_9);
DigitalInOut pin0(P0_8);
DigitalInOut pin1(P0_9);
DigitalInOut pin2(P0_18);
DigitalInOut pin3(P0_7);
DigitalInOut pin4(P0_12);
DigitalInOut pin5(P0_12);


// Hub pin numbers for 24 ports
static PinName hub_pin[] = {
  P0_16, //D1
  P0_15, //D2 *SWDIO (select pin function 1 for GPIO)
  P1_22, //D3
  P0_14, //D4
  P0_13, //D5
  P0_12, //D6
  P0_11, //D7
  P1_29, //D8
  P0_22, //D9
  P0_10, //D10 *SWDCLK (select pin function 1 for GPIO)
  P0_9,  //D11 
  P0_8,  //D12
  P0_23, //D13
  P1_15, //D14
  P0_18, //D15
  P0_19, //D16
  P1_25, //D17
  P0_20, //D18
  P0_2,  //D19
  P1_26, //D20
  P1_27, //D21
  P1_23, //D22
  P1_24, //D23
  P0_7,  //D24
};

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
    led_a = ( ( (unsigned char) c & (1<<bit) ) ? 1 : 0 );
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
    led_a = ( ( (unsigned char) c & (1<<bit) ) ? 1 : 0 );
    wait_us(t_bit);
  }
  wait_us(t_bit); // wait for the stopbit to complete
  return c;
}

class Watchdog {
public:
    void kick(float s) {
     __disable_irq();
        LPC_WWDT->CLKSEL = 0x1;                // Set CLK src to PCLK
        uint32_t clk = 6000/2;    // WD has a fixed /4 prescaler, PCLK default is /4 
        LPC_WWDT->TC = 0xFFF; // s * (float)clk;         
        LPC_WWDT->MOD = 0x3;                   // Enabled and Reset        
        kick();
         __enable_irq();
    }
    
    void kick() {
        LPC_WWDT->FEED = 0xAA;
        LPC_WWDT->FEED = 0x55;
    }
};
 
Watchdog w;
  
 /**
 *** iotest()
 *** Toggle all outputs
 **/
 void iotest()
 {
  int i;
  w.kick(2.5);
  wait(1);

  while(1)
  {
    i++;
    led_a = 1;
    led_b = 0;
    for(i=0; i<(sizeof(hub_pin)/sizeof(hub_pin[0])); i++)
    {
      DigitalOut hub(hub_pin[i]);
      pin_function(P0_15,1);
      pin_function(P0_10,1);
      hub = 1;
    }
    wait(0.1);
    led_a = 0;
    led_b = (button ? 0 : 1);
    for(i=0; i<(sizeof(hub_pin)/sizeof(hub_pin[0])); i++)
    {
      DigitalOut hub(hub_pin[i]);
      pin_function(P0_15,1);
      pin_function(P0_10,1);
      hub = 0;
    }
    wait(0.1);
  }
}


void rs485_test()
{
  char buf[128];
  int n=0;
  //DigitalOut de(P0_17);
  DigitalOut de(NC);
  serial_init( &stdio_uart, P1_13, P1_14); // pins: tx, rx
  LPC_USART->RS485CTRL |= (1<<4) | (1<<5); // Enable bit4: Automatic Direction control
  pin_function(P0_17,1); // RTS
  memset(buf,0,sizeof(buf));
  printf("Hello World!\n");
  while(1)
  {    
    de = 0;
    while ( serial_readable( &stdio_uart ) && n >= 0 )
    {     
     led_a = 1;
     buf[n] = serial_getc(&stdio_uart);
     de = 1;
     serial_putc(&stdio_uart, buf[n]);
     de = 0;
     if ( buf[n] == '\r' ) 
       n = -1;
     else
       n++;       
    }
    if ( n < 0 )
    {     
      led_a = 0;
      n = 0;
      de = 1; // driver enable
      printf("Hello World!\n");
      printf(buf);
      memset(buf,0,sizeof(buf));
    }
  }
}



/**
*** Main function
**/
int main(void) {
  static int i;
  char c;
 //iotest();
  rs485_test();
  //led = 1;


  /*
  while(1)
  { 
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
    c = receive(pin1);
    if ( c ) usb.putc( c );  
	 
    led = ( (i++) & 256 ? 1 : 0 );
  }
  */
}

