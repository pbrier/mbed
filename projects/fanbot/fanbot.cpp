/*

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
#include "USBHID.h"
#include "serial_api.h"
#include "pwmout_api.h"
#include "pinmap.h"
#include "IAP.h"

// externals 
extern int stdio_uart_inited;
extern serial_t stdio_uart;

//We declare a USBHID device
USBHID hid(
 64,  // out 
 64,  // in
 0x1234, // 0x1FC9, // vid  (NXP)  
 0x6, // 0x8066 , // pid (Fanbot)
 0x0001, // p_release
 false // connect
);

// Globals
HID_REPORT send_report, recv_report; //This report will contain data to be sent and received
PwmOut servo1(P0_19);
PwmOut servo2(P0_18);
Ticker tic; // msec ticker
unsigned char pos1=0, pos2=0;
unsigned char loop_count=0, prog_step=0;

// fanbot IO
BusOut leds(P1_19, P1_25, P0_8, P0_9, P0_22, P0_13, P0_14); // Leds
DigitalIn  button(P0_1);   // Button
DigitalIn  uart_rx(P1_26); // rx pin as input, so we can disable the pull-up before enumeration


// IR led and receiver
// 1st 5 protos had IR on P1.16: DigitalIn ir_rx(P1_16);
DigitalIn ir_rx(P1_14);
DigitalOut ir_tx(P1_13);
PwmOut ir_pwm(P1_13);


// IAP and EEPROM defines
#define MEM_SIZE            256
#define PROGRAM_ADDRESS       0
#define NAME_ADDRESS   MEM_SIZE
#define NAME_SIZE            32
IAP iap;

#define FAN_STEPS 20 // nr of program steps
unsigned short int program[128]; // buffer for EEPROM, 256 bytes, need to be word aligned

// Globals
volatile int read_res=0, send_res=0, counter=0;
int serial_nr = 0; // the robot serial# (from LPC device)
char name_string[NAME_SIZE]; // the robot name (read from EEPROM)

// Prototypes
void set_servo(char n, int val);

class Watchdog {
public:
    void kick(float s) {
     __disable_irq();
        LPC_WWDT->CLKSEL = 0x1;                // Set CLK src to PCLK
        //uint32_t clk = 6000/2;    // WD has a fixed /4 prescaler, PCLK default is /4 
        LPC_WWDT->TC = 0xFF; // s * (float)clk;         
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


// USB send Tick handler, send a report (every 100 msec)
// The report format is:
// 0: 0
// 1: status code (0=OK)
// 2: Counter (0..255)
// 3: Button state (0=not pressed)
// 4: Led state (0..127)
// 5: Servo 1 position (0..255)
// 6: Servo 2 position (0..255)
// 7: n.a.
// 8..11: Serial #
// 12 .. 44: Name (content of memory 1)
// 48: Memory index
// 49: Memory content (led state)
// 50: Memory content (servo state)

void tic_handler() 
{
  static int idx = 0; // program memory index
  
  send_report.data[2] = counter++;
  send_report.data[3] = (button == 0 ? 1 : 0); 
  send_report.data[4] = leds;
  send_report.data[5] = pos1; 
  send_report.data[6] = pos2;
  send_report.data[48] = idx;
  send_report.data[49] = program[idx] & 0xFF;
  send_report.data[50] = program[idx] >> 8;
  
  if ( hid.configured() )
    send_res = hid.send(&send_report);	
 
  if ( ++idx >= FAN_STEPS ) idx = 0;
 
  // The program sequencer:
  if ( counter % 10 == 0 ) // every 500 msec
  {  
    if ( prog_step < 0 || prog_step > FAN_STEPS ) prog_step = 0;
    if ( loop_count > 0 )
    {
      leds = program[prog_step] & 0xFF;
      set_servo('A', program[prog_step]>>8);
      prog_step++;
      if ( prog_step >= FAN_STEPS || program[prog_step] == 0 )
      {
        loop_count--;
        prog_step = 0;
      }    
    }
  }    
}

// Software UART send. mark is LOW space is HIGH
#define BAUD 9600
#define t_bit (1000000/BAUD) // 9600 bps, ~10 microseconds bit time
void send(DigitalOut &p, const char c)
{
  p = 1; // 1 start bit
  wait_us(t_bit);
  for(int bit=0; bit<8; bit++) // 8 bits
  { 
    p = ( ( (unsigned char) c & (1<<bit) ) ? 0 : 1 );
    wait_us(t_bit);
  }
  p = 0; // 1 stop bit
  wait_us(t_bit);
}

// Transmit a string in serial mode
void tx(DigitalOut &p, const char *s)
{
  wait_us(2000);
  while ( *s ) 
  {
    send(p, *s++);
    wait_us(1000);
  }
}

// send a single nibble
void tx_nibble(DigitalOut &p, const int n)
{
  wait_us(2000);
  send(p, "0123456789ABCDEF"[n & 0xF] );
  wait_us(2000);
}

// Play the sequence, set the loop_count, the actual playing is performed in the 1msec Ticker
// If a sequence was already playing we just set the loop_count, so the sequence will continue without stopping
void play(int n)
{
  if ( loop_count == 0)
    loop_count = n;
}

//  Stop playing the sequence, reset the program step
void stop()
{
  loop_count = 0;
  prog_step = 0;
}

// Set servo position (clamped to range for Fanbot)
void inline set_servo(char n, int val)
{
  PwmOut *p = (n == 'A' ? &servo1 : &servo2 );
  int a = (val ? 1300 : 0);  
  p->pulsewidth_us(a + val);
}

// get value, wait max 30msec
int get_val()
{
  for(int i=0; i<30 && !serial_readable( &stdio_uart );i++ )
    wait(0.001);
  return serial_getc(&stdio_uart); 
}

/**
*** run serial communication mode
*** Commands:
*** 'p' Play sequence
*** 's' Stop
*** 'L' [val]  select LED value (0..127)
*** 'l' [val]  select LED number (0..7)
*** 'A' [val]  arm position [servo 1] (0 is off, 1=left, 127=center, 255=right)
*** 'B' [val]  arm position [servo 2] (same as servo 1)
*** 'n' Report serial nr (send as 8 digit HEX  number)
*** 'N' Report name (send ASCII)
***
**/
void do_serial()
{
  volatile static int i;
  DigitalOut usb_conn(P0_6); // we control the USB connect output now
  pwmout_t _pwm;  // dummy PWM struct to re-init the PWM ports (reselects the pin functions after serial init)
   
  serial_init( &stdio_uart, P1_27, P1_26); // pins: tx, rx
  pin_mode( P1_26, PullNone); // disable weak pullup for RX
  // pin_mode( P0_6, PullNone); // disable weak pullup for USB_CON
  
  pwmout_init(&_pwm, P0_18);
  pwmout_init(&_pwm, P0_19);
  leds = 127;
  
  while( 1 )
  {

    if ( button == 0 )
    {
      leds = 1;
      play(1);
    }
    if ( serial_readable( &stdio_uart ) )
    {
      char c = 0;
      while ( serial_readable( &stdio_uart ) )
        c = serial_getc(&stdio_uart); 
      switch( c )
      {
        case 0: 
          break;
        case 'n':  // send serialnr as hex value, values are echoed back: so clean the rx FIFO after each TX
          for(int b=28; b>=0; b-=4) 
            tx_nibble(usb_conn, serial_nr >> b);
          wait(0.01);
          serial_clear(&stdio_uart);
          break;
        case 'N': // send name as ASCII
          for(int i=0; i<NAME_SIZE; i++) 
            send(usb_conn, name_string[i]);
          wait(0.01);
          serial_clear(&stdio_uart);
          break;
        case 'p': // play sequence (10 times, or until stopped)
          play(10);
          break;
        case 's': // stop
          stop();
          break;
        case 'L': // led value
          stop();
          leds = get_val();
          break;
        case 'l': // led number
          stop();
          leds = get_val() - '0';
          break;
        case 'A': // first servo
        case 'B': // second servo
          stop();
          set_servo( c, get_val() );
          break;
        default:
          break;
      }
	  }
  }
} // do_serial()


// Read name from EEPROM, and store it in to the global vars
void read_name()
{
  iap.read_eeprom( (char*)NAME_ADDRESS, (char *)program, MEM_SIZE );
  memcpy(name_string, program, sizeof(name_string) );
  memcpy(&send_report.data[12], name_string, sizeof(name_string) );
}

void  check_program()
{


}


/**
*** Main program
*** work in 3 modes: 
***   1) HID mode (tried first): receive commands from USB
***   2) Serial mode (if USB/HID handshake fails within a certain time): Receive commands from serial port
***   3) Stand alone mode: if the serial loopback fails it is not connected to a hub: do stand-alone operation.
***      Any serial character received causes serial mode to be activated
*** In addition the chip has a USB/MSD bootloader that can be activated by holding the program swith when the chip
*** comes out of reset. 
**/
int main(void) {
 
  // Read serial# from device eeprom
  serial_nr = iap.read_serial();
  
  // Fill send_report
  send_report.length = 64;  // Note: 1st byte is index, next 64 is data, so buffer needs to be 65 elements
  memset(send_report.data, 0, sizeof(send_report.data) );
  memcpy(&send_report.data[8], &serial_nr, sizeof(serial_nr) );
  
  // Read name from EEPROM and store it in name string and send report
  read_name();
  iap.read_eeprom( (char*)PROGRAM_ADDRESS, (char *)program, MEM_SIZE );
  check_program(); // see if there is a default program loaded
   
  // enable IR transmitter
  ir_pwm.period(1.0/38000.0);
  ir_pwm.pulsewidth(1.0/72000.0);

	button.mode(PullUp);
	uart_rx.mode(PullNone); // disable pull-up, so we can enumerate 


  // Set the servo update period
  servo1.period_ms(15);
  servo2.period_ms(15);
	set_servo('A', 0);
  set_servo('B', 0);
	
  // Say 'hello'
	for(int l=0, p=1; l<7;l++, p|= (1<<l))
	{
	  leds = p;
	  wait(0.01);
	}

  // Connect to USB
	hid.connect(false); // do not block
	for(int i=0; i<7 && !hid.configured(); i++)
	{
	  wait(0.1);
	  leds = 1<<i;
	  wait(0.1);
	}	 
  tic.attach(tic_handler, 0.05);
	
  if ( !hid.configured() )
    do_serial();
  
  while (1)
	{
	  if ( button == 0 ) // button pressed == LOW 
	    play(1);

    if (hid.readNB(&recv_report)) 
	  {
	    read_res++;
      switch ( recv_report.data[0] ) 
      {
        case 0: // set outputs [A] [B] [L]
          stop();
          leds = recv_report.data[1];
          set_servo('A', recv_report.data[2]);
          set_servo('B', recv_report.data[3]);
          break;
        case 1: // Save progam data to eeprom 
          for(int i=0; i<FAN_STEPS; i++)
            program[i] = ((unsigned short int)recv_report.data[1 + 2*i]) | ((unsigned short int)recv_report.data[2 + 2*i] << (unsigned short int)8);
          send_report.data[1] = iap.write_eeprom( (char*)program, (char*)PROGRAM_ADDRESS, MEM_SIZE );
          break;
        case 2: // Run program, n-times
          if (  recv_report.data[1] == 0 )  
            recv_report.data[1] = 1;
          play( recv_report.data[1] );          
          break;
        case 4: // Stop
         stop();        
          break;           
        case 3: // Save name to eeprom 
          memcpy(&program, &recv_report.data[1], NAME_SIZE);
          send_report.data[1] = iap.write_eeprom( (char*)program, (char*)NAME_ADDRESS, MEM_SIZE );
          read_name();
          break;
        default: // unkown command
          break;
        }			
    }
	 // printf("send: %d receive: %d counter: %d\n", send_res, read_res, counter);
	  wait_ms(1);
	}
} // main
