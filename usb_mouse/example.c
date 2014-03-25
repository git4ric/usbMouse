/* Mouse example with debug channel, for Teensy USB Development Board
 * http://www.pjrc.com/teensy/usb_mouse.html
 * Copyright (c) 2009 PJRC.COM, LLC
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
//#include "usb_mouse.h"

#include "usb_serial.h"
#include "sampling.h"

#define LED_CONFIG	(DDRD |= (1<<6))
#define LED_ON		(PORTD &= ~(1<<6))
#define LED_OFF		(PORTD |= (1<<6))
#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))
#define CPU_16MHz	0x00
#define CPU_125kHz	0x07
#define HEX(n) (((n) < 10) ? ((n) + '0') : ((n) + 'A' - 10))
#define ZERO_G_X	300
#define ZERO_G_Y	475

#define ACC_ERROR_SENSITIVITY 4
#define MAX_ACC_CHANGE 60
#define MULT_FACTOR_OF_DELTA 1

int8_t circle[];

signed long valX;
signed long valY;
int countx,county;
signed long accelerationx[2], accelerationy[2];
int direction;
signed long velocityx[2], velocityy[2];
signed long positionX[2];
signed long positionY[2];
signed long positionZ[2];

signed long deltaX;
signed long deltaY;


void movement_end_check(void)   
{
  if (accelerationx[1]==0)   //we count the number of acceleration samples that equals 0 
    { countx++;}
  else { countx =0;}
  
  if (countx>=25){          //if this number exceeds 25, we can assume that velocity is 0 
	   velocityx[1]=0;
	   velocityx[0]=0;
   }
   
   if (accelerationy[1]==0)   //we do the same for the Y axis
   { county++;}
  else { county =0;}
  
  if (county>=25){ 
	   velocityy[1]=0;
	   velocityy[0]=0;
   } 
}

// Get an average 1024 values of x and y axis
void Calibrate(void) 
{
  unsigned int count1;
  count1 = 0;
  do{  
	  adc_start(ADC_MUX_PIN_F0, ADC_REF_POWER); //Read X // Y
	  valX = valX + adc_read();  
	  adc_start(ADC_MUX_PIN_F1, ADC_REF_POWER); //Read Y // Z
	  valY = valY + adc_read();             // Accumulate Samples
	  count1++;
  }while(count1!=0x0800);                    // 2048 times
  valX=valX>>11;                       // division between 2048
  valY=valY>>11;
  
  accelerationx[0] = valX;
  accelerationy[0] = valY;
}

void print(signed long value){
	char buf[5]={'0','0','0','0','0'};
	int i = 4;
	
	if(value < 0){
		buf[0] = '-';
		value = -value;
	}
	else{
		buf[0] = ' ';
	}
	while(value != 0){
		int mod = value % 10;
		value = value / 10;
		buf[i] = mod + '0';
		i--;
	}
	usb_serial_write((unsigned char *)buf, 5);	
}

void position(void)  
{
unsigned int count2 ;
char buf[4];
char buf1[4];
count2=0;
    
    do{
		adc_start(ADC_MUX_PIN_F0, ADC_REF_POWER); //Read X  
		accelerationx[1]=accelerationx[1] + adc_read(); //filtering routine for noise attenuation
		
		adc_start(ADC_MUX_PIN_F1, ADC_REF_POWER); //Read Y
		accelerationy[1]=accelerationy[1] + adc_read(); //128 samples are averaged. The resulting average represents the acceleration of an instant
		count2++;                                       
    }while (count2!=0x080);                        // 128 sums of the acceleration sample  
    accelerationx[1]= accelerationx[1]>>7;          // division by 128
    accelerationy[1]= accelerationy[1]>>7;
  
    // accelerationx[1] = accelerationx[1] - (int)valX; //eliminating zero reference offset of the acceleration data
    // accelerationy[1] = accelerationy[1] - (int)valY; // to obtain positive and negative acceleration
  
    // if ((accelerationx[1] <=3)&&(accelerationx[1] >= -3)) //Discrimination window applied to the X axis acceleration
      // {accelerationx[1] = 0;}                       
     
    // if ((accelerationy[1] <=3)&&(accelerationy[1] >= -3)) 
      // {accelerationy[1] = 0;} 
  /*
	  //first X integration:
	velocityx[1]= velocityx[0]+ accelerationx[0]+ ((accelerationx[1] -accelerationx[0])>>1);    
	  //second X integration:
	positionX[1]= positionX[0] + velocityx[0] + ((velocityx[1] - velocityx[0])>>1);
	  //first Y integration:       
	velocityy[1] = velocityy[0] + accelerationy[0] + ((accelerationy[1] -accelerationy[0])>>1);
	  //second Y integration:       
	positionY[1] = positionY[0] + velocityy[0] + ((velocityy[1] - velocityy[0])>>1);  
  
    accelerationx[0] = accelerationx[1];  //The current acceleration value must be sent to the previous acceleration 
    accelerationy[0] = accelerationy[1];     //variable in order to introduce the new acceleration value.
    
    velocityx[0] = velocityx[1];          //Same done for the velocity variable
    velocityy[0] = velocityy[1];
  */
  
  
    //first X integration:
	
	if (accelerationx[1] <= valX + ACC_ERROR_SENSITIVITY
		&& accelerationx[1] >= valX - ACC_ERROR_SENSITIVITY){
		accelerationx[1] = valX;
		velocityx[1] = 0;
		positionX[1] = positionX[0];
		deltaX = 0;
	}
	else {
		
		// gating for max change in acceleration due to tilt
		if (accelerationx[1] > valX + MAX_ACC_CHANGE){
			accelerationx[1] = valX + MAX_ACC_CHANGE;
		} else if (accelerationx[1] < valX - MAX_ACC_CHANGE){
			accelerationx[1] = valX - MAX_ACC_CHANGE ;
		}
	
	
		// velocityx[1] = velocityx[0] + (accelerationx[1] - accelerationx[0]);
		// velocityx[1] = velocityx[0] + (accelerationx[1] - valX);
		velocityx[1] = (accelerationx[1] - valX);
		positionX[1] = positionX[0] + velocityx[1];
		deltaX = velocityx[1] - velocityx[0];
		deltaX = deltaX * MULT_FACTOR_OF_DELTA;
		
		 // if(((velocityx[1] - velocityx[0]) <= 2) && ((velocityx[1] - velocityx[0]) >= -2)){
		// //if(abs(velocityx[1] - velocityx[0]) <= 2){
			// positionX[1] = positionX[0];
			// deltaX = 0;
		// }		
		// else{
			// positionX[1] = positionX[0] + velocityx[1];
			// // deltaX = positionX[1] - positionX[0];
			// deltaX = velocityx[1];
			// deltaX = deltaX * MULT_FACTOR_OF_DELTA;
		// }	
	}
	
	

	
	if (accelerationy[1] <= valY + ACC_ERROR_SENSITIVITY
		&& accelerationy[1] >= valY - ACC_ERROR_SENSITIVITY){
		accelerationy[1] = valY;
		velocityy[1] = 0;
		positionY[1] = positionY[0];
		deltaY = 0;
	}
	else {
			
		// gating for max change in acceleration due to tilt
		if (accelerationy[1] > valY + MAX_ACC_CHANGE){
			accelerationy[1] = valY + MAX_ACC_CHANGE;
		} else if (accelerationy[1] < valY - MAX_ACC_CHANGE){
			accelerationy[1] = valY - MAX_ACC_CHANGE ;
		}
	
	
		// velocityy[1] = velocityy[0] + (accelerationy[1] - accelerationy[0]);
		velocityy[1] = (accelerationy[1] - accelerationy[0]);
		positionY[1] = positionY[0] + velocityy[1];
		deltaY = velocityy[1] - velocityy[0];
		deltaY = deltaY * MULT_FACTOR_OF_DELTA;
		
		// if(((velocityy[1] - velocityy[0]) <= 2) && ((velocityy[1] - velocityy[0]) >= -2)){
			// positionY[1] = positionY[0];
			// deltaY = 0;
		// }		
		// else{
			// positionY[1] = positionY[0] + velocityy[1];
			// // deltaY = positionY[1] - positionY[0];
			// deltaY = velocityy[1];
			// deltaY = deltaY * MULT_FACTOR_OF_DELTA;
		// }
	}
	
	
    accelerationx[0] = accelerationx[1];  //The current acceleration value must be sent to the previous acceleration 
    accelerationy[0] = accelerationy[1];     //variable in order to introduce the new acceleration value.
    velocityx[0] = velocityx[1];          //Same done for the velocity variable
    velocityy[0] = velocityy[1];
	positionX[0] = positionX[1];          //actual position data must be sent to the  
    positionY[0] = positionY[1];      //previous position
   
  
    // positionX[1] = positionX[1]<<18;      //The idea behind this shifting (multiplication) is a sensibility adjustment.
    // positionY[1] = positionY[1]<<18;      //Some applications require adjustments to a particular situation i.e. mouse application
    // data_transfer();  
    // positionX[1] = positionX[1]>>18;      //once the variables are sent them must return to 
    // positionY[1] = positionY[1]>>18;      //their original state
	
	
	usb_serial_putchar('G');
	usb_serial_putchar('x');
	usb_serial_putchar(' ');
	print(valX);
	usb_serial_putchar(' ');
	
	usb_serial_putchar(' ');
	usb_serial_putchar('G');
	usb_serial_putchar('y');
	usb_serial_putchar(' ');
	print(valY);		
	usb_serial_putchar(' ');
	
	usb_serial_putchar(' ');
	
	usb_serial_putchar(' ');
	usb_serial_putchar('A');
	usb_serial_putchar('x');
	usb_serial_putchar(' ');
	print(accelerationx[1]);
	usb_serial_putchar(' ');
	
	usb_serial_putchar(' ');
	usb_serial_putchar('A');
	usb_serial_putchar('y');
	usb_serial_putchar(' ');
	print(accelerationy[1]);
	usb_serial_putchar(' ');
	
	usb_serial_putchar(' ');
	
	usb_serial_putchar(' ');
	usb_serial_putchar('V');
	usb_serial_putchar('x');
	usb_serial_putchar(' ');
	print(velocityx[1]);
	usb_serial_putchar(' ');
	
	usb_serial_putchar(' ');
	usb_serial_putchar('V');
	usb_serial_putchar('y');
	usb_serial_putchar(' ');
	print(velocityy[1]);
	usb_serial_putchar(' ');
	
	usb_serial_putchar(' ');
	
	
	usb_serial_putchar(' ');
	usb_serial_putchar('X');
	usb_serial_putchar(' ');
	print(positionX[1]);
	usb_serial_putchar(' ');
	
	usb_serial_putchar(' ');
	usb_serial_putchar('Y');
	usb_serial_putchar(' ');
	print(positionY[1]);
	usb_serial_putchar(' ');
	
	usb_serial_putchar(' ');
	
	usb_serial_putchar(' ');
	usb_serial_putchar('D');
	usb_serial_putchar('x');
	usb_serial_putchar(' ');
	print(deltaX);
	usb_serial_putchar(' ');
	
	usb_serial_putchar(' ');
	usb_serial_putchar('D');
	usb_serial_putchar('y');
	usb_serial_putchar(' ');
	print(deltaY);
	usb_serial_putchar(' ');
		
	usb_serial_putchar('\n');
	
//	usb_mouse_move(deltaX,0, 0); // Move the mouse
//	_delay_ms(10);
 
//  movement_end_check();
    
    // positionX[0] = positionX[1];          //actual position data must be sent to the  
    // positionY[0] = positionY[1];      //previous position
   
    direction = 0;                        // data variable to direction variable reset 
}    

int main(void)
{
	int16_t val_x;
	int16_t val_y;
	uint16_t val_left;
	uint16_t val_right;
	
	int16_t prev_x;
	
	prev_x = ZERO_G_X;
	
	char buf[4];

	CPU_PRESCALE(CPU_125kHz);
	_delay_ms(1);		// allow slow power supply startup
	CPU_PRESCALE(CPU_16MHz); // set for 16 MHz clock
	
	LED_CONFIG;
	LED_OFF;

	// Initialize the USB, and then wait for the host to set configuration.
	// If the Teensy is powered without a PC connected to the USB port,
	// this will wait forever.
	usb_init();
	while (!usb_configured()) /* wait */ ;
	
	// Wait an extra second for the PC's operating system to load drivers
	// and do whatever it does to actually be ready for input
	_delay_ms(1000);
	
	valX = 0;
	valY = 0;
	
	accelerationx[0] = 0;
	accelerationx[1] = 0;
	accelerationy[0] = 0;
	accelerationy[1] = 0;
	velocityx[0] = 0;
	velocityx[1] = 0;
	velocityy[0] = 0;
	velocityy[1] = 0;
	positionX[0] = 0;
	positionX[1] = 0;
	positionY[0] = 0;
	positionX[1] = 0;
	
	
	Calibrate();

	while (1) {
	
		position();
		_delay_ms(10);		
		/* Teensy mouse move code
		_delay_ms(1000);
		LED_ON;  // turn the LED on while moving the mouse
		
		p = circle;
		for (i=0; i<36; i++) {
			x = pgm_read_byte(p++);
			y = pgm_read_byte(p++);
			usb_mouse_move(x, y, 0);
			_delay_ms(20);
		}
		LED_OFF;
		_delay_ms(9000);
		// This sequence creates a right click
		//usb_mouse_buttons(0, 0, 1);
		//_delay_ms(10);
		//usb_mouse_buttons(0, 0, 0);*/
	}
}

int8_t PROGMEM circle[] = {
16, -1,
15, -4,
16, -1,
14, -7,
13, -9,
11, -11,
9, -13,
7, -14,
4, -15,
1, -16,
-1, -16,
-4, -15,
-7, -14,
-9, -13,
-11, -11,
-13, -9,
-14, -7,
-15, -4,
-16, -1,
-16, 1,
-15, 4,
-14, 7,
-13, 9,
-11, 11,
-9, 13,
-7, 14,
-4, 15,
-1, 16,
1, 16,
4, 15,
7, 14,
9, 13,
11, 11,
13, 9,
14, 7,
15, 4,
16, 1
};
