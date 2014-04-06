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
#define MAX_LOW 90
#define MAX_HIGH 250

#define ACC_ERROR_SENSITIVITY 7
#define MAX_ACC_CHANGE 95
#define MULT_FACTOR_OF_DELTA 2

signed long valX;
signed long valY;
int countx,county;
signed long accelerationx[2], accelerationy[2];
signed long left_click[2], right_click[2];
signed long clickDefault_left;
signed long clickDefault_right;

int direction;
signed long velocityx[2], velocityy[2];
signed long positionX[2];
signed long positionY[2];
signed long positionZ[2];

signed long deltaX;
signed long deltaY;

unsigned int click_left;
unsigned int click_right;

unsigned long max;

unsigned short  left_click_pressed;
unsigned short  left_click_released;

unsigned short right_click_released;
unsigned short right_click_pressed;
int flag;

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

// Get an average 2048 values of x and y axis
void Calibrate(void) 
{
  unsigned int count1;
  count1 = 0;
  
  // for calibration of zero g values from accelerometer
  do{  
	  adc_start(ADC_MUX_PIN_F3, ADC_REF_POWER); //Read X // Y
	  valX = valX + adc_read();  
	  
	  adc_start(ADC_MUX_PIN_F5, ADC_REF_POWER); //Read Y // Z
	  valY = valY + adc_read();             // Accumulate Samples	  
	  count1++;
  }while(count1!=0x0800);                    // 2048 times
  valX=valX>>11;                       // division between 2048
  valY=valY>>11;
  
	//_delay_ms(100);
  //count1 = 0;
  // do{  	  	  
	  // adc_start(ADC_MUX_PIN_F6, ADC_REF_POWER);
	  // clickDefault_left = clickDefault_left + adc_read();

	  // adc_start(ADC_MUX_PIN_F7, ADC_REF_POWER);
	  // clickDefault_right = clickDefault_right + adc_read(); 
	  // count1++;
  // }while(count1!=0x080);                    // 128 times
  
  // clickDefault_left = clickDefault_left >> 11;
  // clickDefault_right = clickDefault_right >> 11;
  
  accelerationx[0] = valX;
  accelerationy[0] = valY;
  
  // left_click[0] = clickDefault_left;
  // right_click[0] = clickDefault_right;
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
	
	// Set right click to 0 everytime, unless we detect a 1
	click_right = 0;	
		
	// get x and y
    do{
		adc_start(ADC_MUX_PIN_F3, ADC_REF_POWER); //Read X  
		accelerationx[1]=accelerationx[1] + adc_read(); //filtering routine for noise attenuation
		
		adc_start(ADC_MUX_PIN_F5, ADC_REF_POWER); //Read Y
		accelerationy[1]=accelerationy[1] + adc_read(); //64 samples are averaged. The resulting average represents the acceleration of an instant
			
		count2++;                                       
    }while (count2!=0x040);                        // 64 sums of the acceleration sample  
    accelerationx[1]= accelerationx[1]>>6;          // division by 64
    accelerationy[1]= accelerationy[1]>>6;
	
	_delay_ms(10);
	
	count2=0;
	
	// get left and right click
    do{
		adc_start(ADC_MUX_PIN_F7, ADC_REF_POWER);
		left_click[1] = left_click[1] + adc_read();
		
		adc_start(ADC_MUX_PIN_F6, ADC_REF_POWER); // 64 sums of the clicks sample  
		right_click[1] = right_click[1] + adc_read();
		count2++;                                       
    }while (count2!=0x080); 
		
	left_click[1] = left_click[1] >> 8; // ideally needs to be 7, but 8 works better
	right_click[1] = right_click[1] >> 8;
	
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
	
		//velocityx[1] = velocityx[0] + (accelerationx[1] - accelerationx[0]);
		velocityx[1] = (accelerationx[1] - valX);
		positionX[1] = positionX[0] + velocityx[1];
		deltaX = velocityx[1];
		deltaX = deltaX * MULT_FACTOR_OF_DELTA;
		
		if(deltaX > 127){
			deltaX = 127;
		}
		
		if(deltaX < -127){
			deltaX = -127;
		}
			
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
	
	
		//velocityy[1] = velocityy[0] + (accelerationy[1] - accelerationy[0]);
		velocityy[1] = (accelerationy[1] - valY);
		positionY[1] = positionY[0] + velocityy[1];
		deltaY = velocityy[1];
		deltaY = deltaY * MULT_FACTOR_OF_DELTA;
		
		if(deltaY > 127){
			deltaY = 127;
		}
		
		if(deltaY < -127){
			deltaY = -127;
		}
		
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
	
	/*  |       Press Event          _  <-- Release Event
	    |        |                 /   \
		|        v               /       \  
		|        _             /           \
		|      /   \         /              \
		|    /       \ _ _ /                 \
		|  /  slope 1             slope 2     \
		|/________________________________________________
		
		First If statement updates the max till the press event is reached.
		Press Event is detected when the current left click value starts becoming less than Max.
		Once Press Event is reached, we check for max value to be between 90 and 125. If it is, we update click_left to 1
		
		Then we wait for the Release event. Release Event is defined as current left click value becoming less than Max for the second time
		Once Release Event is reached, we release the click button by resetting click_left to 0.
	*/	
	if(left_click[1] > max){
		max = left_click[1];
		flag = 1;
	}
	else{
		if((max >= MAX_LOW) && (click_left == 0)){
			click_left = 1;
			usb_serial_putchar('L');
			usb_serial_putchar('e');
			usb_serial_putchar('f');
			usb_serial_putchar('t');
			usb_serial_putchar(' ');
			usb_serial_putchar('P');
			usb_serial_putchar('r');
			usb_serial_putchar('e');
			usb_serial_putchar('s');
			usb_serial_putchar('s');
			usb_serial_putchar('!');
			usb_serial_putchar('\n');
			_delay_ms(40);
			flag = 0;
		}
		if((max > MAX_LOW) && (click_left == 1) && (flag == 1)){
			click_left = 0;
			usb_serial_putchar('L');
			usb_serial_putchar('e');
			usb_serial_putchar('f');
			usb_serial_putchar('t');
			usb_serial_putchar(' ');
			usb_serial_putchar('R');
			usb_serial_putchar('e');
			usb_serial_putchar('l');
			usb_serial_putchar('e');
			usb_serial_putchar('a');
			usb_serial_putchar('s');
			usb_serial_putchar('e');
			usb_serial_putchar('\n');
		}
	}
	

	left_click[0] = left_click[1];
	right_click[0] = right_click[1];
   
    //To make it work!
	if(right_click[1] < 120){
		if((right_click[1] > 60) && (right_click[1] < 90)){
			click_right = 1;
			usb_serial_putchar('R');
			usb_serial_putchar('i');
			usb_serial_putchar('g');
			usb_serial_putchar('h');
			usb_serial_putchar('t');
			usb_serial_putchar(' ');
			usb_serial_putchar('B');
			usb_serial_putchar('i');
			usb_serial_putchar('t');
			usb_serial_putchar('c');
			usb_serial_putchar('h');
			usb_serial_putchar('e');
			usb_serial_putchar('s');
			usb_serial_putchar('!');
			usb_serial_putchar('\n');
		}					
	}
    	
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
	
	usb_serial_putchar(' ');
	usb_serial_putchar('l');
	usb_serial_putchar('c');
	usb_serial_putchar(' ');
	print(left_click[1]);
	usb_serial_putchar(' ');
		
	usb_serial_putchar(' ');
	usb_serial_putchar('r');
	usb_serial_putchar('c');
	usb_serial_putchar(' ');
	print(right_click[1]);
	usb_serial_putchar(' ');	
		
	usb_serial_putchar('\n');
	
	// usb_mouse_move(deltaY,deltaX,0); // Move the mouse- changed X to Y, Y to X to accomodate change in pins
	// usb_mouse_buttons(click_left,0,click_right);
//	_delay_ms(10);
 
//  movement_end_check();
    
    positionX[0] = positionX[1];          //actual position data must be sent to the  
    positionY[0] = positionY[1];      //previous position
   
}    

int main(void)
{	
	click_left = 0;
	flag = 0;
	max = 0;
	
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
	left_click[1] = 0;
	right_click[1] = 0;
	
	
	click_left = 0;
	click_right = 0;

	max = 0;

	left_click_pressed = 0;
	left_click_released = 0;

	right_click_released = 0;
	right_click_pressed = 0;
		
	Calibrate();

	while (1) {
	
		position();
		_delay_ms(10);		
	}
}

