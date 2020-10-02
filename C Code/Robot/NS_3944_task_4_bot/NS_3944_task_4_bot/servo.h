
/*
 * Team Id: eYRC#NS3944
 * Author List: Sagar Narkhede , Omkar Sutar, Pooja Katkar, Ashlesha Borade 
 * Filename:ROBOT_#NS3944 
 * Theme: Nutty Squirrel(eYRC)
*Filename	:	Servo.h
*Functions  :	servo_1(unsigned char degrees) , servo_2(unsigned char degrees) , 
				servo_2_free (void) , servo_2_free (void),servo1_pin_config (void),servo2_pin_config (void),
				timer3_init(void).
   
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

//Configure PORTE4  pin for servo motor 1 operation
void servo1_pin_config (void)
	{
		DDRE  = DDRE | 0x10;  //making PORTE 4 pin output
		PORTE = PORTE | 0x10; //setting PORTE 4 pin to logic 1	
	}
//Configure PORTE 5 pin for servo motor 2 operation
void servo2_pin_config (void)
	{
		DDRE  = DDRE | 0x20;  //making PORTE 5 pin output
		PORTE = PORTE | 0x20; //setting PORTE 5 pin to logic 1
	}


//TIMER3 initialization in 10 bit fast PWM mode  
//prescale:256
// WGM: 7) PWM 10bit fast, TOP=0x03FF
// actual value: 52.25Hz 

void timer3_init(void)
{
 TCCR3B = 0x00; //stop
 TCNT3H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT3L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
 OCR3AH = 0x03;	//Output compare Register high value for servo 1
 OCR3AL = 0xFF;	//Output Compare Register low Value For servo 1
 OCR3BH = 0x03;	//Output compare Register high value for servo 2
 OCR3BL = 0xFF;	//Output Compare Register low Value For servo 2
 OCR3CH = 0x03;	//Output compare Register high value for servo 3
 OCR3CL = 0xFF;	//Output Compare Register low Value For servo 3
 ICR3H  = 0x03;	
 ICR3L  = 0xFF;
 TCCR3A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR3C = 0x00;
 TCCR3B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}


	//Function to rotate Servo 1 by a specified angle in the multiples of 1.86 degrees
void servo_1(unsigned char degrees)
{
	float PositionPanServo = 0;
	PositionPanServo = ((float)degrees / 1.86) + 35.0;
	OCR3BH = 0x00;
	OCR3BL = (unsigned char) PositionPanServo;
}
 
//Function to rotate Servo 2 by a specified angle in the multiples of 1.86 degrees
void servo_2(unsigned char degrees)
{
	float PositionTiltServo = 0;
	PositionTiltServo = ((float)degrees / 1.86) + 35.0;
	OCR3CH = 0x00;
	OCR3CL = (unsigned char) PositionTiltServo;
}

//servo_free functions unlocks the servo motors from the any angle 
//and make them free by giving 100% duty cycle at the PWM. This function can be used to 
//reduce the power consumption of the motor if it is holding load against the gravity.

void servo_1_free (void) //makes servo 1 free rotating
{
	OCR3BH = 0x03;
	OCR3BL = 0xFF; //Servo 1 off
}

void servo_2_free (void) //makes servo 2 free rotating
{
	OCR3CH = 0x03;
	OCR3CL = 0xFF; //Servo 2 off
}
