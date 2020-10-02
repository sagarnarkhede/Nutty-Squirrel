#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>







//ADC pin configuration
void adc_pin_config (void)
{
	DDRF = 0x00;
	PORTF = 0x00;
	DDRK = 0x00;
	PORTK = 0x00;
}

void port_init()
{
	lcd_port_config();
	adc_pin_config();
	motion_pin_config();
}


void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=96)
}
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
	unsigned char PortARestore = 0;

	Direction &= 0x0F;	// removing upper nibbel for the protection
	PortARestore = PORTA; 		// reading the PORTA original status
	PortARestore &= 0xF0; 		// making lower direction nibbel to 0
	PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
	PORTA = PortARestore; 		// executing the command
}

void forward (void)
{
	motion_set (0x06);
}

void stop (void)
{
	motion_set (0x00);
}
void back (void) //both wheels backward
{
	motion_set(0x09);
}

void left (void) //Left wheel backward, Right wheel forward
{
	velocity(200,200);
	motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
	velocity(200,200);
	motion_set(0x0A);
}

void soft_left (void) //Left wheel stationary, Right wheel forward
{
	motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
	motion_set(0x02);
}

void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
	motion_set(0x01);
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
	motion_set(0x08);
}




//Function For ADC Conversion

void init_devices (void)
{
	cli(); //Clears the global interrupts
	port_init();
	adc_init();
	timer5_init();
	sei();   //Enables the global interrupts
}
void buzzer_on(void)
{
	PORTA = PORTA | 0x10;
}

void buzzer_off(void)
{
	PORTA = PORTA & 0xEF;
}
void buzz_blink(void)
{
	buzzer_on();
	delay(100);
	buzzer_off();
	delay(100);
}
void motion_pin_config (void)
{
	DDRA = DDRA | 0xFF;
	PORTA = PORTA & 0xF0;
	DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
	PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

void delay(long int a)
{
	long int i=0;
	long int j=0;
	for (i=0;i<=a;i++)
	{
		for (j=0;j<=2000;j++)
		{
		}
	}
}

