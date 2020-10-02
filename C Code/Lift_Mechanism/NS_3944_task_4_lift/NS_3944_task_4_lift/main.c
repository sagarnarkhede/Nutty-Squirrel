

/*
*
*team id:3944
*author list: sagar narkhede,omkar sutar,pooja katakar,ashlesha borade
*file name:LIFTING_#NS3944
*theme: nutty squirrel
*functions:ISR(TIMER0_OVF_vect),void down (void),void stop (void);void reverse (void)
*global variable:double PWM,unsigned int count,int ir.
*
*/


#define F_CPU	16000000
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

double PWM = 0;
unsigned int count = 0;
int ir;




/*
*
*function name:ISR
*input:TIMER0_OVF_vect
*output:none
*logic:detect the overflow
*
*/
ISR(TIMER0_OVF_vect)
{
	OCR0A = (PWM);
}

/*
*
*function name:up
*input:none
*output:none
*logic:rotate the motor in anticlockwise direction 
*
*/

void up (void)
{
	PWM = 255;
	PORTC = 0x01;
	_delay_ms(3300);
	PWM = 00;
	PORTC = 0x00;
	PWM = 100;
	PORTC = 0x01;
	_delay_ms(50);
	PWM = 00;
	PORTC = 0x00;
	
}


/*
*
*function name:down
*input:none
*output:none
*logic:rotate the motor in clockwise direction
*
*/
void down(void)
{
	PWM = 125;
	PORTC = 0x02;
	_delay_ms(2450);
	PWM = 00;
	PORTC = 0x00;
}



/*
*
*function name:stop
*input:none
*output:none
*logic:stop the rotation of motor
*
*/
void stop (void)
{
	PWM = 00;
	PORTC = 0x00;
	_delay_ms(1000);
}


/*
*
*function name:reverse
*input:none
*output:none
*logic:rotate the motor in anticlockwise direction for small delay
*
*/
void reverse(void)
{
	_delay_ms(2500);
	PWM = 50;
	PORTC = 0x02;
	_delay_ms(400);
	PWM = 03;
	PORTC = 0x00;

}

int main(void)
 
{	
	DDRD = 0x40;//SET  PWM PIN D8 AS OUT PUT PIN
	PORTD = 0x00;
	
	DDRC = 0XFF;//SET A IN1=A0 7 IN2=A1 PIN AS OUT PUT PIN
	PORTC =0X00;
	
	DDRB = 0X00;//SET A PIN B4 AS OUTPUT PIN & CONNECT IR OUTPUT
	PORTB= 0XFF;
	
	TCCR0A = 0x83;
	TIMSK0 = 0x01;
	OCR0A = (PWM);
	sei();
	TCCR0B = 0x05;
	
    while(1)
    {

    ir = (PINB & 0X01); //READ THE VALU OF IR
    if((ir)== 1)
    {
	    
	    count++; //INCRECEMENT THE COUNT

	    
	    while (ir == 1)
	    {
		    ir = (PINB & 0X01);
		  
	    }
    }
    else
    {
	    
	    count++;
	   
	    while (ir == 0)
	    {
		    ir = (PINB & 0X01);
          
		}
	}
	
	
	
	/*************CHECK THE COUNT & BRING THE LIFT AT UPWARD & DOWNWORD***************/
	
	
 switch(count)
 
		{
			
			case 1:
					_delay_ms(1000);
			       up();
		           break;

			case 2:
			       reverse();
			       break;

			case 3:
				  _delay_ms(1000);
			      down();
			      break;

			case 4:
			       stop();
			       break;
				   
				
				case 5:
				_delay_ms(1000);
				up();
				break;

				case 6:
				reverse();
				break;

				case 7:
				_delay_ms(1000);
				down();
				break;

				case 8:
				stop();
				break;	   
			
			default:
		           stop();
			       break;
		}
						
   }
}


