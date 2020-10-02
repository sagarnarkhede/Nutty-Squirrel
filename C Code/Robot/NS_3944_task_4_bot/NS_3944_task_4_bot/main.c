


/*
 *
 * Team Id    : 3944
 * Author List: Sagar Narkhede , Omkar Sutar Pooja Katkar, Ashlesha Borade 
 * Filename   :  ROBOT_#NS3944 
 * Theme      : Nutty Squirrel
 * Functions  :void adc_pin_config (void),void port_init(),void adc_init(),void timer5_init(),void velocity (unsigned char left_motor, unsigned char right_motor),void motion_set (unsigned char Direction),void forward (void),
               void stop (void),void left (void),void back (void),void right (void),void soft_left (void),void soft_right (void),void soft_left_2 (void)void soft_right_2 (void) ,unsigned char ADC_Conversion(unsigned char Ch),
               void init_devices (void),void buzzer_on(void),void buzzer_off(void),void buzz_blink(void),void motion_pin_config (void),void blackline_data(void)int blackline(),int blackline_lifting(),void node(void),void back_turn(void),
               int check_condition (int pre_node, int curr_node, int next_node),void initialize (),void push (int x),int minDistance (int dist[], bool sptSet[]),void printPath (int parent[], int j),int printSolution (int dist[], int n, int parent[], int s, int i),
               void dijkstra (int graph[V][V], int s, int d),void find_path (int s, int d, int ob11, int ob12, int ob21, int ob22, int ob31, int ob32),int graph[40][40]int check_path (void),void find_size_of_bot (void),void find_size_of_path (void),int followpath(int source, int destination),
               void get_path (int source, int destination),unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading),void pick(void)void place(void),void scan_pick(void),void scan_pick2(void),void left_turn(),void forward_wls(void),void backward_wls(void);void right_turn(void);int check_obstacle(),


 * GlobalVariables:unsigned char Left_white_line ,unsigned char Center_white_line ,unsigned char Right_white_line,unsigned int count1,int sizepath ,int sizebot ,int pushvarible int destination1,int source1,int obstacle,int destination1, 
                   unsigned char sharp, distance, adc_reading,int bot_index,
 * 
 * 
 */


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include "colorsence.h"
#include "servo.h"
#include <stdio.h>
#include <limits.h>
#include<stdbool.h>
#include "lcd.h"
 //int 0x96 = 100;
 unsigned char ADC_Conversion(unsigned char Ch);
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0; // related to white line sensor
unsigned char Right_white_line = 0;
unsigned int count1=0;

 #define V 40
int path[40];
int pass[20];
int sizepath = 0;
int sizebot = 0;
int bot[20];
int pushvarible = 0;

int start=27;
int deposit_zon_S1[2]={34,35};//r,b,g.
int deposit_zon_S2[2]={36,37};
int deposit_zon_S3[2]={38,39};
int S1=34,S2=36,S3=38;
int Redindication = 0;

int greenindication	= 0;

int blueindication =0;



int obstacle;
int destination1, source1;

unsigned char sharp, distance, adc_reading;

int bot_index=0;
//void port_init();
//void timer5_init();

//function decleration

void velocity(unsigned char, unsigned char);
void motors_delay();
int pick_up_zon[6]={33,32,31,30,29,28};
int deposit_zon[6]={37,36,35,34,38,39};//r,b,g.	
void adc_pin_config (void);
void port_init();
void adc_init();
void timer5_init();
void velocity (unsigned char left_motor, unsigned char right_motor);
void motion_set (unsigned char Direction);
void forward (void);	
void stop (void);
void left (void);
void back (void);
void right (void);
void soft_left (void);
void soft_right (void);
void soft_left_2 (void);
void soft_right_2 (void) ;
unsigned char ADC_Conversion(unsigned char Ch);
void init_devices (void);
void buzzer_on(void);
void buzzer_off(void);
void buzz_blink(void);
void motion_pin_config (void);
void blackline_data(void);
int blackline();
int blackline_lifting();
void node(void);
void back_turn(void);
int check_condition (int pre_node, int curr_node, int next_node);
void initialize ();
void push (int x);
int minDistance (int dist[], bool sptSet[]);
void printPath (int parent[], int j);
int printSolution (int dist[], int n, int parent[], int s, int i);
void dijkstra (int graph[V][V], int s, int d);
void find_path (int s, int d, int ob11, int ob12, int ob21, int ob22, int ob31, int ob32);
int check_path (void);
void find_size_of_bot (void);
void find_size_of_path (void);
int followpath(int source, int destination);
void get_path (int source, int destination);
unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading);
void pick(void);
void place(void);
void scan_pick(void);
void left_turn();
void forward_wls(void);
void backward_wls(void);
void right_turn(void);
int check_obstacle();


//ADC pin configuration
void adc_pin_config (void)
{
	DDRF = 0x00;
	PORTF = 0x00;
	DDRK = 0x00;
	PORTK = 0x00;
}

//Function to configure ports to enable robot's motion


//Function to Initialize PORTS
void port_init()
{
	color_sensor_pin_config();
	color_sensor_pin_interrupt_init();
	lcd_port_config();
	adc_pin_config();
	motion_pin_config();
	rgb_port_config();
	servo1_pin_config();
	servo2_pin_config();
}
// Function to initilize adc
void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
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

	Direction &= 0x0F;	// removing upper nibble for the protection
	PortARestore = PORTA; 		// reading the PORTA original status
	PortARestore &= 0xF0; 		// making lower direction nibble to 0
	PortARestore |= Direction; // adding lower nibble for forward command and restoring the PORTA status
	PORTA = PortARestore; 		// executing the command
}

void forward (void)
{
	motion_set (0x06); //Both wheels forward.
}

void stop (void)
{
	motion_set (0x00); //Both wheels are stop.
}
void back (void) 
{
	motion_set(0x09); //both wheels backward
}

void left (void) 
{
	velocity(200,200);
	motion_set(0x05); ////Left wheel backward, Right wheel forward
}

void right (void) 
{
	velocity(200,200);
	motion_set(0x0A);//Left wheel forward, Right wheel backward
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
unsigned char ADC_Conversion(unsigned char Ch)
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;
	ADMUX= 0xE0| Ch;
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	//_delay_ms(40);
	return a;
}

void init_devices (void)
{
	cli(); //Clears the global interrupts
	port_init();
	servo2_pin_config();
	servo1_pin_config();
	adc_init();
	timer5_init();
	timer3_init();
	sei();   //Enables the global interrupts
}
//Function to on the buzzer
void buzzer_on(void)
{
	PORTA = PORTA | 0x10;
}

//Function to off the buzzer
void buzzer_off(void)
{
	PORTA = PORTA & 0xEF;
}
//function to on the buzzer on detection of obstacle
void buzz_blink(void)
{
	buzzer_on();
	delay(5000);
	buzzer_off();
	delay(5000);
}
void motion_pin_config (void)
{
	DDRA = DDRA | 0xFF;
	PORTA = PORTA & 0xF0;
	DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
	PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}


//to get data from white line sensor
void blackline_data(void)
{
	Left_white_line = ADC_Conversion(1);	//Getting data of Left WL Sensor
	Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
	Right_white_line = ADC_Conversion(3);

	print_sensor(1,1,3);	//Prints value of White Line Sensor1
	print_sensor(1,5,2);	//Prints Value of White Line Sensor2
	print_sensor(1,9,1);	//Prints Value of White Line Sensor3

}

//follows the blackline
int blackline()
{

	{
		blackline_data();
		
	if(Left_white_line > 0x96 &&  Center_white_line <0x96 && Right_white_line <0x96)
		{
			while(((Left_white_line < 0x96 &&  Center_white_line >0x96 && Right_white_line <0x96)||(Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 ))!=1)
				{	
					blackline_data();
//					lcd_cursor(2,1);
	//				lcd_string("left turn");
					
					forward();
					velocity(0,200);
					delay(30);
				}

		}


	 else if(Left_white_line<0x96 && Center_white_line > 0x96 && Right_white_line <0x96)
		{
			blackline_data();
			//lcd_cursor(2,1);
			//lcd_string("forward");
			
			forward();
			velocity(255,255);
		}

	 else if(Left_white_line< 0x96 && Center_white_line <0x96 && Right_white_line >0x96)
		{
			while(((Left_white_line < 0x96 &&  Center_white_line >0x96 && Right_white_line <0x96)||(Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 ))!=1)
				{
					blackline_data();
				//	lcd_cursor(2,1);
					//lcd_string("right turn");
					
					forward();
					velocity(220,0);
					delay(30);
				}
		}
		

	 else
		{
			
			
			forward();
			velocity(255,255);
           //lcd_cursor(2,1);
             //lcd_string("defoult");
		}
	}
}



int blackline_lifting()
{

	{
		blackline_data();
		
	if(Left_white_line > 0x96 &&  Center_white_line <0x96 && Right_white_line <0x96)
		{
			while(((Left_white_line < 0x96 &&  Center_white_line >0x96 && Right_white_line <0x96)||(Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 ))!=1)
				{	
					blackline_data();
//					lcd_cursor(2,1);
	//				lcd_string("left turn");
					
					forward();
					velocity(0,130);
					delay(30);
				}

		}


	 else if(Left_white_line<0x96 && Center_white_line > 0x96 && Right_white_line <0x96)
		{
			blackline_data();
			//lcd_cursor(2,1);
			//lcd_string("forward");
			
			forward();
			velocity(130,130);
		}

	 else if(Left_white_line< 0x96 && Center_white_line <0x96 && Right_white_line >0x96)
		{
			while(((Left_white_line < 0x96 &&  Center_white_line >0x96 && Right_white_line <0x96)||(Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 ))!=1)
				{
					blackline_data();
				//	lcd_cursor(2,1);
					//lcd_string("right turn");
					
					forward();
					velocity(130,0);
					delay(30);
				
		}
	}
	 else
		{
			
			
			forward();
			velocity(130,130);
           //lcd_cursor(2,1);
             //lcd_string("defoult");
		}
  check_obstacle();
	
	
	}
}




void node(void)
{
	
}
void back_turn(void)
{
	right();;
	delay(1800);
	while(ADC_Conversion(2) < 0x96);
	stop();
} 


///checks the previous, current , next node & sets the direction (forward, reverse, left, right).

int check_condition (int pre_node, int curr_node, int next_node)

{
  int f = 1, l = 2, r = 3,b=4,sl=5 ,sr=6,sf=7,bb=8, bf=9,lf=10,ll=11,rx=12,lx=13,lsl=14,lr=15,lr1=16 , ll1 = 17,lr2 = 18;
  
   if (pre_node == 0 && curr_node == 1 && next_node == 2)
   {
	   return (f);
   }
   else if (pre_node == 0 && curr_node == 1 && next_node == 12)
   {
	   return (l);
   }
   else if (pre_node == 0 && curr_node == 13 && next_node == 12)
   {
	   return (r);
   }
   else if (pre_node == 0 && curr_node == 13 && next_node == 14)
   {
	   return (f);
   }
   else if (pre_node == 0 && curr_node == 1 && next_node == 29)
   {
	   return (sr);
   }
   //1
   else if (pre_node == 1 && curr_node == 0 && next_node == 13)
   {
	   return (r);
   }
   else if (pre_node == 1 && curr_node == 2 && next_node == 3)
   {
	   return (f);
   }
   else if (pre_node == 1 && curr_node == 12 && next_node == 13)
   {
	   return (l);
   }
   else if (pre_node == 1 && curr_node == 12 && next_node == 11)
   {
	   return (r);
   }
   else if (pre_node == 1 && curr_node == 0 && next_node == 28)
   {
	   return (sl);
   }
   else if (pre_node == 1 && curr_node == 2 && next_node == 30)
   {
	   return (sr);
   }
   //2
   else if (pre_node == 2 && curr_node == 1 && next_node == 0)
   {
	   return (f);
   }
   else if (pre_node == 2 && curr_node == 1 && next_node == 12)
   {
	   return (r);
   }
   else if (pre_node == 2 && curr_node == 3 && next_node == 4)
   {
	   return (f);
   }
   else if (pre_node == 2 && curr_node == 3 && next_node == 10)
   {
	   return (l);
   }
   else if (pre_node == 2 && curr_node == 1&& next_node == 29)
   {
	   return (sl);
   }
   //3
   else if (pre_node == 3 && curr_node == 2 && next_node == 1)
   {
	   return (f);
   }
   else if (pre_node == 3 && curr_node == 4 && next_node == 5)
   {
	   return (f);
   }
   else if (pre_node == 3 && curr_node == 10 && next_node == 11)
   {
	   return (l);
   }
   else if (pre_node == 3 && curr_node == 10 && next_node == 9)
   {
	   return (r);
   }
   else if (pre_node == 3 && curr_node == 10 && next_node == 27)
   {
	   return (f);
   }
   else if (pre_node == 3 && curr_node == 2 && next_node == 30)
   {
	   return (sl);
   }
   else if (pre_node == 3 && curr_node == 4 && next_node == 31)
   {
	   return (sr);
   }
   //4
   else if (pre_node == 4 && curr_node == 3 && next_node == 2)
   {
	   return (f);
   }
   else if (pre_node == 4 && curr_node == 3 && next_node == 10)
   {
	   return (r);
   }
   else if (pre_node == 4 && curr_node == 5 && next_node == 6)
   {
	   return (f);
   }
   else if (pre_node == 4 && curr_node == 5 && next_node == 8)
   {
	   return (l);
   }
   else if (pre_node == 4 && curr_node == 5 && next_node == 32)
   {
	   return (sr);
   }
   //5
   else if (pre_node == 5 && curr_node == 4 && next_node == 3)
   {
	   return (f);
   }
   else if (pre_node == 5 && curr_node == 8 && next_node == 9)
   {
	   velocity(255,255);
	   forward();
	   delay(100);
	   return (l);
   }
   else if (pre_node == 5 && curr_node == 8 && next_node == 7)
   {
	   return (r);
   }
   else if (pre_node == 5 && curr_node == 6 && next_node == 7)
   {
	   return (l);
   }
   else if (pre_node == 5 && curr_node == 4 && next_node == 31)
   {
	   return (sl);
   }
   else if (pre_node == 5 && curr_node == 6 && next_node == 33)
   {
	   return (sr);
   }
   //6
   else if (pre_node == 6 && curr_node == 7 && next_node == 8)
   {
	   return (l);
   }
   else if (pre_node == 6 && curr_node == 7 && next_node == 20)
   {
	   return (f);
   }
   else if (pre_node == 6 && curr_node == 5 && next_node == 8)
   {
	   return (r);
   }
   else if (pre_node == 6 && curr_node == 5 && next_node == 4)
   {
	   return (f);
   }
   else if (pre_node == 6 && curr_node == 5 && next_node == 32)
   {
	   return (sl);
   }
   //7
   else if (pre_node == 7 && curr_node == 6 && next_node == 5)
   {
	   return (r);
   }
   else if (pre_node == 7 && curr_node == 8 && next_node == 5)
   {
	   return (l);
   }
   else if (pre_node == 7 && curr_node == 8 && next_node == 9)
   {
	   return (r);
   }
   else if (pre_node == 7 && curr_node == 20 && next_node == 11)
   {
	   return (l);
   }
   else if (pre_node == 7 && curr_node == 20 && next_node == 21)
   {
	   return (f);
   }
   else if (pre_node == 7 && curr_node == 6 && next_node == 33)
   {
	   return (sf);
   }
   //8
   else if (pre_node == 8 && curr_node == 9 && next_node == 10)
   {
	   return (l);
   }
   else if (pre_node == 8 && curr_node == 9 && next_node == 18)
   {
	   return (r);
   }
   else if (pre_node == 8 && curr_node == 7 && next_node == 20)
   {
	   return (lsl);
   }
   else if (pre_node == 8 && curr_node == 7 && next_node == 6)
   {
	   return (r);
   }
   else if (pre_node == 8 && curr_node == 5 && next_node == 4)
   {
	   return (r);
   }
   else if (pre_node == 8 && curr_node == 5 && next_node == 6)
   {
	   return (l);
   }
   else if (pre_node == 8 && curr_node == 5 && next_node == 32)
   {
	   return (sf);
   }
   else if (pre_node == 9 && curr_node == 8 && next_node == 5)
   {
	   forward();
	   velocity(150,150);
	   delay(100);
	   return (r);
   }
   else if (pre_node == 9 && curr_node == 8 && next_node == 7)
   {
	   return (l);

   }
   else if (pre_node == 9 && curr_node == 10 && next_node == 11)
   {
	    forward();
	    velocity(0,150);
delay(100);
	    while(ADC_Conversion(2)<0x96);
	   return (f);
   }
   else if (pre_node == 9 && curr_node == 10 && next_node == 27)
   {
	   return (r);
   }
   else if (pre_node == 9 && curr_node == 10 && next_node == 3)
   {
	   return (l);
   }
   else if (pre_node == 9 && curr_node == 18 && next_node == 17)
   {
	    forward();
	    velocity(0,150);
		delay(100);
	    while(ADC_Conversion(2)<0x96);
	   return (f);
   }
   else if (pre_node == 9 && curr_node == 18 && next_node == 19)
   {
	   return (r);
   }
   else if (pre_node == 10 && curr_node == 9 && next_node == 8)
   {
	   return (r);
   }
   else if (pre_node == 10 && curr_node == 9 && next_node == 18)
   {
	    forward();
	    velocity(0,150);
		delay(100);
	    while(ADC_Conversion(2)<0x96);
	   return (f);
   }
   else if (pre_node == 10 && curr_node == 3 && next_node == 2)
   {
	   return (r);
   }
   else if (pre_node == 10 && curr_node == 3 && next_node == 4)
   {
	   return (l);
   }
   else if (pre_node == 10 && curr_node == 11 && next_node == 12)
   {
	   return (l);
   }
   else if (pre_node == 10 && curr_node == 11 && next_node == 16)
   {
	    forward();
	    velocity(0,150);
		delay(100);
	    while(ADC_Conversion(2)<0x96);
	   return (f);
   }
   else if (pre_node == 11 && curr_node == 10 && next_node == 27)
   {
	   return (l);
   }
   else if (pre_node == 11 && curr_node == 10 && next_node == 3)
   {
	   return (r);
   }
   else if (pre_node == 11 && curr_node == 10 && next_node == 9)
   {
	   return (f);
   }
   else if (pre_node == 11 && curr_node == 12 && next_node == 1)
   {
	   return (l);
   }
   else if (pre_node == 11 && curr_node == 12 && next_node == 13)
   {
	   return (f);
   }
   else if (pre_node == 11 && curr_node == 16 && next_node == 15)
   {
	   return (l);
   }
   else if (pre_node == 11 && curr_node == 16 && next_node == 17)
   {
	    forward();
	    velocity(0,150);
		delay(100);
	    while(ADC_Conversion(2)<0x96);
	   return (f);
   }
   else if (pre_node == 12 && curr_node == 11 && next_node == 10)
   {
	   return (f);
   }

   else if (pre_node == 12 && curr_node == 11 && next_node == 16)
   {
	   return (l);
   }
   else if (pre_node == 12 && curr_node == 13 && next_node == 0)
   {
	   return (l);
   }
   else if (pre_node == 12 && curr_node == 1 && next_node == 0)
   {
	   return (r);
   }
   else if (pre_node == 12 && curr_node == 1 && next_node == 2)
   {
	   return (l);
   }
   else if (pre_node == 12 && curr_node == 1 && next_node == 29)
   {
	   return (sf);
   }
   else if (pre_node == 13 && curr_node == 12 && next_node == 1)
   {
	   return (r);
   }

   else if (pre_node == 13 && curr_node == 12 && next_node == 11)
   {
	   return (f);
   }
   else if (pre_node == 13 && curr_node == 0 && next_node == 1)
   {
	   return (l);
   }
   else if (pre_node == 13 && curr_node == 14 && next_node == 22)
   {
	   return (r);
   }
   else if (pre_node == 13 && curr_node == 14 && next_node == 15)
   {
	   return (r);
   }
   else if (pre_node == 13 && curr_node == 0 && next_node == 28)
   {
	   return (sf);
   }
   else if (pre_node == 14 && curr_node == 15 && next_node == 16)
   {
	   return (f);
   }
   else if (pre_node == 14 && curr_node == 13 && next_node == 12)
   {
	   return (l);
   }
   else if (pre_node == 14 && curr_node == 13 && next_node == 0)
   {
	   return (f);
   }
   else if (pre_node == 14 && curr_node == 22 && next_node == 34)
   {
	   return (sf);
   }
   else if (pre_node == 14 && curr_node == 15 && next_node == 35)
   {
	   return (l);
   }
   else if (pre_node == 15 && curr_node == 14 && next_node == 13)
   {
	   return (l);
   }
   else if (pre_node == 15 && curr_node == 14 && next_node == 22)
   {
	   return (r);
   }
   else if (pre_node == 15 && curr_node == 16 && next_node == 11)
   {
	   return (r);
   }
   else if (pre_node == 15 && curr_node == 16 && next_node == 17)
   {
	   return (l);
   }
   else if (pre_node == 15 && curr_node == 16 && next_node == 22)
   {
	   return (r);
   }
   else if (pre_node == 16 && curr_node == 15 && next_node == 14)
   {
	   return (f);
   }
   else if (pre_node == 16 && curr_node == 11 && next_node == 12)
   {
	   return (r);
   }
   else if (pre_node == 16 && curr_node == 11 && next_node == 10)
   {
	    forward();
	    velocity(0,150);
		delay(100);
	    while(ADC_Conversion(2)<0x96);
	   return (f);
   }
   else if (pre_node == 16 && curr_node == 17 && next_node == 23)
   {
	   return (ll);
   }
   else if (pre_node == 16 && curr_node == 17 && next_node == 18)
   {
	    forward();
	    velocity(0,150);
		delay(100);
	    while(ADC_Conversion(2)<0x96);
	   return (f);
   }
   else if (pre_node == 16 && curr_node == 15 && next_node == 35)
   {
	   return (sr);
   }
   else if (pre_node == 17 && curr_node == 16 && next_node == 15)
   {
	   return (r);
   }
   else if (pre_node == 17 && curr_node == 16 && next_node == 11)
   {
	   forward();
	   velocity(0,150);
	   delay(100);
	  while(ADC_Conversion(2)<0x96);
	   return (f);
   }
   else if (pre_node == 17 && curr_node == 18 && next_node == 19)
   {
	   return (r);
   }
   else if (pre_node == 17 && curr_node == 18 && next_node == 9)
   {
	    forward();
	    velocity(0,150);
		delay(100);
	    while(ADC_Conversion(2)<0x96);
	   return (f);
   }
   else if (pre_node == 17 && curr_node == 23 && next_node == 24)
   {
	   return (lf);
   }
   else if (pre_node == 18 && curr_node == 19 && next_node == 20)
   {
	   return (f);
   }
   else if (pre_node == 18 && curr_node == 9 && next_node == 10)
   {
	   return (f);
   }
   else if (pre_node == 18 && curr_node == 9 && next_node == 8)
   {
	   return (l);
   }
   else if (pre_node == 18 && curr_node == 17 && next_node == 16)
   {
	    forward();
	    velocity(0,150);
		delay(100);
	    while(ADC_Conversion(2)<0x96);
	   return (l);
   }
   else if (pre_node == 18 && curr_node == 17 && next_node == 23)
   {
	   return (lr);
   }
   else if (pre_node == 18 && curr_node == 19 && next_node == 36)
   {
	   return (sl);
   }
   else if (pre_node == 19 && curr_node == 20 && next_node == 21)
   {
	   return (l);
   }
   else if (pre_node == 19 && curr_node == 20 && next_node == 7)
   {
	   return (r);
   }
   else if (pre_node == 19 && curr_node == 18 && next_node == 17)
   {
	   return (l);
   }
   else if (pre_node == 19 && curr_node == 18 && next_node == 9)
   {
	   return (l);
   }
   else if (pre_node == 20 && curr_node == 19 && next_node == 18)
   {
	   return (f);
   }
   else if (pre_node == 20 && curr_node == 7 && next_node == 8)
   {
	   return (r);
   }
   else if (pre_node == 20 && curr_node == 7 && next_node == 6)
   {
	   return (f);
   }
   else if (pre_node == 20 && curr_node == 19 && next_node == 36)
   {
	   return (r);
   }
   else if (pre_node == 20 && curr_node == 21 && next_node == 37)
   {
	   return (sf);
   }
   else if (pre_node == 21 && curr_node == 20 && next_node == 19)
   {
	   return (r);
   }
   else if (pre_node == 21 && curr_node == 20 && next_node == 7)
   {
		blackline_data();
		while(1!=((Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 )))
		{
			blackline_data();
			blackline_lifting();
		}
	   return (0);
   }
   else if (pre_node == 22 && curr_node == 14 && next_node == 13)
   {
	   return (f);
   }
   else if (pre_node == 22 && curr_node == 14 && next_node == 15)
   {
	   return (l);
   }
   else if (pre_node == 23 && curr_node == 17 && next_node == 16)
   {
	   return (r);
   }
   else if (pre_node == 23 && curr_node == 17 && next_node == 18)
   {
	   return (l);
   }
   else if (pre_node == 23 && curr_node == 24 && next_node == 25)
   {
	   return (ll1);
   }
   else if (pre_node == 23 && curr_node == 24 && next_node == 26)
   {
	   return (lr1);
   }
   else if (pre_node == 24 && curr_node == 23 && next_node == 17)
   {
	   return (f);
   }
   else if (pre_node == 24 && curr_node == 25 && next_node == 38)
   {
	   return (sf);
   }
   else if (pre_node == 24 && curr_node == 26 && next_node == 39)
   {
	   return (sf);
   }
   else if (pre_node == 25 && curr_node == 24 && next_node == 23)
   {
	   return (lr2);
   }
   else if (pre_node == 26 && curr_node == 24 && next_node == 23)
   {
	   return (ll);
   }
   else if (pre_node == 27 && curr_node == 10 && next_node == 3)
   {
	   return (f);
   }
   else if (pre_node == 27 && curr_node == 10 && next_node == 11)
   {
	   return (r);
   }
   else if (pre_node == 27 && curr_node == 10 && next_node == 9)
   {
	   return (l);
   }
   else if (pre_node == 27 && curr_node == 27 && next_node == 10)
   {
	   return (f);
   }

   else if (pre_node == 28 && curr_node == 0 && next_node == 13)
   {
	   return (bf);
   }

   else if (pre_node == 28 && curr_node == 0 && next_node == 1)
   {
	   return (lx);
   }
   else if (pre_node == 29 && curr_node == 1 && next_node == 0)
   {
	   return (rx);
   }
   else if (pre_node == 29 && curr_node == 1 && next_node == 2)
   {
	   return (lx);
   }
   else if (pre_node == 29 && curr_node == 1 && next_node == 12)
   {
	   return (bf);
   }
   else if (pre_node == 30 && curr_node == 2 && next_node == 3)
   {
	   return (lx);
   }
   else if (pre_node == 30 && curr_node == 2 && next_node == 1)
   {
	   return (rx);
   }
   else if (pre_node == 31 && curr_node == 4 && next_node == 5)
   {
	   return (lx);
   }
   else if (pre_node == 31 && curr_node == 4 && next_node == 3)
   {
	   return (rx);
   }
   else if (pre_node == 32 && curr_node == 5 && next_node == 6)
   {
	   return (lx);
   }
   else if (pre_node == 32 && curr_node == 5 && next_node == 4)
   {
	   return (rx);
   }
   else if (pre_node == 32 && curr_node == 5 && next_node == 8)
   {
	   return (bf);
   }
   else if (pre_node == 33 && curr_node == 6 && next_node == 5)
   {
	   return (rx);
   }
   else if (pre_node == 33 && curr_node == 6 && next_node == 7)
   {
	   return (bf);
   }
   else if (pre_node == 34 && curr_node == 22 && next_node == 14)
   {
	   return (f);
   }
   else if (pre_node == 35 && curr_node == 15 && next_node == 16)
   {
	   return (rx);
   }
   else if (pre_node == 35 && curr_node == 15 && next_node == 14)
   {
	   return (l);
   }
   else if (pre_node == 36 && curr_node == 19 && next_node == 20)
   {
	   return (r);
   }
   else if (pre_node == 36 && curr_node == 19 && next_node == 18)
   {
	   return (lx);
   }
   else if (pre_node == 37 && curr_node == 21 && next_node == 20)
   {
	   return (f);
   }
   else if (pre_node == 38 && curr_node == 25 && next_node == 24)
   {
	   return (bf);
   }
   else if (pre_node == 39 && curr_node == 26 && next_node == 24)
   {
	   return (bf);
   }
   
   //conditions for obstacles
   
   
   
   else if (pre_node ==0  && curr_node ==0 && next_node ==1)
   {
	   return (f);
   }
   else if (pre_node ==0  && curr_node ==0 && next_node ==28)
   {
	   return (f);
   }
   
   else if (pre_node == 0 && curr_node ==0 && next_node ==13)
   {
	   return (f);
   }
   else if (pre_node ==1  && curr_node ==1 && next_node ==0)
   {
	   return (f);
   }
   
   else if (pre_node == 1 && curr_node ==1 && next_node ==29)
   {
	   return (f);
   }
   
   else if (pre_node ==1 && curr_node ==1 && next_node ==12)
   {
	   return (f);
   }
   
   else if (pre_node == 1 && curr_node ==1 && next_node ==2)
   {
	   return (f);
   }
   
   else if (pre_node == 2&& curr_node ==2 && next_node ==1)
   {
	   return (f);
   }
   
   else if (pre_node ==2  && curr_node ==2 && next_node ==3)
   {
	   return (f);
   }
   
   else if (pre_node ==2  && curr_node ==2 && next_node ==30)
   {
	   return (f);
   }
   
   else if (pre_node ==3  && curr_node ==3 && next_node ==2)
   {
	   return (f);
   }
   
   else if (pre_node ==3&& curr_node ==3 && next_node ==4)
   {
	   return (f);
   }
   
   else if (pre_node ==3  && curr_node ==3 && next_node ==10)
   {
	   return (f);
   }
   
   else if (pre_node ==4 && curr_node ==4 && next_node ==3)
   {
	   return (f);
   }
   
   else if (pre_node ==4  && curr_node ==4 && next_node == 5)
   {
	   return (f);
   }
   
   else if (pre_node ==4  && curr_node ==4 && next_node ==31)
   {
	   return (f);
   }
   
   else if (pre_node ==5  && curr_node ==5 && next_node == 4)
   {
	   return (f);
   }
   
   else if (pre_node ==5&& curr_node ==5 && next_node ==6)
   {
	   return (f);
   }
   
   else if (pre_node ==5  && curr_node ==5 && next_node ==32)
   {
	   return (sf);
   }
    else if (pre_node ==5  && curr_node ==5 && next_node ==8)
    {
	    return (f);
    }
   else if (pre_node ==6  && curr_node ==6 && next_node ==5)
   {
	   return (f);
	   
   }
   else if (pre_node ==6 && curr_node ==6 && next_node ==33)
   {
	   return (f);
   }
   
   else if (pre_node ==6  && curr_node ==6 && next_node ==7)
   {
	   return (f);
   }
   else if (pre_node == 7 && curr_node ==7 && next_node ==6)
   {
	   return (f);
   }
   
   else if (pre_node ==7  && curr_node ==7 && next_node ==8)
   {
	   return (f);
   }
   
   else if (pre_node ==7  && curr_node ==7 && next_node == 20)
   {
	   return (f);
   }
   
   else if (pre_node == 8 && curr_node ==8 && next_node ==7)
   {
	   return (f);
   }
   
   else if (pre_node ==8  && curr_node ==8 && next_node ==9)
   {
	   return (f);
   }
   
   else if (pre_node ==8  && curr_node ==8 && next_node ==5)
   {
	   return (f);
   }
   
   else if (pre_node ==9  && curr_node ==9 && next_node ==8)
   {
	   return (f);
   }
   
   else if (pre_node ==9  && curr_node ==9 && next_node ==10)
   {
	   return (f);
   }
   
   else if (pre_node == 9 && curr_node ==9 && next_node ==18)
   {
	   return (f);
   }
   
   else if (pre_node ==10 && curr_node ==10&& next_node ==11)
   {
	   return (f);
   }
   
   else if (pre_node == 10&& curr_node ==10&& next_node ==27)
   {
	   return (f);
   }
   
   else if (pre_node ==11 && curr_node ==11&& next_node ==10)
   {
	   return (f);
   }
   
   else if (pre_node ==11 && curr_node ==11&& next_node ==12)
   {
	   return (f);
   }
   
   else if (pre_node ==11 && curr_node ==11&& next_node ==16)
   {
	   return (f);
   }
   
   else if (pre_node ==12&& curr_node ==12&&next_node ==13)
   {
	   return (f);
   }
   
   else if (pre_node ==12 && curr_node ==12&& next_node ==11)
   {
	   return (f);
   }
   
   else if (pre_node ==12 && curr_node ==12&& next_node ==1)
   {
	   return (f);
   }
   else if (pre_node ==13 && curr_node ==13&& next_node ==14)
   {
	   return (f);
   }
   
   else if (pre_node ==13 && curr_node ==13&& next_node ==12)
   {
	   return (f);
   }
   else if (pre_node ==13 && curr_node ==13&& next_node == 0)
   {
	   return (f);
   }
   
   else if (pre_node ==14 && curr_node ==14&& next_node ==13)
   {
	   return (f);
   }
   
   else if (pre_node ==14 && curr_node ==14&& next_node ==15)
   {
	   return (f);
   }
   
   else if (pre_node ==14 && curr_node ==14&& next_node ==22)
   {
	   return (f);
   }
   
   else if (pre_node ==15&& curr_node ==15&& next_node ==14)
   {
	   return (f);
   }
   
   else if (pre_node ==15 && curr_node ==15&& next_node ==16)
   {
	   return (f);
   }
   
   else if (pre_node ==15 && curr_node ==115& next_node ==35)
   {
	   return (f);
   }
   
   else if (pre_node ==16 && curr_node ==16&& next_node ==15)
   {
	   return (f);
   }
   
   else if (pre_node ==16 && curr_node ==16&& next_node ==17)
   {
	   return (f);
   }
   
   else if (pre_node ==16 && curr_node ==16&& next_node ==11)
   {
	   return (f);
   }
   
   else if (pre_node ==17 && curr_node ==17  && next_node ==16 )
   {
	   return (f);
   }
   else if (pre_node ==17  && curr_node ==17  && next_node ==18 )
   {
	   return (f);
   }
   else if (pre_node ==17  && curr_node ==17  && next_node ==23 )
   {
	   return (f);
   }
   else if (pre_node == 18 && curr_node ==18  && next_node ==17 )
   {
	   return (f);
   }
   
   else if (pre_node ==18  && curr_node ==18  && next_node ==9 )
   {
	   return (f);
   }
   else if (pre_node ==18  && curr_node ==18  && next_node ==19 )
   {
	   return (f);
   }
   else if (pre_node == 19 && curr_node ==19  && next_node ==20 )
   {
	   return (f);
   }
   else if (pre_node == 19 && curr_node ==19  && next_node ==20 )
   {
	   return (f);
   }
   else if (pre_node ==19  && curr_node ==19  && next_node ==18 )
   {
	   return (f);
   }
   else if (pre_node ==19  && curr_node ==19  && next_node ==36 )
   {
	   return (f);
   }
   else if (pre_node ==20  && curr_node ==20  && next_node ==19 )
   {
	   return (f);
   }
   else if (pre_node == 20 && curr_node ==20  && next_node ==7 )
   {
	   return (f);
   }
   else if (pre_node ==20  && curr_node == 20 && next_node ==21 )
   {
	   return (f);
   }
   else if (pre_node == 21  && curr_node ==21  && next_node == 37 )
   {
	   return (f);
   }
   else if (pre_node ==21  && curr_node ==21  && next_node ==20 )
   {
	   return (f);
   }
   else if (pre_node ==22  && curr_node ==22  && next_node ==14 )
   {
	   return (f);
   }else if (pre_node ==22  && curr_node ==22  && next_node ==34 )
   {
	   return (f);
   }
   else if (pre_node ==23  && curr_node ==23  && next_node ==24 )
   {
	   return (f);
   }
   else if (pre_node ==23  && curr_node ==23  && next_node ==17 )
   {
	   return (f);
   }
   else if (pre_node ==24  && curr_node ==24  && next_node ==25 )
   {
	   return (f);
   }else if (pre_node ==24  && curr_node ==24  && next_node ==26 )
   {
	   return (f);
   }
   else if (pre_node ==24  && curr_node ==24  && next_node ==23 )
   {
	   return (f);
   }
   else if (pre_node ==25  && curr_node ==25  && next_node ==24 )
   {
	   return (f);
   }
   else if (pre_node == 25 && curr_node ==25  && next_node ==38 )
   {
	   return (f);
   }
   else if (pre_node ==26  && curr_node ==26  && next_node ==24 )
   {
	   return (f);
   }
   else if (pre_node ==26  && curr_node ==26  && next_node ==39 )
   {
	   return (f);
   }
   else if (pre_node ==27  && curr_node == 27  && next_node ==10 )
   {
	   return (f);
   }
   else if (pre_node ==28  && curr_node ==28  && next_node == 0)
   {
	    node();
	    return(0);
   }
   else if (pre_node ==29  && curr_node == 29 && next_node == 1)
   {
	  node();
	  return(0);
   }
   else if (pre_node ==30  && curr_node ==30  && next_node ==2 )
   {
	    node();
	    return(0);
   }
   else if (pre_node ==31  && curr_node == 31 && next_node ==4 )
   {
	    node();
	    return(0);
   }
   else if (pre_node ==32  && curr_node ==32  && next_node ==5 )
   {
	   node();
	   return(0);
   }
   else if (pre_node ==33  && curr_node ==33  && next_node ==6 )
   {
	   node();
	   return(0);
   }
   else if (pre_node ==34  && curr_node ==34  && next_node ==22 )
   {
	   return (bb);
   }
   else if (pre_node ==35  && curr_node == 35 && next_node ==15 )
   {
	   node();
	   return(0);
   }
   else if (pre_node == 36 && curr_node == 36 && next_node ==19 )
   {
	   node();
	   return(0);
   }
   else if (pre_node ==37  && curr_node ==37  && next_node ==21 )
   {
	   return (bb);
	   
   }
   else if (pre_node =38  && curr_node ==38  && next_node ==25 )
   {
	     node();
	     return(0);
   }
   else if (pre_node ==39  && curr_node ==39  && next_node ==26 )
   {
	  node();
	  return(0);
   }

   else if (pre_node == 0 && curr_node == 0 && next_node == 0)
   {
	   return (f);
   }
    else if (pre_node == 0 && curr_node == 0 && next_node == 13)
      {
	return (f);
      }
   else if (pre_node == 5 && curr_node == 5 && next_node == 8)
	  {
		  return (f);
	  }
   }

// to initialise array
void initialize ()
{
 
  for (int i = 0; i < 40; i++)
    {
      path[i] = -999;
    }
  for (int i = 0; i < 20; i++)
    {
      bot[i] = -999;
    }

}

// to initialise array
void push (int x)
{
  for (int i = 0; i < 40; i++)
    {
      if (path[i] == -999)
	{
	  path[i] = x;
	  break;
	}
    }
}
// push the elemnts(nodes) of array
void pushp (int x)
{
  for (int i = 0; i < 20; i++)
    {
      if (bot[i] == -999)
	{
	  bot[i] = x;
	  break;
	}
    }
}

//to calculate the minimum distance from its adjacent nodes
int minDistance (int dist[], bool sptSet[])
{
  int min = INT_MAX, min_index;

  for (int v = 0; v < V; v++)
    if (sptSet[v] == false && dist[v] <= min)
      min = dist[v], min_index = v;

  return min_index;
}

void printPath (int parent[], int j)
{
  if (parent[j] == -1)
    return;
  printPath (parent, parent[j]);
  printf ("%d ", j);
  push (j);
}

int printSolution (int dist[], int n, int parent[], int s, int i)
{
  {
    printf ("\n%d -> %d \t\t %d\t\t%d ", s, i, dist[i], s);

    printPath (parent, i);
  }
}

///to calculate the shortest path by using dijkshtra algorithm
void dijkstra (int graph[V][V], int s, int d)
{
  int dist[V];
  bool sptSet[V];
  int parent[V];

  for (int i = 0; i < V; i++)
    {
      parent[s] = -1;
      dist[i] = INT_MAX;
      sptSet[i] = false;
    }

  dist[s] = 0;

  for (int count = 0; count < V - 1; count++)
    {
      int u = minDistance (dist, sptSet);
      sptSet[u] = true;
      for (int v = 0; v < V; v++)
	if (!sptSet[v] && graph[u][v] && dist[u] + graph[u][v] < dist[v])
	  {
	    parent[v] = u;
	    dist[v] = dist[u] + graph[u][v];
	  }
    }
  printSolution (dist, V, parent, s, d);
}

//to find the path from source node to destination node
void find_path (int s, int d, int ob11, int ob12, int ob21, int ob22, int ob31, int ob32)
{
  if (pushvarible >= 1)
    {
      push (pass[6]);
    }
  pushvarible++;
  push (s);
int graph[40][40] = {
    {0, 30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 57, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	//0
    {30, 0, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 48, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	//1
    {0, 22, 0, 20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0},	//2
    {0, 0, 20, 0, 20, 0, 0, 0, 0, 0, 52, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	//3
    {0, 0, 0, 20, 0, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 13, 0, 0, 0, 0, 0, 0, 0},	//4                                                                       
    {0, 0, 0, 0, 22, 0, 30, 0, 48, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 13, 0, 0, 0, 0, 0, 0, 0},	//5
    {0, 0, 0, 0, 0, 30, 0, 57, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 13, 0, 0, 0, 0, 0, 0},	//6
    {0, 0, 0, 0, 0, 0, 57, 0, 33, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 40, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	//7
    {0, 0, 0, 0, 0, 48, 0, 33, 0, 26, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	//8
    {0, 0, 0, 0, 0, 0, 0, 0, 26, 0, 29, 0, 0, 0, 0, 0, 0, 0, 17, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	//9
    {0, 0, 0, 52, 0, 0, 0, 0, 0, 29, 0, 29, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	//10
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 29, 0, 26, 0, 0, 0, 17, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	//11
    {0, 48, 0, 0, 0, 0, 0, 0, 0, 0, 0, 26, 0, 33, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	//12
    {57, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 33, 0, 40, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	//13
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 40, 0, 23, 0, 0, 0, 0, 0, 0, 17,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	//14
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 23, 0, 20, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 13, 0, 0, 0, 0},	//15
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 17, 0, 0, 0, 20, 0, 50, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	//16
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 50, 0, 50, 0, 0, 0, 0,
     37, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	//17
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 17, 0, 0, 0, 0, 0, 0, 0, 50, 0, 20, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	//18
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 20, 0, 23, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 13, 0, 0, 0},	//19
    {0, 0, 0, 0, 0, 0, 0, 40, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 23, 0, 17, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	//20
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 17, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 13, 0, 0},	//21
     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 17, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 13, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 37, 0, 0, 0, 0, 0, 0,
     37, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	//23
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 37,
     0, 60, 60, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	//24
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     60, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 13, 0},	//25
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     60, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 13},	//26
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	//27
    {13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0, 0},	//28
    {0, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	//29
    {0, 0, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	//30
    {0, 0, 0, 0, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	// 31
    {0, 0, 0, 0, 0, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	// 32
    {0, 0, 0, 0, 0, 0, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	//33
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	//34
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	//35
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	// 36
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	//37
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	//38
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	//39
};
 
 graph[ob11][ob12] = 0;
 graph[ob12][ob11] = 0;
 graph[ob21][ob22] = 0;
 graph[ob22][ob21] = 0;
 graph[ob31][ob32] = 0;
 graph[ob32][ob31] = 0;
 
  dijkstra (graph, s, d);
}


//
int check_path (void)
{
  int p = 0, c = 0, n = 1, y = 0;
  while ((n == sizepath) != 1)
    {
      int z = 0;
      z = check_condition (path[p], path[c], path[n]);
      p = c;
      c = n;
      n++;
      pushp (z);
	
    }
}
//array to store the left &right conditions
void find_size_of_bot (void)
{
  sizebot = 00;
  for (int i = 0; i < 29; i++)
    {
      if (bot[i] == -999)
	break;
      sizebot++;
    }
  printf ("\nsize of bot=%d", sizebot);

}

// array to store the sequence of path
void find_size_of_path (void)
{
  sizepath = 00;
  for (int i = 0; i < 40; i++)
    {
      if (path[i] == -999)
	break;
      sizepath++;
    }
  printf ("\nsize of path=%d", sizepath);

}

// to follow the bot from source node to destination node
int followpath(int source, int destination)

{ 
  initialize ();
 find_path (source, destination, pass[0], pass[1], pass[2], pass[3], pass[4],
	     pass[5]);
  find_size_of_path ();
  check_path ();
  find_size_of_bot ();

  for (bot_index = 0; bot_index < sizebot; bot_index++)
  {
	  	  lcd_print(2,4,path[(bot_index)-1],2);
	  	  lcd_print(2,8,path[bot_index],2);
	  	  lcd_print(2,12,path[(bot_index)+1],2);
	  	  

	  		lcd_print(1,4,bot[(bot_index)-1],2);
	  		lcd_print(1,8,bot[bot_index],2);
	  		lcd_print(1,12,bot[(bot_index)+1],2);
			 
	    switch (bot[bot_index])
	  {
		  case 1:
		   //velocity(150,150);
		   //forward();
		   //delay(500);
		  blackline_data();
		  blackline();
		  while(1!=((Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 )))
		  {
			  delay(50);
			  while(1!=((Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 )))
			  {
			//obstacle = check_obstacle();
			  blackline_data();
			  blackline();
		  }
		  }
		  node();
		  delay(50);
		  //stop();
		  //delay(150);
		  printf ("%d\t", 2);
		  break;
		  
		  case 2	:
		    velocity(255,255);
		    forward();
		    delay(280);
		  left();
		  delay(500);
		  while(ADC_Conversion(1) < 0x96);
		  //stop();
		  //delay(100);
		  blackline_data();
		    while(1!=((Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 )))
		    {delay(50);
			    while(1!=((Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 )))
			    {
				    //obstacle = check_obstacle();
				    blackline_data();
				    blackline();
			    }
		    }
			node();
		  delay(50);
		  //stop();
		  //delay(150);
		  break;
		  
		  case 3 :
		  velocity(255,255);
		  forward();
		  delay(280);
		  right();
		  delay(500);
		  while(ADC_Conversion(2) < 0x96);
		  //stop();
		  //delay(100);
		  blackline_data();
		    while(1!=((Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 )))
		    {delay(50);
			    while(1!=((Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 )))
			    {
				    //obstacle = check_obstacle();
				    blackline_data();
				    blackline();
			    }
		    }
			node();
		  delay(50);
		  break;
	case 4:
		   velocity(255,255);
		    right();
			delay(1000);
		   while(ADC_Conversion(2) < 0x96);
		  // stop();
		  node();
		   //delay(150);
		   printf ("%d\t", 2);
		   break;
		   
	case 5:
		    velocity(255,255);
		    forward();
		    delay(380);
		    left();
		    delay(400);
		    while(ADC_Conversion(2) < 0x96);
		    stop();
		    delay(1000);
			node();
			break;
		  
	case 6:
		  velocity(255,255);
		  forward();
		  delay(280);
		  right();
		  delay(350);
		  while(ADC_Conversion(2) < 0x96);
		  stop();
		  delay(1000);
		  node();
		  break;
		  
	case 7:
		   velocity(255,255);
		   forward();
		   delay(300);
		   stop();
		   //delay(500);
		   node();
		   break;
		  
	case 8:
		  
		   right();
		    velocity(255,255);
		   delay(600);
		   while(ADC_Conversion(2) < 0x96);
		   stop();
		   delay(100);
		   node();
		   node();
		   break;
		  
	case 9:
		   velocity(255,255);
		    right();
			delay(1700);
		   while(ADC_Conversion(2) < 0x96);
		   //stop();
		   //delay(150);
		   
		   blackline_data();
		   blackline();
		     while(1!=((Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 )))
		     {delay(50);
			     while(1!=((Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 )))
			     {
				     //obstacle = check_obstacle();
				     blackline_data();
				     blackline();
			     }
		     }
		   node();
		   delay(50);
		   //stop();
		   //delay(150);
		   break;
		   
		case 10:						//lf
		   
		   blackline_data();
		   blackline_lifting();
		     while(1!=((Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 )))
		     {delay(50);
			     while(1!=((Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 )))
			     {
				     //obstacle = check_obstacle();
				     blackline_data();
				     blackline();
			     }
		     }
			delay(30);
		     velocity(150,150);
		     forward();
		     delay(200);
		  	    blackline_data();
		      while(1!=((Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 )))
		      {delay(50);
			      while(1!=((Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 )))
			      {
				      //obstacle = check_obstacle();
				      blackline_data();
				      blackline();
			      }
		      }
			node();
			delay(30);
					
		   break;
		   
	 case 11:					
								//ll
			   velocity(255,255);
			   forward();
			   delay(280);
			   left();
			   delay(500);
			   while(ADC_Conversion(1) < 0x96);
			  blackline_data();
			    while(1!=((Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 )))
			    {delay(50);
				    while(1!=((Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 )))
				    {
					    //obstacle = check_obstacle();
					    blackline_data();
					    blackline();
				    }
			    }
			  node();
			  delay(30);
			  velocity(150,150);
			  forward();
			  delay(100);
			  stop();
			  delay(5000);
			 // DDRK = 0x00;
			
		   break;
		  		  
		  
	case 12 :
		 
		  right();
		  delay(500);
		  while(ADC_Conversion(2) < 0x96);
		  //stop();
		  //delay(100);
		  blackline_data();
		    while(1!=((Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 )))
		    {delay(50);
			    while(1!=((Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 )))
			    {
				    //obstacle = check_obstacle();
				    blackline_data();
				    blackline();
			    }
		    }
		  node();
		  delay(50);
		  //stop();
		  //delay(150);
		  break;
		  
	case 13:
		  
		  left();
		  delay(500);
		  while(ADC_Conversion(2) < 0x96);
		  //stop();
		  //delay(100);
		  blackline_data();
		    while(1!=((Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 )))
		    {delay(50);
			    while(1!=((Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 )))
			    {
				    //obstacle = check_obstacle();
				    blackline_data();
				    blackline();
			    }
		    }
		  node();
		 delay(50);
		  //stop();
		  //delay(150);
		  break;

	case 14:
			velocity(150,150);
			forward();
			delay(700);
			left();
			delay(100);
			while(ADC_Conversion(2) < 0x96);
			stop();
			delay(50);
			blackline_data();
			  while(1!=((Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 )))
			  {delay(50);
				  while(1!=((Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 )))
				  {
					  //obstacle = check_obstacle();
					  blackline_data();
					  blackline();
				  }
			  }
			node();
			stop();
			delay(300);
			break;				  

	case 15:								//lr
			velocity(255,255);
			forward();
			delay(280);
			right();
			delay(500);
			while(ADC_Conversion(3) < 0x96);
			blackline_data();
			  while(1!=((Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 )))
			  {delay(50);
				  while(1!=((Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 )))
				  {
					  //obstacle = check_obstacle();
					  blackline_data();
					  blackline();
				  }
			  }
			node();
			delay(30);
			velocity(150,150);
			forward();
			delay(100);
			stop();
			delay(5000);


			break;

	case 16:								//lr1
			velocity(255,255);
			forward();
			delay(280);
			blackline_data();
			  while(1!=((Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 )))
			  {delay(50);
				  while(1!=((Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 )))
				  {
					  //obstacle = check_obstacle();
					  blackline_data();
					  blackline();
				  }
			  }
			node();
			delay(30);
			forward();
			velocity(150,150);
			blackline_data();
			  while(1!=((Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 )))
			  {
				  delay(50);
				  while(1!=((Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 )))
				  {
					  //obstacle = check_obstacle();
					  blackline_data();
					  blackline();
				  }
			  }
			node();
			delay(30);
			velocity(255,255);
			forward();
			delay(280);
			right();
			while (ADC_Conversion(3) < 0x96);
			blackline_data();
			  while(1!=((Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 )))
			  {delay(50);
				  while(1!=((Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 )))
				  {
					  //obstacle = check_obstacle();
					  blackline_data();
					  blackline();
				  }
			  }
			node();
			delay(30);

			break;

		  
	case 17:								//ll1
		  velocity(150,150);
		  forward();
		  delay(400);
		  left();
		  delay(300);
		  while (ADC_Conversion(1) < 0x96);
		  blackline_data();
		    while(1!=((Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 )))
		    {delay(50);
			    while(1!=((Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 )))
			    {
				    //obstacle = check_obstacle();
				    blackline_data();
				    blackline();
			    }
		    }
					  node();
		  delay(30);
		  
		  break;
		  
		  
	case 18:								//lr
			velocity(255,255);
			forward();
			delay(280);
			right();
			delay(500);
			while(ADC_Conversion(3) < 0x96);
			//stop();
			//delay(100);
			blackline_data();
			  while(1!=((Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 )))
			  {delay(50);
				  while(1!=((Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 )))
				  {
					  //obstacle = check_obstacle();
					  blackline_data();
					  blackline();
				  }
			  }			//node();
			delay(30);
			forward();
			velocity(150,150);
			delay(200);
			blackline_data();
			  while(1!=((Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 )))
			  {delay(50);
				  while(1!=((Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 )))
				  {
					  //obstacle = check_obstacle();
					  blackline_data();
					  blackline();
				  }
			  }
			node();
			delay(30);
			velocity(150,150);
			forward();
			delay(100);
			stop();
			delay(5000);


			break;

		  
default:
		
		  lcd_cursor(1,1);
		  lcd_string("break");
		  printf ("%d\t", 0);
		  break;


	  }
}
}
void get_path (int source, int destination)
{
  pushvarible = 0;
  destination1=destination;
 followpath(source, destination);

    }
  

unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading)
{
	float distance;
	unsigned int distanceInt;
	distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
	distanceInt = (int)distance;
	if(distanceInt>800)
	{
		distanceInt=800;
	}
	return distanceInt;
}

// function to pick the nuts using servo motor
void pick(void)
{
	servo_1(60);
	_delay_ms(20000);
	servo_1_free();
	
	servo_2(90);
	_delay_ms(20000);
	servo_2_free();
	
	servo_1(0);
	_delay_ms(20000);
	//servo_1_free();
	
	servo_2(0);
	_delay_ms(20000);
	servo_2_free();
}
// function to place the nuts using servo motor

void place(void)
{
	servo_2(90);
	_delay_ms(20000);
	servo_2_free();
	servo_1(60);
	_delay_ms(20000);
	servo_1_free();
	servo_2(0);
	_delay_ms(20000);
	servo_2_free();
	servo_1(0);
	_delay_ms(20000);
	servo_1_free();
}

// function to scan the sequence of nuts & pick the last nut


void scan_pick(void)
{
	{
		nut_no = 1;
		velocity(130,130);
		forward();
		delay(900);
		velocity(130,130);
		back_turn();
		stop();
		delay(1300);
		colour_detect();
		rgb_on();
		delay(500);
		lcd_print(1,1,nuts_position[nut_no],1);
		
	}
	
	{
		forward_wls();
		blackline_data();
		nut_no = 2;
		colour_detect();
		rgb_on();
		delay(500);
		lcd_print(1,2,nuts_position[nut_no],1);
	}
	
	{
		forward_wls();
		blackline_data();
		nut_no = 3;
		colour_detect();
		rgb_on();
		delay(500);
		lcd_print(1,3,nuts_position[nut_no],1);
	}
	
	forward_wls();
	
	{
		forward_wls();
		blackline_data();
		nut_no = 4;
		colour_detect();
		rgb_on();
		delay(500);
		lcd_print(1,4,nuts_position[nut_no],1);
		
	}
	
	{
		forward_wls();
		blackline_data();
		nut_no = 5;
		colour_detect();
		rgb_on();
		delay(500);
		lcd_print(1,5,nuts_position[nut_no],1);
		
	}
	
	{
		forward_wls();
		blackline_data();
		nut_no = 6;
		colour_detect();
		rgb_on();
		delay(500);
		lcd_print(1,6,nuts_position[nut_no],1);
		
	}
	
	velocity(150,150);
	forward();
	delay(400);
	
	right_turn();
	pick();
}

// to turn the bot in left direction
void left_turn()
	{
	left();
	delay(300);
	while (ADC_Conversion(2) < 0x96);
	stop();
	}



// to move the bot in forward direction according to white line sensor
void forward_wls(void)
{
	blackline_data();
	while(1!=((Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 )))
	{

		blackline_data();
		blackline();
	}
	//stop();
	node();
	delay(30);
	
}
// to move the bot in backward direction according to white line sensor
void backward_wls(void)
{
	while(1!=((Left_white_line > 0x96 && Center_white_line > 0x96) || (Right_white_line > 0x96 && Center_white_line > 0x96) || (Left_white_line > 0x96 && Center_white_line > 0x96 && Right_white_line >  0x96 )))
	{
		blackline_data();
		back();
	}
	stop();
	delay(150);
	
}
//// to turn the bot in right direction according to white line sensor

void right_turn(void)
{
	right();
	delay(100);
	while(ADC_Conversion(2) < 0x96);
	stop();
	delay(100);
		
}

// to detect the obstacle using sharp sensor & follow another path
int check_obstacle()
	{
		sharp = ADC_Conversion(8);
		print_sensor(1,1,8);
		
		if(sharp >= 250)
			{
				stop();
				delay(100);
				buzzer_on();
				delay(500);
				buzzer_off();
				obstacle++;
				char str1[2] = {0};
				itoa(obstacle, str1 ,10);
				lcd_cursor(2,1);
				lcd_string(str1);
				back_turn();
				
				if (obstacle == 1)
				{
					lcd_cursor(1,5);
					lcd_string("ob1_detect");
					 pass[0] = path[(bot_index)+1];
					 pass[1] = path[(bot_index)];
					 pass[6] = path[(bot_index)+1];
					 lcd_print(2,15,pass[1],2);
					followpath (pass[1], destination1);
					return(0);
				}
				if (obstacle == 2)
				{
					 pass[2] = path[(bot_index)+1];
					 pass[3] = path[(bot_index)];
					 pass[6] = path[(bot_index)+1];
					followpath(pass[3], destination1);
 					return(0);
				}

				if (obstacle == 3)
				{ 
					pass[4] = path[(bot_index)+1];
					pass[5] = path[(bot_index)];
					pass[6] = path[(bot_index)+1];
					followpath(pass[5], destination1);
					return(0);
				}
			}

	}
 /*
*
*function name:check_one
*input:none
*output:none
*logic:check the color of the nuts at given location and find the destination path
*
*/
void check_one(void)
{
  if (nuts_position[6] == 1)
  {
    pick();
    get_path(nuts_position[6],S3);
    place();
    get_path(S3,nuts_position[5]);
    S3 = deposit_zon_S2[1];
    Redindication++;
  }
  /*else if (nuts_position[6] == 2)
  {
    return(0);
  }*/
  else if (nuts_position[6] == 3)
  {
    pick();
    get_path(nuts_position[6],S1);
    
    place();
    get_path(S1,nuts_position[5]);
    S1 = deposit_zon_S1[2];
    greenindication++;
  }
  else if (nuts_position[6] == 0)
  {
    if (nuts_position[5] == 1)
    {
        
        get_path(nuts_position[6],nuts_position[5]);
        pick();
        get_path(nuts_position[5],S3);
          place();
          get_path(S3,nuts_position[4]);
          S3 = deposit_zon_S2[1];
          Redindication++;
    }
    /*else if (nuts_position[5] == 2)
    {
      return(0);

    }*/
    else if (nuts_position[5] == 3)
    {
      get_path(nuts_position[6],nuts_position[5]);
      pick();
      get_path(nuts_position[5],S1);
        place();
        get_path(S1,nuts_position[4]);
        S1 = deposit_zon_S1[2];
        greenindication++;
    }
    else if (nuts_position[5] == 0)
    {
      if (nuts_position[4] == 1)
      {
        get_path(nuts_position[6],nuts_position[4]);
        pick();
        get_path(nuts_position[4],S3);
          place();
          get_path(S3,nuts_position[3]);
          S3  = deposit_zon_S2[1];
          Redindication++;
      }
      if (nuts_position[4] == 3)
      {
        get_path(nuts_position[6],nuts_position[4]);
        pick();
        get_path(nuts_position[4],S1);
          place();
          get_path(S1,nuts_position[3]);
          S1 = deposit_zon_S1[2];
          greenindication++;
      }
      
    



    }
  }

  
}   
    
  /*
*
*function name:check_two
*input:none
*output:none
*logic:check the color of the nuts at given location and find the destination path
*
*/    
void check_two(void)
{
    if (nuts_position[5] == 1)
  {
    pick();
    get_path(nuts_position[5],S3);
    place();
    get_path(S3,nuts_position[4]);
    
    
      if(Redindication == 1)
    {
    S3 = deposit_zon_S2[2];
    Redindication++;
    }
    
    
    
  }
  /*else if (nuts_position[6] == 2)
  {
    return(0);
  }*/
  else if (nuts_position[5] == 3)
  {
    pick();
    get_path(nuts_position[5],S1);
    place();
    get_path(S1,nuts_position[4]);
    if(greenindication ==1)
    { 
    S1 = deposit_zon_S1[2];
    greenindication++;
    }
  }
  else if (nuts_position[5] == 0)
  {
    if (nuts_position[4] == 1)
    {
        
        get_path(nuts_position[5],nuts_position[4]);
        pick();
        get_path(nuts_position[4],S3);
          place();
          get_path(S3,nuts_position[3]);
           if(Redindication == 1)
           {
             S3 = deposit_zon_S2[2];
             Redindication++;
           }
      
    }
    /*else if (nuts_position[5] == 2)
    {
      return(0);

    }*/
    else if (nuts_position[4] == 3)
    {
      get_path(nuts_position[5],nuts_position[4]);
      pick();
      get_path(nuts_position[4],S1);
        place();
        get_path(S1,nuts_position[3]);
        if(greenindication ==1)
        {
          S1 = deposit_zon_S1[2];
          greenindication++;
        } 
    }
        
        
    else if (nuts_position[4] == 0)
    {
      if (nuts_position[3] == 1)
      {
        get_path(nuts_position[5],nuts_position[3]);
        pick();
        get_path(nuts_position[3],S3);
          place();
          get_path(S3,nuts_position[2]);
                 if(Redindication == 1)
                 {
                   S3 = deposit_zon_S2[2];
                   Redindication++;
                 }
      }
      if (nuts_position[3] == 3)
      {
        get_path(nuts_position[5],nuts_position[3]);
        pick();
        get_path(nuts_position[3],S1);
          place();
          get_path(S1,nuts_position[2]);
          if(greenindication ==1)
          {
            S1 = deposit_zon_S1[2];
            greenindication++;
          }   
      }
          
    



    }
  }

  
}   


  /*
*
*function name:check_three
*input:none
*output:none
*logic:check the color of the nuts at given location and find the destination path
*
*/

void check_three(void)
{
    if (nuts_position[4] == 1)
  {
    pick();
    get_path(nuts_position[4],S3);
    place();
    get_path(S3,nuts_position[3]);
    
    
      if(Redindication == 1)
    {
    S3 = deposit_zon_S2[1];
    Redindication++;
    }
    if(Redindication == 2)
    {
      S3 = deposit_zon_S2[2];
      Redindication++;
    }
    
    
    
  }
  /*else if (nuts_position[6] == 2)
  {
    return(0);
  }*/
  else if (nuts_position[4] == 3)
  {
    pick();
    get_path(nuts_position[4],S1);
    place();
    get_path(S1,nuts_position[3]);
    if(greenindication ==1)
    { 
    S1 = deposit_zon_S1[2];
    greenindication++;
    }
    
  }
  else if (nuts_position[4] == 0)
  {
    if (nuts_position[3] == 1)
    {
        
        get_path(nuts_position[4],nuts_position[3]);
        pick();
        get_path(nuts_position[3],S3);
          place();
          get_path(S3,nuts_position[2]);
           if(Redindication == 1)
           {
             S3 = deposit_zon_S2[2];
             Redindication++;
           }
            if(Redindication == 2)
            {
              S3 = deposit_zon_S2[1];
              Redindication++;
            }
      
    }
    /*else if (nuts_position[5] == 2)
    {
      return(0);

    }*/
    else if (nuts_position[3] == 3)
    {
      get_path(nuts_position[4],nuts_position[3]);
      pick();
      get_path(nuts_position[3],S1);
        place();
        get_path(S1,nuts_position[2]);
        if(greenindication ==1)
        {
          S1 = deposit_zon_S1[2];
          greenindication++;
        } 
    }
        
        
    else if (nuts_position[3] == 0)
    {
      if (nuts_position[2] == 1)
      {
        get_path(nuts_position[3],nuts_position[2]);
        pick();
        get_path(nuts_position[2],S3);
          place();
          get_path(S3,nuts_position[1]);
                 if(Redindication == 1)
                 {
                   S3 = deposit_zon_S2[2];
                   Redindication++;
                 }
                  if(Redindication == 2)
                  {
                    S3 = deposit_zon_S2[1];
                    Redindication++;
                  }
      }
      if (nuts_position[2] == 3)
      {
        get_path(nuts_position[3],nuts_position[2]);
        pick();
        get_path(nuts_position[2],S1);
          place();
        
          if(greenindication ==1)
          {
            S1 = deposit_zon_S1[2];
            greenindication++;
          }   
      }
          
    



    }
  }

  
}   


  /*
*
*function name:check_four
*input:none
*output:none
*logic:check the color of the nuts at given location and find the destination path
*
*/


void check_four(void)
{
    if (nuts_position[3] == 1)
  {
    pick();
    get_path(nuts_position[3],S3);
    place();
    get_path(S3,nuts_position[2]);
    
    
      if(Redindication == 1)
    {
    S3 = deposit_zon_S2[2];
    Redindication++;
    }
    if(Redindication == 2)
    {
      S3 = deposit_zon_S2[1];
      Redindication++;
    }
    
    
    
  }
  /*else if (nuts_position[6] == 2)
  {
    return(0);
  }*/
  else if (nuts_position[3] == 3)
  {
    pick();
    get_path(nuts_position[3],S1);
    place();
    get_path(S1,nuts_position[2]);
    if(greenindication ==1)
    { 
    S1 = deposit_zon_S1[2];
    greenindication++;
    }
    
  }
  else if (nuts_position[3] == 0)
  {
    if (nuts_position[2] == 1)
    {
        
        get_path(nuts_position[4],nuts_position[2]);
        pick();
        get_path(nuts_position[2],S3);
          place();
          get_path(S3,nuts_position[1]);
           if(Redindication == 1)
           {
             S3 = deposit_zon_S2[2];
             Redindication++;
           }
            if(Redindication == 2)
            {
              S3 = deposit_zon_S2[1];
              Redindication++;
            }
      
    }
    /*else if (nuts_position[5] == 2)
    {
      return(0);

    }*/
    else if (nuts_position[2] == 3)
    {
      get_path(nuts_position[4],nuts_position[2]);
      pick();
      get_path(nuts_position[2],S1);
        place();
        get_path(S1,nuts_position[1]);
        if(greenindication ==1)
        {
          S1 = deposit_zon_S1[2];
          greenindication++;
        } 
    }
        
        
    else if (nuts_position[2] == 0)
    {
      if (nuts_position[1] == 1)
      {
        get_path(nuts_position[2],nuts_position[1]);
        pick();
        get_path(nuts_position[1],S3);
          place();
          get_path(S3,start);
          buzz_blink();
                
      }
      if (nuts_position[1] == 3)
      {
        get_path(nuts_position[2],nuts_position[1]);
        pick();
        get_path(nuts_position[1],S1);
          place();
          get_path(S1,start);
          buzz_blink();
          
      }
          
    



    }
  }

  
}   


  /*
*
*function name:check_five
*input:none
*output:none
*logic:check the color of the nuts at given location and find the destination path
*
*/



void check_five(void)
{
    if (nuts_position[2] == 1)
  {
    pick();
    get_path(nuts_position[2],S3);
    place();
    get_path(S3,nuts_position[1]);
    
    
      if(Redindication == 1)
    {
    S3 = deposit_zon_S2[2];
    Redindication++;
    }
    if(Redindication == 2)
    {
      S3 = deposit_zon_S2[1];
      Redindication++;
    }
    
    
    
  }
  /*else if (nuts_position[6] == 2)
  {
    return(0);
  }*/
  else if (nuts_position[2] == 3)
  {
    pick();
    get_path(nuts_position[2],S1);
    place();
    get_path(S1,nuts_position[1]);
    if(greenindication ==1)
    { 
    S1 = deposit_zon_S1[2];
    greenindication++;
    }
    
  }
  else if (nuts_position[2] == 0)
  {
    if (nuts_position[1] == 1)
    {
        
        //get_path(nuts_position[4],nuts_position[2]);
        pick();
        get_path(nuts_position[1],S3);
          place();
          get_path(S3,start);
          buzz_blink();
          
    }
    /*else if (nuts_position[5] == 2)
    {
      return(0);

    }*/
    else if (nuts_position[1] == 3)
    {
      //get_path(nuts_position[4],nuts_position[2]);
      pick();
      get_path(nuts_position[1],S1);
        place();
        get_path(S1,start);
        buzz_blink();
        
    }
        
        
    else if (nuts_position[1] == 0)
    {
    
        get_path(nuts_position[1],start);
        buzz_blink();   
  
    }
  }

  
}   

  /*
*
*function name:check_six
*input:none
*output:none
*logic:check the color of the nuts at given location and find the destination path
*
*/


void check_six(void)
{
    if (nuts_position[1] == 1)
  {
    pick();
    get_path(nuts_position[1],S3);
    place();
    get_path(S3,start);
    buzz_blink();
    
  }
  else if (nuts_position[1] == 2)
  {
  }
    
  else if (nuts_position[1] == 3)
  {
    pick();
    get_path(nuts_position[1],S1);
    place();
    get_path(S1,start);
    buzz_blink();
  }
  else if (nuts_position[1] == 0)
  {
        get_path(nuts_position[1],start);
        buzz_blink();
  
     } 
  }
	 
int main (void)
{	
	
	init_devices();
	lcd_init();
	color_sensor_scaling();
	delay(3000);
	servo_1(60);
	servo_1_free();
	get_path(start,nuts_position[1]);
check_one();
check_two();
check_three();
check_four();
check_five();
check_six();
}
