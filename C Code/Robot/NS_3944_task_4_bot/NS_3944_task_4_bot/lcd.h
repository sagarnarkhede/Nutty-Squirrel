
/*
* Team Id     : eYRC#NS3944
* Author List : Sagar Narkhede , Omkar Sutar, Pooja Katkar, Ashlesha Borade 
* Filename    :ROBOT_#NS3944
* Theme       : Nutty Squirrel(eYRC)
*Filename	  :	lcd.h
*Functions    :lcd_port_config (void),unsigned char Busy_wait(),print_sensor(char row, char coloumn,unsigned char channel),
              lcd_wr_command(unsigned char data),lcd_wr_char(unsigned char data),lcd_init(),lcd_cursor (char row, char column),lcd_string(char *str),lcd_home(),getPow(unsigned char value, unsigned char multiplier),
              getDigits(const unsigned long int number, unsigned char digit),lcd_print (char row, char coloumn, unsigned long int value, int digits),
			  
*/



#define sbit(reg,bit)		reg |= (1<<bit)						// Macro defined for Setting a bit of any register.
#define cbit(reg,bit)		reg &= ~(1<<bit)					// Macro defined for Clearing a bit of any register.
#define lcd_port 			PORTC
#define RS 0
#define RW 1
#define EN 2
#define cut_off 20


#define DATA_PORT()			sbit(lcd_port,RS)
#define	COMMAND_PORT()		cbit(lcd_port,RS)
#define READ_DATA()			sbit(lcd_port,RW)
#define WRITE_DATA()		cbit(lcd_port,RW)
#define EN_HI()				sbit(lcd_port,EN)
#define EN_LW()				cbit(lcd_port,EN)

unsigned int temp;
unsigned int unit;
unsigned int tens;
unsigned int hundred;
unsigned int thousand;
unsigned int million;
unsigned char ADC_Value;




//Function to configure LCD port
void lcd_port_config (void)
{
	DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
	PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}


// ==============================================================================================================================
//	Function 							: Busy_wait()
//	Return type							: unsigned char
//	Parameters							: none
//	Description 						: This function checks busy flag of LCD and returns zero on success (PORTD7 busy flag pin)
//
// ==============================================================================================================================

unsigned char Busy_wait()
{
	unsigned char temp = 0;
	EN_LW();
	COMMAND_PORT();
	READ_DATA();

	PORTC &= 0x0F;
	DDRC &= 0x0F;
	PORTC |= 0x00;
	
	do{
		EN_HI();
		EN_LW();
		EN_HI();
		EN_LW();
		temp = PORTC;
	}
	while((temp & 0x80) == 0x80);											// wait till PORTD7 is high
	
	EN_LW();
	WRITE_DATA();
	DDRC &= 0x08;
	DDRC |= 0xF7;
	return 0;
}

// ==============================================================================================================================
//	Function 							: lcd_wr_command()
//	Return type							: void
//	Parameters							: unsigned char
//	Description 						: function to send command to LCD
//
// ==============================================================================================================================

//Function To Print Sensor Values At Desired Row And Coloumn Location on LCD
void print_sensor(char row, char coloumn,unsigned char channel)
{
	
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}


void lcd_wr_command(unsigned char data)
{
	unsigned char temp = 0;
	EN_LW();
	COMMAND_PORT();
	WRITE_DATA();
	
	temp = data;
	PORTC &= 0x0F;
	PORTC |= (temp & 0xF0);
	
	EN_HI();
	EN_LW();
	
	temp = data & 0x0F;
	PORTC &= 0x0F;
	PORTC |= (temp) << 4;
	
	EN_HI();
	EN_LW();

	while(Busy_wait());

	_delay_ms(5);
	
}

// ==============================================================================================================================
//	Function 							: lcd_wr_char()
//	Return type							: void
//	Parameters							: unsigned char
//	Description 						: function to write data on LCD
//
// ==============================================================================================================================//

void lcd_wr_char(unsigned char data)
{
	
	unsigned char temp = 0;
	EN_LW();
	DATA_PORT();
	WRITE_DATA();

	temp = data;
	PORTC &= 0x0F;
	PORTC |= (temp & 0xF0);

	EN_HI();
	EN_LW();
	
	temp = data & 0x0F;
	PORTC &= 0x0F;
	PORTC |= (temp) << 4;
	
	EN_HI();
	EN_LW();
	
	_delay_ms(1);
	
}

// ==============================================================================================================================
//	Function 							: lcd_init()
//	Return type							: void
//	Parameters							: void
//	Description 						: Function to initialise LCD
//
// ==============================================================================================================================

void lcd_init()
{
	lcd_wr_command(0x20);
	lcd_wr_command(0x28);
	lcd_wr_command(0x0C);
	lcd_wr_command(0x06);
	lcd_wr_command(0x01);
}

// ==============================================================================================================================
//	Function 							: lcd_cursor()
//	Return type							: void
//	Parameters							: char , char
//	Description 						: Function to set LCD cursor
//
// ==============================================================================================================================

void lcd_cursor (char row, char column)
{
	switch (row) {
		case 1: lcd_wr_command (0x80 + column - 1); break;
		case 2: lcd_wr_command (0xc0 + column - 1); break;
		case 3: lcd_wr_command (0x94 + column - 1); break;
		case 4: lcd_wr_command (0xd4 + column - 1); break;
		default: break;
	}
}

// ==============================================================================================================================
//	Function 							: lcd_string()
//	Return type							: void
//	Parameters							: char pointer
//	Description 						: Function to send string on LCD
//
// ==============================================================================================================================

void lcd_string(char *str)
{
	while(*str != '\0')
	{
		lcd_wr_char(*str);
		str++;
	}
}

// ==============================================================================================================================
//	Function 							: lcd_home()
//	Return type							: void
//	Parameters							: void
//	Description 						: Function to set LCD to home position
//
// ==============================================================================================================================

void lcd_home()
{
	lcd_wr_command(0x80);
}

// ==============================================================================================================================
//	Function 							: lcd_clear()
//	Return type							: void
//	Parameters							: void
//	Description 						: Function to clear LCD screen
//
// ==============================================================================================================================

void lcd_clear()
{
	lcd_wr_command(0x01);
}

// ==============================================================================================================================
//	Function 							: getPow()
//	Return type							: unsigned long int
//	Parameters							: unsigned char, unsigned char
//	Description 						: Function to get power value
//
// ==============================================================================================================================

unsigned long int getPow(unsigned char value, unsigned char multiplier)
{
	unsigned long int val = value;
	
	if(multiplier == 0)
	{
		return val = 1;
	}
	for(char i = multiplier - 1;i >= 1; i--)
	{
		val = val * value;
	}
	return val;
}

// ==============================================================================================================================
//	Function 							: getDigits()
//	Return type							: unsigned char
//	Parameters							: unsigned long int, unsigned char
//	Description 						: Function to extract each digit from number
//
// ==============================================================================================================================

char getDigits(const unsigned long int number, unsigned char digit)
{
	
	unsigned long int c = 0;

	if (digit >= 0)
	{
		c = getPow(10,(digit - 1));
		
		return (char)((number / c) % 10) + 48;
	}
	else
	{
		return 0;
	}
}

// ==============================================================================================================================
//	Function 							: lcd_print()
//	Return type							: void
//	Parameters							: char ,char ,unsigned int ,int
//	Description 						: Function to printf numbers on LCD
//	Note 								: User can print maximum 0 to 4294C87295 numbers on LCD
// ==============================================================================================================================

void lcd_print (char row, char coloumn, unsigned long int value, int digits)
{
	unsigned char i=0;
	
	if(row==0||coloumn==0)
	{
		lcd_home();
	}
	else
	{
		lcd_cursor(row,coloumn);
	}
	
	if(digits > 10)
	{
		lcd_wr_char('X');										// Indicates digits limit has exceeded
	}
	else
	{
		for(i = digits;i >= 1; i--)
		{
			lcd_wr_char(getDigits(value,i));
		}
	}
}
