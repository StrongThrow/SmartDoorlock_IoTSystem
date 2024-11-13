#define F_CPU 16000000
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define DEBOUNCING_DELAY 20

unsigned char x, y = 10;
unsigned char input = 1;
unsigned char lcd_state = 0;
unsigned char sww;


void lcd_func(void) //Function Set
{
	PORTG = 0x01;
	PORTA = 0x38;
	_delay_us(39);
	PORTG = 0x00;
}

void lcd_pic(void) //Display On/Off Control
{
	PORTG = 0x01;
	PORTA = 0x0c;
	_delay_us(39);
	PORTG = 0x00;
}

void lcd_mv(void) //Entry Mode Set
{
	PORTG = 0x01;
	PORTA = 0x06;
	_delay_us(39);
	PORTG = 0x00;
}

void lcd_clear(void)
{
	PORTG = 0x01;
	PORTA = 0x01;
	_delay_ms(2);
	PORTG = 0x00;
}

void lcd_data(unsigned char a, unsigned char b)
{
	unsigned char sw;
	sw = 0x00;
	if(a == 1) sw = 0xc0;
	PORTG = 0x01;
	PORTA = (0x80|b|sw);
	_delay_us(39);
	PORTG = 0x00;
}

void lcd_read(unsigned char a)
{
	PORTG = 0x05;
	PORTA = a;
	_delay_us(43);
	PORTG = 0x04;
}

void menu(void)
{
	if (lcd_state == 0)
	{
		if (y == 11)
		{
			lcd_data(0,0); lcd_read(' '); lcd_read('1'); lcd_read('.');   lcd_read('S');  lcd_read('E');  lcd_read('T'); lcd_read('T'); lcd_read('I'); lcd_read('N'); lcd_read('G'); lcd_read('S'); lcd_data(0,14); lcd_read(' ');
			lcd_data(1,0); lcd_read('>'); lcd_read('2'); lcd_read('.');   lcd_read('T');  lcd_read('E');  lcd_read('M'); lcd_read('P'); lcd_read('&'); lcd_read('H'); lcd_read('U'); lcd_read('M'); lcd_data(1,14); lcd_read(' ');
			if (input == 2)
			{
				lcd_state = 2;
				y = 10;
				input = 1;
			}	
		}
		if (y == 10)
		{
			lcd_data(0,0); lcd_read('>'); lcd_read('1'); lcd_read('.');   lcd_read('S');  lcd_read('E');  lcd_read('T'); lcd_read('T'); lcd_read('I'); lcd_read('N'); lcd_read('G'); lcd_read('S'); lcd_data(0,14); lcd_read(' ');
			lcd_data(1,0); lcd_read(' '); lcd_read('2'); lcd_read('.');   lcd_read('T');  lcd_read('E');  lcd_read('M'); lcd_read('P'); lcd_read('&'); lcd_read('H'); lcd_read('U'); lcd_read('M'); lcd_data(1,14); lcd_read(' ');
			if (input == 2)
			{
				lcd_state = 1;
				y = 10;
				input = 1;
			}
		}
		if(y > 11)
		{
			y = 11;
		}
		if(y < 10)
		{
			y = 10;
		}
	}
	if (lcd_state == 1)
	{
		
		if (y == 10)
		{
			lcd_data(0,0); lcd_read('>'); lcd_read('A'); lcd_read('I');   lcd_read('R');  lcd_read('C');  lcd_read('O'); lcd_read('N'); lcd_read(' '); lcd_read('A'); lcd_read('U'); lcd_read('T'); lcd_read('O'); lcd_read(' '); lcd_read(' ');  lcd_read(' ');  lcd_read(' ');
			lcd_data(1,0); lcd_read(' '); lcd_read('B'); lcd_read('L');   lcd_read('I');  lcd_read('N');  lcd_read('D'); lcd_read(' '); lcd_read('O'); lcd_read('N'); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' ');  lcd_read(' ');  lcd_read(' ');
		}
		if (y == 11)
		{
			lcd_data(0,0); lcd_read(' '); lcd_read('A'); lcd_read('I');   lcd_read('R');  lcd_read('C');  lcd_read('O'); lcd_read('N'); lcd_read(' '); lcd_read('A'); lcd_read('U'); lcd_read('T'); lcd_read('O'); lcd_read(' '); lcd_read(' ');  lcd_read(' ');  lcd_read(' ');
			lcd_data(1,0); lcd_read('>'); lcd_read('B'); lcd_read('L');   lcd_read('I');  lcd_read('N');  lcd_read('D'); lcd_read(' '); lcd_read('O'); lcd_read('N'); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' ');  lcd_read(' ');  lcd_read(' ');
		}
		if (y == 12)
		{
			lcd_data(0,0); lcd_read('>'); lcd_read('M'); lcd_read('A');   lcd_read('I');  lcd_read('L');  lcd_read(' '); lcd_read('A'); lcd_read('L'); lcd_read('A'); lcd_read('R'); lcd_read('M'); lcd_read(' '); lcd_read('O'); lcd_read('N');  lcd_read(' ');  lcd_read(' ');
			lcd_data(1,0); lcd_read(' '); lcd_read('G'); lcd_read('A');   lcd_read('S');  lcd_read(' ');  lcd_read('S'); lcd_read('E'); lcd_read('N'); lcd_read('S'); lcd_read('O'); lcd_read('R'); lcd_read(' '); lcd_read('O'); lcd_read('N');  lcd_read(' ');  lcd_read(' ');
		}
		if (y == 13)
		{
			lcd_data(0,0); lcd_read(' '); lcd_read('M'); lcd_read('A');   lcd_read('I');  lcd_read('L');  lcd_read(' '); lcd_read('A'); lcd_read('L'); lcd_read('A'); lcd_read('R'); lcd_read('M'); lcd_read(' '); lcd_read('O'); lcd_read('N');  lcd_read(' ');  lcd_read(' ');
			lcd_data(1,0); lcd_read('>'); lcd_read('G'); lcd_read('A');   lcd_read('S');  lcd_read(' ');  lcd_read('S'); lcd_read('E'); lcd_read('N'); lcd_read('S'); lcd_read('O'); lcd_read('R'); lcd_read(' '); lcd_read('O'); lcd_read('N');  lcd_read(' ');  lcd_read(' ');
		}
		if(y == 14)
		{
			lcd_data(0,0); lcd_read('>'); lcd_read('H'); lcd_read('U');   lcd_read('M');  lcd_read('I');  lcd_read('D'); lcd_read('I'); lcd_read('D'); lcd_read('F'); lcd_read('I'); lcd_read('E'); lcd_read('R'); lcd_read(' '); lcd_read('O'); lcd_read('N');  lcd_read(' ');
			lcd_data(1,0); lcd_read(' '); lcd_read('S'); lcd_read('E');   lcd_read('C');  lcd_read('U');  lcd_read('R'); lcd_read('I'); lcd_read('T'); lcd_read('Y'); lcd_read(' '); lcd_read('O'); lcd_read('N'); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' ');
		}
		if (y == 15)
		{
			lcd_data(0,0); lcd_read(' '); lcd_read('H'); lcd_read('U');   lcd_read('M');  lcd_read('I');  lcd_read('D'); lcd_read('I'); lcd_read('D'); lcd_read('F'); lcd_read('I'); lcd_read('E'); lcd_read('R'); lcd_read(' '); lcd_read('O'); lcd_read('N');  lcd_read(' ');
			lcd_data(1,0); lcd_read('>'); lcd_read('S'); lcd_read('E');   lcd_read('C');  lcd_read('U');  lcd_read('R'); lcd_read('I'); lcd_read('T'); lcd_read('Y'); lcd_read(' '); lcd_read('O'); lcd_read('N'); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' ');
		}
		if(y == 16)
		{
			lcd_data(0,0); lcd_read('>'); lcd_read('W'); lcd_read('A');   lcd_read('T');  lcd_read('E');  lcd_read('R'); lcd_read('S'); lcd_read('E'); lcd_read('N'); lcd_read('S'); lcd_read('O'); lcd_read('R'); lcd_read(' '); lcd_read('O'); lcd_read('N'); lcd_read(' ');
			lcd_data(1,0); lcd_read(' '); lcd_read('C'); lcd_read('O');   lcd_read('N');  lcd_read('N');  lcd_read('E'); lcd_read('C'); lcd_read('T'); lcd_read(' '); lcd_read('O'); lcd_read('N'); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' ');
		}
		if (y == 17)
		{
			lcd_data(0,0); lcd_read(' '); lcd_read('W'); lcd_read('A');   lcd_read('T');  lcd_read('E');  lcd_read('R'); lcd_read('S'); lcd_read('E'); lcd_read('N'); lcd_read('S'); lcd_read('O'); lcd_read('R'); lcd_read(' '); lcd_read('O'); lcd_read('N'); lcd_read(' ');
			lcd_data(1,0); lcd_read('>'); lcd_read('C'); lcd_read('O');   lcd_read('N');  lcd_read('N');  lcd_read('E'); lcd_read('C'); lcd_read('T'); lcd_read(' '); lcd_read('O'); lcd_read('N'); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' ');
		}
		if (y == 18)
		{
			lcd_data(0,0); lcd_read('>'); lcd_read('B'); lcd_read('A');   lcd_read('C');  lcd_read('K');  lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' ');
			lcd_data(1,0); lcd_read(' '); lcd_read(' '); lcd_read(' ');   lcd_read(' ');  lcd_read(' ');  lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' ');
			if (input == 2)
			{
				lcd_state = 0;
				y = 10;
				input = 1;
				lcd_data(0,0); lcd_read('>'); lcd_read('1'); lcd_read('.');   lcd_read('S');  lcd_read('E');  lcd_read('T'); lcd_read('T'); lcd_read('I'); lcd_read('N'); lcd_read('G'); lcd_read('S'); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' ');
				lcd_data(1,0); lcd_read(' '); lcd_read('2'); lcd_read('.');   lcd_read('T');  lcd_read('E');  lcd_read('M'); lcd_read('P'); lcd_read('&'); lcd_read('H'); lcd_read('U'); lcd_read('M'); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' ');
			}
		}
		if (y > 18)
		{
			y = 18;
		}
		if (y < 10)
		{
			y = 10;
		}
		
	}
}



void pind_3_input(void) 
{
	while((sww = ~PIND & 0x1f) == 0);
	_delay_ms(DEBOUNCING_DELAY);
	while(~PIND & 0x1f);
	_delay_ms(DEBOUNCING_DELAY);
	if (sww == 0x04)
	{
		y--;
	}
	if (sww == 0x08)
	{
		y++;
	}
	if (sww == 0x10)
	{
		input++;
	}
}

int main(void)
{
	DDRD = 0x00;
	DDRB = 0xff;
	DDRG = 0x07;
	DDRA = 0xff;
	sei();
	lcd_pic();
	lcd_mv();
	lcd_func();
	lcd_clear();
	_delay_ms(10);
	
	while (1)
	{
		
		menu();
		pind_3_input();
	}
}