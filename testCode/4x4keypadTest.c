#define F_CPU 16000000
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define BIT_SET(a,b) ((a) |= (1<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1<<(b)))
#define BIT_TOGGLE(a,b) ((a) ^= (1<<(b)))
#define BIT_CHECK(a,b) (!!((a) & (1<<(b))))

#define DEBOUNCING_DELAY 40


unsigned char input_number;
unsigned char input_signal = 0;
unsigned char A_input = 1; 
unsigned char B_input = 2;
unsigned char C_input = 3;
unsigned char D_input = 4;
unsigned char pound_input = 5;
unsigned char star_input = 6;
unsigned char input_number_count = 0;

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

void test(void)// PB4~PB7 => C1~C4 PB0~PD3 => R4~R1
{
	PORTB = ~0x01;
	_delay_ms(DEBOUNCING_DELAY);
	if(!(PINB&0x10)) input_signal = A_input; // A버튼을 눌렀을 때
	if(!(PINB&0x20)) input_number = 3;
	if(!(PINB&0x40)) input_number = 2;
	if(!(PINB&0x80)) input_number = 1; 

    PORTB = ~0x02;
	_delay_ms(DEBOUNCING_DELAY);
	if(!(PINB&0x10)) input_signal = B_input; // B버튼을 눌렀을 떄
	if(!(PINB&0x20))
	{
		input_number = 6;
		input_number_count++;
	}
	if(!(PINB&0x40))
	{
		input_number = 5;
		input_number_count++;
	}
	if(!(PINB&0x80))
	{
		input_number = 4;
		input_number_count++;
	}

	
	PORTB = ~0x04;
	_delay_ms(DEBOUNCING_DELAY);
	if(!(PINB&0x10)) input_signal = C_input; // C버튼을 눌렀을 때
	if(!(PINB&0x20))
	{
		input_number = 9;
		input_number_count++;
	}
	if(!(PINB&0x40))
	{
		input_number = 8;
		input_number_count++;
	}
	if(!(PINB&0x80))
	{
		input_number = 7;
		input_number_count++;
	}

	
    PORTB = ~0x08;
	_delay_ms(DEBOUNCING_DELAY);
	if(!(PINB&0x10)) input_signal = D_input;// D버튼을 눌렀을 때
	if(!(PINB&0x20)) input_signal = pound_input; // 우물정자를 눌렀을 때
	if(!(PINB&0x40))
	{
		input_number = 0;
		input_number_count++;
	}
	if(!(PINB&0x80)) input_signal = star_input; // 별버튼을 눌럿을 때 
}

int main(void)
{
   DDRB = 0x0f;
   lcd_pic();
   lcd_mv();
   lcd_func();
   lcd_clear();
   _delay_ms(10);
   sei();
   
    while (1) 
    {
		test();
		lcd_data(0,0); lcd_read(input_number + '0'); 
		lcd_data(1,0); lcd_read(input_signal + '0');lcd_data(1,4); lcd_read(input_number_count+'0');
		
    }
}

