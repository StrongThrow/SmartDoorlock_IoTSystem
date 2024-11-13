#define F_CPU 14745600UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

int g;
int h;
int data[5][8];
int RH_integral;
int RH_decimal;
int T_integral;
int T_decimal;
int parity;

unsigned char T_10; // 온도의 십의자리
unsigned char T_01; // 온도의 일의자리
unsigned char T_0p1;// 온도의 소수점 첫번째 자리
unsigned char T_0p01;//온도의 소수점 두번째 자리

unsigned char H_10; // 습도의 십의자리
unsigned char H_01; // 습도의 일의자리
unsigned char H_0p1;// 습도의 소수점 첫번째 자리
unsigned char H_0p01;//습도의 소수점 두번째 자리

void send_signal()
{
	DDRC = 0x40;
	PORTC = 0x40;
	PORTC = 0x00;  // low
	_delay_ms(20);  // wait 18ms
	PORTC = 0x40;  // high
	_delay_us(40);  // wait 20~40us
}

int response()
{
	DDRC = 0x00;
	if((PINC & 0x40) != 0)   // low가 아니면
	{
		return 1;
	}
	_delay_us(80);  // until 80us
	if((PINC & 0x40) == 0)  // high가 아니면
	{
		return 1;
	}
	_delay_us(80);  // until 80us
	return 0;
}

void send_data()
{
	if(response() == 0)
	{
		DDRC = 0x00;
		for(g = 0; g < 5; g++)   // RH, T, Parity bit
		{
			for(h=0; h < 8; h++)   // 8bit
			{
				while((PINC & 0x40) == 0)
				{
					;
				}
				_delay_us(30);
				if((PINC & 0x40) == 0)    // 26 ~ 28us
				{
					data[g][h] = 0;
				}
				else   //  70us
				{
					data[g][h] = 1;
					while((PINC & 0x40) == 128)  // while(PINC & (1<<7))
					{
						;
					}
				}
			}
		}
	}
	else
	{
		;
	}
}

void  dht11_work(void)
{
	send_signal();
	response();
	send_data();
	RH_integral = data[0][0] * 128 + data[0][1] * 64 + data[0][2] * 32 + data[0][3] * 16 + data[0][4] * 8 + data[0][5] * 4 + data[0][6] * 2 + data[0][7];
	RH_decimal = data[1][0] * 128 + data[1][1] * 64 + data[1][2] * 32 + data[1][3] * 16 + data[1][4] * 8 + data[1][5] * 4 + data[1][6] * 2 + data[1][7];
	T_integral = data[2][0] * 128 + data[2][1] * 64 + data[2][2] * 32 + data[2][3] * 16 + data[2][4] * 8 + data[2][5] * 4 + data[2][6] * 2 + data[2][7];
	T_decimal = data[3][0] * 128 + data[3][1] * 64 + data[3][2] * 32 + data[3][3] * 16 + data[3][4] * 8 + data[3][5] * 4 + data[3][6] * 2 + data[3][7];
	parity = data[4][0] * 128 + data[4][1] * 64 + data[4][2] * 32 + data[4][3] * 16 + data[4][4] * 8 + data[4][5] * 4 + data[4][6] * 2 + data[4][7];
	if((RH_integral + RH_decimal + T_integral + T_decimal) == parity)
	{   //패리티 검사가 맞으면 온/습도 값을 계산하여 넣음
		H_10 = RH_integral / 10; H_01 = RH_integral % 10; H_0p1 = RH_decimal /10; H_0p01 = RH_decimal % 10;
		T_10 = T_integral / 10; T_01 = T_integral % 10; T_0p1 = T_decimal /10; T_0p01 = T_decimal % 10;
	}
	
}

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
	_delay_us(50);
	PORTG = 0x00;
}

void lcd_read(unsigned char a)
{
	PORTG = 0x05;
	PORTA = a;
	_delay_us(43);
	PORTG = 0x04;
}

int main(void)
{
   DDRC = 0xff;
   PORTC = 0xff;
   DDRB = 0xff;
   PORTB = 0x00;
   lcd_mv();
   lcd_func();
   lcd_pic();
   lcd_clear();
   _delay_ms(10);
   
    while (1) 
    {
		dht11_work();
		lcd_data(0,0); lcd_read(T_10+'0'); lcd_read(T_01+'0'); lcd_read('.'); lcd_read(T_0p1+'0'); lcd_read(T_0p01+'0');
		lcd_data(1,0); lcd_read(H_10+'0'); lcd_read(H_01+'0'); lcd_read('.'); lcd_read(H_0p1+'0'); lcd_read(H_0p01+'0');
    }
}

