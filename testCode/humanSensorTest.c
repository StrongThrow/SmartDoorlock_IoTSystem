#define F_CPU 14745600UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#define BIT_SET(a,b) ((a) |= (1<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1<<(b)))
#define BIT_TOGGLE(a,b) ((a) ^= (1<<(b)))
#define BIT_CHECK(a,b) (!!((a) & (1<<(b))))

unsigned char mode = 0;
unsigned int second_0p01 = 0;
unsigned int second_0p1 = 0;
unsigned int second_1 = 0;
unsigned int second_10 = 0;
unsigned int minute_1 = 0;

void lcd_func(void)
{
	PORTG = 0x01;
	PORTA = 0x38;
	_delay_us(39);
	PORTG = 0x00;
}

void lcd_pic(void)
{
	PORTG = 0x01;
	PORTA = 0x0c;
	_delay_us(39);
	PORTG = 0x00;
}

void lcd_mv(void)
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

void lcd_data(unsigned char a,unsigned char b)
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

void lcd_str(unsigned char *str)
{
	while(*str != 0)
	{
		lcd_read(*str);
		str++;
	}
}

void init_timer2(void) // 오버플로우 발생시 0.01초씩 증가,
{
	TCCR2 = 0;
	TCCR2 |=(1<<WGM21)|(1<<WGM20)|(1<<CS22)|(1<<CS20);// fast PWM 모드 사용, 자동출력 x 오버플로우 인터럽트 활성화
	TIMSK = 0x40;
	TCNT2 = 112;
}

ISR(TIMER2_OVF_vect)
{
	second_0p01++;
	TCNT2 = 112;
}

ISR(INT0_vect)
{
	second_0p01 = 0;
	second_0p1 = 0;
	second_1 = 0;
	second_10 = 0;
	minute_1 = 0;
}


void sleepmode(void)
{
	if(second_1 > 6)
	{
		lcd_clear();
		_delay_ms(10);
		lcd_data(0,0); lcd_read('a');
		sleep_cpu();
	}
}
	
void time_1(void)
{
	if (second_0p01 > 9)
	{
		second_0p1++;
		second_0p01 = 0;
	}
	if (second_0p1 > 9)
	{
		second_1++;
		second_0p1 = 0;
	}
	if (second_1 > 9)
	{
		second_10++;
		second_1 = 0;
	}
	if (second_10 > 6)
	{
		minute_1++;
		second_10 = 0;
	}
	if (minute_1 > 9)
	{
		second_0p01 = 0;
		second_0p1 = 0;
		second_1 = 0;
		second_10 = 0;
		minute_1 = 0;
	}
}	
	
int main(void)
{
	DDRB = 0xff;
	EIMSK = 0x01; // IMT1번핀 외부 인터럽트 활성화
	EICRA = 0x02; // INT1번핀의 Rising edge 때 인터럽트 요구
	DDRD = 0x00;
	PORTB = ~0x00;
	sei();
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_enable();
	init_timer2();
	lcd_func();
	lcd_pic();
	lcd_mv();
	lcd_clear();
	_delay_ms(10);

    while (1) 
    {
		sleepmode();
		time_1();
		lcd_data(0,0); lcd_read(minute_1+'0'); lcd_read(':'); lcd_read(second_10+'0'); lcd_read(second_1+'0'); lcd_read(':');
		lcd_read(second_0p1+'0'); lcd_read(second_0p01+'0'); 
    }
}

