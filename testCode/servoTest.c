#define F_CPU 14745600UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#define BIT_TOGGLE(a,b) ((a) ^= (1<<(b)))

unsigned char a = 0x10;

void timer_counter_1_init(void)
{
	TCCR1A= 0x82; // PB5(OC1A)핀 자동출력, 컴패어 매치 때 클리어, TOP에 도달하였을 때 셋
	TCCR1B=0x1B; // 16비트 Fast PWM 모드 사용, 분주비 64 ICR1  = 4607일때 파형의 주기 20ms
	OCR1A=40;
	ICR1=4607;
	TIMSK = 0x40;
}



int main(void)
{
	DDRB = 0x20;
	DDRG = 0x10;
	

	sei();
	timer_counter_1_init();
    while (1) 
    {
		
		OCR1A = 460;
		_delay_ms(1000);
		OCR1A = 0;
		_delay_ms(3000);
		OCR1A = 240;
		_delay_ms(700);
		OCR1A = 0;
		_delay_ms(3000);
    }
}

