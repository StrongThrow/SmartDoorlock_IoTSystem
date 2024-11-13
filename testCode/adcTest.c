#define F_CPU 16000000
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define ADC_VREF_TYPE 0x00
#define ADC_AVCC_TYPE 0x40
#define ADC_RES_TYPE 0x80
#define ADC_2_56_TYPE 0xc0

void ADC_init_INT(void)
{
	ADCSRA = 0x00;
	ADMUX = 0b01000000; // ADC0 단일 입력 모드 10비트 데이터 우측부터 저장 외부 AVCC단자로 입력된 전압 사용
	ADCSRA = 0b11100111;
}



int main(void)
{
    /* Replace with your application code */
    while (1) 
    {
    }
}

