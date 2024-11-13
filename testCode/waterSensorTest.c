#define F_CPU 14745600UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lcd.h"
int adcRaw = 0;
float adcVoltage = 0;
char Message[40];
 
unsigned int y = 0;
unsigned int x = 0;
unsigned char x_move = 120;
unsigned char y_move = 120;
unsigned char second_0p01 = 0;
unsigned char one006_state = 0;

void init_timer2(void)
{
	TCCR2 = 0;
	TCCR2 |=(1<<WGM21)|(1<<WGM20)|(1<<CS22)|(1<<CS20);
	TIMSK = 0x40;
	TCNT2 = 112;
}

ISR(TIMER2_OVF_vect)
{
	second_0p01++;
	TCNT2 = 112;
}

void ADC_init(void)
{
	ADCSRA = 0x00;
	ADMUX = 0x40 | (0<<ADLAR) | (0 << MUX0);
	ADCSRA = (1<<ADEN)|(1<<ADFR)|(3<<ADPS0);
}

unsigned int Read_ADC_Data(unsigned char adc_input)
{
	unsigned int adc_Data = 0;
	ADMUX &= ~(0x1F);
	ADMUX |= (adc_input & 0x07);
	ADCSRA |= (1<<ADSC);
	while(!(ADCSRA & (1<<ADIF)));
	adc_Data = ADCL;
	adc_Data |= ADCH<<8;
	
	return adc_Data;
}

void one006_test_x(void)
{
	if (one006_state == 0)
	{
		if(x > 800)
		{
			one006_state = 1;
		}
	}
	if(one006_state == 1)
	{
		if(x < 800)
		{
			x_move++;
			one006_state = 0;
		}
	}
}


int main(void)
{
   DDRB = 0xff;
   sei();
   init_timer2();
   ADC_init();
   LCD_Init();
    while (1) 
    {
		adcRaw = Read_ADC_Data(1);
		x = adcRaw;
		one006_test_x();
		adcRaw = Read_ADC_Data(2);
		y = adcRaw;
		sprintf(Message, "X:%04d", x);
		LCD_Pos(0,0); LCD_Str(Message);
		sprintf(Message, "Y:%04d", x_move);
		LCD_Pos(0,8); LCD_Str(Message);
		sprintf(Message, "time:%04d", second_0p01);
		LCD_Pos(1,0); LCD_Str(Message);
		if(second_0p01 >0)
		{
			PORTB = ~0x01;
		}
		if(second_0p01 <200 && second_0p01 >100)
		{
			PORTB = ~0x02;
		}
    }
}

