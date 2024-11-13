#define F_CPU 14745600UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "lcd.h"

#define BIT_SET(a,b) ((a) |= (1<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1<<(b)))
#define BIT_TOGGLE(a,b) ((a) ^= (1<<(b)))
#define BIT_CHECK(a,b) (!!((a) & (1<<(b))))
#define DEBOUNCING_DELAY 20

char Message[40];
unsigned char x = 1;
unsigned char x_1 = 1;
unsigned char y = 10;
unsigned char input = 1;
unsigned char lcd_state = 0;
unsigned char sww = 0;
unsigned char pw1, pw2, pw3, pw4 = 0;
unsigned char input_pw1, input_pw2, input_pw3, input_pw4 = 0;
unsigned char input_rpw1, input_rpw2, input_rpw3, input_rpw4 = 0;
volatile unsigned char flag;
unsigned char mode = 0b00000000;    
/* mode의 각 비트별 설정 [7:에어컨 자동/수동][6:가습기 자동/수동 ][5:조도센서 자동/수동][4:인체 감지 on/off]
                [3:우편물감지on/off][2::수위센서 on/off][1:가스검출기 on/off  ][0:블루투스 on/off ]  1이면 자동/on 0이면 수동/off*/

void USART0_init(void)
{
	UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0) | (1 << TXCIE0); // RXCIE = 1(수신 인터럽트 허가), RXEN0 = 1(수신 허가)
	UBRR0H = 0x00; // 57600bps 보오레이트 설정
	UBRR0L = 0x0f; // 57600bps 보오레이트 설정
}
void uart0_tx(unsigned char pw)
{
	while(!(UCSR0A&(1<<UDRE0)));
	UDR0 = pw;
}

ISR(USART0_RX_vect)
{
	flag = UDR0;
}


//LCD설정부분---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
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
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//LCD인터페이스 부분-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void settings_page(unsigned char mode_sel_1,unsigned char mode_sel_2)
{
	if (BIT_CHECK(mode, mode_sel_1) == 1)
	{
		LCD_Pos(0,13); LCD_Str("ON ");
	}
	if (BIT_CHECK(mode, mode_sel_1) == 0) 
	{
		LCD_Pos(0,13); LCD_Str("OFF");
	}
	if (BIT_CHECK(mode, mode_sel_2) == 1) 
	{
		LCD_Pos(1,13); LCD_Str("ON ");
	}
	if (BIT_CHECK(mode, mode_sel_2) == 0) 
	{
		LCD_Pos(1,13); LCD_Str("OFF");
	}
}

void lcd_cursor(void)
{
	if((y%2) == 1)
	{
		LCD_Pos(0,0); LCD_Str(" ");
		LCD_Pos(1,0); LCD_Str(">");
	}
	if((y%2) == 0)
	{
		LCD_Pos(0,0); LCD_Str(">");
		LCD_Pos(1,0); LCD_Str(" ");
	}
}

void menu(void)
{
	if (lcd_state == 0) //LCD 초기화면일 때
	{
		if (y == 11)
		{
			LCD_Pos(0,1); LCD_Str("1.SETTINGS    ");
			LCD_Pos(1,1); LCD_Str("2.TEMP&HUM    ");
			lcd_cursor();
			if (input == 2)
			{
				lcd_state = 2;
				input = 1;
			}
		}
		if (y == 10)
		{
			LCD_Pos(0,1); LCD_Str("1.SETTINGS    ");
			LCD_Pos(1,1); LCD_Str("2.TEMP&HUM    ");
			lcd_cursor();
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
	if (lcd_state == 1)//LCD에서 SETTINGS 들어갔을 때
	{
		
		if (y == 10)
		{
			LCD_Pos(0,1); LCD_Str("AIRCON      ");
			LCD_Pos(1,1); LCD_Str("BLIND       ");
			lcd_cursor();
			settings_page(7, 5);
			if (input == 2)
			{
				BIT_TOGGLE(mode, 7);
				input = 1;
			}
		}
		if (y == 11)
		{
			LCD_Pos(0,1); LCD_Str("AIRCON      ");
			LCD_Pos(1,1); LCD_Str("BLIND       ");
			lcd_cursor();
			settings_page(7, 5);
			if (input == 2)
			{
				BIT_TOGGLE(mode, 5);
				settings_page(7, 5);
				input = 1;
			}
		}
		if (y == 12)
		{
			LCD_Pos(0,1); LCD_Str("MAIL ALARM  ");
			LCD_Pos(1,1); LCD_Str("GAS SENSOR  ");
			lcd_cursor();
			settings_page(3, 1);
			if (input == 2)
			{
				BIT_TOGGLE(mode, 3);
				settings_page(3, 1);
				input = 1;
			}
		}
		if (y == 13)
		{
			LCD_Pos(0,1); LCD_Str("MAIL ALARM  ");
			LCD_Pos(1,1); LCD_Str("GAS SENSOR  ");
			lcd_cursor();
			settings_page(3, 1);
			if (input == 2)
			{
				BIT_TOGGLE(mode, 1);
				settings_page(3, 1);
				input = 1;
			}
		}
		if(y == 14)
		{
			LCD_Pos(0,1); LCD_Str("HUMIDIFIER  ");
			LCD_Pos(1,1); LCD_Str("SECURITY    ");
			lcd_cursor();
			settings_page(6, 4);
			if (input == 2)
			{
				BIT_TOGGLE(mode, 6);
				settings_page(6, 4);
				input = 1;
			}
		}
		if (y == 15)
		{
			LCD_Pos(0,1); LCD_Str("HUMIDIFIER  ");
			LCD_Pos(1,1); LCD_Str("SECURITY    ");
			lcd_cursor();
			settings_page(6, 4);
			if (input == 2)
			{
				BIT_TOGGLE(mode, 4);
				settings_page(6, 4);
				input = 1;
			}
		}
		if(y == 16)
		{
			LCD_Pos(0,1); LCD_Str("WATERSENSOR ");
			LCD_Pos(1,1); LCD_Str("CONNECT     ");
			lcd_cursor();
			settings_page(2, 0);
			if (input == 2)
			{
				BIT_TOGGLE(mode, 2);
				settings_page(2, 0);
				input = 1;
			}
		}
		if (y == 17)
		{
			LCD_Pos(0,1); LCD_Str("WATERSENSOR ");
			LCD_Pos(1,1); LCD_Str("CONNECT     ");
			lcd_cursor();
			settings_page(2, 0);
			if (input == 2)
			{
				BIT_TOGGLE(mode, 0);
				settings_page(2, 0);
				input = 1;
			}
		}
		if (y == 18)
		{
			LCD_Pos(0,1); LCD_Str("BACK           ");
			LCD_Pos(1,1); LCD_Str("               ");
			lcd_cursor();
			if (input == 2)
			{
				lcd_state = 0;
				y = 10;
				input = 1;
				LCD_Pos(0,1); LCD_Str("1.SETTINGS    ");
				LCD_Pos(1,1); LCD_Str("2.TEMP&HUM    ");
				lcd_cursor();
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
	if (lcd_state == 2)
	{
		if (x < 5)
		{
			LCD_Pos(0,0); LCD_Str("INPUT PWD    ");
			sprintf(Message," %01d",input_pw1);
			LCD_Pos(1,0); LCD_Str(Message);
			sprintf(Message," %01d",input_pw2);
			LCD_Pos(1,2); LCD_Str(Message);
			sprintf(Message," %01d",input_pw3);
			LCD_Pos(1,4); LCD_Str(Message);
			sprintf(Message," %01d   ",input_pw4);
			LCD_Pos(1,6); LCD_Str(Message);
		}
		if (x == 1)
		{
			LCD_Pos(1,0); LCD_Str(">");
		}
		if (x == 2)
		{
			LCD_Pos(1,0); LCD_Str(" "); LCD_Pos(1,2); LCD_Str(">");
		}
		if (x == 3)
		{
			LCD_Pos(1,0); LCD_Str(" "); LCD_Pos(1,2); LCD_Str(" "); LCD_Pos(1,4); LCD_Str(">"); 
		}
		if (x == 4)
		{
			LCD_Pos(1,0); LCD_Str(" "); LCD_Pos(1,2); LCD_Str(" "); LCD_Pos(1,4); LCD_Str(" "); LCD_Pos(1,6); LCD_Str(">");
		}
		if (x >= 5)
		{
			if (input_pw1 != pw1 || input_pw2 != pw2 || input_pw3 != pw3 || input_pw4 != pw4)
			{
				LCD_Pos(0,0); LCD_Str("WRONG");
				LCD_Pos(1,0); LCD_Str("        ");
				_delay_ms(1000);
				x = 1;
				x_1 = 1;
				lcd_state = 2;
				input_pw1 = 0;
				input_pw2 = 0;
				input_pw3 = 0;
				input_pw4 = 0;
			}
			if (input_pw1 == pw1 && input_pw2 == pw2 && input_pw3 == pw3 && input_pw4 == pw4)
			{
				LCD_Pos(0,1); LCD_Str("CHANGE PWD     ");
				LCD_Pos(1,1); LCD_Str("               ");
				LCD_Pos(1,0); LCD_Str(" "); LCD_Pos(1,1); LCD_Str(input_rpw1);
				LCD_Pos(1,2); LCD_Str(" "); LCD_Pos(1,3); LCD_Str(input_rpw2);
				LCD_Pos(1,4); LCD_Str(" "); LCD_Pos(1,5); LCD_Str(input_rpw3);
				LCD_Pos(1,6); LCD_Str(" "); LCD_Pos(1,7); LCD_Str(input_rpw4);
				if (x_1 == 5)
				{
					LCD_Pos(1,0); LCD_Str(">");
				}
				if (x_1 == 6)
				{
					LCD_Pos(1,0); LCD_Str(" "); LCD_Pos(1,2); LCD_Str(">");
				}
				if (x_1 == 7)
				{
					LCD_Pos(1,0); LCD_Str(" "); LCD_Pos(1,2); LCD_Str(" "); LCD_Pos(1,4); LCD_Str(">");
				}
				if (x_1 == 8)
				{
					LCD_Pos(1,0); LCD_Str(" "); LCD_Pos(1,2); LCD_Str(" "); LCD_Pos(1,4); LCD_Str(" "); LCD_Pos(1,6); LCD_Str(">");
				}
				if (x_1 == 9)
				{
					pw1 = input_rpw1;
					pw2 = input_rpw2;
					pw3 = input_rpw3;
					pw4 = input_rpw4;
					lcd_state = 0;
					x = 1;
					y = 11;
					x_1 = 1;
					input = 1;
					input_pw1 = 0;
					input_pw2 = 0;
					input_pw3 = 0;
					input_pw4 = 0;
					input_rpw1 = 0;
					input_rpw2 = 0;
					input_rpw3 = 0;
					input_rpw4 = 0;
				}
			}
			
		}
		if (x < 1)
		{
			x = 1;
		}
		
	}
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//각 모듈 제어 부분------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Aircon_auto_manual(void)
{
	if (BIT_CHECK(mode, 7) == 1) // 에어컨 자동모드 일 떄의 동작 코드 기술
	{
		;
	}
	if (BIT_CHECK(mode, 7) == 0) // 에어컨 수동모드 일 때의 동작 코드 기술
	{
		;
	}
}

void humidifier_auto_manual(void)
{
	if (BIT_CHECK(mode, 6) == 1) // 가습기 자동모드 일 떄의 동작 코드 기술
	{
		;
	}
	if (BIT_CHECK(mode, 6) == 0) // 가습기 자동모드 일 떄의 동작 코드 기술
	{
		;
	}
}

void light_sensor_on_off(void)
{
	if (BIT_CHECK(mode, 5) == 1) // 조도센서 자동모드 일 떄의 동작 코드 기술
	{
		;
	}
	if (BIT_CHECK(mode, 5) == 0) // 조도센서 수동모드 일 때의 동작 코드 기술
	{
		;
	}
}

void human_detect_on_off(void)
{
	if (BIT_CHECK(mode, 4) == 1) // 인체 감지 센서 on일 떄의 동작 코드 기술 
	{
		;
	}
	if (BIT_CHECK(mode, 4) == 0) // 인체 감지 센서 off일 때의 동작 코드 기술
	{
		;
	}
}

void mail_on_off(void)
{
	if (BIT_CHECK(mode, 3) == 1) // 우편물 감지 센서 on일 떄의 동작 코드 기술
	{
		;
	}
	if (BIT_CHECK(mode, 3) == 0) // 우편물 감지 센서 off일 때의 동작 코드 기술
	{
		;
	}
}

void water_sensor_on_off(void)
{
	if (BIT_CHECK(mode, 2) == 1) // 수위 감지 센서 on일 떄의 동작 코드 기술
	{
		;
	}
	if (BIT_CHECK(mode, 2) == 0) // 수위 감지 센서 off일 때의 동작 코드 기술
	{
		;
	}
}

void gas_sensor_on_off(void)
{
	if (BIT_CHECK(mode, 1) == 1) // 가스 감지 센서 on일 떄의 동작 코드 기술
	{
		;
	}
	if (BIT_CHECK(mode, 1) == 0) // 가스 감지 센서 off일 때의 동작 코드 기술
	{
		;
	}
}

void bluetooth_on_off(void)
{
	if (BIT_CHECK(mode, 0) == 1) // 블루투스 on일 떄의 동작 코드 기술
	{
		;
	}
	if (BIT_CHECK(mode, 0) == 0) // 블루투스 off일 때의 동작 코드 기술
	{
		;
	}
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//핀 및 기타 제어 부분----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Set_pin_set_atmega128(void)
{
	DDRA = 0xff;
	DDRB = 0xff;
	DDRC = 0xff; // 0~3번 모터 드라이버 4~8번 릴레이 모듈 제어
    DDRD = 0x00; // 0~7번 4x4 키패드 C4~C1 -> || 0번~3번 R1~R4 -> 4번~7번
	DDRE = 0x00;
	DDRG = 0x07; 
	sei(); // 인터럽트 활성화
	
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//테스트 함수 부분-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void pind_input(void) // PIND 의 입력 받는 함수
{
	sww = ~PIND & 0x1f;
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

void pind_test_fix(void)
{
	if (PIND != 0xff || sww == 0)
	{
		sww = ~PIND & 0xff;
		_delay_ms(DEBOUNCING_DELAY);
		if(PIND == 0xff)
		{
			if (sww == 0x04)
			{
				y--;
				if(lcd_state == 2)
				{
					if (x == 1)
					{
						input_pw1--;
					}
					if (x == 2)
					{
						input_pw2--;
					}
					if (x == 3)
					{
						input_pw3--;
					}
					if (x == 4)
					{
						input_pw4--;
					}
					if (x_1 == 5)
					{
						input_rpw1--;
					}
					if (x_1 == 6)
					{
						input_rpw2--;
					}
					if (x_1 == 7)
					{
						input_rpw3--;
					}
					if (x_1 == 8)
					{
						input_rpw4--;
					}
				}
			}
			if (sww == 0x08)
			{
				y++;
				if(lcd_state == 2)
				{
					if (x == 1)
					{
						input_pw1++;
					}
					if (x == 2)
					{
						input_pw2++;
					}
					if (x == 3)
					{
						input_pw3++;
					}
					if (x == 4)
					{
						input_pw4++;
					}
					if (x_1 == 5)
					{
						input_rpw1++;
					}
					if (x_1 == 6)
					{
						input_rpw2++;
					}
					if (x_1 == 7)
					{
						input_rpw3++;
					}
					if (x_1 == 8)
					{
						input_rpw4++;
					}
				}
			}
			if (sww == 0x10)
			{
				input++;
				if(lcd_state == 2)
				{
					x++;
					x_1++;
				}
			}
		}
	}
	
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
int main(void)
{
	Set_pin_set_atmega128(); 
	USART0_init();
	LCD_Init();
    while (1) 
    {
		Aircon_auto_manual();
		humidifier_auto_manual();
		light_sensor_on_off();
		human_detect_on_off();
		mail_on_off();
		water_sensor_on_off();
		gas_sensor_on_off();
		bluetooth_on_off();
		menu();
		pind_test_fix();
		PORTB = ~mode;
    }
}

