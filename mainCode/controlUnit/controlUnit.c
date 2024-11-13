#define F_CPU 14745600UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <string.h>
#include "TWI.h"

#define BIT_SET(a,b) ((a) |= (1<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1<<(b)))
#define BIT_TOGGLE(a,b) ((a) ^= (1<<(b)))
#define BIT_CHECK(a,b) (!!((a) & (1<<(b))))
#define DEBOUNCING_DELAY 80
#define REC_BUFF_MAX_LENGTH 100

//TWI 부분--------------------------------------------------------
#define COM_REG 0
#define SRF02_1st_Seq_change 160
#define SRF02_2nd_Seq_change 170
#define SRF02_3rd_Seq_change 165
#define SRF02_Return_inch 80
#define SRF02_Return_Cm 81
#define SRF02_Return_microSecond 82

unsigned char ti_Cnt_1ms;
unsigned char LCD_DelCnt_1ms;
int k = 0;
char Sonar_Addr = 0xE2;
unsigned int Sonar_range;
unsigned int Sonar_range_buffer_plus;
unsigned int Sonar_range_avr;
unsigned int Sonar_range_buffer[30];
int readCnt = 0;
unsigned char res = 0;
volatile unsigned char mail_flag = 0;
unsigned char gas_flag = 0;


//dht11 부분------------------------------------------------------ 
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
unsigned char Temp;

unsigned char H_10; // 습도의 십의자리
unsigned char H_01; // 습도의 일의자리
unsigned char H_0p1;// 습도의 소수점 첫번째 자리
unsigned char H_0p01;//습도의 소수점 두번째 자리
unsigned char Humi;

//-----------------------------------------------------------------

//lcd state = 4, 사용자 환경 설정일 때에 사용하는 변수----------------
//초기온도 25도 설정 

char user_temp = 25; // 십의자리 x 10 + 일의자리 
//초기습도 50퍼 설정
char user_hum = 50; //  십의자리 x 10 + 일의자리
//초기 광조도 퍼센트 
char user_Luminous = 5; //사용자가 설정할 광조도 퍼센트, 0~100 사이의 십의 배수들
char Luminous_100; 
int Luminous = 0; //0~1024사이의 값을 가짐, 나누기 100을 하여서 10의 배수로 나타낼 예정 
//물 수위 퍼센트
char user_water_level = 9; // 사용자가 설정할 물 수위 퍼센트
char water_level_100;
int water_level = 9; // 0~1024사이의 값을 가짐, 나누기 100을 하여서 10의 배수로 변환 
unsigned char flooding_flag = 0;

unsigned char env_insert_mode = 0; // 사용자 환경 변경 시 조이스틱을 누르면 1

char str[20];
int adcRaw = 0;
unsigned char send_pw_chksum;
unsigned char reboot_count = 0; // 재부팅 횟수
unsigned char y = 0; // 조이스틱의 y축 좌표 표현
unsigned char x = 1;
char x_lcd_state_5 = 0;
unsigned char pw[4] = {0, 0, 0, 0}; // 도어락 비밀번호
unsigned char pw1 = 0;
unsigned char pw2 = 0;
unsigned char pw3 = 0;
unsigned char pw4 = 0;
unsigned char input_pw1, input_pw2, input_pw3, input_pw4 = 0; // 비밀번호 확인 시 사용자에게 입력받는 비밀번호 
unsigned char input_rpw1, input_rpw2, input_rpw3, input_rpw4 = 0; // 비밀번호 변경 시 사용자에게 입력받는 비밀번호
unsigned int joystick_x = 0;
unsigned int joystick_y = 0;
unsigned char lcd_state = 0;
unsigned char pwd_insert_mode = 0; // 비밀번호 변경시 인풋을 위해 조이스틱을 누르면 1

unsigned char sww = 0; 

unsigned char RecBuff[REC_BUFF_MAX_LENGTH];
unsigned char RecBuffindex;
unsigned char RecBuff_estLength = REC_BUFF_MAX_LENGTH;
unsigned char RecFlg = 0;
unsigned char lRecBuff[100]; // 수신패킷 임시저장용 배열
unsigned char lRecBuffLength = 0;
int usart0_state = 0;

volatile unsigned char flag;
unsigned char USER_WHO = 0;
unsigned char GUEST = 1;
unsigned char USER1 = 2;
unsigned char USER2 = 3;

unsigned char mode_USER1 = 0;
unsigned char mode_USER2 = 0;
unsigned char mode_Guest = 0;
unsigned char mode = 0b00000000;    
/* mode의 각 비트별 설정 [7:DHT-11 ON/OFF][6:가습기 ON/OFF ][5:조도센서 자동/수동][4:x]
                [3:우편물감지on/off][2::수위센서 on/off][1:가스검출기 on/off  ][0:에어컨 on/off ]  1이면 자동/on 0이면 수동/off*/
unsigned char lcd_state_and_joystick = 0b00000000;

unsigned char door_open[1] = {1};
	
unsigned char sleep_flag = 0;

//타이머 카운터--------------------------------------------------------------------------------------------------------
void init_timer0(void)   //타이머 기본 설정
{
	TCCR0 = 0x0f;//(1<<WGM01)|(1<<CS00)|(1<<CS01)|(1<<CS02);
	TCNT0 = 0x00;
	OCR0 = 14;
	TIMSK = 0x14;
	TIMSK |= (1<<OCIE0);
}

ISR(TIMER0_COMP_vect)
{
	ti_Cnt_1ms ++;
	LCD_DelCnt_1ms ++;
}

void init_timer1(void) // 타이머 카운터 1, Fast PWM 16비트 타이머 사용, 4608번 세면 주기 20ms 
{
	TCCR1A = 0x02; // 컴패어 매치 때 클리어, TOP에 도달하였을 때 셋
	TCCR1B = 0x1B; // 16비트 Fast PWM 모드 사용, 분주비 64 ICR1  = 4607일때 파형의 주기 20ms
	OCR1A = 0;
	ICR1 = 4607;
}

ISR(TIMER1_COMPA_vect)
{
	BIT_CLEAR(PORTC, 4);
}

ISR(TIMER1_OVF_vect)
{
	BIT_SET(PORTC, 4);
}

//창문 서보모터 부분----------------------------------------------------------------------------------------------------
void window_open(void)
{
	OCR1A = 550;
}

void window_close(void)
{
	OCR1A = 230;
}

void window_open_and_close(void)
{
	OCR1A = 550;
	_delay_ms(600);
	OCR1A = 230;
	_delay_ms(600);
}

void window_stop(void)
{
	DDRC = 0xE0;
	OCR1A = 0;
}

//SRF02_I2C부분---------------------------------------------------------------------------------------------------------
unsigned char SRF02_I2C_Write(char address, char reg, char data)

{
	unsigned char ret_err = 0;
	ret_err = TWI_Start();
	ret_err = TWI_Write_SLAW(address);
	if(ret_err !=0) return ret_err;
	ret_err = TWI_Write_Data(reg);
	if(ret_err != 0) return ret_err;
	ret_err = TWI_Write_Data(data);
	if(ret_err != 0) return ret_err;
	TWI_Stop();
	return 0;
}

unsigned char SRF02_I2C_Read(char address, char reg, unsigned char* Data)
{
	char read_data = 0;
	unsigned char ret_err = 0;
	ret_err = TWI_Start();

	ret_err = TWI_Write_SLAW(address);
	if(ret_err !=0) return ret_err;
	ret_err = TWI_Write_Data(reg);
	if(ret_err !=0) return ret_err;
	ret_err = TWI_Restart();

	if(ret_err !=0) return ret_err;
	ret_err = TWI_Write_SLAR(address);
	if(ret_err !=0) return ret_err;
	ret_err = TWI_Read_Data_NACK(&read_data);
	if(ret_err !=0) return ret_err;
	TWI_Stop();
	*Data = read_data;
	return 0;
}



unsigned char startRanging(char addr)
{
	return SRF02_I2C_Write(addr, COM_REG, SRF02_Return_Cm);
}

unsigned int getRange(char addr, unsigned int*pDistance)
{
	unsigned char temp1;
	unsigned char res = 0;
	res = SRF02_I2C_Read(addr,2,&temp1);
	if(res) return res;
	*pDistance = temp1 << 8;
	res = SRF02_I2C_Read(addr,3,&temp1);
	if(res) return res;
	*pDistance |= temp1;

	return res;
}


unsigned char change_Sonar_Addr(unsigned char ori, unsigned char des)
{
	unsigned char res = 0;
	switch(des)
	{                        //초음파센서는 FE까지 주소가 가능
		case 0xE0:
		case 0xE2:
		case 0xE4:
		case 0xE6:
		case 0xE8:
		case 0xEA:
		case 0xEC:
		case 0xEE:
		case 0xF0:
		case 0xF2:
		case 0xF4:
		case 0xF6:
		case 0xF8:
		case 0xFA:
		case 0xFC:
		case 0xFE:
		
		res = SRF02_I2C_Write(ori, COM_REG, SRF02_1st_Seq_change);
		if(res) return res;
		res = SRF02_I2C_Write(ori, COM_REG, SRF02_2nd_Seq_change);
		if(res) return res;
		res = SRF02_I2C_Write(ori, COM_REG, SRF02_3rd_Seq_change);
		if(res) return res;
		res = SRF02_I2C_Write(ori,COM_REG,des);
		if(res) return res;
		break;
		default:
		return -1;
	}
	return 0;
}

void SRF02_running(void)
{
	startRanging(Sonar_Addr);
	if(ti_Cnt_1ms > 66){  //66ms이상일 때 동작 // 충분한 시간이 있어야함
		res = getRange(Sonar_Addr, &Sonar_range);
		if(LCD_DelCnt_1ms > 66)
		{ // 거리를 30번 더하고 30으로 나누어 평균값 구함(1번 측정하면 값이 오류가 나는 경우가 많음)
			Sonar_range_buffer_plus += Sonar_range;
			LCD_DelCnt_1ms = 0;
			k++;
		}
		startRanging(Sonar_Addr);
		ti_Cnt_1ms = 0;
		readCnt = (readCnt + 1)%10;
	}
   if(k > 29) // 30번 카운팅 후
   {
	   Sonar_range_avr = Sonar_range_buffer_plus / 30; // Sonar_range_buffer_plus 을 30으로 나누어 평균값 구함
	   // 이후 관련 변수 초기화
	   Sonar_range_buffer_plus = 0;
	   k = 0;
   }
}


//EEPROM설정부분------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void EEPROM_Write(unsigned int EE_Addr, unsigned char EE_Data)
{
   while(EECR&(1<<EEWE));
   EEAR = EE_Addr;
   EEDR = EE_Data;// EEDR <- EE_Data
   cli();// Global Interrupt Disable
   EECR |= (1 << EEMWE);// SET EEMWE
   EECR |= (1 << EEWE); // SET EEWE
   sei(); // Global Interrupt Enable
}

unsigned char EEPROM_Read(unsigned int EE_addr)
{
   while(EECR&(1<<EEWE));
   EEAR = EE_addr;
   EECR |= (1 <<EERE);// SET EERE
   return EEDR; // Return Read Value
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//dht11설정부분------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void send_signal()
{
	DDRC = 0x80;
	PORTC = 0x80;
	PORTC = 0x00;  // low
	_delay_ms(20);  // wait 18ms
	PORTC = 0x80;  // high
	_delay_us(40);  // wait 20~40us
}

int response()
{
	DDRC = 0x00;
	if((PINC & 0x80) != 0)   // low가 아니면
	{
		return 1;
	}
	_delay_us(80);  // until 80us
	if((PINC & 0x80) == 0)  // high가 아니면
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
				while((PINC & 0x80) == 0)
				{
					;
				}
				_delay_us(30);
				if((PINC & 0x80) == 0)    // 26 ~ 28us
				{
					data[g][h] = 0;
				}
				else   //  70us
				{
					data[g][h] = 1;
					while((PINC & 0x80) == 128)  // while(PINC & (1<<7))
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
	{
		H_10 = RH_integral / 10; H_01 = RH_integral % 10; H_0p1 = RH_decimal /10; H_0p01 = RH_decimal % 10;
		T_10 = T_integral / 10; T_01 = T_integral % 10; T_0p1 = T_decimal /10; T_0p01 = T_decimal % 10;
	}
	
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//USART설정부분------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
ISR(USART0_RX_vect)
{
   sleep_flag = 0;
   RecBuff[RecBuffindex] = UDR0;
   RecBuffindex++;
   if(RecBuffindex > 4) // 데이터 길이정보가 수신된 경우 패킷 길이 갱신
   {
      RecBuff_estLength = (RecBuff[4] + 6); // 패킷길이는 데이터 길이+6
   }
   if(RecBuffindex == RecBuff_estLength) // 수신된 데이터의 순서가 패킷 길이와 같으면 패킷 수신완료
   {
      RecFlg = 1; // 수신 완료 플래그 활성화
   }
   //sleep_flag = 0;
}

void USART0_init(void)
{
   UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0) | (1 << TXCIE0); // RXCIE = 1(수신 인터럽트 허가), RXEN0 = 1(수신 허가)
   UBRR0H = 0x00; // 57600bps 보오레이트 설정
   UBRR0L = 0x0f; // 57600bps 보오레이트 설정
}

void putch_USART0(char data)
{
   while( !(UCSR0A & (1<<UDRE0)));
   UDR0 = data;
}

void sendBuff_USART0(char *str,int length) // 문자열 출력 루틴
{
   while (length--)
   {
      putch_USART0(*str); // ‘\0’이 나올 때까지 출력한다.
      str++;
   }
}

unsigned char chksum(char *buff,int length){
   unsigned char sum = 0;
   int i = 0;
   for(i=0 ; i<length ; i++)
   {
      sum += buff[i];
   }
   return sum;
}

int parsingPacket(char *recBuff, int length) // 들어온 패킷을 처리하는 함수  return -1, -2,-3 이면 각각의 종류의 상태이상 발생한것임
{ 
   if(recBuff[0] != 0xff) return -1; // 패킷검사 1. 만약 시작 데이터가 정해진 데이터(0xFF)가 아닌 경우 종료.
   if(recBuff[1] != 0x01) return -2; // 패킷검사 2. 장치의 이름이 1이 아닌 경우 종료, 0x01은 도어락, 0x02는 제어부 아트메가
   if(recBuff[length -1] != chksum(recBuff,length-1)) return -3;
   // 패킷검사 3. 패킷의 체크섬을 확인한뒤 일치하지 않은 경우 종료.
   // 수신된 체크섬 데이터는 recBuff[length -1]
   // 체크섬 계산 값은 체크섬 데이터 앞까지 더하는 chksum(recBuff,length-1)
   
      if(RecBuff[3] == 0x06)
      {
         USER_WHO = RecBuff[5];
         if(USER_WHO == 1) mode = EEPROM_Read(0x01);
         if(USER_WHO == 2) mode = EEPROM_Read(0x02);
         if(USER_WHO == 3) mode = EEPROM_Read(0x03);
      }
   
   return 0;
}

void USART0_process(void)
{
   if(RecFlg == 1)
   {
      // 임시저장 후 또다시 패킷 수신을 위한 버퍼 및 관련변수 초기화
      memcpy(lRecBuff, RecBuff, RecBuff_estLength);
      lRecBuffLength = RecBuff_estLength;
      RecBuff_estLength = REC_BUFF_MAX_LENGTH;
      RecBuffindex = 0;
      RecFlg = 0;
      // 수신된 패킷을 파싱하는 함수
      usart0_state = parsingPacket(lRecBuff,lRecBuffLength);
      memset(RecBuff, 0, REC_BUFF_MAX_LENGTH);
      
      if (usart0_state < 0)
      {
         lcd_clear();
         _delay_ms(10);
      }
   }
}

void send_Packet(unsigned char type, unsigned char rw, unsigned char length, unsigned char *sendData) // 데이터 보내는 함수
{
   unsigned char resBUFF[REC_BUFF_MAX_LENGTH]; // REC_BUFF_MAX_LENGTH는 데이터의 최대 길이, 코딩하고 마지막에 수정
   unsigned char resBUFFLength = 0;
   int i = 0;
   resBUFF[resBUFFLength++] = 0xff; // 시작 바이트 0xff
   resBUFF[resBUFFLength++] = 0x02; // 장치 이름 0x01은 도어락 0x02는 제어부, 이 장치는 제어부임으로 0x02 전송
   resBUFF[resBUFFLength++] = type; // 데이터의 종류, 0x01은 USER데이터, 0x02는 패스워드
   resBUFF[resBUFFLength++] = rw;   // read, write 종류 0x03 읽기, 0x06 쓰기
   resBUFF[resBUFFLength++] = length; // 데이터들의 길이 *****절대 전체의 길이가 아님
   for(i = 0; i<length; i++)
   {
      resBUFF[resBUFFLength++] = sendData[i]; // 데이터를 보냄 send_Pachet(....... ,보내는 데이터)
   }
   resBUFF[resBUFFLength] = chksum(resBUFF, resBUFFLength); // 체크섬 계산 후 체크섬 전송
   resBUFFLength++;
   sendBuff_USART0(resBUFF, resBUFFLength); // 데이터 전송 
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//ADC설정부분---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void ADC_init(void)
{
   ADCSRA = 0x00;
   ADMUX = 0x40 | (0<<ADLAR) | (0 << MUX0);
   ADCSRA = (1<<ADEN)|(3<<ADPS0)|(0<<ADFR); //
}

unsigned int read_ADC4_data(void)
{
	unsigned int adc_Data_ADC0 = 0;
	ADMUX = 0x44;
	ADCSRA |=(1<<ADSC);
	while(!(ADCSRA & (1<<ADIF)));
	adc_Data_ADC0 = ADCL;
    adc_Data_ADC0 |= ADCH<<8;
	 
	return adc_Data_ADC0;
}

unsigned int read_ADC1_data(void)
{
	unsigned int adc_Data_ADC1 = 0;
	ADMUX = 0x41;
	ADCSRA |=(1<<ADSC);
	while(!(ADCSRA & (1<<ADIF)));
	adc_Data_ADC1 = ADCL;
	adc_Data_ADC1 |= ADCH<<8;
	
	return adc_Data_ADC1;
}

unsigned int read_ADC2_data(void)
{
	unsigned int adc_Data_ADC2 = 0;
	ADMUX = 0x42;
	ADCSRA |=(1<<ADSC);
	while(!(ADCSRA & (1<<ADIF)));
	adc_Data_ADC2 = ADCL;
	adc_Data_ADC2 |= ADCH<<8;
	
	return adc_Data_ADC2;
}

unsigned int read_ADC3_data(void)
{
	unsigned int adc_Data_ADC3 = 0;
	ADMUX = 0x43;
	ADCSRA |=(1<<ADSC);
	while(!(ADCSRA & (1<<ADIF)));
	adc_Data_ADC3 = ADCL;
	adc_Data_ADC3 |= ADCH<<8;
	
	return adc_Data_ADC3;
}

//광 조도 센서, 물 수위 센서 ADC설정 부분-----------------------------------------------------------------

void adc_sensor(void)
{
	// 물 수위 센서값 변환 시작 
	water_level = read_ADC4_data();
	water_level_100 = water_level / 100;
	// 조이스틱 변환 시작 
	joystick_x = read_ADC1_data();
	joystick_y = read_ADC2_data();
    // 조도 센서 변환 시작 
	Luminous = read_ADC3_data();
	Luminous_100 = Luminous / 100;
}

//조이스틱,ADC 부분--------------------------------------------------------------------------------------
void joystick(void)
{
    //조이스틱이 오른쪽으로 움직일 때의 동작---------------------------------------------------------------
   if(BIT_CHECK(lcd_state_and_joystick,0) == 0)//조이스틱이 중앙에 있을 때
   {
      if(joystick_x > 700)// 조이스틱이 오른쪽으로 움직였을 때
      {
         BIT_SET(lcd_state_and_joystick,0); // 조이스틱이 오른쪽으로 움직인 상태라는것을 알 수 있게 1로 셋
      }
   }
   if(BIT_CHECK(lcd_state_and_joystick,0) == 1)//조이스틱이 오른쪽에 있을 때
   {
      if(joystick_x < 580)//조이스틱이 중앙으로 돌아왔을 때
      {
          if(BIT_CHECK(pwd_insert_mode, 0) == 0)
          {
            x++;
          }
		  if(BIT_CHECK(env_insert_mode,0)==0)
		  {
			  x_lcd_state_5++;
		  }
		  
         BIT_CLEAR(lcd_state_and_joystick,0);//조이스틱이 중앙에 있을 때의 상태로 돌아감
      }
   }
    //조이스틱이 왼쪽으로 움직일 때의 동작-----------------------------------------------------------------
   if(BIT_CHECK(lcd_state_and_joystick,1) == 0)//조이스틱이 중앙에 있을 때
   {
      if(joystick_x <200)// 조이스틱이 왼쪽으로 움직였을 때
      {
         BIT_SET(lcd_state_and_joystick,1); // 조이스틱이 왼쪽으로 움직인 상태라는것을 알 수 있게 1로 셋
      }
   }
   if(BIT_CHECK(lcd_state_and_joystick,1) == 1)//조이스틱이 왼쪽에 있을 때
   {
      if(joystick_x > 400)//조이스틱이 중앙으로 돌아왔을 때
      {
         if(BIT_CHECK(pwd_insert_mode, 0) == 0)
         {
            x--;
         }
		 if(BIT_CHECK(env_insert_mode,0)==0)
		 {
			 x_lcd_state_5--;
		 }
         BIT_CLEAR(lcd_state_and_joystick,1);//조이스틱이 중앙에 있을 때의 상태로 돌아감
      }
   }
    //조이스틱이 위쪽으로 움직일 때의 동작------------------------------------------------------------------
   if(BIT_CHECK(lcd_state_and_joystick,2) == 0)//조이스틱이 중앙에 있을 때
   {
      if(joystick_y > 800)// 조이스틱이 위쪽으로 움직였을 때
      {
           BIT_SET(lcd_state_and_joystick,2); // 조이스틱이 위쪽으로 움직인 상태라는것을 알 수 있게 1로 셋
      }
   }
   if(BIT_CHECK(lcd_state_and_joystick,2) == 1)//조이스틱이 왼쪽에 있을 때
   {
      if(joystick_y < 600)//조이스틱이 중앙으로 돌아왔을 때
      {
         y++;//y를 1 증가시킴
         if((BIT_CHECK(pwd_insert_mode, 0) == 1) && ((lcd_state == 2)|| lcd_state == 4)) // 비밀번호 변경 모드일 때
         {
            if(x == 1) {input_pw1--; input_rpw1--;} if(x == 2) {input_pw2--;  input_rpw2--;}
            if(x == 3) {input_pw3--; input_rpw3--;} if(x == 4) {input_pw4--;  input_rpw4--;}
         }
		 if((BIT_CHECK(env_insert_mode, 0) == 1) && (lcd_state == 5))
		 {
			  if(x_lcd_state_5 == 0) user_temp--; if(x_lcd_state_5 == 1) user_hum--;
			  if(x_lcd_state_5 == 2) user_Luminous--; if(x_lcd_state_5 == 3) user_water_level--;
		 }
         BIT_CLEAR(lcd_state_and_joystick,2);//조이스틱이 중앙에 있을 때의 상태로 돌아감
      }
   }
   //조이스틱이 아래쪽으로 움직일 때의 동작-------------------------------------------------------------------
   if(BIT_CHECK(lcd_state_and_joystick,3) == 0)//조이스틱이 중앙에 있을 때
   {
      if(joystick_y < 100)// 조이스틱이 아래쪽으로 움직였을 때
      {
         BIT_SET(lcd_state_and_joystick,3); // 조이스틱이 아래쪽으로 움직인 상태라는것을 알 수 있게 1로 셋
      }
   }
   if(BIT_CHECK(lcd_state_and_joystick,3) == 1)//조이스틱이 아래쪽에 있을 때
   {
      if(joystick_y > 350)//조이스틱이 중앙으로 돌아왔을 때
      {
         y--;//y를 1 감소시킴
          if((BIT_CHECK(pwd_insert_mode, 0) == 1) && ((lcd_state == 2)|| lcd_state == 4)) // 비밀번호 변경 모드일 때
          {
             if(x == 1) {input_pw1++; input_rpw1++;} if(x == 2) {input_pw2++; input_rpw2++;}
             if(x == 3) {input_pw3++; input_rpw3++;} if(x == 4) {input_pw4++; input_rpw4++;}
          }
		  if((BIT_CHECK(env_insert_mode, 0) == 1) && (lcd_state == 5))
		  {
			  if(x_lcd_state_5 == 0) user_temp++; if(x_lcd_state_5 == 1) user_hum++;
			  if(x_lcd_state_5 == 2) user_Luminous++; if(x_lcd_state_5 == 3) user_water_level++;
		  }
         BIT_CLEAR(lcd_state_and_joystick,3);//조이스틱이 중앙에 있을 때의 상태로 돌아감
      }
   }
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//슬립모드부분---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void sleepmode(void)
{
	if(sleep_flag == 1)
	{
		TCCR0 = 0;
		PORTB = ~0x00;
		Sonar_range_buffer_plus = 0;
		k = 0;
		Sonar_range_avr = 0;
		lcd_clear();
		_delay_ms(10);
		lcd_data(0,0); lcd_read('S'); lcd_read('L'); lcd_read('E'); lcd_read('E'); lcd_read('P'); lcd_read(' ');
		lcd_read('M'); lcd_read('O'); lcd_read('D'); lcd_read('E');
		sleep_cpu();
	}
	if(sleep_flag = 0)
	{   
		TCCR0 |= (1<<CS00)|(1<<CS01)|(1<<CS02);
	}
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

void lcd_str(unsigned char *str)
{
	while(*str != 0)
	{
		lcd_read(*str);
		str++;
	}
}


//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//LCD인터페이스 부분-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void settings_page(unsigned char mode_sel_1,unsigned char mode_sel_2) // on/off 를 lcd에 표기하기 위해 만든 함수 (lcd 윗줄에 해당하는 비트 번호/ lcd 아랫줄에 해당하는 비트 번호)
{
   if (BIT_CHECK(mode, mode_sel_1) == 1)
   {
      lcd_data(0,13); lcd_read('O'); lcd_read('N'); lcd_read(' '); 
   }
   if (BIT_CHECK(mode, mode_sel_1) == 0) 
   {
      lcd_data(0,13); lcd_read('O'); lcd_read('F'); lcd_read('F'); 
   }
   if (BIT_CHECK(mode, mode_sel_2) == 1)
   {
      lcd_data(1,13); lcd_read('O'); lcd_read('N'); lcd_read(' ');
   }
   if (BIT_CHECK(mode, mode_sel_2) == 0) 
   {
      lcd_data(1,13); lcd_read('O'); lcd_read('F'); lcd_read('F');
   }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//메인메뉴-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void menu(void)
{
   if(mail_flag == 0 && gas_flag == 0)
   {
	   if (lcd_state == 0) //LCD 초기화면일 때
	   {
		   if (y == 0)
		   {
			   lcd_data(0,0); lcd_read('>'); lcd_read('1'); lcd_read('.');   lcd_read('S');  lcd_read('E');  lcd_read('T'); lcd_read('T'); lcd_read('I'); lcd_read('N'); lcd_read('G'); lcd_read('S'); lcd_read(' '); lcd_read(' '); lcd_read(' ');
			   lcd_data(1,0); lcd_read(' '); lcd_read('2'); lcd_read('.');   lcd_read('C');  lcd_read('H');  lcd_read('A'); lcd_read('N'); lcd_read('G'); lcd_read('E'); lcd_read(' '); lcd_read('P'); lcd_read('W'); lcd_read('D');
			   if (BIT_CHECK(lcd_state_and_joystick,4) == 1)
			   {
				   lcd_state = 1; y = 0; BIT_CLEAR(lcd_state_and_joystick, 4);
			   }
		   }
		   if (y == 1)
		   {
			   lcd_data(0,0); lcd_read(' '); lcd_read('1'); lcd_read('.');   lcd_read('S');  lcd_read('E');  lcd_read('T'); lcd_read('T'); lcd_read('I'); lcd_read('N'); lcd_read('G'); lcd_read('S'); lcd_read(' '); lcd_read(' '); lcd_read(' ');
			   lcd_data(1,0); lcd_read('>'); lcd_read('2'); lcd_read('.');   lcd_read('C');  lcd_read('H');  lcd_read('A'); lcd_read('N'); lcd_read('G'); lcd_read('E'); lcd_read(' '); lcd_read('P'); lcd_read('W'); lcd_read('D');
			   if (BIT_CHECK(lcd_state_and_joystick,4) == 1)
			   {
				   lcd_state = 2; x = 1; y = 0;
				   input_pw1 = 0;
				   input_pw2 = 0;
				   input_pw3 = 0;
				   input_pw4 = 0;
				   BIT_CLEAR(lcd_state_and_joystick, 4);
			   }
		   }
		   if (y == 2)
		   {
			   lcd_data(0,0); lcd_read('>'); lcd_read('3'); lcd_read('.');   lcd_read('E');  lcd_read('N');  lcd_read('V'); lcd_read(' '); lcd_read('S'); lcd_read('E'); lcd_read('T'); lcd_read('T'); lcd_read('I'); lcd_read('N'); lcd_read('G'); lcd_data(0, 14); lcd_read(' '); lcd_read(' ');
			   lcd_data(1,0); lcd_read(' '); lcd_read('4'); lcd_read('.');   lcd_read('D');  lcd_read('O');  lcd_read('O'); lcd_read('R'); lcd_read(' '); lcd_read('O'); lcd_read('P'); lcd_read('E'); lcd_read('N'); lcd_read(' ');
			   if (BIT_CHECK(lcd_state_and_joystick,4) == 1)
			   {
				   lcd_state = 5; y = 0; BIT_CLEAR(lcd_state_and_joystick, 4); y = 0; x = 0; x_lcd_state_5 = 0;
				   lcd_clear(); _delay_ms(10);
			   }
		   }
		   if (y == 3)
		   {
			   lcd_data(0,0); lcd_read(' '); lcd_read('3'); lcd_read('.');   lcd_read('E');  lcd_read('N');  lcd_read('V'); lcd_read(' '); lcd_read('S'); lcd_read('E'); lcd_read('T'); lcd_read('T'); lcd_read('I'); lcd_read('N'); lcd_read('G'); lcd_data(0, 14); lcd_read(' '); lcd_read(' ');
			   lcd_data(1,0); lcd_read('>'); lcd_read('4'); lcd_read('.');   lcd_read('D');  lcd_read('O');  lcd_read('O'); lcd_read('R'); lcd_read(' '); lcd_read('O'); lcd_read('P'); lcd_read('E'); lcd_read('N'); lcd_read(' ');
			   if (BIT_CHECK(lcd_state_and_joystick,4) == 1)
			   {
				   lcd_state = 6; y = 0; BIT_CLEAR(lcd_state_and_joystick, 4);
			   }
		   }
		   if (y == 4)
		   {
			   lcd_data(0,0); lcd_read('>'); lcd_read('5'); lcd_read('.');   lcd_read('S');  lcd_read('L');  lcd_read('E'); lcd_read('E'); lcd_read('P'); lcd_read(' '); lcd_read('M'); lcd_read('O'); lcd_read('D'); lcd_read('E'); lcd_read(' '); lcd_data(0, 14); lcd_read(' '); lcd_read(' ');
			   lcd_data(1,0); lcd_read(' '); lcd_read(' '); lcd_read(' ');   lcd_read(' ');  lcd_read(' ');  lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' ');
			   if (BIT_CHECK(lcd_state_and_joystick,4) == 1)
			   {
				   lcd_state = 0; y = 0; sleep_flag = 1; BIT_CLEAR(lcd_state_and_joystick, 4); 
			   }
		   }
		   if(y > 4) y = 4; if(y < 0) y = 0; // y좌표 초과 방지
	   }
	   //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

	   //설정메뉴-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	   if (lcd_state == 1)//LCD에서 SETTINGS 들어갔을 때
	   {
		   
		   if (y == 0)
		   {
			   lcd_data(0,0); lcd_read('>'); lcd_read('D'); lcd_read('H');   lcd_read('T');  lcd_read('_');  lcd_read('1'); lcd_read('1'); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' ');
			   lcd_data(1,0); lcd_read(' '); lcd_read('B'); lcd_read('L');   lcd_read('I');  lcd_read('N');  lcd_read('D'); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' ');
			   settings_page(7, 5);
			   if (BIT_CHECK(lcd_state_and_joystick,4) == 1)
			   {
				   BIT_TOGGLE(mode, 7);
				   BIT_CLEAR(lcd_state_and_joystick, 4);
			   }
		   }
		   if (y == 1)
		   {
			   lcd_data(0,0); lcd_read(' '); lcd_read('D'); lcd_read('H');   lcd_read('T');  lcd_read('_');  lcd_read('1'); lcd_read('1'); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' ');
			   lcd_data(1,0); lcd_read('>'); lcd_read('B'); lcd_read('L');   lcd_read('I');  lcd_read('N');  lcd_read('D'); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' ');
			   settings_page(7, 5);
			   if (BIT_CHECK(lcd_state_and_joystick,4) == 1)
			   {
				   BIT_TOGGLE(mode, 5);
				   settings_page(7, 5);
				   BIT_CLEAR(lcd_state_and_joystick, 4);
			   }
		   }
		   if (y == 2)
		   {
			   lcd_data(0,0); lcd_read('>'); lcd_read('M'); lcd_read('A');   lcd_read('I');  lcd_read('L');  lcd_read(' '); lcd_read('A'); lcd_read('L'); lcd_read('A'); lcd_read('R'); lcd_read('M'); lcd_read(' '); lcd_read(' ');
			   lcd_data(1,0); lcd_read(' '); lcd_read('G'); lcd_read('A');   lcd_read('S');  lcd_read(' ');  lcd_read('S'); lcd_read('E'); lcd_read('N'); lcd_read('S'); lcd_read('O'); lcd_read('R'); lcd_read(' '); lcd_read(' ');
			   settings_page(3, 1);
			   if (BIT_CHECK(lcd_state_and_joystick,4) == 1)
			   {
				   BIT_TOGGLE(mode, 3);
				   settings_page(3, 1);
				   BIT_CLEAR(lcd_state_and_joystick, 4);
			   }
		   }
		   if (y == 3)
		   {
			   lcd_data(0,0); lcd_read(' '); lcd_read('M'); lcd_read('A');   lcd_read('I');  lcd_read('L');  lcd_read(' '); lcd_read('A'); lcd_read('L'); lcd_read('A'); lcd_read('R'); lcd_read('M'); lcd_read(' '); lcd_read(' ');
			   lcd_data(1,0); lcd_read('>'); lcd_read('G'); lcd_read('A');   lcd_read('S');  lcd_read(' ');  lcd_read('S'); lcd_read('E'); lcd_read('N'); lcd_read('S'); lcd_read('O'); lcd_read('R'); lcd_read(' '); lcd_read(' ');
			   settings_page(3, 1);
			   if (BIT_CHECK(lcd_state_and_joystick,4) == 1)
			   {
				   BIT_TOGGLE(mode, 1);
				   settings_page(3, 1);
				   BIT_CLEAR(lcd_state_and_joystick, 4);
			   }
		   }
		   if(y == 4)
		   {
			   lcd_data(0,0); lcd_read('>'); lcd_read('H'); lcd_read('U');   lcd_read('M');  lcd_read('I');  lcd_read('D'); lcd_read('I'); lcd_read('D'); lcd_read('F'); lcd_read('I'); lcd_read('E'); lcd_read('R'); lcd_read(' ');
			   lcd_data(1,0); lcd_read(' '); lcd_read('M'); lcd_read('U');   lcd_read('L');  lcd_read('T');  lcd_read('I'); lcd_read('T'); lcd_read('A'); lcd_read('P'); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' ');
			   settings_page(6, 4);
			   if (BIT_CHECK(lcd_state_and_joystick,4) == 1)
			   {
				   BIT_TOGGLE(mode, 6);
				   settings_page(6, 4);
				   BIT_CLEAR(lcd_state_and_joystick, 4);
			   }
		   }
		   if (y == 5)
		   {
			   lcd_data(0,0); lcd_read(' '); lcd_read('H'); lcd_read('U');   lcd_read('M');  lcd_read('I');  lcd_read('D'); lcd_read('I'); lcd_read('D'); lcd_read('F'); lcd_read('I'); lcd_read('E'); lcd_read('R'); lcd_read(' ');
			   lcd_data(1,0); lcd_read('>'); lcd_read('M'); lcd_read('U');   lcd_read('L');  lcd_read('T');  lcd_read('I'); lcd_read('T'); lcd_read('A'); lcd_read('P'); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' ');
			   settings_page(6, 4);
			   if (BIT_CHECK(lcd_state_and_joystick,4) == 1)
			   {
				   BIT_TOGGLE(mode, 4);
				   settings_page(6, 4);
				   BIT_CLEAR(lcd_state_and_joystick, 4);
			   }
		   }
		   if(y == 6)
		   {
			   lcd_data(0,0); lcd_read('>'); lcd_read('W'); lcd_read('A');   lcd_read('T');  lcd_read('E');  lcd_read('R'); lcd_read('S'); lcd_read('E'); lcd_read('N'); lcd_read('S'); lcd_read('O'); lcd_read('R'); lcd_read(' ');
			   lcd_data(1,0); lcd_read(' '); lcd_read('A'); lcd_read('I');   lcd_read('R');  lcd_read('C');  lcd_read('O'); lcd_read('N'); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' ');
			   settings_page(2, 0);
			   if (BIT_CHECK(lcd_state_and_joystick,4) == 1)
			   {
				   BIT_TOGGLE(mode, 2);
				   settings_page(2, 0);
				   BIT_CLEAR(lcd_state_and_joystick, 4);
			   }
		   }
		   if (y == 7)
		   {
			   lcd_data(0,0); lcd_read(' '); lcd_read('W'); lcd_read('A');   lcd_read('T');  lcd_read('E');  lcd_read('R'); lcd_read('S'); lcd_read('E'); lcd_read('N'); lcd_read('S'); lcd_read('O'); lcd_read('R'); lcd_read(' ');
			   lcd_data(1,0); lcd_read('>'); lcd_read('A'); lcd_read('I');   lcd_read('R');  lcd_read('C');  lcd_read('O'); lcd_read('N'); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' ');
			   settings_page(2, 0);
			   if (BIT_CHECK(lcd_state_and_joystick,4) == 1)
			   {
				   BIT_TOGGLE(mode, 0);
				   settings_page(2, 0);
				   BIT_CLEAR(lcd_state_and_joystick, 4);
			   }
		   }
		   if (y == 8)
		   {
			   lcd_data(0,0); lcd_read('>'); lcd_read('B'); lcd_read('A');   lcd_read('C');  lcd_read('K');  lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' ');
			   lcd_data(1,0); lcd_read(' '); lcd_read(' '); lcd_read(' ');   lcd_read(' ');  lcd_read(' ');  lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' ');
			   if (BIT_CHECK(lcd_state_and_joystick,4) == 1)
			   {
				   lcd_state = 0;
				   y = 0;
				   if(USER_WHO == 1)
				   {
					   mode_Guest = mode;
					   EEPROM_Write(0x01, mode_Guest);
					   mode = EEPROM_Read(0x01);
				   }
				   if(USER_WHO == 2)
				   {
					   mode_USER1 = mode;
					   EEPROM_Write(0x02, mode_USER1);
					   mode = EEPROM_Read(0x02);
				   }
				   if(USER_WHO == 3)
				   {
					   mode_USER2 = mode;
					   EEPROM_Write(0x03, mode_USER2);
					   mode = EEPROM_Read(0x03);
				   }
				   BIT_CLEAR(lcd_state_and_joystick, 4);
			   }
		   }
		   if (y > 8) y = 0; if (y < 0) y = 8;
	   }
	   //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

	   //비밀번호변경---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	   if (lcd_state == 2)
	   {
		   if (input_pw1 > 9) input_pw1 = 0; if (input_pw2 > 9) input_pw2 = 0; if (input_pw3 > 9) input_pw3 = 0; if (input_pw4 > 9) input_pw4 = 0;
		   if (x < 1) x = 1; if (x > 5) x = 5;
		   
		   if (x < 6)
		   {
			   lcd_data(0,0); lcd_read('I'); lcd_read('N'); lcd_read('P'); lcd_read('U'); lcd_read('T'); lcd_read(' '); lcd_read('P'); lcd_read('W'); lcd_read('D'); lcd_read(' '); lcd_read(' '); lcd_read(' ');
			   if(BIT_CHECK(pwd_insert_mode, 0) == 1)
			   {
				   lcd_data(0,0); lcd_read('I'); lcd_read('N'); lcd_read('S'); lcd_read('E'); lcd_read('R'); lcd_read('T'); lcd_read(' '); lcd_read('M'); lcd_read('O'); lcd_read('D'); lcd_read('E'); lcd_read(' '); lcd_read(' ');
			   }
			   lcd_data(1,0); lcd_read(' '); lcd_read(input_pw1+'0'); lcd_read(' '); lcd_read(input_pw2+'0'); lcd_read(' '); lcd_read(input_pw3+'0'); lcd_read(' '); lcd_read(input_pw4+'0'); lcd_read(' '); lcd_read('O'); lcd_read('K'); lcd_read(' '); lcd_read(' ');
		   }
		   if (x == 1)
		   {
			   lcd_data(1,0); lcd_read('>');
			   if (BIT_CHECK(lcd_state_and_joystick,4) == 1)// 조이스틱을 눌렀을 때
			   {
				   BIT_TOGGLE(pwd_insert_mode, 0);
				   BIT_CLEAR(lcd_state_and_joystick, 4);
			   }
		   }
		   if (x == 2)
		   {
			   lcd_data(1,0); lcd_read(' '); lcd_data(1,2); lcd_read('>');
			   if (BIT_CHECK(lcd_state_and_joystick,4) == 1)// 조이스틱을 눌렀을 때
			   {
				   BIT_TOGGLE(pwd_insert_mode, 0);
				   BIT_CLEAR(lcd_state_and_joystick, 4);
			   }
		   }
		   if (x == 3)
		   {
			   lcd_data(1,0); lcd_read(' '); lcd_data(1,2); lcd_read(' '); lcd_data(1,4); lcd_read('>');
			   if (BIT_CHECK(lcd_state_and_joystick,4) == 1)// 조이스틱을 눌렀을 때
			   {
				   BIT_TOGGLE(pwd_insert_mode, 0);
				   BIT_CLEAR(lcd_state_and_joystick, 4);
			   }
		   }
		   if (x == 4)
		   {
			   lcd_data(1,0); lcd_read(' '); lcd_data(1,2); lcd_read(' '); lcd_data(1,4); lcd_read(' '); lcd_data(1,6); lcd_read('>');
			   if (BIT_CHECK(lcd_state_and_joystick,4) == 1)// 조이스틱을 눌렀을 때
			   {
				   BIT_TOGGLE(pwd_insert_mode, 0);
				   BIT_CLEAR(lcd_state_and_joystick, 4);
			   }
		   }
		   if (x == 5)
		   {
			   lcd_data(1,0); lcd_read(' '); lcd_data(1,2); lcd_read(' '); lcd_data(1,4); lcd_read(' '); lcd_data(1,6); lcd_read(' '); lcd_data(1,8); lcd_read('>');
			   if (BIT_CHECK(lcd_state_and_joystick,4) == 1)// 조이스틱을 눌렀을 때
			   {
				   lcd_state = 3;
				   x = 1;
				   input_rpw1 = 0;
				   input_rpw2 = 0;
				   input_rpw3 = 0;
				   input_rpw4 = 0;
				   BIT_CLEAR(lcd_state_and_joystick, 4);
			   }
		   }
	   }
	   
	   if (lcd_state == 3)
	   {
		   if (input_pw1 != pw1 || input_pw2 != pw2 || input_pw3 != pw3 || input_pw4 != pw4)
		   {
			   lcd_clear();
			   _delay_ms(10);
			   lcd_data(0,0); lcd_read('W'); lcd_read('R'); lcd_read('O'); lcd_read('N'); lcd_read('G'); lcd_read(' '); lcd_read('P'); lcd_read('W'); lcd_read('D');
			   _delay_ms(1000);
			   lcd_state = 0;
			   y = 1;
		   }
		   if (input_pw1 == pw1 && input_pw2 == pw2 && input_pw3 == pw3 && input_pw4 == pw4)
		   {
			   lcd_state = 4;
			   input_rpw1 = 0;
			   input_rpw2 = 0;
			   input_rpw3 = 0;
			   input_rpw4 = 0;
		   }
	   }
	   
	   if (lcd_state == 4)
	   {
		   if (input_rpw1 > 9) input_rpw1 = 0; if (input_rpw2 > 9) input_rpw2 = 0; if (input_rpw3 > 9) input_rpw3 = 0; if (input_rpw4 > 9) input_rpw4 = 0;
		   if (x < 1) x = 1;
		   if (x > 5) x = 5;
		   lcd_data(0,0); lcd_read('C'); lcd_read('H'); lcd_read('A'); lcd_read('N'); lcd_read('G'); lcd_read('E'); lcd_read(' '); lcd_read('P'); lcd_read('W'); lcd_read('D'); lcd_read(' ');
		   if(BIT_CHECK(pwd_insert_mode, 0) == 1)
		   {
			   lcd_data(0,0); lcd_read('I'); lcd_read('N'); lcd_read('S'); lcd_read('E'); lcd_read('R'); lcd_read('T'); lcd_read(' '); lcd_read('M'); lcd_read('O'); lcd_read('D'); lcd_read('E'); lcd_read(' '); lcd_read(' ');
		   }
		   lcd_data(1,0); lcd_read(' '); lcd_read(input_rpw1+'0'); lcd_read(' '); lcd_read(input_rpw2+'0'); lcd_read(' '); lcd_read(input_rpw3+'0'); lcd_read(' '); lcd_read(input_rpw4+'0'); lcd_read(' '); lcd_read('O'); lcd_read('K');
		   if (x == 1)
		   {
			   lcd_data(1,0); lcd_read('>');
			   if (BIT_CHECK(lcd_state_and_joystick,4) == 1)// 조이스틱을 눌렀을 때
			   {
				   BIT_TOGGLE(pwd_insert_mode, 0);
				   BIT_CLEAR(lcd_state_and_joystick, 4);
			   }
		   }
		   if (x == 2)
		   {
			   lcd_data(1,2); lcd_read('>');
			   if (BIT_CHECK(lcd_state_and_joystick,4) == 1)// 조이스틱을 눌렀을 때
			   {
				   BIT_TOGGLE(pwd_insert_mode, 0);
				   BIT_CLEAR(lcd_state_and_joystick, 4);
			   }
		   }
		   if (x == 3)
		   {
			   lcd_data(1,4); lcd_read('>');
			   if (BIT_CHECK(lcd_state_and_joystick,4) == 1)// 조이스틱을 눌렀을 때
			   {
				   BIT_TOGGLE(pwd_insert_mode, 0);
				   BIT_CLEAR(lcd_state_and_joystick, 4);
			   }
		   }
		   if (x == 4)
		   {
			   lcd_data(1,6); lcd_read('>');
			   if (BIT_CHECK(lcd_state_and_joystick,4) == 1)// 조이스틱을 눌렀을 때
			   {
				   BIT_TOGGLE(pwd_insert_mode, 0);
				   BIT_CLEAR(lcd_state_and_joystick, 4);
			   }
		   }
		   if (x == 5)
		   {
			   lcd_data(1,8); lcd_read('>');
			   if (BIT_CHECK(lcd_state_and_joystick,4) == 1)// 조이스틱을 눌렀을 때
			   {  //1.입력된 번호들을 비밀번호로 입력
				   pw1 = input_rpw1; pw2 = input_rpw2; pw3 = input_rpw3; pw4 = input_rpw4;
				   //2. EEPROM 0x10~0x40에 pw1~pw4 저장
				   EEPROM_Write(0x10,pw1); EEPROM_Write(0x20,pw2); EEPROM_Write(0x30,pw3); EEPROM_Write(0x40,pw4);
				   //3. usart통신을 이용하여 도어락에 비밀번호 전송을 위하여 array형태로 비밀번호를 변환
				   pw[0] = pw1; pw[1] = pw2; pw[2] = pw3; pw[3] = pw4;
				   //4. 도어락에 비밀번호 전송
				   send_Packet(0x02, 0x06, 0x04, pw);
				   //5. lcd_state = 1로 하여 초기화면으로 돌아감
				   lcd_state = 0;
				   //6. x = 1, y = 1로 하여 조이스틱 좌표 초기화
				   x = 1; y = 1;
				   BIT_CLEAR(lcd_state_and_joystick, 4); // 조이스틱 눌림 해제
			   }
		   }
	   }
	   if (lcd_state == 5)
	   {   if(x_lcd_state_5 < 0) x_lcd_state_5 = 0; if (x_lcd_state_5 > 4) x_lcd_state_5 = 4;
		   lcd_data(0,2); lcd_read('T'); lcd_read('Y'); lcd_read('P'); lcd_read('E'); lcd_data(0,11); lcd_read('S'); lcd_read('E'); lcd_read('T');
		   if(BIT_CHECK(env_insert_mode, 0) == 1)
		   {
			   lcd_data(0,0); lcd_read(' '); lcd_data(0,15); lcd_read(' ');
			   lcd_data(1,11); lcd_read('>');
		   }
		   if(x_lcd_state_5 == 0)
		   {
			   if(BIT_CHECK(env_insert_mode, 0) == 0)
			   {
				   lcd_data(0,0); lcd_read(' '); lcd_data(0,15); lcd_read('>');
				   lcd_data(1,11); lcd_read(' ');
			   }
			   lcd_data(1,0); lcd_read(' '); lcd_read('T'); lcd_read('e'); lcd_read('m'); lcd_read('p'); lcd_read(':'); lcd_read(T_10+'0'); lcd_read(T_01+'0'); lcd_read('.'); lcd_read(T_0p1+'0'); lcd_read(T_0p01+'0'); sprintf(str,"%03d",user_temp);
			   lcd_data(1,12); lcd_str(str);
			   if(user_temp < 18) user_temp = 18; if (user_temp > 30) user_temp = 30;
			   if (BIT_CHECK(lcd_state_and_joystick,4) == 1)
			   {
				   BIT_TOGGLE(env_insert_mode, 0);
				   BIT_CLEAR(lcd_state_and_joystick, 4);
			   }
			   
		   }
		   if(x_lcd_state_5 == 1)
		   {
			   if(BIT_CHECK(env_insert_mode, 0) == 0)
			   {
				   lcd_data(0,0); lcd_read('<'); lcd_data(0,15); lcd_read('>');
				   lcd_data(1,11); lcd_read(' ');
			   }
			   lcd_data(1,0); lcd_read(' '); lcd_read('H'); lcd_read('u'); lcd_read('m'); lcd_read('i'); lcd_read(':'); lcd_read(H_10+'0'); lcd_read(H_01+'0'); lcd_read('.'); lcd_read(H_0p1+'0'); lcd_read(H_0p01+'0'); sprintf(str,"%03d",user_hum);
			   lcd_data(1,12); lcd_str(str);
			   if(user_hum < 10) user_hum = 10; if (user_hum > 90) user_hum = 90;
			   if (BIT_CHECK(lcd_state_and_joystick,4) == 1)
			   {
				   BIT_TOGGLE(env_insert_mode, 0);
				   BIT_CLEAR(lcd_state_and_joystick, 4);
			   }
			   
		   }
		   if(x_lcd_state_5 == 2)
		   {
			   if(BIT_CHECK(env_insert_mode, 0) == 0)
			   {
				   lcd_data(0,0); lcd_read('<'); lcd_data(0,15); lcd_read('>');
				   lcd_data(1,11); lcd_read(' ');
			   }
			   lcd_data(1,0); lcd_read(' '); lcd_read('L'); lcd_read('u'); lcd_read('m'); lcd_read('i'); lcd_read(':'); lcd_read(Luminous_100+'0'); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); sprintf(str,"%03d",user_Luminous);
			   lcd_data(1,12); lcd_str(str);
			   if(user_Luminous < 1) user_Luminous = 1; if (user_Luminous > 9) user_Luminous = 9;
			   if (BIT_CHECK(lcd_state_and_joystick,4) == 1)
			   {
				   BIT_TOGGLE(env_insert_mode, 0);
				   BIT_CLEAR(lcd_state_and_joystick, 4);
			   }
		   }
		   if(x_lcd_state_5 == 3)
		   {
			   if(BIT_CHECK(env_insert_mode, 0) == 0)
			   {
				   lcd_data(0,0); lcd_read('<'); lcd_data(0,15); lcd_read('>');
				   lcd_data(1,11); lcd_read(' ');
			   }
			   lcd_data(1,0); lcd_read(' '); lcd_read('W'); lcd_read('a'); lcd_read('t'); lcd_read('e'); lcd_read('r'); lcd_read(' '); lcd_read('L'); lcd_read(':'); lcd_read(water_level_100+'0'); lcd_read(' '); sprintf(str,"%03d",user_water_level);
			   lcd_data(1,12); lcd_str(str);
			   if(user_water_level < 1) user_water_level = 1; if (user_water_level > 9) user_water_level = 9;
			   if (BIT_CHECK(lcd_state_and_joystick,4) == 1)
			   {
				   BIT_TOGGLE(env_insert_mode, 0);
				   BIT_CLEAR(lcd_state_and_joystick, 4);
			   }
		   }
		   if(x_lcd_state_5 == 4)
		   {
			   lcd_clear();
			   _delay_ms(10);
			   lcd_data(1,0); lcd_read('>'); lcd_read('B'); lcd_read('A'); lcd_read('C'); lcd_read('K');
			   lcd_data(1,11); lcd_read(' '); lcd_read(' '); lcd_read(' ');
			   if (BIT_CHECK(lcd_state_and_joystick,4) == 1)
			   {
				   lcd_state = 0; y = 0; x = 0;
				   BIT_CLEAR(lcd_state_and_joystick, 4);
			   }
		   }
		   
	   }
	   if(lcd_state == 6)
	   {
		   send_Packet(0x03, 0x06, 0x01, door_open);
		   lcd_clear();
		   _delay_ms(10);
		   lcd_data(0,0); lcd_read('D'); lcd_read('O'); lcd_read('O'); lcd_read('R'); lcd_read(' '); lcd_read('O'); lcd_read('P'); lcd_read('E'); lcd_read('N');
		   _delay_ms(2010);
		   lcd_state = 0; y = 0; x = 0;
	   }
   }
   if(mail_flag == 1 && gas_flag == 0)
   {
	   lcd_data(0,0); lcd_str("Mail arrived        ");
	   sprintf(str, "%04d %02d %03d     ", Sonar_range_buffer_plus, k, Sonar_range_avr);
	   lcd_data(1,0); lcd_str(str);
   }
   if(gas_flag == 1)
   {
	    lcd_data(0,0); lcd_str("Gas!!Gas!!Gas!! ");
	    lcd_data(1,0); lcd_str("Gas!!Gas!!Gas!! ");
   }
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//각 모듈 제어 부분------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void dht11_on_off(void)
{
   if (BIT_CHECK(mode, 7) == 1) // dht_11 on 일 때 코드 
   {
       dht11_work();
	   if(user_temp < T_integral) BIT_SET(mode, 0); // 설정 온도가 현재 온도보다 높을때 에어컨 on
	   if(user_temp > T_integral) BIT_CLEAR(mode, 0); //설정 온도가 현재 온도보다 낮을때 에어컨 off
	   if(user_hum > RH_integral ) BIT_SET(mode, 6); // 설정 습도가 현재 습도보다 높을 때 가습기 on
	   if(user_hum < RH_integral) BIT_CLEAR(mode, 6); // 설정 습도가 현재 습도보다 낮을 때 가습기 off
   }
}

void humidifier_auto_manual(void)
{
   if (BIT_CHECK(mode, 6) == 1) // 가습기 켜짐
   {
      ;
   }
   if (BIT_CHECK(mode, 6) == 0) // 가습기 꺼짐 
   {
      ;
   }
}

void light_sensor_on_off(void)
{
   if (BIT_CHECK(mode, 5) == 1) // 조도센서 자동모드 일 떄의 동작 코드 기술
   {
      if(Luminous_100 > user_Luminous) window_open();
	  if(Luminous_100 < user_Luminous) window_close();
   }
   if (BIT_CHECK(mode, 5) == 0) // 조도센서 수동모드 일 때의 동작 코드 기술
   {
      if(BIT_CHECK(mode,1) == 0)
	  {
		  window_stop();
	  }
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
	   SRF02_running();
	   if(Sonar_range_avr > 25)
	   {
		   mail_flag = 0;
	   }
	   if(Sonar_range_avr < 25 && Sonar_range_avr > 0)
	   {
		   mail_flag = 1;
	   } 
   }
   if (BIT_CHECK(mode, 3) == 0) // 우편물 감지 센서 off일 때의 동작 코드 기술
   {
      mail_flag = 0;
   }
}

void water_sensor_on_off(void)
{
   if (BIT_CHECK(mode, 2) == 1) // 수위 감지 센서 on일 떄의 동작 코드 기술
   {
      if(water_level_100 > user_water_level)
	  {
		  flooding_flag = 1;
	  }
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
      if(BIT_CHECK(PINC, 1) == 0)
	  {
		  window_open_and_close();
		  gas_flag = 1;
	  }
	  if(BIT_CHECK(PINC, 1) == 1)
	  {
		  gas_flag = 0;
		  if(BIT_CHECK(mode,5) == 0)
		  {
			  window_stop();
		  }
	  }
   }
   if (BIT_CHECK(mode, 1) == 0) // 가스 감지 센서 off일 때의 동작 코드 기술
   {
      gas_flag = 0;
   }
}

void aircon_on_off(void)
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
   DDRC = 0xf0;
   DDRF = 0x00; // ADC 변환 채널 
   DDRD|= 0x03; 
   DDRE = 0x00;
   DDRG = 0x07; 
   set_sleep_mode(SLEEP_MODE_IDLE);
   sleep_enable();
   sei(); // 인터럽트 활성화
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//테스트 함수 부분-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void pind_input(void) // PIND 의 입력 받는 함수
{
   sww = ~PIND & 0x80;
   while(~PIND & 0x80);
   _delay_ms(DEBOUNCING_DELAY);
   if (sww == 0x80)
   {
      BIT_SET(lcd_state_and_joystick,4);
   }
}

void pind_test_fix(void)
{
   if (PIND != 0xff)
   {
      sww = ~PIND & 0xff;
      _delay_ms(DEBOUNCING_DELAY);
         if (sww == 0x80)
         {
            BIT_SET(lcd_state_and_joystick,4);
            if(lcd_state == 2)
            {
               x++;
            }
         }
   }
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
int main(void)
{
   Set_pin_set_atmega128(); 
   reboot_count = EEPROM_Read(0x50);
   if(reboot_count > 100)
   {
      reboot_count = 1;
   }
   reboot_count++; // atmega128의 전원이 켜지면 1씩 증가시킴
   EEPROM_Write(0x50,reboot_count);  // 재부팅 횟수를 저장함
   USART0_init();
   ADC_init();
   lcd_pic();
   lcd_mv();
   lcd_func();
   lcd_clear();
   _delay_ms(10);
   
   init_timer1();
   //twi 부분
   init_timer0();
   Init_TWI();
   _delay_ms(1000);
   startRanging(Sonar_Addr);
   ti_Cnt_1ms = 0;
   LCD_DelCnt_1ms = 0;
   if(reboot_count >= 1) // 1번 이상 다시 켜졌으면 eeprom에 저장된 비밀번호를 불러옴 
   {
      pw1 = EEPROM_Read(0x10);
      pw2 = EEPROM_Read(0x20);
      pw3 = EEPROM_Read(0x30);
      pw4 = EEPROM_Read(0x40);
   }
    while (1) 
    {
	  if(flooding_flag == 0)
	  { // 침수가 되지 않았을 떄
		  //sleepmode();
		  adc_sensor();
		  joystick();
		  dht11_on_off();
		  humidifier_auto_manual();
		  light_sensor_on_off();
		  human_detect_on_off();
		  mail_on_off();
		  water_sensor_on_off();
		  gas_sensor_on_off();
		  aircon_on_off();
		  menu();
		  pind_input();
		  USART0_process();
		  PORTB = ~mode;
	  }
      if(flooding_flag == 1)
	  { // 침수가 되었을 때
		  DDRC = 0x40;
		  PORTC = 0x40;
	  }
	  
      	 
	 
    }
}
