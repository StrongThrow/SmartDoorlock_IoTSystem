#define F_CPU 14745600UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <string.h>
#include "UARTControl.h"
#include "RFIDControl.h"

#define DEBOUNCING_DELAY 50
#define REC_BUFF_MAX_LENGTH 100
#define BIT_SET(a,b) ((a) |= (1<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1<<(b)))
#define BIT_TOGGLE(a,b) ((a) ^= (1<<(b)))
#define BIT_CHECK(a,b) (!!((a) & (1<<(b))))

unsigned char rfid_ic_ver;
unsigned char card_byte_USER[4];
unsigned char card_USER1_USER2[8];
unsigned char card_buffer_for_open_door[4] = {1, 2, 3, 4};
unsigned char USER_WHO[1] = {0};
unsigned char card_check_buffer[4];



unsigned char RecBuff[REC_BUFF_MAX_LENGTH];
unsigned char mode = 0b00000000;
unsigned char lcd_state = 0;
unsigned char pw1, pw2, pw3, pw4 = 0;
unsigned char input_pw1, input_pw2, input_pw3, input_pw4;
unsigned char sww = 0;
unsigned char rec_pwd = 0;
volatile unsigned char flag;
volatile unsigned pwd_num= 0;
unsigned char input_number;
unsigned char input_signal = 0;
unsigned char A_input = 1;
unsigned char B_input = 2;
unsigned char C_input = 3;
unsigned char D_input = 4;
unsigned char pound_input = 5;
unsigned char star_input = 6;
unsigned char input_number_count = 0;
int16_t res_SPI = 0;


unsigned char RecBuff[REC_BUFF_MAX_LENGTH];
unsigned char RecBuffindex;
unsigned char RecBuff_estLength = REC_BUFF_MAX_LENGTH;
unsigned char RecFlg = 0;
unsigned char lRecBuff[100]; // 수신패킷 임시저장용 배열
unsigned char lRecBuffLength = 0;
unsigned int second_0p01 = 0;
int usart0_state = 0;


//EEPROM설정부분----------------------------------------------------
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

//타이머카운터 설정부분----------------------------------------------
void init_timer2(void) // 오버플로우 발생시 0.01초씩 증가, 타이머 카운터 2
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

void init_timer1(void) // 타이머 카운터 1, Fast PWM 16비트 타이머 사용, 4608번 세면 주기 20ms PB5 자동출력
{
	TCCR1A= 0x82; // PB5(OC1A)핀 자동출력, 컴패어 매치 때 클리어, TOP에 도달하였을 때 셋
	TCCR1B=0x1B; // 16비트 Fast PWM 모드 사용, 분주비 64 ICR1  = 4607일때 파형의 주기 20ms
	OCR1A=0;
	ICR1=4607;
}

//-----------------------------------------------------------------

//문 설정부분---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void door_open(void)
{
	OCR1A = 550; //  20% duty
}

void door_close(void)
{
	OCR1A = 230; // 10% duty
	_delay_ms(800);
	OCR1A = 0; // 다시 서보모터가 작동하면 안되므로 duty 0
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


//--------------------------------------------------------------------------------------------

//슬립 모드 설정부분------------------------------------------------
ISR(INT7_vect)
{
   second_0p01 = 0; // 사람이 감지되면 시간 초기화
   TCCR2 |=(1<<WGM21)|(1<<WGM20)|(1<<CS22)|(1<<CS20);
}

void sleepmode(void)
{
   if(second_0p01 > 1000)
   {  //10초가 넘으면 슬립모드 활성화
	  TCCR2 = 0;
      lcd_clear();
      _delay_ms(10);
      lcd_data(0,0); lcd_read('S'); lcd_read('L'); lcd_read('E'); lcd_read('E'); lcd_read('P'); lcd_read(' ');
      lcd_read('M'); lcd_read('O'); lcd_read('D'); lcd_read('E');
      sleep_cpu();
   }
}
//-----------------------------------------------------------------

//USART설정부분------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
ISR(USART0_RX_vect)
{
   second_0p01 = 0;
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
   if(recBuff[1] != 0x02) return -2; // 패킷검사 2. 장치의 이름이 1이 아닌 경우 종료, 0x01은 도어락, 0x02는 제어부 아트메가
   if(recBuff[length -1] != chksum(recBuff,length-1)) return -3;
   // 패킷검사 3. 패킷의 체크섬을 확인한뒤 일치하지 않은 경우 종료.
   // 수신된 체크섬 데이터는 recBuff[length -1]
   // 체크섬 계산 값은 체크섬 데이터 앞까지 더하는 chksum(recBuff,length-1)
   if( recBuff[3] == 0x06)
   { // 데이터가 쓰기인 경우
	   if(recBuff[2] == 0x02)
	   {
		   pw1 = RecBuff[5];
		   pw2 = RecBuff[6];
		   pw3 = RecBuff[7];
		   pw4 = RecBuff[8];
		   EEPROM_Write(0x10, pw1);
		   EEPROM_Write(0x11, pw2);
		   EEPROM_Write(0x12, pw3);
		   EEPROM_Write(0x13, pw4);
	   }
	   if(recBuff[2] == 0x03)
	   {
		   door_open();
		   lcd_clear();
		   _delay_ms(10);
		   lcd_data(0,0); lcd_read('D'); lcd_read('O'); lcd_read('O'); lcd_read('R'); lcd_read(' '); lcd_read('O'); lcd_read('P'); lcd_read('E'); lcd_read('N');
		   _delay_ms(2000);
		   door_close();
	   }
      
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
         ; // 상태 이상 알림
      }
   }
}

void send_Packet(unsigned char type, unsigned char rw, unsigned char length, unsigned char *sendData) // 데이터 보내는 함수
{
   unsigned char resBUFF[REC_BUFF_MAX_LENGTH]; // REC_BUFF_MAX_LENGTH는 데이터의 최대 길이, 코딩하고 마지막에 수정
   unsigned char resBUFFLength = 0;
   int i = 0;
   resBUFF[resBUFFLength++] = 0xff; // 시작 바이트 0xff
   resBUFF[resBUFFLength++] = 0x01; // 장치 이름 0x01은 도어락 0x02는 제어부, 이 장치는 제어부임으로 0x02 전송
   resBUFF[resBUFFLength++] = type; // 데이터의 종류, 0x02는 패스워드, 0x01은 유저 데이터 0x03은 문 열림 데이터
   resBUFF[resBUFFLength++] = rw;   // read, write 종류
   resBUFF[resBUFFLength++] = length; // 데이터들의 길이 절대 전체의 길이가 아님
   for(i = 0; i<length; i++)
   {
      resBUFF[resBUFFLength++] = sendData[i]; // 데이터를 보냄 send_Pachet(....... ,보낼 데이터)
   }
   resBUFF[resBUFFLength] = chksum(resBUFF, resBUFFLength); // 체크섬 계산 후 체크섬 전송
   resBUFFLength++;
   sendBuff_USART0(resBUFF, resBUFFLength); // 데이터 전송
}

//핀 및 기타 제어 부분----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Set_pin_set_atmega128(void)
{
   DDRA = 0xff;
   DDRC = 0xff; // 0~3번 모터 드라이버 4~8번 릴레이 모듈 제어
   DDRD = 0x00;
   DDRG = 0x07;
   DDRF = 0x0f;
   //DDRB = 0x27;

   
   //set_sleep_mode(SLEEP_MODE_IDLE);
   //sleep_enable();
   //EIMSK = 0x80; // IMT1번핀 외부 인터럽트 활성화
   //EICRB = 0xC0; // INT1번핀의 falling edge 때 인터럽트 요구
   
   sei(); // 인터럽트 활성화
   
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//테스트 함수 부분-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void num_pad_test(void)// PB4~PB7 => C1~C4 PB0~PD3 => R4~R1
{
   PORTF = ~0x01;
   _delay_ms(DEBOUNCING_DELAY); 
   if(!(PINF&0x10)) input_signal = star_input;
   if(!(PINF&0x20))
   {
      input_number = 7;//
      input_number_count++;
   }
   if(!(PINF&0x40))
   {
      input_number = 4;//
      input_number_count++;
   }
   if(!(PINF&0x80))
   {
      input_number = 1;//
      input_number_count++;
   }

   PORTF = ~0x02;
   _delay_ms(DEBOUNCING_DELAY);
   if(!(PINF&0x10)) 
    {
       input_number = 0;//
       input_number_count++;
    }
   if(!(PINF&0x20))
   {
      input_number = 8;//
      input_number_count++;
   }
   if(!(PINF&0x40))
   {
      input_number = 5;
      input_number_count++;
   }
   if(!(PINF&0x80))
   {
      input_number = 2;//
      input_number_count++;
   }

   
   PORTF = ~0x04;
   _delay_ms(DEBOUNCING_DELAY);
   if(!(PINF&0x10)) input_signal = pound_input; // #버튼을 눌렀을 때
   if(!(PINF&0x20))
   {
      input_number = 9;//
      input_number_count++;
   }
   if(!(PINF&0x40))
   {
      input_number = 6;//
      input_number_count++;
   }
   if(!(PINF&0x80))
   {
      input_number = 3;
      input_number_count++;
   }

   
   PORTF = ~0x08;
   _delay_ms(DEBOUNCING_DELAY);
   if(!(PINF&0x10)) input_signal = D_input;// D버튼을 눌렀을 때
   if(!(PINF&0x20)) input_signal = C_input; // C버튼 눌렀을 때
   if(!(PINF&0x40)) input_signal = B_input; //B버튼 눌렀을 때
   if(!(PINF&0x80)) input_signal = A_input; // A버튼 눌렀을 때
}

void input_pw(void)
{
   if (input_number_count == 1) input_pw1 = input_number;
   if (input_number_count == 2) input_pw2 = input_number;
   if (input_number_count == 3) input_pw3 = input_number;
   if (input_number_count == 4) input_pw4 = input_number;
   
}
//RFID설정부분------------------------------------------------------
int16_t RFID_loop()
{
	// Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle.
	if ( ! PICC_IsNewCardPresent()) {
		return 0;
	}
	
	// Select one of the cards
	if ( ! PICC_ReadCardSerial()) {
		return 1;
	}
	
	// Dump debug info about the card; PICC_HaltA() is automatically called
	PICC_DumpToSerial(&uid);
	return 2;
}

void RFID_settings(void)
{
	unsigned char rfid_ic_ver;
	
	SPI_Master_Init();
	Init_USART1_IntCon(9600);
	RFID_Init();
	
}

void RFID_new_reading()
{
	// RFID_loop()는 카드가 감지되면 2를 리턴함
	// 이떄 RFID_loop()함수 내부에서 UART로 카드정보 출력함.
	int f = 0;
	if (res_SPI == 2)
	{
		for(f = 0; f < 4; f++)
		{
			card_byte_USER[f] = uid.uidByte[f];
		}
		if(lcd_state == 3)
		{   // 사용자가 USER1 입력 모드 선택하여 card_USER1_USER2 [0~3]에 rfid 카드 정보 입력
			card_USER1_USER2[0] = card_byte_USER[0];  card_USER1_USER2[1] = card_byte_USER[1];
			card_USER1_USER2[2] = card_byte_USER[2];  card_USER1_USER2[3] = card_byte_USER[3]; 
			// EEPROM 0x01~0x04에 rfid 카드 정보 저장
			EEPROM_Write(0x01,card_USER1_USER2[0]);   EEPROM_Write(0x02,card_USER1_USER2[1]);
			EEPROM_Write(0x03,card_USER1_USER2[2]);   EEPROM_Write(0x04,card_USER1_USER2[3]);
			//card_check_buffer에 EEPROM에 저장한 카드 정보를 불러옴
			card_check_buffer[0] = EEPROM_Read(0x01); card_check_buffer[1] = EEPROM_Read(0x02);
			card_check_buffer[2] = EEPROM_Read(0x03); card_check_buffer[3] = EEPROM_Read(0x04);
			// EEPROM에 제대로 저장되었는지 확인 되면
			if(card_check_buffer[0] == card_USER1_USER2[0] && card_check_buffer[1] == card_USER1_USER2[1]&&
			card_check_buffer[2] == card_USER1_USER2[2] && card_check_buffer[3] == card_USER1_USER2[3])
			{   //카드 등록이 성공하였다는 메세지 출력 후 초기 화면으로 돌아감 이하 아래 lcd_state = 4의 경우도 동일하게 작동함.
				lcd_clear();
				_delay_ms(10);
				lcd_data(0,0); lcd_read('R'); lcd_read('F'); lcd_read('I'); lcd_read('D'); lcd_read(' '); lcd_read('I'); lcd_read('N'); lcd_read('F'); lcd_read('O'); lcd_read('R'); lcd_read('M'); lcd_read('A'); lcd_read('T'); lcd_read('I'); lcd_read('O'); lcd_read('N');
				lcd_data(1,0); lcd_read('S'); lcd_read('A'); lcd_read('V'); lcd_read('E'); lcd_read('D'); lcd_read(' '); lcd_read('S'); lcd_read('U'); lcd_read('C'); lcd_read('C'); lcd_read('E'); lcd_read('E'); lcd_read('D');
				_delay_ms(2000);
				lcd_state = 0; input_number_count = 0; input_signal = 0;
			}
			
		}
		if(lcd_state == 4)
		{
			card_USER1_USER2[4] = card_byte_USER[0];  card_USER1_USER2[5] = card_byte_USER[1];
			card_USER1_USER2[6] = card_byte_USER[2];  card_USER1_USER2[7] = card_byte_USER[3];
			EEPROM_Write(0x05,card_USER1_USER2[4]);   EEPROM_Write(0x06,card_USER1_USER2[5]);
			EEPROM_Write(0x07,card_USER1_USER2[6]);   EEPROM_Write(0x08,card_USER1_USER2[7]);
			card_check_buffer[0] = EEPROM_Read(0x05); card_check_buffer[1] = EEPROM_Read(0x06);
			card_check_buffer[2] = EEPROM_Read(0x07); card_check_buffer[3] = EEPROM_Read(0x08);
			if(card_check_buffer[0] == card_USER1_USER2[4] && card_check_buffer[1] == card_USER1_USER2[5]&&
			card_check_buffer[2] == card_USER1_USER2[6] && card_check_buffer[3] == card_USER1_USER2[7])
			{
				lcd_clear();
				_delay_ms(10);
				lcd_data(0,0); lcd_read('R'); lcd_read('F'); lcd_read('I'); lcd_read('D'); lcd_read(' '); lcd_read('I'); lcd_read('N'); lcd_read('F'); lcd_read('O'); lcd_read('R'); lcd_read('M'); lcd_read('A'); lcd_read('T'); lcd_read('I'); lcd_read('O'); lcd_read('N');
				lcd_data(1,0); lcd_read('S'); lcd_read('A'); lcd_read('V'); lcd_read('E'); lcd_read('D'); lcd_read(' '); lcd_read('S'); lcd_read('U'); lcd_read('C'); lcd_read('C'); lcd_read('E'); lcd_read('E'); lcd_read('D');
				_delay_ms(2000);
				lcd_state = 0; input_number_count = 0; input_signal = 0;
			}
			
		}
	}
	
}

void RFID_opendoor(void)
{
	unsigned char c;
	if (res_SPI == 2)
	{
		for(c = 0; c < 4; c++)
		{
			card_buffer_for_open_door[c] = uid.uidByte[c];
		}
		if (card_buffer_for_open_door[0] == card_USER1_USER2[0] && card_buffer_for_open_door[1] == card_USER1_USER2[1] &&
		card_buffer_for_open_door[2] == card_USER1_USER2[2] && card_buffer_for_open_door[3] == card_USER1_USER2[3])
		{
			door_open();
			lcd_clear();
			_delay_ms(10);
			lcd_data(0,0); lcd_read('D'); lcd_read('O'); lcd_read('O'); lcd_read('R'); lcd_read(' '); lcd_read('O'); lcd_read('P'); lcd_read('E'); lcd_read('N');
			lcd_data(1,0); lcd_read('W'); lcd_read('E'); lcd_read('L'); lcd_read('C'); lcd_read('O'); lcd_read('M'); lcd_read('E'); lcd_read(' '); lcd_read('U'); lcd_read('S'); lcd_read('E'); lcd_read('R'); lcd_read('1');
			_delay_ms(2000);
			door_close();
			USER_WHO[0] = 2;
			send_Packet(0x01, 0x06, 0x01, USER_WHO);
			input_number_count = 0; input_signal = 0; lcd_state = 0;
			
		}
		if (card_buffer_for_open_door[0] == card_USER1_USER2[4] && card_buffer_for_open_door[1] == card_USER1_USER2[5] &&
		card_buffer_for_open_door[2] == card_USER1_USER2[6] && card_buffer_for_open_door[3] == card_USER1_USER2[7])
		{
			door_open();
			lcd_clear();
			_delay_ms(10);
			lcd_data(0,0); lcd_read('D'); lcd_read('O'); lcd_read('O'); lcd_read('R'); lcd_read(' '); lcd_read('O'); lcd_read('P'); lcd_read('E'); lcd_read('N');
			lcd_data(1,0); lcd_read('W'); lcd_read('E'); lcd_read('L'); lcd_read('C'); lcd_read('O'); lcd_read('M'); lcd_read('E'); lcd_read(' '); lcd_read('U'); lcd_read('S'); lcd_read('E'); lcd_read('R'); lcd_read('2');
			_delay_ms(2000);
			door_close();
			USER_WHO[0] = 3;
			send_Packet(0x01, 0x06, 0x01, USER_WHO);
			input_number_count = 0; input_signal = 0; lcd_state = 0;
			
		}
	}
}
//-----------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void menu(void)
{
   if (lcd_state == 0)
   {
      if(input_number_count == 0)
      {
         lcd_data(0,0); lcd_read('T'); lcd_read('A'); lcd_read('G'); lcd_read(' '); lcd_read('R'); lcd_read('F'); lcd_read('I'); lcd_read('D'); lcd_read(' '); lcd_read('O'); lcd_read('R'); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' ');
         lcd_data(1,0); lcd_read('P'); lcd_read('U'); lcd_read('T'); lcd_read(' '); lcd_read('P'); lcd_read('W'); lcd_read('D'); lcd_read(':'); lcd_read('_'); lcd_read('_'); lcd_read('_'); lcd_read('_'); lcd_read(' ');
      }
      if (input_number_count == 1)
      {
         lcd_data(1,8); lcd_read(input_pw1+'0');
      }
      if (input_number_count == 2)
      {
         lcd_data(1,9); lcd_read(input_pw2+'0');
      }
      if (input_number_count == 3)
      {
         lcd_data(1,10); lcd_read(input_pw3+'0');
      }
      if (input_number_count == 4)
      {
         lcd_data(1,11); lcd_read(input_pw4+'0');  
		 if(input_signal == star_input)
		 {
			 lcd_state = 1;
			 input_signal = 0;
		 }
		 if(input_signal == pound_input)
		 {
			 lcd_state = 2;
			 input_signal = 0;
		 }
      }
	  RFID_opendoor();
   }
   
   if (lcd_state == 1)
   {
	   if (input_pw1 != pw1 || input_pw2 != pw2 || input_pw3 != pw3 || input_pw4 != pw4)
	   {
		   lcd_clear();
		   _delay_ms(10);
		   lcd_data(0,0); lcd_read('W'); lcd_read('R'); lcd_read('O'); lcd_read('N'); lcd_read('G'); lcd_read(' '); lcd_read('P'); lcd_read('W'); lcd_read('D');
		   _delay_ms(1000);
		   lcd_state = 0; input_number_count = 0; input_signal = 0;
	   }
	   if (input_pw1 == pw1 && input_pw2 == pw2 && input_pw3 == pw3 && input_pw4 == pw4)
	   {
		   door_open();
		   lcd_clear();
		   _delay_ms(10);
		   lcd_data(0,0); lcd_read('D'); lcd_read('O'); lcd_read('O'); lcd_read('R'); lcd_read(' '); lcd_read('O'); lcd_read('P'); lcd_read('E'); lcd_read('N');
		   lcd_data(1,0); lcd_read('W'); lcd_read('E'); lcd_read('L'); lcd_read('C'); lcd_read('O'); lcd_read('M'); lcd_read('E');
		   _delay_ms(2000);
		   door_close();
		   input_number_count = 0; input_signal = 0; lcd_state = 0;
		   USER_WHO[0] = 1;
		   send_Packet(0x01, 0x06, 0x01, USER_WHO);
		   
	   }
   }
   
   
   if (lcd_state == 2)
   {
	   if (input_pw1 != pw1 || input_pw2 != pw2 || input_pw3 != pw3 || input_pw4 != pw4)
	   {
		   lcd_clear();
		   _delay_ms(10);
		   lcd_data(0,0); lcd_read('W'); lcd_read('R'); lcd_read('O'); lcd_read('N'); lcd_read('G'); lcd_read(' '); lcd_read('P'); lcd_read('W'); lcd_read('D');
		   _delay_ms(1000);
		   lcd_state = 0; input_number_count = 0; input_signal = 0;
	   }
	   if (input_pw1 == pw1 && input_pw2 == pw2 && input_pw3 == pw3 && input_pw4 == pw4)
	   {
		    lcd_data(0,0); lcd_read('A'); lcd_read('.'); lcd_read('U'); lcd_read('S'); lcd_read('E'); lcd_read('R'); lcd_read('1'); lcd_read(' '); lcd_read('B'); lcd_read('.'); lcd_read('U'); lcd_read('S'); lcd_read('E'); lcd_read('R'); lcd_read('2');
		    lcd_data(1,0); lcd_read('D'); lcd_read('.'); lcd_read('R'); lcd_read('E'); lcd_read('S'); lcd_read('E'); lcd_read('T'); lcd_read(' '); lcd_read('R'); lcd_read('F'); lcd_read('I'); lcd_read('D'); lcd_read(' '); lcd_read(' ');
		    if(input_signal == A_input)
		    {
			    lcd_state = 3; input_number_count = 0; input_signal = 0;
		    }
			if(input_signal == B_input)
			{
				lcd_state = 4; input_number_count = 0; input_signal = 0;
			}
			if(input_signal == D_input)
			{
				unsigned char e;
				for(e = 0; e<8; e++)
				{
					card_USER1_USER2[e] = 0;
					EEPROM_Write(e+1, 0);
					lcd_state = 0; input_number_count = 0; input_signal = 0;
				}
			}
	   }
	}
	if(lcd_state == 3)
	{
		lcd_data(0,0); lcd_read('T'); lcd_read('A'); lcd_read('G'); lcd_read(' '); lcd_read('N'); lcd_read('E'); lcd_read('W'); lcd_read(' '); lcd_read('C'); lcd_read('A'); lcd_read('R'); lcd_read('D'); lcd_read(' '); lcd_read(' '); lcd_read(' ');
		lcd_data(1,0); lcd_read('B'); lcd_read('A'); lcd_read('C'); lcd_read('K'); lcd_read(':'); lcd_read('#'); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' ');
		RFID_new_reading();
		if(input_signal == pound_input)
		{
			input_number_count = 0; input_signal = 0; lcd_state = 0;
		}
	}
	
	if(lcd_state == 4)
	{
		lcd_data(0,0); lcd_read('T'); lcd_read('A'); lcd_read('G'); lcd_read(' '); lcd_read('N'); lcd_read('E'); lcd_read('W'); lcd_read(' '); lcd_read('C'); lcd_read('A'); lcd_read('R'); lcd_read('D'); lcd_read(' '); lcd_read(' '); lcd_read(' ');
		lcd_data(1,0); lcd_read('B'); lcd_read('A'); lcd_read('C'); lcd_read('K'); lcd_read(':'); lcd_read('#'); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' '); lcd_read(' ');
		RFID_new_reading();
		if(input_signal == pound_input)
		{
			input_number_count = 0; input_signal = 0; lcd_state = 0;
		}
	}

}

int main(void)
{
   /*EEPROM_Write(0x10, 0);
   EEPROM_Write(0x11, 0);
   EEPROM_Write(0x12, 0);
   EEPROM_Write(0x13, 0);*/ //비밀번호 초기화 코드 각주 지우고 사용하면 EEPROM에 0000저장
   
   pw1 = EEPROM_Read(0x10);
   pw2 = EEPROM_Read(0x11);
   pw3 = EEPROM_Read(0x12);
   pw4 = EEPROM_Read(0x13);
   // 도어락이 만약 꺼져도 eeprom에 저장된 비밀번호를 불러옴, 비밀번호 변경 데이터가 수신되면
   // eeprom에 각 비밀번호가 저장됨.
   
   // 기존 사용자의 RFID 정보를 EEPROM으로부터 읽어옴 USER1는 0x01~0x04, USER2는 0x05~0x08에 정보 저장됨
   card_USER1_USER2[0] = EEPROM_Read(0x01); card_USER1_USER2[1] = EEPROM_Read(0x02); card_USER1_USER2[2] = EEPROM_Read(0x03); card_USER1_USER2[3] = EEPROM_Read(0x04);
   card_USER1_USER2[4] = EEPROM_Read(0x05); card_USER1_USER2[5] = EEPROM_Read(0x06); card_USER1_USER2[6] = EEPROM_Read(0x07); card_USER1_USER2[7] = EEPROM_Read(0x08);
   // EEPROM에 저장되었던 카드 2개의 정보를 읽어 태그하는 카드와 비교 할 수 있게 함
   
   Set_pin_set_atmega128();
   USART0_init();
   init_timer1();
   init_timer2();
   
   RFID_settings();
   lcd_pic();
   lcd_mv();
   lcd_func();
   rfid_ic_ver = PCD_DumpVersionToSerial();
   DDRB |= 0x20; // RFID에서 DDRB 레지스터 제어하기 때문에 자동출력 PORTB5번핀 출력 활성화 
    lcd_clear();
    _delay_ms(10);
   while (1)
   {
      //sleepmode();
      menu();
      num_pad_test();
      input_pw();
      USART0_process();
	  res_SPI = RFID_loop();
	  if(input_number_count > 4 ) input_number_count = 4;
	 
   }
}