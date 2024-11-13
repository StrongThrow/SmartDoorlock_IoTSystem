#define F_CPU 14745600UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include <stdio.h>
#include <string.h>
#include "lcd.h"
#include "RFIDControl.h"
#include "UARTControl.h"

#define  max_card_byte_pcs 100
unsigned char card_byte[max_card_byte_pcs];
unsigned char card_byte_buffer_0 = 0;
unsigned char card_byte_buffer_1 = 0;
unsigned char card_byte_buffer_2 = 0;
unsigned char card_byte_buffer_3 = 0;
unsigned char User = 0;
unsigned char User_1 = 1;
unsigned char User_2 = 2;

char STR_forLCD[20]; // LCD 출력 문자 저장 위함

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

int16_t RFID_loop() {
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


int main(void)
{
    int f=0;
    int16_t res_SPI = 0;
    char str[100];
    unsigned char rfid_ic_ver;
    
    SPI_Master_Init();
    LCD_Init(); // LCD 초기화
    Init_USART1_IntCon(9600);
    
    
    LCD_Pos(0,0);
    LCD_Str("HELLO LCD");     // LCD 동작 확인을 위함
    _delay_ms(2000);
    LOG_str("HELLOUART\r\n");  // UART 통신 동작 확인을 위함
    _delay_ms(2000);
    
    // RFID 초기화
    RFID_Init();
    while (1) 
    {
        
        // RFID_loop()는 카드가 감지되면 2를 리턴함
        // 이떄 RFID_loop()함수 내부에서 UART로 카드정보 출력함.
        // LCD에 출력하기 위해서 아래 LCD 출력함수 작성홤. 
        res_SPI = RFID_loop();
        
		//rfid 카드 새로 등록 모드일 때, 사용자 1,2 각 두명의 rfid 카드 정보를 등록할 수 있도록 함 USER1, USER2 또한 
        if(res_SPI == 2)
		{
			EEPROM_Read = 
	        for(f = 0; f < 4; f++)
			{
				card_byte[f] = uid.uidByte[f];
				EEPROM_Write(f+1, card_byte[f]);
			}
			
        }
    }
}

