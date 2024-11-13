#define F_CPU 14745600UL

#include <avr/io.h>

#include <avr/interrupt.h>

#include <util/delay.h>​

 

#include "TWI.h"

#include "lcd.h"

 

#define COM_REG 0

#define SRF02_1st_Seq_change 160

#define SRF02_2nd_Seq_change 170

#define SRF02_3rd_Seq_change 165

#define SRF02_Return_inch 80

#define SRF02_Return_Cm 81

#define SRF02_Return_microSecond 82

 

unsigned char ti_Cnt_1ms;

unsigned char LCD_DelCnt_1ms;

 

ISR(TIMER0_COMP_vect)

{

   ti_Cnt_1ms ++;

   LCD_DelCnt_1ms ++;

}

 

void Timer0_Init()   //타이머 기본 설정

{

   TCCR0 = (1<<WGM01)|(1<<CS00)|(1<<CS01)|(1<<CS02);

   TCNT0 = 0x00;

   OCR2 = 14;

   TIMSK = (1<<OCIE0);

}

 

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

   PORTB|= 0x08;

   if(ret_err !=0) return ret_err;

   

   ret_err = TWI_Write_SLAR(address);

   PORTB |= 0x10;

   if(ret_err !=0) return ret_err;

   

   ret_err = TWI_Read_Data_NACK(&read_data);

   PORTB |= 0x20;

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

   unsigned char temp;

   unsigned char res = 0;

   res = SRF02_I2C_Read(addr,2,&temp);

   if(res) return res;

   *pDistance = temp << 8;

   res = SRF02_I2C_Read(addr,3,&temp);

   if(res) return res;

   *pDistance |= temp;

   

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

 

 

void main(void)

{

   char Sonar_Addr = 0xE2;

   unsigned int Sonar_range;

   char Message[40];

   int readCnt = 0;

   unsigned char res = 0;

   DDRD |= 0x03;

   

   LCD_Init();

   Timer0_Init();

   Init_TWI();

   _delay_ms(1000);

   sei();

   

   startRanging(Sonar_Addr);

   ti_Cnt_1ms = 0;

   LCD_DelCnt_1ms = 0;

   while (1)

   {

      if(ti_Cnt_1ms > 66){  //66ms이상일 때 동작 // 충분한 시간이 있어야함

         res = getRange(Sonar_Addr, &Sonar_range);

         /*if(res)

         {

            LCD_Pos(0,0);

            LCD_Str("Measured Dist.= ");

            

            LCD_Pos(1,5);

            LCD_Str("ERR       ");

            

         }

         else if(LCD_DelCnt_1ms > 66)

         {

            LCD_Pos(0,0);

            LCD_Str("Measured Dist.= ");

            

            sprintf(Message, "%01d  %03d cm",readCnt,Sonar_range);

            LCD_Pos(1,5);  //거리는 센티미터 단위로 출력

            LCD_Str(Message);

            

            LCD_DelCnt_1ms = 0;

         }*/

         if(LCD_DelCnt_1ms > 66)

         {

            LCD_Pos(0,0);

            LCD_Str("Measured Dist.= ");

            

            sprintf(Message, "%01d  %03d cm",readCnt,Sonar_range);

            LCD_Pos(1,0);  //거리는 센티미터 단위로 출력

            LCD_Str(Message);

            

            LCD_DelCnt_1ms = 0;

         }

         

         startRanging(Sonar_Addr);

         ti_Cnt_1ms = 0;

         readCnt = (readCnt + 1)%10;

      }

   }

}