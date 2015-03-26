#include <asf.h>
#include <stdio.h>
#include <string.h>
#define F_CPU 32000000UL
#include <util/delay.h>

#include "initialize.h"
#include "nrf24l01_L.h"
#include "nrf24l01_R.h"
#include "Menu.h"
#include "lcd.h"
#include "transmitter.h"

char Buf_Tx_R[Max_Robot][_Buffer_Size] ;
char Buf_Rx_R[Max_Robot][_Buffer_Size];
char Buff_L[_Buffer_Size];
char Buff_R[_Buffer_Size];
int tmprid;
int wireless_reset;

char Address[_Address_Width] = { 0x11, 0x22, 0x33, 0x44, 0x55};
char str[200];
uint8_t count;
int display_counter = 0;
int16_t M3RPM;
int test_motor;
int Robot_Select;
int time;
int timer=0;
int data_flag = 0;

uint16_t pck_timeout[Max_Robot];
//float P_temp,I_temp,D_temp,P,I,D,a=0,ki=0,kp=0.2,kd=0.07,M1,M1_temp;//ki=1.34,kp=1,kd=0.02,;
	
int main (void)
{
	En_RC32M();
	PORT_init();
	//LCDInit();
	TimerD0_init();
	PMIC_CTRL |=PMIC_LOLVLEN_bm|PMIC_MEDLVLEN_bm;

	wdt_enable();

	USART_R_init();
	USART_L_init();
	NRF24L01_L_CE_LOW;       //disable transceiver modes
	NRF24L01_R_CE_LOW;
	/////////////////////////////////////////////////////////////spi setting

	spi_xmega_set_baud_div(&NRF24L01_L_SPI,8000000UL,F_CPU);
	spi_enable_master_mode(&NRF24L01_L_SPI);
	spi_enable(&NRF24L01_L_SPI);

	spi_xmega_set_baud_div(&NRF24L01_R_SPI,8000000UL,F_CPU);
	spi_enable_master_mode(&NRF24L01_R_SPI);
	spi_enable(&NRF24L01_R_SPI);

	sei();

	////////////////////////////////////////////////////////////////////////

	_delay_us(10);
	//power on reset delay needs 100ms
	_delay_ms(100);
	NRF24L01_L_Clear_Interrupts();
	NRF24L01_R_Clear_Interrupts();
	NRF24L01_L_Flush_TX();
	NRF24L01_R_Flush_TX();
	NRF24L01_L_Flush_RX();
	NRF24L01_R_Flush_RX();

	NRF24L01_L_CE_LOW;
	NRF24L01_L_Init_milad(_TX_MODE, _CH_L, _2Mbps, Address, _Address_Width, _Buffer_Size, RF_PWR_MAX);
	NRF24L01_L_WriteReg(W_REGISTER | DYNPD,0x01);//0x07
	NRF24L01_L_WriteReg(W_REGISTER | FEATURE,0x06);//0x06
	NRF24L01_L_CE_HIGH;


	NRF24L01_R_CE_LOW;
	NRF24L01_R_Init_milad(_TX_MODE, _CH_R, _2Mbps, Address, _Address_Width, _Buffer_Size, RF_PWR_MAX);
	NRF24L01_R_WriteReg(W_REGISTER | DYNPD,0x01);
	NRF24L01_R_WriteReg(W_REGISTER | FEATURE,0x06);
	NRF24L01_R_CE_HIGH;

	_delay_us(130);

	for (uint8_t i=0;i<Max_Robot;i++)
	{
		Robot_D_tmp[i].RID=12;
	}

	Buff_L[31]='N';
	while (1)
	{
		if (data_flag ==1)
		{
			// 			count = sprintf(str,"%d,%d,%d,%d,%d\r",
			// 			((int)(Buf_Rx_R[Robot_Select][0]<<8) & 0xff00) | ((int)(Buf_Rx_R[Robot_Select][1]) & 0x0ff),
			// 			((int)(Buf_Rx_R[Robot_Select][2]<<8) & 0xff00) | ((int)(Buf_Rx_R[Robot_Select][3]) & 0x0ff),
			// 			((int)(Buf_Rx_R[Robot_Select][4]<<8) & 0xff00) | ((int)(Buf_Rx_R[Robot_Select][5]) & 0x0ff),
			// 			((int)(Buf_Rx_R[Robot_Select][6]<<8) & 0xff00) | ((int)(Buf_Rx_R[Robot_Select][7]) & 0x0ff),
			// 			 time);
			// 			for (uint8_t i=0;i<count;i++)
			// 			usart_putchar(&USARTE0,str[i]);
			//data_flag = 0 ;
		}
	
	
	}
}

		
ISR(TCD0_OVF_vect)
{
	wdt_reset();
	wireless_reset++;
	time++;
	if(time ==200)time=0;
	
	/////////////////////////////////packeting three robot data in one packet
	Buff_L[0]='L';
	for (int i = 1 ; i < 11 ; i++)
	{
		Buff_L[i] = Buf_Tx_R[6][i] ;
	}

	for (int i = 1 ; i < 11 ; i++)
	{
		Buff_L[i+10] = Buf_Tx_R[7][i] ;
	}
	
	for (int i = 1 ; i < 11 ; i++)
	{
		Buff_L[i+20] = Buf_Tx_R[8][i] ;
	}

	Buff_R[0]='L';
	for (int i = 1 ; i < 11 ; i++)
	{
		Buff_R[i] = Buf_Tx_R[3][i] ;
	}

	for (int i = 1 ; i < 11 ; i++)
	{
		Buff_R[i+10] = Buf_Tx_R[4][i] ;
	}
	
	for (int i = 1 ; i < 11 ; i++)
	{
		Buff_R[i+20] = Buf_Tx_R[5][i] ;
	}
	//////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////sending packet
	// 	if (wireless_reset==1)
	// 	{
	NRF24L01_L_Write_TX_Buf(Buff_L, _Buffer_Size);
	NRF24L01_L_RF_TX();
	
	NRF24L01_R_Write_TX_Buf(Buff_R, _Buffer_Size);
	if (Buff_R[29]!=0)
	{
		LED_White_R_PORT.OUTSET = LED_White_R_PIN_bm;
	}

	NRF24L01_R_RF_TX();
	wireless_reset = 0;
	/*	}*/
	////////////////////////////////////////////////////////////////////////////////////////
	
	

	for (uint8_t i=0;i<Max_Robot;i++)
	{
		//if there is no order from pc for a robot
		//an order of brake and then free wheel will
		//be send to that robot
		pck_timeout[i]++;
		if (pck_timeout[i]>=300)
		{
			if(pck_timeout[i]<=600)
			
			{//brake order
				Buf_Tx_R[i][0] = i;
				Buf_Tx_R[i][1] = 0;
				Buf_Tx_R[i][2] = 1;
				Buf_Tx_R[i][3] = 0;
				Buf_Tx_R[i][4] = 1;
				Buf_Tx_R[i][5] = 0;
				Buf_Tx_R[i][6] = 1;
				Buf_Tx_R[i][7] = 0;
				Buf_Tx_R[i][8] = 1;
				Buf_Tx_R[i][9] = 0;
				Buf_Tx_R[i][10] = 0;
			}
			else
			{//freewheel order
				Buf_Tx_R[i][0] = i;
				Buf_Tx_R[i][1] = 1;
				Buf_Tx_R[i][2] = 2;
				Buf_Tx_R[i][3] = 3;
				Buf_Tx_R[i][4] = 4;
				Buf_Tx_R[i][5] = 0;
				Buf_Tx_R[i][6] = time;
				Buf_Tx_R[i][7] = 0;
				Buf_Tx_R[i][8] = 0;
				Buf_Tx_R[i][9] = 0;
				Buf_Tx_R[i][10]= 0;
			}
		}
	}
	while(Menu_PORT.IN & Menu_Side_Switch_PIN_bm);
	wdt_reset();
}

ISR(PRX_R)
{
	wdt_reset();
	uint8_t status_R = NRF24L01_R_ReadReg(STATUSe);
	if((status_R & _RX_DR) == _RX_DR)
	{
		LED_White_R_PORT.OUTTGL = LED_White_R_PIN_bm;
		//		tmprid = ((status_R&0x0e)>>1);
		//1) read payload through SPI,
		NRF24L01_R_Read_RX_Buf(Buf_Rx_R[Robot_Select], _Buffer_Size);
		//2) clear RX_DR IRQ,
		status_R=NRF24L01_R_WriteReg(W_REGISTER | STATUSe, _RX_DR );
		//3) read FIFO_STATUS to check if there are more payloads available in RX FIFO,
		//4) if there are more data in RX FIFO, repeat from step 1).Buf_Tx_R
		count = sprintf(str,"%d,%d,%d,%d,%d\r",
		((int)(Buf_Rx_R[Robot_Select][0]<<8) & 0xff00) | ((int)(Buf_Rx_R[Robot_Select][1]) & 0x0ff),
		((int)(Buf_Rx_R[Robot_Select][2]<<8) & 0xff00) | ((int)(Buf_Rx_R[Robot_Select][3]) & 0x0ff),
		((int)(Buf_Rx_R[Robot_Select][4]<<8) & 0xff00) | ((int)(Buf_Rx_R[Robot_Select][5]) & 0x0ff),
		((int)(Buf_Rx_R[Robot_Select][6]<<8) & 0xff00) | ((int)(Buf_Rx_R[Robot_Select][7]) & 0x0ff),
		time);
		for (uint8_t i=0;i<count;i++)
		usart_putchar(&USARTE0,str[i]);
	}
	
	status_R = NRF24L01_R_WriteReg(W_REGISTER|STATUSe,_TX_DS|_MAX_RT);
	if((status_R&_TX_DS) == _TX_DS)
	{
		LED_Green_R_PORT.OUTTGL = LED_Green_R_PIN_bm;
	}
	if ((status_R&_MAX_RT) == _MAX_RT)
	{
		NRF24L01_R_Flush_TX();
	}
	data_flag = 1;
}

ISR(PRX_L)
{
	wdt_reset();
	uint8_t status_L = NRF24L01_L_ReadReg(STATUSe);
	if((status_L & _RX_DR) == _RX_DR)
	{
		LED_White_L_PORT.OUTTGL = LED_White_L_PIN_bm;
		//		tmprid = ((status_L&0x0e)>>1);
		//1) read payload through SPI,
		NRF24L01_L_Read_RX_Buf(Buf_Rx_R[Robot_Select], _Buffer_Size);
		//2) clear RX_DR IRQ,
		status_L=NRF24L01_L_WriteReg(W_REGISTER | STATUSe, _RX_DR );
		//3) read FIFO_STATUS to check if there are more payloads available in RX FIFO,
		//4) if there are more data in RX FIFO, repeat from step 1).
		count = sprintf(str,"%d,%d,%d,%d,%d\r",
		((int)(Buf_Rx_R[Robot_Select][0]<<8) & 0xff00) | ((int)(Buf_Rx_R[Robot_Select][1]) & 0x0ff),
		((int)(Buf_Rx_R[Robot_Select][2]<<8) & 0xff00) | ((int)(Buf_Rx_R[Robot_Select][3]) & 0x0ff),
		((int)(Buf_Rx_R[Robot_Select][4]<<8) & 0xff00) | ((int)(Buf_Rx_R[Robot_Select][5]) & 0x0ff),
		((int)(Buf_Rx_R[Robot_Select][6]<<8) & 0xff00) | ((int)(Buf_Rx_R[Robot_Select][7]) & 0x0ff),
		time);
		for (uint8_t i=0;i<count;i++)
		usart_putchar(&USARTE0,str[i]);
	}
	
	status_L = NRF24L01_L_WriteReg(W_REGISTER|STATUSe,_TX_DS|_MAX_RT);
	if((status_L&_TX_DS) == _TX_DS)
	{
		LED_Green_L_PORT.OUTTGL = LED_Green_L_PIN_bm;
	}
	if ((status_L&_MAX_RT) == _MAX_RT)
	{
		NRF24L01_L_Flush_TX();
	}
	data_flag = 1;
}

ISR(USART_R_RXC_vect)
{
	GetNewData(USARTC0_DATA);
}

ISR(USART_R_DRE_vect) //Wireless_R_USART
{

}

ISR(USART_L_RXC_vect)
{
	char data;
	data=USARTE0_DATA;


	switch (data)// used character : {w,s,123456(for robot id),!@#$(for motors id)}
	{
		
		case 'w':
		M3RPM+=10;
		Buf_Tx_R[0][7] = (char)((M3RPM>>8) & 0x0ff) ;
		Buf_Tx_R[0][8] = (char)(M3RPM & 0x0ff) ;
		count = sprintf(str,"M3RPM: %d\r",M3RPM);
		for (uint8_t i=0;i<count;i++)
		usart_putchar(&USARTE0,str[i]);
		break;
		
		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		Robot_Select = data - '0' ;
		if (Robot_Select < 3)
		{
			NRF24L01_L_WriteReg(W_REGISTER | EN_AA, 0x01);
		}
		else
		{
			NRF24L01_L_WriteReg(W_REGISTER | EN_AA, 0x00);
		}
		
		if (Robot_Select > 2)
		{
			NRF24L01_R_WriteReg(W_REGISTER | EN_AA, 0x01);
		}
		else
		{
			NRF24L01_R_WriteReg(W_REGISTER | EN_AA, 0x00);
		}

		Buff_L[31] = Robot_Select;
		Buff_R[31] = Robot_Select;
		break;
		
		case '`'://non of robots send ACK to wireless board
		Robot_Select = 7 ;//- '0';
		NRF24L01_L_WriteReg(W_REGISTER | EN_AA, 0x00);
		NRF24L01_R_WriteReg(W_REGISTER | EN_AA, 0x00);
		Buff_L[31] = data;
		Buff_R[31] = data;
		break;
		
		case '!':
		//		Robot_Motor = 0;
		// 		NRF24L01_L_WriteReg(W_REGISTER | EN_AA, 0x00);
		// 		Buff_L[31] = 0;
		break;
		
		case '@':
		//		Robot_Motor = 1;
		// 		Buff_L[31] = 1;
		// 		NRF24L01_L_WriteReg(W_REGISTER | EN_AA, 0x01);
		break;
		
		case '#':
		//		Robot_Motor = 2;
		break;
		
		case '$':
		//		Robot_Motor = 3;
		break;
		
	};

}
ISR(USART_L_DRE_vect) //Wireless_R_USART
{

}