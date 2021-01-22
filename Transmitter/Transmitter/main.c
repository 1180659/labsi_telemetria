#define F_CPU    10000000UL //para j�

#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <avr/interrupt.h> 
#include <util/delay.h>
#include <inttypes.h>									/* Include integer type header file */									/* Include standard library file */
#include "MPU6050_res_define.h"							/* Include MPU6050 register define file */
#include "I2C_Master_H_file.h"							/* Include I2C Master header file */

float Acc_x,Acc_y,Acc_z,Temperature,Gyro_x,Gyro_y,Gyro_z;

#define SCK    5
#define MISO   4
#define MOSI   3
#define SS_NRF 2
#define CE     1


#define LEDstatus 3
#define LEDerror 2
#define IRQ 3

#define CONFIGnrf      0x00 +0x20 //(soma-se 0x20 pois o comando W_REGISTER = 001X XXXX,
#define EN_AA       0x01 +0x20 // sendo que XXXXX � o binario do address em que queremos escrever)
#define EN_RXADDR   0x02 +0x20
#define SETUP_AW    0x03 +0x20
#define SETUP_RETR  0x04 +0x20
#define RF_CH       0x05 +0x20
#define RF_SETUP    0x06 +0x20
#define STATUS      0x07 +0x20
#define RX_ADDR_P0  0x0A +0x20
#define TX_ADDR     0x10 +0x20 //(para ler os registos vamos subtrair 0x20 aos valores destes #defines
#define RX_PW_P0    0x11 +0x20 //pois o comando R_REGISTER = 000X XXXX)
#define FIFO_STATUS 0x17 +0x20

#define R_RX_PAYLOAD 0b01100001
#define W_TX_PAYLOAD 0b10100000
#define FLUSH_TX     0b11100001
#define FLUSH_RX     0b11100010


//#define END_RX_Byte2  0x69
//#define END_RX_Byte1  0xED
//#define END_RX_Byte0  0x75

#define END_TX_Byte2  0x69
#define END_TX_Byte1  0xED
#define END_TX_Byte0  0x75


//volatile uint16_t sample12bit = 1; //ser� que nao � necessario o valatile

void config_portas_atmega()
{
	//PORTS:
	DDRB = 0;
	DDRB |= (1<<SCK) | (1<<MOSI) | (1<<SS_NRF) | (1<<CE); //outputs
	DDRB &= ~(1<<MISO); //input (vem do adc e do nrf)
	
	
	PORTB = 0;
	PORTB |= (1<<SS_NRF); //come�am inativos os salves selects
	PORTB &= ~(1<<CE); //zero o pin Chip Enable (CE)
	
	DDRD = 0;	
	DDRC |= (1<<LEDerror) | (1<<LEDstatus); //LED output para teste ( + LED para ver se o status disse que o TX_FIFO foi enviado)

	DDRD &= ~(1<<IRQ);

	PORTC &= ~(1<<LEDstatus); //LED come�a desligado

}

void config_spi_atmega()
{
//SPI:

	//SPCR |= (1<<SPIE) | (1<<SPE) | (0<<DORD) | (1<<MSTR) | (0<<CPOL) | (0<<CPHA) | (0<<SPR1) | (0<<SPR0); MODO 0
	SPCR = 0b11010000; //SPIE com interrupt n�? assim sempre que l� mandamos fazer algo? SPE enable spi. DORD primeiro o MSB e no fim o LSB. MSTR definir para ser o master. Sck freq = Fosc/2

	//SPSR |= (1<<SPIF) | (0<<WCOL) | (1<<SPI2X);
	SPSR = 0b10000001; //com a flag SPIF para sabermos quando terminou de enviar a data pelo SPDR. Sem colision flag (WCOL). Double SPI speed bit a 1 para a Sck freq = Fosc/2
}





uint8_t spi_read(uint8_t junkdata)
{
	SPDR = junkdata; // Write data to SPI data register //Mandamos por que ordem
	while(!(SPSR & (1<<SPIF)));		// Wait till transmission complete 
	return(SPDR);
}


void spi_write(uint8_t data)
{
	SPDR = data;					/* Write data to SPI data register */ //Mandamos por que ordem???
	while(!(SPSR & (1<<SPIF)));		/* Wait till transmission complete */
}


void spi_write_x_bytes(char data[])
{
	
	int i;
	
	for(i=0;i<31;i++)
	{
		SPDR = data[i];
		while(!(SPSR & (1<<SPIF)));		
	}
	
}

void configuracao_do_nrf24L01_TX()
{
	
	PORTB &= ~(1<<CE);
	
	_delay_ms(12); // tempo de espera após alimentação do modulo
	
	PORTB &= ~(1<<SS_NRF); //SETUP_AW
	spi_write(SETUP_AW);
	spi_write(0b00000001); //3 bytes para RX e TX address (o minimo)
	PORTB |= (1<<SS_NRF);
		
	PORTB &= ~(1<<SS_NRF); //RX_ADDR_P0 no recetor address de 3 bytes
	spi_write(RX_ADDR_P0);
	spi_write(END_TX_Byte2);
	spi_write(END_TX_Byte1);
	spi_write(END_TX_Byte0);
	PORTB |= (1<<SS_NRF);
	
	
	PORTB &= ~(1<<SS_NRF); //TX_ADDR address de 3 bytes
	spi_write(TX_ADDR);
	spi_write(END_TX_Byte2);
	spi_write(END_TX_Byte1);
	spi_write(END_TX_Byte0);
	PORTB |= (1<<SS_NRF);
	
	
	PORTB &= ~(1<<SS_NRF); //EN_AA
	spi_write(EN_AA);
	spi_write(0); //NO EN_AA
	PORTB |= (1<<SS_NRF);
	

	PORTB &= ~(1<<SS_NRF); //EN_RXADDR
	spi_write(EN_RXADDR);
	spi_write(0b00000001); //enable of data pipe 0
	PORTB |= (1<<SS_NRF);


	PORTB &= ~(1<<SS_NRF); //SETUP_RETR 
	spi_write(SETUP_RETR);
	spi_write(0b00000000); //Re-Transmit disabled
	PORTB |= (1<<SS_NRF);
	

	PORTB &= ~(1<<SS_NRF); //RF_CH
	spi_write(RF_CH);
	spi_write(0); //Channel frequency at 2.4GHZ + 0MHZ = 2.40GHz
	PORTB |= (1<<SS_NRF);
	

	PORTB &= ~(1<<SS_NRF); //RF_SETUP
	spi_write(RF_SETUP);
	spi_write(0b00000110); //air data rate = 1Mbps, output power in TX mode = 0dBm, "setup LNA gain disabled"
	PORTB |= (1<<SS_NRF);
	

	PORTB &= ~(1<<SS_NRF); //RX_PW_P0
	spi_write(RX_PW_P0);
	spi_write(0b00000001);  //1 bytes
	PORTB |= (1<<SS_NRF);
	
	
	PORTB &= ~(1<<SS_NRF); //CONFIG
	spi_write(CONFIGnrf);
	spi_write(0b01011010); //CRC (des)ligado, Power UP, modo PTX, IRQ -> MASK_TX_DS  /* spi_write(0b01010010); //CRC ligado, Power UP, modo PTX //*/ 
	PORTB |= (1<<SS_NRF);
	
	_delay_ms(2); //tempo para entrar em standby após power up
}

void MPU6050_Init()										/* Gyro initialization function */
{
	_delay_ms(150);										/* Power up time >100ms */
	I2C_Start_Wait(0xD0);								/* Start with device write address */
	I2C_Write(SMPLRT_DIV);								/* Write to sample rate register */
	I2C_Write(0x07);									/* 1KHz sample rate */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(PWR_MGMT_1);								/* Write to power management register */
	I2C_Write(0x01);									/* X axis gyroscope reference frequency */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(CONFIG);									/* Write to Configuration register */
	I2C_Write(0x00);									/* Fs = 8KHz */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(GYRO_CONFIG);								/* Write to Gyro configuration register */
	I2C_Write(0x18);									/* Full scale range +/- 2000 degree/C */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(INT_ENABLE);								/* Write to interrupt enable register */
	I2C_Write(0x01);
	I2C_Stop();
}

void MPU_Start_Loc()
{
	I2C_Start_Wait(0xD0);								/* I2C start with device write address */
	I2C_Write(ACCEL_XOUT_H);							/* Write start location address from where to read */
	I2C_Repeated_Start(0xD1);							/* I2C start with device read address */
}

void Read_RawValue()
{
	MPU_Start_Loc();									/* Read Gyro values */
	Acc_x = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Acc_y = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Acc_z = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Temperature = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Gyro_x = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Gyro_y = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Gyro_z = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Nack());
	I2C_Stop();
}

void enviar_sample(char data[]) 
{
	
	
	
	PORTB &= ~(1<<CE);
	
	
	PORTB &= ~(1<<SS_NRF);
	spi_write(FLUSH_TX); //FLUSH_TX
	PORTB |= (1<<SS_NRF);
	
	
	PORTB &= ~(1<<SS_NRF); //ENVIAR SAMPLE
	spi_write(W_TX_PAYLOAD); //W_TX_PAYLOAD
	spi_write_x_bytes(data);
	PORTB |= (1<<SS_NRF);
	
	
	PORTB |= (1<<CE);
	_delay_us(10); 
	PORTB &= ~(1<<CE);
								
								
								while ( (PIND & (1<<IRQ)) ) //enquanto fifo n�o for limpo (enviado)
								{
									PORTC &= ~(1<<LEDstatus);
									PORTC |= (1<<LEDerror);
									_delay_us(5);
									
								}
									
								PORTC &= ~(1<<LEDerror);	
								PORTC |= (1<<LEDstatus); 
									
								
									
								
								
	PORTB &= ~(1<<SS_NRF); //STATUS dar reset/limpar
	spi_write(STATUS);
	spi_write(0b00100000); //clear bit do TX FIFO
	PORTB |= (1<<SS_NRF);	
	
	
	PORTB &= ~(1<<SS_NRF);
	spi_write(FLUSH_TX); //FLUSH_TX
	PORTB |= (1<<SS_NRF);

}



int main(void)
{

	
	PORTB &= ~(1<<CE);
	
	
	float Xa,Ya,Za,t;
	float Xg=0,Yg=0,Zg=0;
	
	config_portas_atmega();
	config_spi_atmega();
	configuracao_do_nrf24L01_TX();
	I2C_Init();											/* Initialize I2C */
	MPU6050_Init();
	
	
	
	while (1)
	{	
		Read_RawValue();

		Xa = Acc_x/16384.0;								/* Divide raw value by sensitivity scale factor to get real values */
		Ya = Acc_y/16384.0;
		Za = Acc_z/16384.0;
		
		Xg = Gyro_x/16.4;
		Yg = Gyro_y/16.4;
		Zg = Gyro_z/16.4;

		t = (Temperature/340.00)+36.53;					/* Convert temperature in �/c using formula */
		
		char ax_text[31];
		char ay_text[31];
		char az_text[31];
		char t_text[31];
		char gx_text[31];
		char gy_text[31];
		char gz_text[31];

		dtostrf(Xa, 10, 10, ax_text);
		dtostrf(Ya, 10, 10, ay_text);
		dtostrf(Za, 10, 10, az_text);
		dtostrf(t, 10, 10, t_text);
		dtostrf(Xg, 10, 10, gx_text);
		dtostrf(Yg, 10, 10, gy_text);
		dtostrf(Zg, 10, 10, gz_text);
		
		ax_text[30]=1;
		ay_text[30]=2;
		az_text[30]=3;
		t_text[30]=4;
		gx_text[30]=5;
		gy_text[30]=6;
		gz_text[30]=7;
		
		_delay_us(100); //dar tempo para a leitura
		enviar_sample(ax_text);
		_delay_us(100);
		enviar_sample(ay_text);
		_delay_us(100);
		enviar_sample(az_text);
		_delay_us(100);
		enviar_sample(t_text);
		_delay_us(100);
		enviar_sample(gx_text);
		_delay_us(100);
		enviar_sample(gy_text);
		_delay_us(100);
		enviar_sample(gz_text);
				
	}
}


