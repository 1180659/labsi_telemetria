#define F_CPU  10000000UL

#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h> 
#include <avr/interrupt.h>
#include <util/delay.h>



#define SCK    5
#define MISO   4
#define MOSI   3
#define SS_NRF 2
#define CE     1

#define LEDstatus 5
#define LEDerror 4
#define LEDblink 0

#define IRQ 3


#define CONFIGnrf   0x00 +0x20 //(soma-se 0x20 é o comando de escrita W_REGISTER 
#define EN_AA       0x01 +0x20 
#define EN_RXADDR   0x02 +0x20
#define SETUP_AW    0x03 +0x20
#define SETUP_RETR  0x04 +0x20
#define RF_CH       0x05 +0x20
#define RF_SETUP    0x06 +0x20
#define STATUS      0x07 +0x20
#define RX_ADDR_P0  0x0A +0x20
#define TX_ADDR     0x10 +0x20 //
#define RX_PW_P0    0x11 +0x20 //
#define FIFO_STATUS 0x17 +0x20

#define R_RX_PAYLOAD 0b01100001
#define W_TX_PAYLOAD 0b10100000
#define FLUSH_TX     0b11100001
#define FLUSH_RX     0b11100010


#define END_RX_Byte2  0x69
#define END_RX_Byte1  0xED
#define END_RX_Byte0  0x75

char buffer[31];
int count;


void config_portas_atmega()
{
	DDRB = 0;
	DDRB |= (1<<SCK) | (1<<MOSI) | (1<<SS_NRF) | (1<<CE) | (1<<LEDblink); //outputs
	DDRB &= ~(1<<MISO); //input 
	
	PORTB = 0;
	PORTB |= (1<<SS_NRF); //começa inativo o SS
	PORTB &= ~(1<<CE); //começa a zero o Chip enable
	
	
	DDRC |= (1<<LEDerror) | (1<<LEDstatus); //LED output para status da comunicação
	DDRD &= ~(1<<IRQ); //ler interrupt IRQ do RX_DS
	PORTC &= ~(1<<LEDerror) | (1<<LEDstatus);
	

	TCCR0A =0;
	TCCR0A |= (1 << WGM01); //CTC MODE
	TIMSK0 |= (1 << OCIE0A); //ISR COMP VECT
	TCCR0B |= (1 << CS01) | (1 << CS00); //1024 PRESCALER
	OCR0A=200;

	
}


/**************UART FUNCTIONS****************/
void UART_init(unsigned int ubrr)
{

	// set baudrate in UBRR
	UBRR0L = (uint8_t)ubrr;
	UBRR0H = (uint8_t)(ubrr >> 8);

	// enable the transmitter and receiver
	UCSR0B |= (1 << RXEN0) | (1 << TXEN0);
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}


void UART_writec(uint8_t data)
{
	// wait for transmit buffer to be empty
	while(!(UCSR0A & (1 << UDRE0)));

	// load data into transmit register
	UDR0 = data;

}


void UART_writeU8(uint8_t val)
{
	uint8_t dig1 = '0', dig2 = '0';

	// count value in 100s place
	while(val >= 100)
	{
		val -= 100;
		dig1++;
	}

	// count value in 10s place
	while(val >= 10)
	{
		val -= 10;
		dig2++;
	}

	// print first digit (or ignore leading zeros)
	if(dig1 != '0') UART_writec(dig1);

	// print second digit (or ignore leading zeros)
	if((dig1 != '0') || (dig2 != '0')) UART_writec(dig2);

	// print final digit
	UART_writec(val + '0');
}


void UART_writeU16(uint16_t val)
{
	uint8_t dig1 = '0', dig2 = '0', dig3 = '0', dig4 = '0';

	// count value in 10000s place
	while(val >= 10000)
	{
		val -= 10000;
		dig1++;
	}

	// count value in 1000s place
	while(val >= 1000)
	{
		val -= 1000;
		dig2++;
	}

	// count value in 100s place
	while(val >= 100)
	{
		val -= 100;
		dig3++;
	}

	// count value in 10s place
	while(val >= 10)
	{
		val -= 10;
		dig4++;
	}

	// was previous value printed?
	uint8_t prevPrinted = 0;

	// print first digit (or ignore leading zeros)
	if(dig1 != '0') {UART_writec(dig1); prevPrinted = 1;}

	// print second digit (or ignore leading zeros)
	if(prevPrinted || (dig2 != '0')) {UART_writec(dig2); prevPrinted = 1;}

	// print third digit (or ignore leading zeros)
	if(prevPrinted || (dig3 != '0')) {UART_writec(dig3); prevPrinted = 1;}

	// print third digit (or ignore leading zeros)
	if(prevPrinted || (dig4 != '0')) {UART_writec(dig4); prevPrinted = 1;}

	// print final digit
	UART_writec(val + '0');
}


void UART_writeS(char* s)
{
	// transmit character until NULL is reached
	while(*s > 0) UART_writec(*s++);
}

/********* SPI FUNCTIONS **********/
void config_spi_atmega()
{
//SPI:

	//SPCR |= (1<<SPIE) | (1<<SPE) | (0<<DORD) | (1<<MSTR) | (0<<CPOL) | (0<<CPHA) | (0<<SPR1) | (0<<SPR0);
	SPCR = 0b11010000; //

	//SPSR |= (1<<SPIF) | (0<<WCOL) | (1<<SPI2X);
	SPSR = 0b10000001; //com a flag SPIF para sabermos quando terminou de enviar a data pelo SPDR 
}

uint8_t spi_read(uint8_t junkdata)
{
	SPDR = junkdata; // Write data to SPI data register 
	while(!(SPSR & (1<<SPIF)));		// Wait till transmission complete 
	return(SPDR);
}

void spi_read_X_bytes()  //tempo estimado de envio de 24us
{

	for(int i=0;i<31;i++)
	{
		
		SPDR = 0xAA;
		while(!(SPSR & (1<<SPIF)));		// Wait till transmission complete
		buffer[i] = SPDR;															
	}
	
}

void spi_write(uint8_t data)
{	
	SPDR = data;					/* Write data to SPI data register */ 
	while(!(SPSR & (1<<SPIF)));		/* Wait till transmission complete */	
}


/******* NRF FUNCTIONS *****/
void configuracao_do_nrf24L01_RX()
{
	
	PORTB &= ~(1<<CE);
	
	_delay_ms(12); // tempo de espera após alimentação do modulo
		
	PORTB &= ~(1<<SS_NRF); //SETUP_AW
	spi_write(SETUP_AW);
	spi_write(0b00000001); //3 bytes para RX e TX address (o minimo)
	PORTB |= (1<<SS_NRF);
	
	
	PORTB &= ~(1<<SS_NRF); //RX_ADDR_P0 address de 3 bytes
	spi_write(RX_ADDR_P0);
	spi_write(END_RX_Byte2);
	spi_write(END_RX_Byte1);
	spi_write(END_RX_Byte0);
	PORTB |= (1<<SS_NRF);
	

	
	PORTB &= ~(1<<SS_NRF); //EN_AA nao se usa o Enhanced Shockburst
	spi_write(EN_AA);
	spi_write(0); //nao usa auto acknowledgement 
	PORTB |= (1<<SS_NRF);
	

	PORTB &= ~(1<<SS_NRF); //EN_RXADDR
	spi_write(EN_RXADDR);
	spi_write(0b00000001); //enable data pipe 0
	PORTB |= (1<<SS_NRF);


	PORTB &= ~(1<<SS_NRF); //SETUP_RETR 
	spi_write(SETUP_RETR);
	spi_write(0); //Re-Transmit disabled
	PORTB |= (1<<SS_NRF);
	

	PORTB &= ~(1<<SS_NRF); //RF_CH
	spi_write(RF_CH);
	spi_write(0); //frequencia do canal fica a 2.4GHz
	PORTB |= (1<<SS_NRF);
	

	PORTB &= ~(1<<SS_NRF); //RF_SETUP
	spi_write(RF_SETUP);
	spi_write(0b00000110); //air data rate = 1Mbps, output power in TX mode = 0dBm, setup LNA gain disabled
	PORTB |= (1<<SS_NRF);
	

	PORTB &= ~(1<<SS_NRF); //STATUS dar reset/limpar
	spi_write(STATUS);
	spi_write(0b01000000); //clear bit do RX FIFO
	PORTB |= (1<<SS_NRF);
	

	PORTB &= ~(1<<SS_NRF); //RX_PW_P0
	spi_write(RX_PW_P0);
	spi_write(0b00011111); //31 bytes no RX payload do pipe 0
	PORTB |= (1<<SS_NRF);
	
	
	PORTB &= ~(1<<SS_NRF);
	spi_write(FLUSH_RX); //FLUSH_RX
	PORTB |= (1<<SS_NRF);
	
	
	PORTB &= ~(1<<SS_NRF);
	spi_write(FLUSH_TX); //FLUSH_TX
	PORTB |= (1<<SS_NRF);
	
	
	PORTB &= ~(1<<SS_NRF); //CONFIG
	spi_write(CONFIGnrf);
	spi_write(0b00110011); //CRC (des)ligado, Power UP, IRQ -> RX_DS, PRIM_RX = 1
	PORTB |= (1<<SS_NRF);
	
	_delay_ms(2); //tempo para entrar em standby após power up
}


void recebe_sample()
{
	
		PORTB |= (1<<CE);
					
					while( (PIND & (1<<IRQ)) )
					{
						_delay_us(8);
						PORTC &= ~(1<<LEDstatus);
						PORTC |= (1<<LEDerror);
					}
					
					if ( !(PIND & (1<<IRQ)) )
					{
						
						PORTC |= (1<<LEDstatus); // ligar o LEDteste1
						PORTC &= ~(1<<LEDerror);
								
									
						PORTB &= ~(1<<CE);//desligar modo RX para ler o RX_FIFO	
						
												
						PORTB &= ~(1<<SS_NRF);
						spi_write(R_RX_PAYLOAD); //R_RX_PAYLOAD
						spi_read_X_bytes();
						PORTB |= (1<<SS_NRF);
						
						
						PORTB |= (1<<CE);
						
						
						PORTB &= ~(1<<SS_NRF); //STATUS dar reset/limpar
						spi_write(STATUS);
						spi_write(0b01000000); //clear bit do RX FIFO
						PORTB |= (1<<SS_NRF);
						
						
						PORTB &= ~(1<<SS_NRF);
						spi_write(FLUSH_RX); //FLUSH_RX
						PORTB |= (1<<SS_NRF);
						
						
					}					
					
					else
					{
						
						PORTC &= ~(1<<LEDstatus);
					}
				
}




int main(void)
{
	
	config_portas_atmega();
	UART_init(64);
	config_spi_atmega();
	configuracao_do_nrf24L01_RX();
	
	char ax_text[30];
	char ay_text[30];
	char az_text[30];
	char t_text[30];
	char gx_text[30];
	char gy_text[30];
	char gz_text[30];
	int i,j,id;
	
	while (1)
	{
		
		for (i=0; i<8; i++){
			
			recebe_sample();
			id=buffer[30];
			
			switch (id)
			{
				case 1:
				
				for(j=0; j<30 ;j++){
					
					ax_text[j]=buffer[j];
				}
				break;
				
				case 2:
				
				for(j=0; j<30 ;j++){
					
					ay_text[j]=buffer[j];
				}
				break;
				
				case 3:
				
				for(j=0; j<30 ;j++){
					
					az_text[j]=buffer[j];
				}
					break;
					
				case 4:
				
				for(j=0; j<30 ;j++){
					
					t_text[j]=buffer[j];
				}
					break;
					
				case 5:
				
				for(j=0; j<30 ;j++){
					
					gx_text[j]=buffer[j];
				}
					break;
					
				case 6:
				
				for(j=0; j<30 ;j++){
					
					gy_text[j]=buffer[j];
				}
					break;
					
				case 7:
				
				for(j=0; j<30 ;j++){
					
					gz_text[j]=buffer[j];
				}
					break;
					
					
			}
			
			
			
		
		}
		
		char text[218];
		snprintf(text, 218, "%s,%s,%s,%s,%s,%s,%s", ax_text, ay_text, az_text, t_text, gx_text, gy_text, gz_text);
		UART_writeS(text);
		UART_writeS("\n\r");
			
	}
	
	return 0;
}


ISR(TIMER0_COMPA_vect)
{
	if (count == 25)
	{
		PORTB ^= (1<<LEDblink);
		count=0;
	}
	else
	count++;
	
}