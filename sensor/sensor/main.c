/**
 * @file main.c
 * @author Mateusz Januszewski
 * @brief Low-power temperature node using ATtiny series, MCP9808 and NRF24L01.
 * @date 2025-07-18
 */

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <avr/cpufunc.h>
#include <stdbool.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

//byte mask pin
#define Led PIN0_bm
#define CSN PIN4_bm
#define MOSI PIN1_bm
#define MISO PIN2_bm
#define SCK PIN3_bm
#define CE PIN5_bm

//Addresses
#define MCP9808_ADDR 0x18 //0b0011000

//NRF24 registers
#define CONFIG 0x00
#define EN_AA 0x01
#define EN_RXADDR 0x02
#define SETUP_AW 0x03
#define SETUP_RETR 0x04
#define RF_CH 0x05
#define RF_SETUP 0x06
#define STATUS 0x07
#define OBSERVE_TX 0x08
#define RPD 0x09
#define RX_ADDR_P0 0x0A
#define RX_ADDR_P1 0x0B
#define RX_ADDR_P2 0x0C
#define RX_ADDR_P3 0x0D
#define RX_ADDR_P4 0x0E
#define RX_ADDR_P5 0x0F
#define TX_ADDR 0x10
#define RX_PW_P0 0x11
#define RX_PW_P1 0x12
#define RX_PW_P2 0x13
#define RX_PW_P3 0x14
#define RX_PW_P4 0x15
#define RX_PW_P5 0x16
#define FIFO_STATUS 0x17
#define DYNPD 0x1C
#define FEATURE 0x1D

//NRF24 commands
#define R_REGISTER 0x00
#define W_REGISTER 0x20
#define MASK_REGISTER 0x1F
#define R_RX_PAYLOAD 0x61
#define W_TX_PAYLOAD 0xA0
#define FLUSH_TX 0xE1
#define FLUSH_RX 0xE2
#define REUSE_TX_PL 0xE3
#define R_RX_PL_WID 0x60
#define W_ACK_PAYLOAD 0xA8
#define W_TX_PAYLOAD_NOACK 0xB0
#define NOP 0xFF





//Function TWI -->
void TwiInit(void);
int TwiWrite(uint8_t Data);//write the data into the registry twi.mdata
int TwiStart(uint8_t Addr, uint8_t Read);//if 1 read
void TwiStop(void);// bus release
int TwiRead(bool Ack);// Read byte from bus, send ACK if ack=true
void TwiRepeatedStart(void);// Issue repeated start condition
//<--

int16_t Mcp9808ReadTemp(void);// the function responsible for reading the temperature from the MCP9808

//Function SPI -->
void SpiInit(void);
uint8_t SpiData(uint8_t Data);

//<--

//Function NRF24l01 -->
void NrfWriteReg(unsigned char Reg, unsigned char val);
unsigned char NrfReadReg(unsigned char Reg);
void Nrf24ConfigTX(unsigned char address[]);
void NrfUpTX(void);
void NrfSend(unsigned char Payload[],int Length);
int NrfAck(void);
void NrfResetStatus(void);
//<--

void RtcPitInit(void);


uint8_t Address0 [5] ={0xE7,0xE7,0xE7,0xE7,0xE7}; 

volatile uint8_t WakeCounter = 0;

int main(void)
{	
	ccp_write_io((void*)&CLKCTRL.MCLKCTRLB, 0x1 );// Prescaler /3 enabled: 10 MHz / 3 = 3.33 MHz
	//Function for setting pins -->
	for (uint8_t i = 0; i < 8; i++) {
		(&PORTA.PIN0CTRL)[i] = PORT_PULLUPEN_bm; 
	}
	for (uint8_t i = 0; i < 8; i++) {
		(&PORTB.PIN0CTRL)[i] = PORT_PULLUPEN_bm;
	}
	for (uint8_t i = 0; i < 8; i++) {
		(&PORTC.PIN0CTRL)[i] = PORT_PULLUPEN_bm;
	}
	//<--
	
	RtcPitInit();
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sei();
	//Settings input/output
	PORTC.DIR |= Led;
	PORTA.DIR |= CSN; // SPI Chip Select pin
	PORTA.DIR |= MOSI;
	PORTA.DIR &=~MISO;
	PORTA.DIR |= SCK;
	PORTA.DIR |=CE;
	//pin control

	//initializations
	TwiInit();
	SpiInit();
	Nrf24ConfigTX(Address0);
	
	float Temp;
	
    while(1)
    {
			sleep_mode();
			if(WakeCounter>=10) // Wake every ~10s (PIT period 1s x 10)
			{
				WakeCounter=0;
				
				Temp =(float)Mcp9808ReadTemp();
				Temp = Temp/16.0f; // MCP9808 resolution is 0.0625°C per LSB (1/16)
				uint8_t RadioBuffer[5]; //Length: 5 bytes
				float *pTemp = &Temp;

				// Copy 4 bytes of a float to a buffer
				RadioBuffer[0] = ((uint8_t*)pTemp)[0];
				RadioBuffer[1] = ((uint8_t*)pTemp)[1];
				RadioBuffer[2] = ((uint8_t*)pTemp)[2];
				RadioBuffer[3] = ((uint8_t*)pTemp)[3];
				RadioBuffer[4] = 0x00; // Fifth byte is free (e.g. packet counter)

				NrfSend(RadioBuffer, 5);
				NrfResetStatus();
				_delay_ms(10);
			}
    }

}

void TwiInit()
{
	uint8_t BAUD= ((F_CPU / (2 * 100000)) - 5);//Set the communication frequency to 100 kHz
	TWI0.MBAUD= BAUD;
	TWI0.CTRLA = TWI_SDAHOLD_300NS_gc;
	TWI0.MCTRLA |= TWI_ENABLE_bm;
	TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;
}
int TwiStart(uint8_t Addr, uint8_t Read)
{
	if (Read)
	{
		TWI0.MADDR = ( Addr <<1) | 1;//Read
	}
	else
	{
		TWI0.MADDR=(Addr<<1);//Write
	}
	uint16_t TimeOut = 2000;
	while(!(TWI0.MSTATUS&TWI_WIF_bm)&&--TimeOut)
	{
		_delay_us(1);
	}
	if (TimeOut==0)
	{
		TwiStop();
		return 0;
	}
	if(TWI0.MSTATUS&TWI_RXACK_bm)
	{
		TwiStop();
		return 0;//Failed reception (transmission interrupted)
	}
	if (TWI0.MSTATUS&TWI_ARBLOST_bm)
	{
		TwiStop();
		return 0; //wait until the bus is idle 
	}
	if (TWI0.MSTATUS&TWI_CLKHOLD_bm)
	{
		return 1;//Successful data reception (data transmission) 
	}
	while(1)
	{
		PORTC.OUT |=Led;
		_delay_ms(500);
		PORTC.OUT&=~Led;
	}
		return 0;//Something went wrong 
		
}
int TwiWrite(uint8_t Data)
{
	uint16_t TimeOut = 2000;
	TWI0.MDATA=Data;
	while(!(TWI0.MSTATUS&TWI_WIF_bm)&&--TimeOut);
	
	if (TimeOut==0)
	{
		TwiStop();
		return 0; // Timeout
	}
	if(TWI0.MSTATUS&TWI_RXACK_bm)
	{
		TwiStop();
		return 0; // Slave not respond (NACK)
	}
	return 1;// Success – data accepted
}
void TwiStop(void)
{
	TWI0.MCTRLB = TWI_MCMD_STOP_gc;
}
void TwiRepeatedStart(void)
{
	TWI0.MCTRLB = TWI_MCMD_REPSTART_gc;
}

int TwiRead(bool Ack)
{
	uint8_t Data;
	uint16_t TimeOut=2000;
	while (!(TWI0.MSTATUS & TWI_RIF_bm) && --TimeOut){
		_delay_us(1);
	}
	if(TimeOut==0)
	{
		return 0;
	}
		Data =TWI0.MDATA;
		// Send an ACK or NACK and prepare for the next byte or STOP
		if(Ack) //ack
		{
			TWI0.MCTRLB =TWI_MCMD_RECVTRANS_gc;
		}else //nack
		{
			TWI0.MCTRLB = TWI_ACKACT_NACK_gc |TWI_MCMD_STOP_gc;
		}
		
	return Data;
}
int16_t Mcp9808ReadTemp(void)
{
	uint8_t UpperByte;
	uint8_t LowerByte;
	int16_t Temperature;
	TwiStart(MCP9808_ADDR,0);
	TwiWrite(0x05);	// MCP9808 register pointer: Ta (Ambient Temperature Register)
	TwiRepeatedStart();
	TWI0.MADDR = (MCP9808_ADDR << 1) | 1; // Send address with Read bit
	uint16_t TimeOut = 2000;
	while (!((TWI0.MSTATUS & TWI_RIF_bm) || (TWI0.MSTATUS & TWI_WIF_bm))&& --TimeOut)
	{
		_delay_us(1);
	}
	if (TimeOut==0)
	{
		TwiStop();
		return 0;
	}
	if (TWI0.MSTATUS & (TWI_ARBLOST_bm|TWI_BUSERR_bm|TWI_RXACK_bm))
	{
		TwiStop();
		return 0;
	}
	UpperByte = (uint8_t)TwiRead(1);
	LowerByte = (uint8_t)TwiRead(0);
	Temperature = ((UpperByte & 0x1F) << 8) | LowerByte; // Mask flag bits [15:13], keep 13-bit temp data
	if (Temperature & 0x1000) { 

		Temperature -= 0x2000; // Two's complement sign extension: 2^13 = 8192
		
	}
	return Temperature;
}
void SpiInit()
{
	PORTC.DIR |= Led;
	PORTA.DIR |= CSN;// SPI Chip Select pin
	PORTA.DIR |= MOSI;
	PORTA.DIR &=~MISO;
	PORTA.DIR |= SCK;
	SPI0.CTRLA |= SPI_MASTER_bm | SPI_CLK2X_bm | SPI_ENABLE_bm;//Configure the Attiny as the master, set the clock to 5 MHz, and start SPI operation
	SPI0.CTRLB = 0;

	
}
uint8_t SpiData(uint8_t Data)
{
	SPI0.DATA=Data;
	while(!(SPI0.INTFLAGS & SPI_IF_bm));
	return SPI0.DATA;
}
void NrfWriteReg(unsigned char Reg, unsigned char Val)
{
	PORTA.OUT &=~CE;
	PORTA.OUT &=~CSN;
	SpiData(W_REGISTER|(MASK_REGISTER & Reg));
	SpiData(Val);
	PORTA.OUT |=CSN;
}
unsigned char NrfReadReg(unsigned char Reg)
{
	uint8_t value;
	PORTA.OUT &=~CSN;
    SpiData(R_REGISTER|(MASK_REGISTER & Reg));
	value = SpiData(0x00);
	PORTA.OUT |=CSN;
	return value;
}
void Nrf24ConfigTX(unsigned char address[])
{
	_delay_ms(120);
	PORTA.OUT &=~CE;
	NrfWriteReg(CONFIG,0x0E);		// PWR_UP=1, PRIM_RX=0 (TX mode), CRC enabled (2 bytes)
	_delay_ms(2);
	NrfWriteReg(EN_AA,0x00);		// Auto-Acknowledgment disabled on all pipes
	NrfWriteReg(SETUP_AW,0x03);		// Address width: 5 bytes
	NrfWriteReg(SETUP_RETR, 0x3F);	// Retransmit: delay=1000us (0x3<<4), count=15 (0xF)
	NrfWriteReg(RF_CH,0x02);		// RF channel 2 (2.402 GHz)
	NrfWriteReg(RF_SETUP,0x26);		// 250 kbps data rate, 0 dBm TX power
	NrfWriteReg(STATUS,0x70);		// Clear RX_DR, TX_DS, MAX_RT interrupt flags
	
	PORTA.OUT &=~CSN;
	SpiData((W_REGISTER|(MASK_REGISTER & RX_ADDR_P0)));
	for (int i=4; i>=0; i--){
		SpiData(address[i]);
	}
	PORTA.OUT |=CSN;
	
	PORTA.OUT &=~CSN;
	SpiData((W_REGISTER|(MASK_REGISTER & TX_ADDR)));
	for (int i=4; i>=0; i--){
		SpiData(address[i]);
	}
	PORTA.OUT |=CSN;
	
	NrfWriteReg(RX_PW_P0, 5);      // RX payload width on pipe 0: 5 bytes (matches TX payload)
	NrfWriteReg(DYNPD, 0x00);      // Disabled Dynamic Payload
	NrfWriteReg(FEATURE, 0x00);    // Disabled special features

}
void NrfUpTX(void)
{
	PORTA.OUT &=~CE;
	PORTA.OUT &=~CSN;
	SpiData((W_REGISTER | (MASK_REGISTER & CONFIG)));	
	SpiData(0x0E); // CONFIG value: PWR_UP=1, TX mode, 2-byte CRC
	PORTA.OUT |=CSN;
}
void NrfSend(unsigned char Payload[],int Length)
{
	PORTA.OUT &=~CE;
	NrfUpTX();
	PORTA.OUT &=~CSN;
	SpiData(FLUSH_TX);
	PORTA.OUT |=CSN;
	PORTA.OUT &=~CSN;
	SpiData(W_TX_PAYLOAD);
	for (int i =0;i<Length;i++)
	{
		SpiData(Payload[i]);
	}
	PORTA.OUT |=CSN;
	PORTA.OUT |=CE;
	_delay_us(20);
	PORTA.OUT &=~CE;
}
int NrfAck(void)
{	
	uint8_t status;
	status=NrfReadReg(STATUS);
	if ((status&0x20)==0x20)
	{
		return 1;
	}else {
		return 0;
	}
}
void NrfResetStatus(void)
{
	NrfWriteReg(STATUS,0x70);
}

void RtcPitInit(void)
{
	
	RTC.CLKSEL=RTC_CLKSEL_INT32K_gc; // Internal 32.768 kHz oscillator as RTC clock source
	
	RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc | RTC_PITEN_bm;
	RTC.PITINTCTRL = RTC_PI_bm;
}




//interrupts
ISR(RTC_PIT_vect) {
	// Clear the interrupt flag (as described in the RTC.PITINTFLAGS section)
	RTC.PITINTFLAGS = RTC_PI_bm;
	WakeCounter++;
}