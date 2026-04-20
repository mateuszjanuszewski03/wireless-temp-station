/**
 * @file main.c
 * @author Mateusz Januszewski
 * @date 11/18/2025
 * * @brief Wireless temperature receiver using nRF24L01+ (SPI) and SH1106 OLED (I2C).
 *
 * This program configures an ATmega328P to act as a receiver node. It listens 
 * for a 4-byte float payload via the nRF24L01+ radio module and displays the 
 * formatted temperature on an SH1106 OLED display.
 * * Target MCU: ATmega328P @ 8 MHz (internal oscillator, prescaler disabled)
 */

#include <xc.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/twi.h>
#include <stdbool.h>
#include <avr/pgmspace.h>
#include <float.h>
#include <string.h>

/* ??? nRF24L01+ GPIO pin mapping ??????????????????????????????????????????? */
#define CE_PIN   PC0   /**< Chip Enable (RX/TX mode control) */
#define CSN_PIN  PB2   /**< SPI Chip Select (active low) */
#define MOSI_PIN PB3   /**< SPI Master Out Slave In */
#define MISO_PIN PB4   /**< SPI Master In Slave Out */
#define SCK_PIN  PB5   /**< SPI Clock */

#define CE_LOW    PORTC &= ~(1<<CE_PIN)
#define CE_HIGH   PORTC |= (1<<CE_PIN)
#define CSN_LOW   PORTB &= ~(1<<CSN_PIN)
#define CSN_HIGH  PORTB |= (1<<CSN_PIN)

/* ??? nRF24L01+ register addresses ????????????????????????????????????????? */
#define CONFIG      0x00   // Configuration register
#define EN_AA       0x01   // Enable Auto Acknowledgement
#define EN_RXADDR   0x02   // Enable RX addresses
#define SETUP_AW    0x03   // Address width setup
#define SETUP_RETR  0x04   // Retransmit setup
#define RF_CH       0x05   // RF channel
#define RF_SETUP    0x06   // RF setup (data rate, power)
#define STATUS      0x07   // Status register
#define RX_ADDR_P0  0x0A   // RX address pipe 0
#define TX_ADDR     0x10   // TX address
#define RX_PW_P0    0x11   // Payload width for pipe 0
#define FIFO_STATUS 0x17   // FIFO status register
#define DYNPD       0x1C   // Enable dynamic payload length
#define FEATURE     0x1D   // Feature register

/* ??? nRF24L01+ SPI commands ??????????????????????????????????????????????? */
#define R_REGISTER    0x00   // Read register
#define W_REGISTER    0x20   // Write register
#define R_RX_PAYLOAD  0x61   // Read RX payload
#define W_TX_PAYLOAD  0xA0   // Write TX payload
#define FLUSH_TX      0xE1   // Flush TX FIFO
#define FLUSH_RX      0xE2   // Flush RX FIFO
#define NOP           0xFF   // No operation (used to read STATUS)
#define R_RX_PL_WID   0x60   // Read RX payload width

/* ??? SH1106 OLED (I2C) ????????????????????????????????????????????????????? */
#define AdressOled 0b0111100          /**< I2C address of the SH1106 controller */
#define ACK   (TWCR = (1<<TWEN)|(1<<TWEA)|(1<<TWINT))   /**< Send ACK after byte */
#define NoACK (TWCR = (1<<TWEN)|(1<<TWINT))             /**< Send NACK after byte */

/* ??? FontTable mapping ????????????????????????????????????????????????????? */
#define CHAR_SPACE   10
#define CHAR_CELSIUS 11
#define CHAR_MINUS   12
#define CHAR_COMMA   13

#define F_CPU 8000000UL   /**< CPU frequency: 8 MHz */

/**
 * @brief Font table stored in program memory (Flash).
 * * Each entry is a 5-byte column bitmap for a 5x8 pixel character.
 * Indices: 0-9 = digits, 10 = space, 11 = '°C', 12 = '-', 13 = ','
 */
const uint8_t FontTable[][5] PROGMEM={
	{0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
	{0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
	{0x42, 0x61, 0x51, 0x49, 0x46}, // 2
	{0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
	{0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
	{0x27, 0x45, 0x45, 0x45, 0x39}, // 5
	{0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
	{0x01, 0x71, 0x09, 0x05, 0x03}, // 7
	{0x36, 0x49, 0x49, 0x49, 0x36}, // 8
	{0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
	{0x00, 0x00, 0x00, 0x00, 0x00}, // Space (index 10)
	{0x00, 0x63, 0x14, 0x08, 0x00}, // 'C' (st. Celsius) (index 11)
	{0x08, 0x08, 0x08, 0x08, 0x08}, // '-' (minus) (index 12)
	{0x00, 0x50, 0x30, 0x00, 0x00}, // ',' (comma) (index 13)
};

/* ??? Function prototypes ??????????????????????????????????????????????????? */

// System
void ClockChange(void);

// I2C (TWI)
void TwiInit(void);
void TwiStart(void);
void TwiAddress(uint8_t ADR, uint8_t RW);
void TwiWrite(uint8_t Data);
void TwiStop(void);
uint8_t TwiRead(uint8_t *Data,bool Ack);//If Ack = 1, then ACK; if Ack = 0, then NoACK

// SH1106 OLED display
void ControlByte(bool DC,bool C0);// Check byte: are we writing to RAM or performing a command operation?
void SH1106Init(void);
void SH1106ColumnSet(uint8_t Num);
void SH1106DisplayNumber(uint8_t Number);
void SH1106PrintTemp(float Temp);
void SH106ClearPage(uint8_t Page);

// SPI
void SpiInit(void);
uint8_t SpiData(uint8_t Data);

// nRF24L01+
void NrfWriteReg(uint8_t Reg, uint8_t Value);
uint8_t NrfReadReg(uint8_t Reg);
void NrfWriteBuf(uint8_t Reg, uint8_t *BufferValues, uint8_t Byte);
void NrfReadBuf(uint8_t Reg, uint8_t *BufferValues, uint8_t Byte);
void Nrf24ConfigRX(unsigned char address[]);
void NrfStartListening(void);
void NrfStopListening(void);
int NrfDataReady(void);
int NrfPayloadLength(void);
void NrfRead(unsigned char payload[],int length);



/* ??????????????????????????????????????????????????????????????????????????? */

/**
 * @brief Main application entry point.
 * @return int 
 */
int main(void)
{
	uint8_t Address0 [5] ={0xE7,0xE7,0xE7,0xE7,0xE7}; // Must match the TX address
	uint8_t Length;
	uint8_t Received_data[32]="";
	float Temperature = 0;
	
	ClockChange();		// Switch CPU to 8 MHz (disable clock prescaler)
	SH1106Init();		// Initialize the OLED display via I2C
	SpiInit();			// Initialize SPI peripheral
	Nrf24ConfigRX(Address0);	// Configure nRF24L01+ in RX mode
	
	// Clear all 8 display pages before showing anything
	for (int i =0;i<8;i++)
	{
		SH106ClearPage(i);
	}
	
	SH1106PrintTemp(Temperature);	// Show initial temperature (0.00 °C)
	NrfStartListening();			// Put nRF24L01+ into active RX mode
    while(1)
    {
			if (NrfDataReady())	// Check if a new packet has been received
			{
				Length=NrfPayloadLength();	// Expected Payload size: 4 bytes float + 1 byte extra
				
				if (Length > 0 && Length <= 32) 
				{
					NrfRead(Received_data, Length);	// Read Payload from RX FIFO
					
					// Manually flush RX FIFO to ensure no leftover data
					CSN_LOW;
					SpiData(FLUSH_RX);
					CSN_HIGH;
					
					// Reinterpret the first 4 bytes of the Payload as a float
					memcpy(&Temperature, Received_data, sizeof(float));
					
					
					SH1106PrintTemp(Temperature);	// Update temperature on the display
					}
				}
				_delay_ms(1);
			}	
}

/* ??? System ???????????????????????????????????????????????????????????????? */
 
/**
 * @brief Disables the clock prescaler to run the MCU at the full 8 MHz.
 * * @note The two-write sequence to CLKPR is required by the AVR hardware 
 * to prevent accidental clock changes.
 */
void ClockChange(void)
{
	cli();
	CLKPR = 0x80;
	CLKPR = 0x00;
	sei();
}

/* ??? I2C (TWI) ????????????????????????????????????????????????????????????? */

/**
 * @brief Initializes the TWI (I2C) peripheral for 100 kHz bus speed.
 */
void TwiInit(void)
{
	TWBR=((F_CPU/100000)-16)/2;
	TWCR |=(1<<TWEN);
	TWSR &=~((1<<TWPS1)|(1<<TWPS0));
}

/**
 * @brief Generates an I2C START condition and waits for it to complete.
 * * @note Triggers a watchdog reset if the bus does not respond within the timeout.
 */
void TwiStart(void)
{
	TWCR=(1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	uint16_t TimeOut=2000;
	while(!(TWCR&(1<<TWINT)) && TimeOut--)
	;
	if (TimeOut==0)
	{
		wdt_enable(WDTO_15MS);
		while(1);
	}
}

/**
 * @brief Sends the 7-bit slave address + R/W bit onto the I2C bus.
 * * @param ADR 7-bit I2C address of the device.
 * @param RW Read/Write bit (0 for Write, 1 for Read).
 */
void TwiAddress(uint8_t ADR, uint8_t RW)
{
	TWDR = (ADR<<1)|RW;
	TWCR=(1<<TWINT)|(1<<TWEN);
	uint16_t TimeOut=2000;
	while((!(TWCR&(1<<TWINT)) && TimeOut--))
	;
	if (TimeOut==0||((TWSR&0xF8)!= TW_MT_SLA_ACK&&(TWSR&0xF8)!= TW_MR_SLA_ACK)) //0x18 ||0x40
	{
		wdt_enable(WDTO_15MS);
		while(1);
	}
}

/**
 * @brief Transmits one byte over I2C and waits for completion.
 * * @param Data The byte to be transmitted.
 */
void TwiWrite(uint8_t Data)
{
	TWDR=Data;
	TWCR=(1<<TWINT)|(1<<TWEN);
	uint16_t TimeOut=2000;
	while((!(TWCR&(1<<TWINT)) && TimeOut--))
	;
	if (TimeOut==0)
	{
		wdt_enable(WDTO_15MS);
		while(1);
	}
	if ((TWSR&0xF8)!= TW_MT_DATA_ACK) //0x28
	{
		TwiStop();
	}
}

/**
 * @brief Generates an I2C STOP condition to release the bus.
 */
void TwiStop(void)
{
	TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
}

/**
 * @brief Reads one byte from the I2C bus.
 * * @param Data Pointer to store the received byte.
 * @param Ack If true, sends ACK. If false, sends NACK + STOP.
 * @return uint8_t The received byte.
 */
uint8_t TwiRead(uint8_t *Data,bool Ack)
{
		if (Ack)
	{
		ACK;
	}else
	{
		NoACK;
	}
	uint16_t TimeOut=2000;
	while(!(TWCR&(1<<TWINT)) && TimeOut--)
	;
	if (TimeOut==0)
	{
		wdt_enable(WDTO_15MS);
		while(1);
	}
	*Data=TWDR;
	if(!Ack)
	{
		TwiStop();
	}
	return *Data;		
}

/* ??? SH1106 OLED display ??????????????????????????????????????????????????? */

/**
 * @brief Sends the SH1106 I2C control byte that precedes command or data streams.
 * * @param DC true = display RAM data, false = commands.
 * @param C0 true = only one byte follows, false = a stream of bytes follows.
 */
void ControlByte(bool DC, bool C0)
{
	if (DC)
	{
		if (C0)
		{
			TwiWrite(0b11000000);
		}else{
			TwiWrite(0b01000000);
		}
	}else{
		if (C0)
		{
			TwiWrite(0b10000000);
		}else{
			TwiWrite(0b00000000);
		}
	}
	
}

/**
 * @brief Initializes the SH1106 OLED controller via I2C.
 * * Sets display geometry, segment remapping, contrast, charge pump, 
 * and turns the display on.
 */
void SH1106Init(void)
{
	TwiInit();
	TwiStart();
	TwiAddress(AdressOled,0);
	ControlByte(false,false);
	TwiWrite(0xAE); // Display OFF
	TwiWrite(0x02); // Set Column Address Low
	TwiWrite(0x10); // Set Column Address High 
	TwiWrite(0x40); // Set Display Start Line: 0 
	TwiWrite(0xB0); // Set Page Address: 0 

	TwiWrite(0xA1); // Segment Re-map: (flip left/right) 
	TwiWrite(0xC8); // Common Output Scan Direction: (flip up/down)
	
	TwiWrite(0x81); // Set Contrast Control 
	TwiWrite(0x80); // Contrast value (midpoint)
	
	TwiWrite(0xAD); // DC-DC Control Mode Set 
	TwiWrite(0x8B); // DC-DC ON 
	
	TwiWrite(0xA4); // Set Entire Display ON/OFF (resume from RAM) 
	TwiWrite(0xA6); // Set Normal/Reverse Display (Normal) 
	TwiWrite(0xAF); // Display ON 

	TwiStop();
}

/**
 * @brief Sets the column start address for the next write operation.
 * * @param Num Column number (0-127).
 * @note A hardware offset of +2 is applied internally for 1.3" SH1106 panels.
 */
void SH1106ColumnSet(uint8_t Num)
{
	Num+=2;
	uint8_t LowBit = Num & 0x0F;
	uint8_t UperBit = (Num>>4) & 0x0F;
	
	TwiStart();
	TwiAddress(AdressOled,0);
	ControlByte(false,false);
	
	TwiWrite(0x00 | LowBit);
	TwiWrite(0x10 | UperBit);
	
}

/**
 * @brief Draws a single 5x8 character from the font table.
 * * @param Number Index of the character in the PROGMEM array.
 */
void SH1106DisplayNumber(uint8_t Number)
{
	TwiStart();
	TwiAddress(AdressOled,0);
	ControlByte(true,false);
	for (int i = 0; i <5;i++)
	{
		TwiWrite(pgm_read_byte(&FontTable[Number][i]));
	}
	TwiWrite(0x00);
	TwiStop();
}

/**
 * @brief Formats a float temperature value and renders it on page 3.
 * * Format: [-]DD,CC °C (e.g., -12,57 °C or 25,03 °C)
 * * @param Temp Temperature value to display.
 */
void SH1106PrintTemp(float Temp)
{	
	SH106ClearPage(3);
	TwiStart();
	TwiAddress(AdressOled,0);
	ControlByte(false,false);
	TwiWrite(0xB3);
	SH1106ColumnSet(45);
	TwiStop();
	
	if (Temp < 0)
	{
		SH1106DisplayNumber(CHAR_MINUS);
		Temp = -Temp;
	}

	// Breaking down into whole numbers and decimal parts
	int IntegerPart = (int)Temp;
	
	// We extract the hundredths (multiply by 100)
	// Example: 25.57 -> 0.57 * 100 = 57.0 -> +0.5 for rounding
	int Hundredths = (int)((Temp - (float)IntegerPart) * 100.0f + 0.5f);
	
	// Handle overflow (e.g., if the result is 100, increment the total by 1)
	if (Hundredths >= 100) {
		IntegerPart++;
		Hundredths = 0;
	}

	// 1. Display the entire content
	if (IntegerPart / 10 > 0) {
		SH1106DisplayNumber(IntegerPart / 10);
	}
	SH1106DisplayNumber(IntegerPart % 10);

	// 2. Display the COMMA (index 13)
	SH1106DisplayNumber(CHAR_COMMA);

	// 3. Displaying tenths and hundredths
	// Important: We must include a leading zero; for example, 0.05 should be displayed as "05", not "5"
	SH1106DisplayNumber(Hundredths / 10); 
	SH1106DisplayNumber(Hundredths % 10); 

	// 4. Unit
	SH1106DisplayNumber(CHAR_SPACE);
	SH1106DisplayNumber(CHAR_CELSIUS);
}

/**
 * @brief Fills an entire 128-pixel display page with zeros.
 * * @param Page Page index to clear (0-7).
 */
void SH106ClearPage(uint8_t Page)
{
	TwiStart();
	TwiAddress(AdressOled,0);
	ControlByte(false,false);
	TwiWrite(0xB0 | Page);
	SH1106ColumnSet(0);
	TwiStop();
	
	TwiStart();
	TwiAddress(AdressOled,0);
	ControlByte(true,false);
	
	for (uint8_t i = 0;i<128;i++)
	{
		TwiWrite(0x00);
	}
	TwiStop();
}

/* ??? SPI ??????????????????????????????????????????????????????????????????? */

/**
 * @brief Initializes the SPI peripheral in Master mode (F_CPU/16).
 */
void SpiInit(void)
{
	DDRC |= (1<<CE_PIN);
	DDRB |= (1<<MOSI_PIN)|(1<<CSN_PIN)|(1<<SCK_PIN);
	DDRB &= ~(1<<MISO_PIN);
	
	CE_LOW;
	CSN_HIGH;
	
	SPCR |= (1<<SPE)|(1<<MSTR)|(1<<SPR0); 

}

/**
 * @brief Transmits one byte over SPI and returns the received byte.
 * * @param Data Byte to transmit.
 * @return uint8_t Byte received from the slave.
 */
uint8_t SpiData(uint8_t Data)
{
	SPDR = Data;
	while (!(SPSR & (1<<SPIF)));
	
	return SPDR;
}

/* ??? nRF24L01+ ????????????????????????????????????????????????????????????? */

/**
 * @brief Writes a single byte to a nRF24L01+ register.
 * * @param Reg Register address (0-31).
 * @param Value Data to write.
 */
void NrfWriteReg(uint8_t Reg, uint8_t Value)
{
	CSN_LOW;
	SpiData(W_REGISTER | (Reg & 0x1F));
	SpiData(Value);
	CSN_HIGH;
	_delay_us(1);
}

/**
 * @brief Reads a single byte from a nRF24L01+ register.
 * * @param Reg Register address (0-31).
 * @return uint8_t The read value.
 */
uint8_t NrfReadReg(uint8_t Reg)
{
	uint8_t Value;
	CSN_LOW;
	SpiData(R_REGISTER | (Reg & 0b00011111));
	Value = SpiData(NOP);
	CSN_HIGH;
	return Value;
}

/**
 * @brief Writes multiple bytes to a nRF24L01+ register.
 * * @param Reg Register address.
 * @param BufferValues Pointer to the data array to write.
 * @param Byte Number of bytes to write.
 */
void NrfWriteBuf(uint8_t Reg, uint8_t *BufferValues, uint8_t Byte)
{
	CSN_LOW;
	SpiData(W_REGISTER | (Reg & 0b00111111));
	for (int i =0; i<Byte;i++)
	{
		SpiData(*BufferValues++);
	}
	CSN_HIGH;
}

/**
 * @brief Reads multiple bytes from a nRF24L01+ register.
 * * @param Reg Register address.
 * @param BufferValues Pointer to the buffer where data will be stored.
 * @param Byte Number of bytes to read.
 */
void NrfReadBuf(uint8_t Reg, uint8_t *BufferValues, uint8_t Byte)
{
	CSN_LOW;
	SpiData(R_REGISTER | (Reg & 0b00011111));
	for (int i = 0;i<Byte;i++)
	{
		*BufferValues++=SpiData(NOP);
	}
	CSN_HIGH;
}

/**
 * @brief Configures the nRF24L01+ module as a receiver (RX mode).
 * * @param address A 5-byte array containing the RX pipe 0 address.
 */
void Nrf24ConfigRX(unsigned char address[])
{
	_delay_ms(120);
	CE_LOW;
	NrfWriteReg(CONFIG,0x0F); // PWR_UP=1, PRIM_RX=1 (RX mode), CRC enabled (2 bytes)

	_delay_ms(2);
	NrfWriteReg(EN_AA,0x00);		// Auto-Acknowledgment disabled on all pipes
	NrfWriteReg(EN_RXADDR , 0x01); 
	NrfWriteReg(SETUP_AW, 0x03);	// Address width: 5 bytes
	NrfWriteReg(SETUP_RETR, 0x3F);	// Retransmit: delay=1000us (0x3<<4), count=15 (0xF)
	NrfWriteReg(RF_CH, 0x02);		// RF channel 2 (2.402 GHz)
	NrfWriteReg(RF_SETUP, 0x26);	// 250 kbps data rate, 0 dBm TX power
	NrfWriteReg(STATUS, 0x70);		// Clear RX_DR, TX_DS, MAX_RT interrupt flags
	
	NrfWriteReg(RX_PW_P0, 5);		// RX Payload width on pipe 0: 5 bytes (matches TX Payload)
	
	NrfWriteBuf(RX_ADDR_P0, address,5); // RX address pipe 0
	

	NrfWriteReg(DYNPD, 0x00);		// Enable dynamic Payload Length
	NrfWriteReg(FEATURE, 0x00);		// Feature register   
}

/**
 * @brief Activates RX mode by asserting CE high.
 */
void NrfStartListening(void)
{
	CE_HIGH;
	_delay_ms(10);
}

/**
 * @brief Deactivates RX mode by pulling CE low (returns to Standby-I).
 */
void NrfStopListening(void)
{
	CE_LOW;
	_delay_ms(10);
}

/**
 * @brief Checks if the RX FIFO contains a new packet.
 * * @return int 1 if data is available, 0 otherwise.
 */
int NrfDataReady(void)
{
	uint8_t status;
	status=NrfReadReg(STATUS);

	if ((status & 0x40) == 0x40){
		return 1;
	}
	else {
		return 0;
	}
}

/**
 * @brief Reads the width of the top Payload in the RX FIFO.
 * * @return int The Payload Length in bytes (0–32).
 */
int NrfPayloadLength(void)
{
	int Length;
	CSN_LOW;
	SpiData(R_RX_PL_WID);
	Length=SpiData(0x00);
	CSN_HIGH;
	return Length;
}

/**
 * @brief Reads bytes from the RX FIFO into a buffer and clears interrupt flags.
 * * @param Payload Pointer to the buffer array.
 * @param Length Number of bytes to read.
 */
void NrfRead(unsigned char Payload[],int Length)
{
	CSN_LOW;
	SpiData(R_RX_PAYLOAD);
	for (int i=0;i<Length;i++)
	{
		Payload[i]=SpiData(NOP);
	}
	NrfWriteReg(STATUS,0x70);
	NrfWriteReg(STATUS,0x70);
}