# wireless-temp-station

A wireless temperature monitoring system built from scratch using two AVR microcontroller nodes communicating over a 2.4 GHz RF link. The transmitter node reads ambient temperature from a precision sensor and broadcasts it as a compact binary payload; the receiver node decodes the data and renders it in real time on an OLED display. The design prioritises low power consumption on the sensor side, achieving a sleep-dominant duty cycle of roughly 10 seconds per measurement.
<img width="2048" height="1536" alt="image" src="https://github.com/user-attachments/assets/68e066bc-4329-414d-903e-b15e6f8a9e9a" />
<img width="2048" height="1536" alt="image" src="https://github.com/user-attachments/assets/cb11de6e-4806-4aee-aa97-fcbe10e954fd" />
<img width="2048" height="1536" alt="image" src="https://github.com/user-attachments/assets/fa5e3eda-a5ff-4428-89ed-dac1ec11ef37" />

---

## System Overview

```
┌───────────────────────────────┐                       ┌───────────────────────────────┐
│          TX NODE              │   2.402 GHz RF link   │          RX NODE              │
│                               │──────────────────────▶│                               │
│  ATtiny1626  @ 3.33 MHz       │   nRF24L01+           │  ATmega328P @ 8 MHz           │
│  MCP9808 temperature sensor   │   250 kbps            │  SH1106 1.3" OLED display     │
│  Deep sleep + RTC PIT wakeup  │   5-byte payload      │  Real-time temperature output │
└───────────────────────────────┘                       └───────────────────────────────┘
```

## Firmware

### Transmitter

The transmitter firmware is structured around a low-power event loop. The MCU enters `SLEEP_MODE_PWR_DOWN` after each transmission and is woken periodically by the RTC Peripheral Interrupt Timer (PIT). A software counter accumulates wakeup events; once ten ticks are counted (~10 s), the firmware reads the sensor, serialises the result, transmits the packet, and returns to sleep immediately.

The MCP9808 returns a signed 13-bit fixed-point value. The firmware masks the three flag bits, performs two's complement sign extension where needed, and converts the raw value to a 32-bit IEEE 754 float by dividing by 16:

```
temperature [°C] = raw_13bit_signed / 16.0f
```

The float is then copied byte-for-byte into a 5-byte radio buffer and dispatched over the nRF24L01+ SPI interface.

**Key implementation details:**
- All unused GPIO pins configured with internal pull-ups to prevent floating inputs and reduce leakage current
- Clock prescaler set to ÷3 at startup (10 MHz oscillator → 3.33 MHz) to reduce active-mode power
- RTC PIT driven by the internal 32.768 kHz oscillator; period set to 32 768 cycles (≈1 s per tick)
- Float serialised via direct byte-array copy — no pointer aliasing, no undefined behaviour

### Receiver

The receiver polls the nRF24L01+ STATUS register in the main loop. When the RX_DR flag is asserted, the firmware reads the payload width register, fetches the bytes from the RX FIFO, and reconstructs the float using `memcpy` — avoiding undefined behaviour from type-punning.

The temperature value is then formatted and written to the OLED. The display driver communicates with the SH1106 over I²C (TWI), sends initialisation commands on startup, and updates a single 8-pixel page per refresh. A 5×8 pixel bitmap font stored in program memory (`PROGMEM`) is used to render digits, the degree symbol, minus sign, and decimal comma.

**Rendering logic (`SH1106PrintTemp`):**
1. Clears page 3 of the display buffer
2. Prepends a minus sign for negative values
3. Decomposes the absolute value into integer and two-decimal-place fractional parts
4. Handles rounding overflow (e.g. `x.995 → (x+1).00`)
5. Suppresses the leading zero for single-digit integers
6. Applies a fixed +2 column hardware offset required by 1.3″ SH1106 panels


---

## Pin Mapping

### Transmitter — ATtiny1626 (PORTA / PORTC)

| Signal | Pin |
|---|---|
| SPI MOSI | PA1 |
| SPI MISO | PA2 |
| SPI SCK | PA3 |
| nRF24 CSN | PA4 |
| nRF24 CE | PA5 |
| Status LED | PC0 |
| I²C SDA / SCL | TWI0 default |

### Receiver — ATmega328P

| Signal | Pin |
|---|---|
| nRF24 CE | PC0 |
| SPI CSN | PB2 |
| SPI MOSI | PB3 |
| SPI MISO | PB4 |
| SPI SCK | PB5 |
| I²C SDA | PC4 |
| I²C SCL | PC5 |


## Author

**Mateusz Januszewski** — November 2025
