#include <Arduino.h>
#include <SPI.h>


// Two SPI masters on one ESP32
SPIClass spiA(VSPI); // will talk to SLAVE VSPI pins (GPIO 18/19/23/5 on slave)
SPIClass spiB(HSPI); // will talk to SLAVE HSPI pins (GPIO 14/12/13/15 on slave)

// ===============================
// MASTER PINS (ESP32 #2 MASTER)
// Set these to the GPIOs you physically wired.
// ===============================

// Bus A (writes into FIFO via slave VSPI)
#define A_SCLK 18
#define A_MISO 19
#define A_MOSI 23
#define A_CS   5

// Bus B (reads from FIFO via slave HSPI)
#define B_SCLK 14
#define B_MISO 12
#define B_MOSI 13
#define B_CS   15

static inline void cs_low(int pin)  { digitalWrite(pin, LOW); }
static inline void cs_high(int pin) { digitalWrite(pin, HIGH); }

uint8_t sendByteToVSPI(uint8_t b)
{
  spiA.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  cs_low(A_CS);
  uint8_t rx = spiA.transfer(b);
  cs_high(A_CS);
  spiA.endTransaction();
  return rx;
}

uint8_t readByteFromHSPI()
{
  spiB.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  cs_low(B_CS);
  uint8_t b = spiB.transfer(0x00); // dummy byte to clock out 1 byte from slave
  cs_high(B_CS);
  spiB.endTransaction();
  return b;
}

void setup()
{
  Serial.begin(115200);
  delay(500);

  pinMode(A_CS, OUTPUT); cs_high(A_CS);
  pinMode(B_CS, OUTPUT); cs_high(B_CS);

  // Init buses with explicit pins
  spiA.begin(A_SCLK, A_MISO, A_MOSI, A_CS);
  spiB.begin(B_SCLK, B_MISO, B_MOSI, B_CS);

  Serial.println("Dual SPI Master ready.");
}

void loop()
{
  static uint8_t x = 0x00; // start value

  // 1) Write one byte into FIFO (VSPI -> slave)
  uint8_t vspi_reply = sendByteToVSPI(x);

  // 2) Read one byte out of FIFO (HSPI <- slave)
  uint8_t out = readByteFromHSPI();

  Serial.printf("Sent: 0x%02X  (VSPI reply: 0x%02X)  Read: 0x%02X\n",
                x, vspi_reply, out);

  x++; // change value each loop to prove FIFO ordering
  delay(200);
}