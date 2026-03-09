#include <Arduino.h>
#include <SPI.h>

SPIClass spiA(VSPI); // Producer: writes to slave VSPI
SPIClass spiB(HSPI); // Consumer: reads from slave HSPI

// Adjust to your MASTER wiring
#define A_SCLK 18
#define A_MISO 19
#define A_MOSI 23
#define A_CS   5

#define B_SCLK 14
#define B_MISO 12
#define B_MOSI 13
#define B_CS   15

// Slave -> Master GPIO (wire from SLAVE DRAIN_REQ_PIN)
#define DRAIN_REQ_PIN 27

static inline void cs_low(int pin)  { digitalWrite(pin, LOW); }
static inline void cs_high(int pin) { digitalWrite(pin, HIGH); }

volatile bool drain_requested = false;

void IRAM_ATTR onDrainReqRise()
{
  drain_requested = true;
}

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
  uint8_t b = spiB.transfer(0x00); // dummy clocks to read 1 byte
  cs_high(B_CS);
  spiB.endTransaction();
  return b;
}

void drainWhileRequested()
{
  // Drain until slave lowers DRAIN_REQ (i.e., fifo_count <= LOW_WM)
  // This naturally handles "intaking new bytes while draining".
  uint32_t n = 0;

  while (digitalRead(DRAIN_REQ_PIN) == HIGH) {
    (void)readByteFromHSPI();   // <-- capture the byte
    n++;

    // Optional: avoid Serial spam; print occasionally
    if ((n % 256) == 0) {
      Serial.printf("Drained %lu bytes...\n", (unsigned long)n);
    }
  }

  Serial.printf("Drain cycle complete. Drained %lu bytes.\n", (unsigned long)n);
}

void setup()
{
  Serial.begin(115200);
  delay(500);

  pinMode(A_CS, OUTPUT); cs_high(A_CS);
  pinMode(B_CS, OUTPUT); cs_high(B_CS);

  pinMode(DRAIN_REQ_PIN, INPUT); // consider INPUT_PULLDOWN if line floats during reset

  spiA.begin(A_SCLK, A_MISO, A_MOSI, A_CS);
  spiB.begin(B_SCLK, B_MISO, B_MOSI, B_CS);

  attachInterrupt(digitalPinToInterrupt(DRAIN_REQ_PIN), onDrainReqRise, RISING);

  Serial.println("Master ready: drain when FIFO full.");
}

void loop()
{
  // Example producer: keep writing bytes into FIFO
  static uint8_t x = 0;
  sendByteToVSPI(x++);
  // Adjust producer rate to match your real application
  // If you produce too fast and never drain enough, you will overflow.
  delayMicroseconds(200); 

  // Drain only when requested
  if (drain_requested) {
    drain_requested = false;

    // If it’s still high, do the drain cycle
    if (digitalRead(DRAIN_REQ_PIN) == HIGH) {
      Serial.println("DRAIN_REQ high -> starting drain cycle...");
      drainWhileRequested();
    }
  }
}