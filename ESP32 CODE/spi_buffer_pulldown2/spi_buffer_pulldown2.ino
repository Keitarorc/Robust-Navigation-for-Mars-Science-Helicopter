#include "driver/spi_slave.h"
#include <Arduino.h>

#define DRAIN_REQ_PIN    27   // OUTPUT: tells external SPI master to drain
#define DRAIN_ENABLE_PIN 26   // INPUT: LOW = allow draining, HIGH = stop draining

// ===============================
// VSPI SLAVE PINS (ESP32 #1 SLAVE)
// ===============================
#define V_MOSI 23
#define V_MISO 19
#define V_SCLK 18
#define V_CS   5

// ===============================
// HSPI SLAVE PINS (ESP32 #1 SLAVE)
// ===============================
#define H_MOSI 13
#define H_MISO 12
#define H_SCLK 14
#define H_CS   15

// ===============================
// FIFO (RING BUFFER)
// ===============================
#define FIFO_SIZE 2048
static uint8_t fifo[FIFO_SIZE];
static volatile uint32_t fifo_head = 0;
static volatile uint32_t fifo_tail = 0;
static volatile uint32_t fifo_count = 0;

static portMUX_TYPE fifoMux = portMUX_INITIALIZER_UNLOCKED;

static inline bool fifo_push(uint8_t b)
{
  bool ok = false;
  portENTER_CRITICAL(&fifoMux);

  if (fifo_count < FIFO_SIZE) {
    fifo[fifo_head] = b;
    fifo_head = (fifo_head + 1) % FIFO_SIZE;
    fifo_count++;
    ok = true;
  }

  portEXIT_CRITICAL(&fifoMux);
  return ok;
}

static inline bool fifo_pop(uint8_t *b)
{
  bool ok = false;
  portENTER_CRITICAL(&fifoMux);

  if (fifo_count > 0) {
    *b = fifo[fifo_tail];
    fifo_tail = (fifo_tail + 1) % FIFO_SIZE;
    fifo_count--;
    ok = true;
  }

  portEXIT_CRITICAL(&fifoMux);
  return ok;
}

static inline uint32_t fifo_level()
{
  uint32_t c;
  portENTER_CRITICAL(&fifoMux);
  c = fifo_count;
  portEXIT_CRITICAL(&fifoMux);
  return c;
}

// ===============================
// 1-BYTE BUFFERS
// ===============================
#define RX_CHUNK 1
#define TX_CHUNK 1

static uint8_t v_rx[RX_CHUNK];
static uint8_t v_tx[RX_CHUNK];
static spi_slave_transaction_t v_t;

static uint8_t h_rx[TX_CHUNK];
static uint8_t h_tx[TX_CHUNK];
static spi_slave_transaction_t h_t;

// optional shared state for debug
static volatile bool g_drain_allowed = true;

// ===============================
// VSPI SLAVE TASK: RECEIVE -> FIFO
// ===============================
void vspi_slave_task(void *arg)
{
  v_tx[0] = 0xAA;

  while (1)
  {
    memset(&v_t, 0, sizeof(v_t));
    v_t.length = 8;
    v_t.tx_buffer = v_tx;
    v_t.rx_buffer = v_rx;

    esp_err_t err = spi_slave_transmit(VSPI_HOST, &v_t, portMAX_DELAY);
    if (err != ESP_OK) continue;

    uint32_t before = fifo_level();
    bool ok = fifo_push(v_rx[0]);
    uint32_t after = fifo_level();

    Serial.printf("VSPI RX: 0x%02X push_ok=%d fifo %u->%u\n",
                  v_rx[0], ok, before, after);
  }
}

// ===============================
// FAST GPIO MIRROR TASK
// GPIO26 LOW  -> GPIO27 HIGH
// GPIO26 HIGH -> GPIO27 LOW
// ===============================
void drain_gpio_task(void *arg)
{
  int lastEnable = -1;

  while (1)
  {
    int en = digitalRead(DRAIN_ENABLE_PIN);
    bool drain_allowed = (en == LOW);
    g_drain_allowed = drain_allowed;

    digitalWrite(DRAIN_REQ_PIN, drain_allowed ? HIGH : LOW);

    if (en != lastEnable) {
      Serial.printf("GPIO update: GPIO26=%d -> GPIO27=%d\n",
                    en,
                    drain_allowed ? HIGH : LOW);
      lastEnable = en;
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ===============================
// HSPI SLAVE TASK: FIFO -> TRANSMIT
// Uses latest drain state, but does not control GPIO27
// ===============================
void hspi_slave_task(void *arg)
{
  while (1)
  {
    bool drain_allowed = g_drain_allowed;

    uint32_t before = fifo_level();

    uint8_t b = 0x00;
    bool ok = false;

    if (drain_allowed) {
      ok = fifo_pop(&b);
    }

    h_tx[0] = ok ? b : 0x00;

    uint32_t after = fifo_level();

    Serial.printf("HSPI TX: 0x%02X pop_ok=%d drain_en=%d req=%d fifo %u->%u\n",
                  h_tx[0],
                  ok,
                  digitalRead(DRAIN_ENABLE_PIN),
                  digitalRead(DRAIN_REQ_PIN),
                  before,
                  after);

    memset(&h_t, 0, sizeof(h_t));
    h_t.length = 8;
    h_t.tx_buffer = h_tx;
    h_t.rx_buffer = h_rx;

    esp_err_t err = spi_slave_transmit(HSPI_HOST, &h_t, portMAX_DELAY);
    if (err != ESP_OK) continue;
  }
}

void setup()
{
  pinMode(DRAIN_REQ_PIN, OUTPUT);
  digitalWrite(DRAIN_REQ_PIN, LOW);

  pinMode(DRAIN_ENABLE_PIN, INPUT); // external pulldown resistor on GPIO26

  Serial.begin(115200);
  delay(500);

  Serial.println("Initializing Dual SPI Slaves + FIFO (1-byte)...");
  Serial.println("GPIO27 = drain request output");
  Serial.println("GPIO26 = drain enable input");
  Serial.println("GPIO26 LOW  -> GPIO27 HIGH -> drain FIFO");
  Serial.println("GPIO26 HIGH -> GPIO27 LOW  -> stop draining");

  // VSPI
  spi_bus_config_t v_buscfg = {};
  v_buscfg.mosi_io_num = V_MOSI;
  v_buscfg.miso_io_num = V_MISO;
  v_buscfg.sclk_io_num = V_SCLK;
  v_buscfg.quadwp_io_num = -1;
  v_buscfg.quadhd_io_num = -1;

  spi_slave_interface_config_t v_slvcfg = {};
  v_slvcfg.spics_io_num = V_CS;
  v_slvcfg.queue_size = 2;
  v_slvcfg.mode = 0;

  spi_slave_initialize(VSPI_HOST, &v_buscfg, &v_slvcfg, 0);

  // HSPI
  spi_bus_config_t h_buscfg = {};
  h_buscfg.mosi_io_num = H_MOSI;
  h_buscfg.miso_io_num = H_MISO;
  h_buscfg.sclk_io_num = H_SCLK;
  h_buscfg.quadwp_io_num = -1;
  h_buscfg.quadhd_io_num = -1;

  spi_slave_interface_config_t h_slvcfg = {};
  h_slvcfg.spics_io_num = H_CS;
  h_slvcfg.queue_size = 2;
  h_slvcfg.mode = 0;

  spi_slave_initialize(HSPI_HOST, &h_buscfg, &h_slvcfg, 0);

  // Tasks
  xTaskCreatePinnedToCore(vspi_slave_task, "vspi_slave", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(hspi_slave_task, "hspi_slave", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(drain_gpio_task, "drain_gpio", 2048, NULL, 3, NULL, 1);

  Serial.println("Both SPI slaves ready.");
}

void loop() {}