#include "driver/spi_slave.h"
#include <Arduino.h>

// =====================================================
// HANDSHAKE PINS
// =====================================================
// Main system
#define MAIN_DRAIN_ENABLE_PIN 26   // INPUT  : LOW = main should drain FIFO
#define MAIN_DRAIN_REQ_PIN    27   // OUTPUT : HIGH = ESP32 requests main to drain

// Backup system
#define BACKUP_DRAIN_ENABLE_PIN 25 // INPUT  : LOW = backup should drain FIFO
#define BACKUP_DRAIN_REQ_PIN    33 // OUTPUT : HIGH = ESP32 requests backup to drain

// =====================================================
// VSPI PINS = MAIN SYSTEM BUS
// =====================================================
#define V_MOSI 23
#define V_MISO 19
#define V_SCLK 18
#define V_CS   5

// =====================================================
// HSPI PINS = BACKUP SYSTEM BUS
// =====================================================
#define H_MOSI 13
#define H_MISO 12
#define H_SCLK 14
#define H_CS   15

// =====================================================
// FIFO (RING BUFFER)
// =====================================================
#define FIFO_SIZE 2048
static uint8_t fifo[FIFO_SIZE];
static volatile uint32_t fifo_head = 0;
static volatile uint32_t fifo_tail = 0;
static volatile uint32_t fifo_count = 0;

static portMUX_TYPE fifoMux = portMUX_INITIALIZER_UNLOCKED;

// =====================================================
// DRAIN TARGET ARBITRATION
// =====================================================
enum DrainTarget
{
  DRAIN_NONE = 0,
  DRAIN_MAIN,
  DRAIN_BACKUP
};

static volatile DrainTarget g_target = DRAIN_NONE;
static TaskHandle_t arbiterTaskHandle = NULL;

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

static inline DrainTarget compute_drain_target()
{
  bool main_wants_drain   = (digitalRead(MAIN_DRAIN_ENABLE_PIN)   == LOW);
  bool backup_wants_drain = (digitalRead(BACKUP_DRAIN_ENABLE_PIN) == LOW);

  // Priority if both request at once
  if (main_wants_drain)   return DRAIN_MAIN;
  if (backup_wants_drain) return DRAIN_BACKUP;
  return DRAIN_NONE;
}

static inline void update_request_pins(DrainTarget target)
{
  digitalWrite(MAIN_DRAIN_REQ_PIN,   (target == DRAIN_MAIN)   ? HIGH : LOW);
  digitalWrite(BACKUP_DRAIN_REQ_PIN, (target == DRAIN_BACKUP) ? HIGH : LOW);
}

// =====================================================
// ISR: wake arbiter immediately when enable pins change
// =====================================================
void IRAM_ATTR onEnablePinChangeISR()
{
  BaseType_t hpTaskWoken = pdFALSE;
  if (arbiterTaskHandle != NULL) {
    vTaskNotifyGiveFromISR(arbiterTaskHandle, &hpTaskWoken);
    if (hpTaskWoken) {
      portYIELD_FROM_ISR();
    }
  }
}

// =====================================================
// ARBITER TASK
// Immediately mirrors enable-pin changes into request pins
// =====================================================
void arbiter_task(void *arg)
{
  DrainTarget lastTarget = DRAIN_NONE;

  // Initial update
  g_target = compute_drain_target();
  update_request_pins((DrainTarget)g_target);
  lastTarget = (DrainTarget)g_target;

  Serial.printf("ARB init: target=%d main_en=%d backup_en=%d\n",
                (int)g_target,
                digitalRead(MAIN_DRAIN_ENABLE_PIN),
                digitalRead(BACKUP_DRAIN_ENABLE_PIN));

  while (1)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    DrainTarget newTarget = compute_drain_target();
    g_target = newTarget;
    update_request_pins(newTarget);

    if (newTarget != lastTarget) {
      Serial.printf("ARB change: target=%d main_en=%d backup_en=%d req_main=%d req_backup=%d\n",
                    (int)newTarget,
                    digitalRead(MAIN_DRAIN_ENABLE_PIN),
                    digitalRead(BACKUP_DRAIN_ENABLE_PIN),
                    digitalRead(MAIN_DRAIN_REQ_PIN),
                    digitalRead(BACKUP_DRAIN_REQ_PIN));
      lastTarget = newTarget;
    }
  }
}

// =====================================================
// 1-BYTE BUFFERS
// =====================================================
#define CHUNK 1

static uint8_t v_rx[CHUNK];
static uint8_t v_tx[CHUNK];
static spi_slave_transaction_t v_t;

static uint8_t h_rx[CHUNK];
static uint8_t h_tx[CHUNK];
static spi_slave_transaction_t h_t;

// =====================================================
// RX FILTER
// =====================================================
static inline bool rx_byte_is_valid(uint8_t b)
{
  (void)b;
  return true;
}

// =====================================================
// VSPI TASK = MAIN SYSTEM BUS
// =====================================================
void vspi_slave_task(void *arg)
{
  while (1)
  {
    DrainTarget target = (DrainTarget)g_target;
    bool main_is_drain_target = (target == DRAIN_MAIN);

    uint32_t before = fifo_level();

    uint8_t txb = 0x00;
    bool pop_ok = false;

    if (main_is_drain_target) {
      pop_ok = fifo_pop(&txb);
    }

    v_tx[0] = pop_ok ? txb : 0x00;

    memset(&v_t, 0, sizeof(v_t));
    v_t.length = 8;
    v_t.tx_buffer = v_tx;
    v_t.rx_buffer = v_rx;

    esp_err_t err = spi_slave_transmit(VSPI_HOST, &v_t, portMAX_DELAY);
    if (err != ESP_OK) {
      continue;
    }

    bool push_ok = false;
    if (rx_byte_is_valid(v_rx[0])) {
      push_ok = fifo_push(v_rx[0]);
    }

    uint32_t after = fifo_level();

    Serial.printf(
      "VSPI  target=%d tx=0x%02X pop_ok=%d rx=0x%02X push_ok=%d fifo %u->%u\n",
      (int)target, v_tx[0], pop_ok, v_rx[0], push_ok, before, after
    );
  }
}

// =====================================================
// HSPI TASK = BACKUP SYSTEM BUS
// =====================================================
void hspi_slave_task(void *arg)
{
  while (1)
  {
    DrainTarget target = (DrainTarget)g_target;
    bool backup_is_drain_target = (target == DRAIN_BACKUP);

    uint32_t before = fifo_level();

    uint8_t txb = 0x00;
    bool pop_ok = false;

    if (backup_is_drain_target) {
      pop_ok = fifo_pop(&txb);
    }

    h_tx[0] = pop_ok ? txb : 0x00;

    memset(&h_t, 0, sizeof(h_t));
    h_t.length = 8;
    h_t.tx_buffer = h_tx;
    h_t.rx_buffer = h_rx;

    esp_err_t err = spi_slave_transmit(HSPI_HOST, &h_t, portMAX_DELAY);
    if (err != ESP_OK) {
      continue;
    }

    bool push_ok = false;
    if (rx_byte_is_valid(h_rx[0])) {
      push_ok = fifo_push(h_rx[0]);
    }

    uint32_t after = fifo_level();

    Serial.printf(
      "HSPI  target=%d tx=0x%02X pop_ok=%d rx=0x%02X push_ok=%d fifo %u->%u\n",
      (int)target, h_tx[0], pop_ok, h_rx[0], push_ok, before, after
    );
  }
}

// =====================================================
// SETUP
// =====================================================
void setup()
{
  pinMode(MAIN_DRAIN_REQ_PIN, OUTPUT);
  pinMode(BACKUP_DRAIN_REQ_PIN, OUTPUT);
  digitalWrite(MAIN_DRAIN_REQ_PIN, LOW);
  digitalWrite(BACKUP_DRAIN_REQ_PIN, LOW);

  // You said you have external pulldowns
  pinMode(MAIN_DRAIN_ENABLE_PIN, INPUT);
  pinMode(BACKUP_DRAIN_ENABLE_PIN, INPUT);

  Serial.begin(115200);
  delay(500);

  Serial.println("Initializing dual SPI slaves with immediate request-pin response...");
  Serial.println("VSPI   = main system bus");
  Serial.println("HSPI   = backup system bus");
  Serial.println("GPIO26 = MAIN drain enable  (LOW = main drains)");
  Serial.println("GPIO27 = MAIN drain request (HIGH = ESP32 requests main)");
  Serial.println("GPIO25 = BACKUP drain enable  (LOW = backup drains)");
  Serial.println("GPIO33 = BACKUP drain request (HIGH = ESP32 requests backup)");
  Serial.println("If both enable pins are LOW, MAIN has priority.");

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

  esp_err_t err = spi_slave_initialize(VSPI_HOST, &v_buscfg, &v_slvcfg, 0);
  if (err != ESP_OK) {
    Serial.printf("VSPI init failed: %d\n", err);
    while (1) { delay(1000); }
  }

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

  err = spi_slave_initialize(HSPI_HOST, &h_buscfg, &h_slvcfg, 0);
  if (err != ESP_OK) {
    Serial.printf("HSPI init failed: %d\n", err);
    while (1) { delay(1000); }
  }

  xTaskCreatePinnedToCore(arbiter_task,   "arbiter",    3072, NULL, 4, &arbiterTaskHandle, 1);
  xTaskCreatePinnedToCore(vspi_slave_task,"vspi_slave", 4096, NULL, 2, NULL,               0);
  xTaskCreatePinnedToCore(hspi_slave_task,"hspi_slave", 4096, NULL, 2, NULL,               1);

  attachInterrupt(digitalPinToInterrupt(MAIN_DRAIN_ENABLE_PIN),   onEnablePinChangeISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BACKUP_DRAIN_ENABLE_PIN), onEnablePinChangeISR, CHANGE);

  // Force one initial arbiter pass
  xTaskNotifyGive(arbiterTaskHandle);

  Serial.println("Both SPI slaves ready.");
}

void loop()
{
}