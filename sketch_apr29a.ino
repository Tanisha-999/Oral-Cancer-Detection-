/*
 * ============================================================
 *  ORAL CANCER SCREENING DEVICE — HARDWARE FIRMWARE
 *  XIAO ESP32-S3 Sense | FreeRTOS Dual-Core | No ML
 * ============================================================
 *
 *  SOFTWARE REQUIRED:
 *  - Arduino IDE 2.x (https://www.arduino.cc/en/software)
 *  - Board Package: esp32 by Espressif v2.0.8 or higher
 *    URL: https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
 *  - Libraries (install via Sketch > Include Library > Manage Libraries):
 *      1. "Adafruit SSD1306"    — OLED display driver
 *      2. "Adafruit GFX Library" — graphics primitives (dependency)
 *
 *  BOARD SETTINGS (Tools menu):
 *  - Board:            XIAO_ESP32S3
 *  - PSRAM:            OPI PSRAM   ← CRITICAL for camera frame buffer
 *  - Flash Size:       8MB (64Mb)
 *  - Partition Scheme: Huge APP (3MB No OTA/1MB SPIFFS)
 *  - Upload Speed:     921600
 *
 *  HARDWARE CONNECTIONS:
 *  - Camera:     Built-in OV2640 on Sense expansion board (no wiring needed)
 *  - 405nm LED:  GPIO 26 → NPN transistor base (via 1kΩ resistor) → LED+ → 3.7V
 *  - Capture Btn:GPIO 1  → GND (active LOW, use INPUT_PULLUP)
 *  - OLED (opt): SDA → GPIO 5, SCL → GPIO 6  (I2C, 0x3C address)
 *  - SD Card:    Built-in on Sense expansion board (SPI)
 *
 *  FREERTOS DUAL-CORE STRATEGY (from your Phase 1 report):
 *  - Core 0: Camera DMA acquisition, LED control, Button polling, OLED display
 *  - Core 1: SD card writes, Serial data forwarding (Edge Impulse bridge)
 * ============================================================
 */

// ============================================================
//  INCLUDES
// ============================================================
#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

// Camera
#include "esp_camera.h"

// OLED (optional — comment out if not using display)
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// SD Card
#include "FS.h"
#include "SD.h"
#include "SPI.h"

// ============================================================
//  PIN DEFINITIONS  (XIAO ESP32-S3 Sense)
// ============================================================
#define LED_405NM_PIN     26    // 405 nm excitation LED via transistor
#define CAPTURE_BTN_PIN   1     // Capture button (active LOW)

// OLED (I2C)
#define OLED_SDA          5
#define OLED_SCL          6
#define OLED_WIDTH        128
#define OLED_HEIGHT       64
#define OLED_ADDR         0x3C

// SD Card SPI pins (Sense expansion board)
#define SD_CS_PIN         21
#define SD_MOSI_PIN       9
#define SD_MISO_PIN       8
#define SD_CLK_PIN        7

// ============================================================
//  CAMERA PIN DEFINITIONS  (OV2640 on XIAO Sense)
// ============================================================
#define CAM_PIN_PWDN     -1
#define CAM_PIN_RESET    -1
#define CAM_PIN_XCLK      10
#define CAM_PIN_SIOD      40
#define CAM_PIN_SIOC      39
#define CAM_PIN_D7        48
#define CAM_PIN_D6        11
#define CAM_PIN_D5        12
#define CAM_PIN_D4        14
#define CAM_PIN_D3        16
#define CAM_PIN_D2        18
#define CAM_PIN_D1        17
#define CAM_PIN_D0        15
#define CAM_PIN_VSYNC     38
#define CAM_PIN_HREF      47
#define CAM_PIN_PCLK      13

// ============================================================
//  RTOS OBJECTS
// ============================================================
// Semaphore: signals Core 1 that a new frame is ready to save
SemaphoreHandle_t xFrameReadySemaphore;

// Mutex: protects shared frame buffer pointer
SemaphoreHandle_t xFrameMutex;

// Queue: passes status messages to OLED task
QueueHandle_t xStatusQueue;

// Task handles
TaskHandle_t xCameraTaskHandle   = NULL;
TaskHandle_t xSDWriteTaskHandle  = NULL;
TaskHandle_t xOLEDTaskHandle     = NULL;
TaskHandle_t xLEDTaskHandle      = NULL;

// ============================================================
//  SHARED STATE (protected by xFrameMutex)
// ============================================================
camera_fb_t* sharedFrameBuffer = NULL;
volatile bool newFramePending   = false;
volatile uint32_t captureCount  = 0;

// ============================================================
//  OLED OBJECT
// ============================================================
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
bool oledAvailable = false;

// ============================================================
//  STATUS MESSAGE TYPE
// ============================================================
typedef struct {
  char message[32];
  bool isError;
} StatusMsg_t;

// ============================================================
//  CAMERA INITIALISATION
// ============================================================
bool initCamera() {
  camera_config_t config;

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = CAM_PIN_D0;
  config.pin_d1       = CAM_PIN_D1;
  config.pin_d2       = CAM_PIN_D2;
  config.pin_d3       = CAM_PIN_D3;
  config.pin_d4       = CAM_PIN_D4;
  config.pin_d5       = CAM_PIN_D5;
  config.pin_d6       = CAM_PIN_D6;
  config.pin_d7       = CAM_PIN_D7;
  config.pin_xclk     = CAM_PIN_XCLK;
  config.pin_pclk     = CAM_PIN_PCLK;
  config.pin_vsync    = CAM_PIN_VSYNC;
  config.pin_href     = CAM_PIN_HREF;
  config.pin_sccb_sda = CAM_PIN_SIOD;
  config.pin_sccb_scl = CAM_PIN_SIOC;
  config.pin_pwdn     = CAM_PIN_PWDN;
  config.pin_reset    = CAM_PIN_RESET;

  config.xclk_freq_hz = 20000000;    // 20 MHz XCLK
  config.pixel_format = PIXFORMAT_JPEG;

  // Use PSRAM for large frame buffers (OPI PSRAM must be enabled in Tools!)
  if (psramFound()) {
    config.frame_size   = FRAMESIZE_UXGA;  // 1600x1200 — full resolution for LAF imaging
    config.jpeg_quality = 10;              // 0=best, 63=worst
    config.fb_count     = 2;              // Double-buffer for DMA
    config.fb_location  = CAMERA_FB_IN_PSRAM;
    Serial.println("[Camera] PSRAM found — using UXGA resolution");
  } else {
    // Fallback: no PSRAM (OPI PSRAM not enabled in Tools!)
    config.frame_size   = FRAMESIZE_SVGA;  // 800x600 fallback
    config.jpeg_quality = 20;
    config.fb_count     = 1;
    config.fb_location  = CAMERA_FB_IN_DRAM;
    Serial.println("[Camera] WARNING: No PSRAM! Enable OPI PSRAM in Tools. Falling back to SVGA.");
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[Camera] Init FAILED: 0x%x\n", err);
    return false;
  }

  // Sensor fine-tuning for autofluorescence imaging
  sensor_t* s = esp_camera_sensor_get();
  s->set_brightness(s, 1);      // Slightly brighter to catch weak emission signal
  s->set_saturation(s, 0);      // Neutral saturation
  s->set_contrast(s, 1);        // Slight contrast boost for LAF dark patches
  s->set_whitebal(s, 0);        // Disable auto white balance (405nm illumination is fixed)
  s->set_awb_gain(s, 0);
  s->set_exposure_ctrl(s, 0);   // Disable AEC — manual exposure for consistent results
  s->set_aec_value(s, 400);     // Fixed exposure value
  s->set_gain_ctrl(s, 0);       // Disable AGC
  s->set_agc_gain(s, 5);        // Fixed gain

  Serial.println("[Camera] Initialized OK");
  return true;
}

// ============================================================
//  SD CARD INITIALISATION
// ============================================================
bool initSD() {
  SPI.begin(SD_CLK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("[SD] Mount FAILED — check card or wiring");
    return false;
  }
  Serial.printf("[SD] Card size: %llu MB\n", SD.cardSize() / (1024 * 1024));

  // Create directory for scans
  if (!SD.exists("/scans")) {
    SD.mkdir("/scans");
    Serial.println("[SD] Created /scans directory");
  }
  return true;
}

// ============================================================
//  TASK 1 — CAMERA ACQUISITION (Core 0)
//  Polls capture button, fires LED, grabs frame, signals Core 1
// ============================================================
void vCameraTask(void* pvParameters) {
  Serial.println("[Task] CameraTask started on Core: " + String(xPortGetCoreID()));

  StatusMsg_t msg;
  bool btnPrevState = HIGH;

  while (1) {
    bool btnState = digitalRead(CAPTURE_BTN_PIN);

    // Detect falling edge (button press)
    if (btnState == LOW && btnPrevState == HIGH) {
      Serial.println("[Camera] Button pressed — starting capture");

      // ── 1. Turn ON 405 nm LED ──────────────────────────
      digitalWrite(LED_405NM_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(150)); // Allow tissue to fluoresce (~150ms warm-up)

      // ── 2. Flush stale frame from buffer ──────────────
      camera_fb_t* stale = esp_camera_fb_get();
      if (stale) esp_camera_fb_return(stale);

      // ── 3. Capture fresh frame ─────────────────────────
      camera_fb_t* fb = esp_camera_fb_get();

      // ── 4. Turn OFF LED immediately after capture ──────
      //    (Duty cycling — saves power & prevents overheating)
      digitalWrite(LED_405NM_PIN, LOW);

      if (!fb) {
        Serial.println("[Camera] Frame capture FAILED");
        snprintf(msg.message, sizeof(msg.message), "Capture FAILED!");
        msg.isError = true;
        xQueueSend(xStatusQueue, &msg, 0);
      } else {
        Serial.printf("[Camera] Frame captured: %u bytes\n", fb->len);

        // ── 5. Hand frame to SD task via shared buffer ───
        if (xSemaphoreTake(xFrameMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
          // Return any previous unread frame
          if (sharedFrameBuffer != NULL) {
            esp_camera_fb_return(sharedFrameBuffer);
          }
          sharedFrameBuffer = fb;
          newFramePending   = true;
          captureCount++;
          xSemaphoreGive(xFrameMutex);

          // Signal SD task
          xSemaphoreGive(xFrameReadySemaphore);

          snprintf(msg.message, sizeof(msg.message), "Saving #%lu...", captureCount);
          msg.isError = false;
          xQueueSend(xStatusQueue, &msg, 0);
        } else {
          // Mutex timeout — SD task busy, discard frame
          esp_camera_fb_return(fb);
          Serial.println("[Camera] Mutex timeout — frame discarded");
        }
      }

      vTaskDelay(pdMS_TO_TICKS(500)); // Debounce
    }

    btnPrevState = btnState;
    vTaskDelay(pdMS_TO_TICKS(20)); // Poll button at 50 Hz
  }
}

// ============================================================
//  TASK 2 — SD CARD WRITER (Core 1)
//  Waits for signal, saves JPEG to SD, forwards via Serial
// ============================================================
void vSDWriteTask(void* pvParameters) {
  Serial.println("[Task] SDWriteTask started on Core: " + String(xPortGetCoreID()));

  StatusMsg_t msg;
  char filepath[64];

  while (1) {
    // Block until CameraTask signals a new frame
    if (xSemaphoreTake(xFrameReadySemaphore, portMAX_DELAY) == pdTRUE) {

      camera_fb_t* fbToSave = NULL;
      uint32_t frameNum = 0;

      // Take frame ownership
      if (xSemaphoreTake(xFrameMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        fbToSave        = sharedFrameBuffer;
        sharedFrameBuffer = NULL;
        newFramePending  = false;
        frameNum         = captureCount;
        xSemaphoreGive(xFrameMutex);
      }

      if (fbToSave == NULL) {
        Serial.println("[SD] No frame to save");
        continue;
      }

      // ── Write JPEG to SD card ──────────────────────────
      snprintf(filepath, sizeof(filepath), "/scans/scan_%04lu.jpg", frameNum);
      File f = SD.open(filepath, FILE_WRITE);
      if (!f) {
        Serial.printf("[SD] Failed to open %s\n", filepath);
        snprintf(msg.message, sizeof(msg.message), "SD Write FAIL!");
        msg.isError = true;
        xQueueSend(xStatusQueue, &msg, 0);
      } else {
        size_t written = f.write(fbToSave->buf, fbToSave->len);
        f.close();
        Serial.printf("[SD] Saved %s (%u bytes)\n", filepath, written);

        snprintf(msg.message, sizeof(msg.message), "Saved scan_%04lu", frameNum);
        msg.isError = false;
        xQueueSend(xStatusQueue, &msg, 0);
      }

      // ── Edge Impulse Data Forwarder Bridge ─────────────
      // Streams raw JPEG bytes over Serial for dataset collection
      // Usage: run `edge-impulse-data-forwarder` on your PC
      Serial.write(fbToSave->buf, fbToSave->len);
      Serial.println(); // newline delimiter for forwarder

      // Return frame buffer to camera DMA pool
      esp_camera_fb_return(fbToSave);
    }
  }
}

// ============================================================
//  TASK 3 — OLED DISPLAY (Core 0)
//  Shows status messages from the queue
// ============================================================
void vOLEDTask(void* pvParameters) {
  Serial.println("[Task] OLEDTask started on Core: " + String(xPortGetCoreID()));

  StatusMsg_t msg;
  uint32_t lastHeapReport = 0;

  // Boot screen
  if (oledAvailable) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("OralScan v1.0");
    display.println("Hardware Ready");
    display.println("Press button to");
    display.println("capture scan.");
    display.display();
  }

  while (1) {
    // Non-blocking queue receive with 500ms timeout
    if (xQueueReceive(xStatusQueue, &msg, pdMS_TO_TICKS(500)) == pdTRUE) {
      if (oledAvailable) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);

        if (msg.isError) {
          display.println("!! ERROR !!");
        } else {
          display.println("OralScan v1.0");
        }
        display.println(msg.message);
        display.printf("Scans: %lu\n", captureCount);
        display.display();
      }
      Serial.printf("[OLED] Status: %s\n", msg.message);
    }

    // Periodic heap stats every 10 seconds
    if (millis() - lastHeapReport > 10000) {
      Serial.printf("[Heap] Free: %u bytes | Min: %u bytes\n",
                    esp_get_free_heap_size(),
                    esp_get_minimum_free_heap_size());
      lastHeapReport = millis();
    }
  }
}

// ============================================================
//  TASK 4 — SYSTEM WATCHDOG / IDLE (Core 0)
//  Monitors stack watermarks for debugging
// ============================================================
void vWatchdogTask(void* pvParameters) {
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(30000)); // Check every 30 seconds

    if (xCameraTaskHandle)  Serial.printf("[WD] Camera task stack HWM: %u\n",  uxTaskGetStackHighWaterMark(xCameraTaskHandle));
    if (xSDWriteTaskHandle) Serial.printf("[WD] SD task stack HWM:     %u\n",  uxTaskGetStackHighWaterMark(xSDWriteTaskHandle));
    if (xOLEDTaskHandle)    Serial.printf("[WD] OLED task stack HWM:   %u\n",  uxTaskGetStackHighWaterMark(xOLEDTaskHandle));
  }
}

// ============================================================
//  SETUP  (runs on Core 1 before scheduler)
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n============================");
  Serial.println(" OralScan Hardware Firmware");
  Serial.println("============================");

  // ── GPIO Setup ────────────────────────────────────────────
  pinMode(LED_405NM_PIN,  OUTPUT);
  pinMode(CAPTURE_BTN_PIN, INPUT_PULLUP);
  digitalWrite(LED_405NM_PIN, LOW); // LED off by default

  // ── I2C & OLED ────────────────────────────────────────────
  Wire.begin(OLED_SDA, OLED_SCL);
  if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    oledAvailable = true;
    Serial.println("[OLED] Display initialized");
  } else {
    Serial.println("[OLED] Not found — continuing without display");
  }

  // ── Camera Init ───────────────────────────────────────────
  if (!initCamera()) {
    Serial.println("[FATAL] Camera init failed. Check Sense expansion board.");
    while (1) { delay(1000); } // Halt
  }

  // ── SD Card Init ──────────────────────────────────────────
  if (!initSD()) {
    Serial.println("[WARNING] SD card not available. Images won't be saved.");
    // Not fatal — device can still stream via Serial
  }

  // ── RTOS Objects ──────────────────────────────────────────
  xFrameReadySemaphore = xSemaphoreCreateBinary();
  xFrameMutex          = xSemaphoreCreateMutex();
  xStatusQueue         = xQueueCreate(5, sizeof(StatusMsg_t));

  if (!xFrameReadySemaphore || !xFrameMutex || !xStatusQueue) {
    Serial.println("[FATAL] RTOS object creation failed!");
    while (1) { delay(1000); }
  }

  // ── Create FreeRTOS Tasks ─────────────────────────────────
  //
  //  xTaskCreatePinnedToCore(
  //    function,    name,         stack(bytes), param, priority, handle, core)
  //
  //  Priority: 0=lowest, configMAX_PRIORITIES-1=highest
  //  Camera and SD should be higher priority than display
  //
  xTaskCreatePinnedToCore(
    vCameraTask,   "CameraTask",   8192,  NULL, 3, &xCameraTaskHandle,  0);

  xTaskCreatePinnedToCore(
    vSDWriteTask,  "SDWriteTask",  8192,  NULL, 3, &xSDWriteTaskHandle, 1);

  xTaskCreatePinnedToCore(
    vOLEDTask,     "OLEDTask",     4096,  NULL, 1, &xOLEDTaskHandle,    0);

  xTaskCreatePinnedToCore(
    vWatchdogTask, "WatchdogTask", 2048,  NULL, 1, NULL,                0);

  Serial.println("[Setup] All tasks created. System running.");
  Serial.println("[Setup] Press the capture button to take a scan.");
  Serial.println("[Setup] Core 0: Camera + LED + OLED | Core 1: SD Writer");
}

// ============================================================
//  LOOP  — Empty: FreeRTOS tasks handle everything
// ============================================================
void loop() {
  vTaskDelay(portMAX_DELAY); // Yield forever — don't waste Core 1 cycles
}

/*
 * ============================================================
 *  QUICK-START CHECKLIST
 * ============================================================
 *
 *  [ ] 1. Install Arduino IDE 2.x
 *  [ ] 2. Add ESP32 board URL in File > Preferences
 *  [ ] 3. Install board: Tools > Board Manager > "esp32" by Espressif
 *  [ ] 4. Install libraries: "Adafruit SSD1306" + "Adafruit GFX Library"
 *  [ ] 5. Select Board: XIAO_ESP32S3
 *  [ ] 6. Set PSRAM: OPI PSRAM  ← do NOT skip this!
 *  [ ] 7. Set Partition: Huge APP (3MB No OTA)
 *  [ ] 8. Wire 405nm LED to GPIO 26 via NPN transistor
 *  [ ] 9. Wire capture button between GPIO 1 and GND
 *  [ ] 10. Perform the Lens Hack (remove IR filter, add Yellow Gel)
 *  [ ] 11. Upload and open Serial Monitor at 115200 baud
 *  [ ] 12. Press button — LED fires, image captured, saved to SD
 *
 *  FOR EDGE IMPULSE DATASET COLLECTION:
 *  - Run: edge-impulse-data-forwarder --frequency 1
 *  - Each button press streams one JPEG frame via Serial
 *
 * ============================================================
 */
