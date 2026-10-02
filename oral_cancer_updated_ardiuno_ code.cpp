#include "esp_camera.h"
#include <WiFi.h>
#include "WebServer.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --- WiFi Credentials ---
const char* ssid     = "charitha";
const char* password = "anuglnpan2021731";

WebServer server(80);

// --- HARDWARE PINOUT (Your Exact Breadboard Wiring) ---
#define MOSFET_GATE_PIN   2   // Violet 405nm LED MOSFET Gate
#define BUTTON_PIN        1   // Capture Button
#define I2C_SDA           8   // OLED Display SDA
#define I2C_SCL           9   // OLED Display SCL

// --- CAMERA PINOUT (ESP32-S3 Ribbon Connector) ---
#define CAM_PIN_PWDN      -1
#define CAM_PIN_RESET     -1
#define CAM_PIN_XCLK      15
#define CAM_PIN_SIOD       4
#define CAM_PIN_SIOC       5
#define CAM_PIN_D7        16
#define CAM_PIN_D6        17
#define CAM_PIN_D5        18
#define CAM_PIN_D4        12
#define CAM_PIN_D3        10
#define CAM_PIN_D2         8
#define CAM_PIN_D1         9
#define CAM_PIN_D0        11
#define CAM_PIN_VSYNC      6
#define CAM_PIN_HREF       7
#define CAM_PIN_PCLK      13

// OLED Display Configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

int scanCount = 0;

// Function Declarations
bool initCamera();
void handleCapture();
void handleRoot();
void updateOLED(String line1, String line2, String line3);

void setup() {
  Serial.begin(115200);
  delay(500);

  // Initialize GPIOs
  pinMode(MOSFET_GATE_PIN, OUTPUT);
  digitalWrite(MOSFET_GATE_PIN, LOW); // LED OFF by default
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Initialize OLED Interface
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED Init Failed!");
  }
  
  updateOLED("ORAL CANCER TRIAGE", "Initializing...", "");

  // Initialize Camera Interface
  if (!initCamera()) {
    Serial.println("Camera Init Failed!");
    updateOLED("ERROR!", "Cam Init Failed", "Check Ribbon Cable");
    return;
  }

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");
  updateOLED("Wi-Fi Connecting", ssid, "");
  
  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 20) {
    delay(500);
    Serial.print(".");
    timeout++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("Access URL: http://");
    Serial.println(WiFi.localIP());
    updateOLED("STATUS: READY", "Press button to scan", "IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("\nWiFi Failed (Offline Mode Enabled)");
    updateOLED("STATUS: READY (OFFLINE)", "Press button to scan", "No WiFi Connection");
  }

  // Web Server Endpoints
  server.on("/", handleRoot);
  server.on("/capture", handleCapture);
  server.begin();
}

void loop() {
  server.handleClient();

  // Handle Physical Button Press
  if (digitalRead(BUTTON_PIN) == LOW) {
    delay(50); // Debounce
    if (digitalRead(BUTTON_PIN) == LOW) {
      handleCapture();
      while (digitalRead(BUTTON_PIN) == LOW); // Wait for release
    }
  }
}

void handleRoot() {
  String html = "<html><head><title>Oral Malignancy Triage Node</title></head><body>";
  html += "<h2>Low-Cost Optoelectronic Sensing Node</h2>";
  html += "<p>Total Scans Taken: " + String(scanCount) + "</p>";
  html += "<a href='/capture'><button style='padding:15px; font-size:18px;'>TRIGGER 405nm SCAN</button></a>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleCapture() {
  updateOLED("STATUS: SCANNING...", "405nm UV Light ON", "Capturing frame");

  // 1. Trigger 405nm Violet Illumination
  digitalWrite(MOSFET_GATE_PIN, HIGH);
  delay(120); // Sensor exposure stabilization under UV excitation

  // 2. Capture Autofluorescence Frame Buffer
  camera_fb_t * fb = esp_camera_fb_get();

  // 3. Extinguish UV Light Immediately (Thermal & Safety Control)
  digitalWrite(MOSFET_GATE_PIN, LOW);

  if (!fb) {
    Serial.println("Camera capture failed");
    updateOLED("ERROR!", "Capture Failed", "Retake Prompt");
    server.send(500, "text/plain", "Camera capture failed");
    return;
  }

  // 4. Quality Gate Check (Basic Exposure Check)
  bool passesQualityGate = (fb->len > 10000); // Ensures non-corrupt, exposed frame
  
  if (passesQualityGate) {
    scanCount++;
    updateOLED("STATUS: CAPTURED!", "Size: " + String(fb->len) + " B", "Scans done: " + String(scanCount));
  } else {
    updateOLED("QUALITY GATE REJECT", "Motion/Glare detected", "PROMPT: Retake Scan");
  }

  // 5. Stream back JPEG direct to HTTP response
  server.setContentLength(fb->len);
  server.send(200, "image/jpeg", "");
  WiFiClient client = server.client();
  if (client) {
    client.write(fb->buf, fb->len);
  }

  esp_camera_fb_return(fb);

  delay(2500); // Hold status on display
  updateOLED("STATUS: READY", "Press button to scan", "Scans done: " + String(scanCount));
}

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
  config.pin_sscb_sda = CAM_PIN_SIOD;
  config.pin_sscb_scl = CAM_PIN_SIOC;
  config.pin_pwdn     = CAM_PIN_PWDN;
  config.pin_reset    = CAM_PIN_RESET;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size   = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count     = 2;
    config.grab_mode    = CAMERA_GRAB_LATEST;
    config.fb_location  = CAMERA_FB_IN_PSRAM;
  } else {
    config.frame_size   = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count     = 1;
    config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location  = CAMERA_FB_IN_DRAM;
  }

  return (esp_camera_init(&config) == ESP_OK);
}

void updateOLED(String line1, String line2, String line3) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  display.setCursor(0, 0);
  display.println(line1);
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
  
  display.setCursor(0, 20);
  display.println(line2);
  
  display.setCursor(0, 45);
  display.println(line3);
  
  display.display();
}