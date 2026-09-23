/*
 * ARK-5 ESP32-CAM OV3660 — MJPEG Video Stream
 *
 * Connects to home router (STA mode).
 * Laptop opens: http://<cam-ip>:81/stream
 *
 * Flash via FTDI USB (GPIO0 → GND at reset for download mode).
 * Wiring: wiring/WIRING_GUIDE.txt Section 5
 */

#include "config.h"
#include "esp_camera.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include "esp_http_server.h"

httpd_handle_t streamHttpd = NULL;

// ── Camera init (OV3660) ──────────────────────────────────────────────────────

static bool initCamera() {
  camera_config_t cfg = {};
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer   = LEDC_TIMER_0;
  cfg.pin_d0       = Y2_GPIO_NUM;
  cfg.pin_d1       = Y3_GPIO_NUM;
  cfg.pin_d2       = Y4_GPIO_NUM;
  cfg.pin_d3       = Y5_GPIO_NUM;
  cfg.pin_d4       = Y6_GPIO_NUM;
  cfg.pin_d5       = Y7_GPIO_NUM;
  cfg.pin_d6       = Y8_GPIO_NUM;
  cfg.pin_d7       = Y9_GPIO_NUM;
  cfg.pin_xclk     = XCLK_GPIO_NUM;
  cfg.pin_pclk     = PCLK_GPIO_NUM;
  cfg.pin_vsync    = VSYNC_GPIO_NUM;
  cfg.pin_href     = HREF_GPIO_NUM;
  cfg.pin_sccb_sda = SIOD_GPIO_NUM;
  cfg.pin_sccb_scl = SIOC_GPIO_NUM;
  cfg.pin_pwdn     = PWDN_GPIO_NUM;
  cfg.pin_reset    = RESET_GPIO_NUM;
  cfg.xclk_freq_hz = 20000000;
  cfg.frame_size   = CAM_FRAME_SIZE;
  cfg.pixel_format = PIXFORMAT_JPEG;
  cfg.grab_mode    = CAMERA_GRAB_LATEST;
  cfg.fb_location  = CAMERA_FB_IN_PSRAM;
  cfg.jpeg_quality = CAM_JPEG_QUALITY;
  cfg.fb_count     = 2;

  esp_err_t err = esp_camera_init(&cfg);
  if (err != ESP_OK) {
    Serial.printf("[CAM] Init failed: 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_framesize(s, CAM_FRAME_SIZE);
    // OV3660 PID check — fallback settings
    if (s->id.PID == OV3660_PID) {
      Serial.println("[CAM] OV3660 detected");
    }
  }
  Serial.println("[CAM] Camera OK");
  return true;
}

// ── MJPEG stream handler ──────────────────────────────────────────────────────

static esp_err_t streamHandler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  char partBuf[128];

  res = httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=frame");
  if (res != ESP_OK) return res;

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("[CAM] Frame capture failed");
      res = ESP_FAIL;
      break;
    }

    snprintf(partBuf, sizeof(partBuf),
             "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
             fb->len);
    res = httpd_resp_send_chunk(req, partBuf, strlen(partBuf));
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, "\r\n", 2);

    esp_camera_fb_return(fb);
    if (res != ESP_OK) break;
    yield();
  }
  return res;
}

static void startStreamServer() {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port = STREAM_PORT;
  cfg.ctrl_port   = STREAM_PORT + 1;
  cfg.max_uri_handlers = 4;

  if (httpd_start(&streamHttpd, &cfg) == ESP_OK) {
    httpd_uri_t uri = {};
    uri.uri     = "/stream";
    uri.method  = HTTP_GET;
    uri.handler = streamHandler;
    uri.user_ctx = NULL;
    httpd_register_uri_handler(streamHttpd, &uri);
    Serial.printf("[HTTP] Stream ready: http://%s:%d/stream\n",
                  WiFi.localIP().toString().c_str(), STREAM_PORT);
  } else {
    Serial.println("[HTTP] Stream server start FAILED");
  }
}

// ── WiFi STA ──────────────────────────────────────────────────────────────────

static void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

#if USE_STATIC_IP
  IPAddress local_IP(STATIC_IP_ADDR);
  IPAddress gateway(GATEWAY_IP_ADDR);
  IPAddress subnet(SUBNET_MASK);
  IPAddress dns(DNS_IP_ADDR);
  if (!WiFi.config(local_IP, gateway, subnet, dns)) {
    Serial.println("[WiFi] Static IP config failed, using DHCP fallback.");
  } else {
    Serial.print("[WiFi] Static IP set to: ");
    Serial.println(local_IP);
  }
#endif

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("[WiFi] Connecting to '%s' ", WIFI_SSID);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[WiFi] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
#ifdef MDNS_HOSTNAME
    if (MDNS.begin(MDNS_HOSTNAME)) {
      MDNS.addService("http", "tcp", STREAM_PORT);
      Serial.printf("[mDNS] Stream URL: http://%s.local:%d/stream\n", MDNS_HOSTNAME, STREAM_PORT);
    }
#endif
    Serial.printf("[HTTP] Stream ready: http://%s:%d/stream\n", WiFi.localIP().toString().c_str(), STREAM_PORT);
  } else {
    Serial.println("\n[WiFi] FAILED — WIFI_SSID/WIFI_PASSWORD config.h mein check karo.");
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n╔══════════════════════════════════════╗");
  Serial.println("║  ARK-5 ESP32-CAM OV3660 Stream       ║");
  Serial.println("╚══════════════════════════════════════╝");

  if (!initCamera()) {
    Serial.println("[FATAL] Camera init failed — check power (stable 5V).");
    return;
  }
  setupWiFi();
  startStreamServer();
  Serial.println("[READY] MJPEG stream active.");
}

void loop() {
  delay(1000);
  // Stream runs in HTTP handler task
}
