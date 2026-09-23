/*
 * ARK-5 ESP32-CAM (OV2640 / OV3660 / OV7670) — MJPEG Video Stream
 *
 * Connects to home router (STA mode).
 * Laptop opens: http://<cam-ip>:81/stream
 *
 * Auto-handles:
 *  - Hardware JPEG sensors (OV2640, OV3660, OV5640)
 *  - Non-JPEG sensors (OV7670 etc.) via software JPEG conversion (frame2jpg)
 */

#include "config.h"
#include "esp_camera.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include "esp_http_server.h"

httpd_handle_t streamHttpd = NULL;
static bool s_software_jpeg = false;

// ── Camera init ──────────────────────────────────────────────────────────────

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
  cfg.xclk_freq_hz = 16500000; // 16.5 MHz — OV2640 ke liye bohot stable
  cfg.frame_size   = CAM_FRAME_SIZE;
  cfg.pixel_format = PIXFORMAT_JPEG;
  cfg.grab_mode    = CAMERA_GRAB_LATEST;
  cfg.fb_location  = CAMERA_FB_IN_PSRAM;
  cfg.jpeg_quality = CAM_JPEG_QUALITY;
  cfg.fb_count     = 2;

  // 1st Attempt: Hardware JPEG
  esp_err_t err = esp_camera_init(&cfg);

  // 2nd Attempt: Agar JPEG format support nahi karta (Error 0x106), RGB565 + Software JPEG use karo
  if (err == ESP_ERR_NOT_SUPPORTED || err == 0x106) {
    Serial.println("[CAM] Hardware JPEG not supported on this sensor. Trying RGB mode + Software JPEG...");
    esp_camera_deinit();
    delay(100);
    cfg.pixel_format = PIXFORMAT_RGB565;
    cfg.fb_count     = 1;
    s_software_jpeg  = true;
    err = esp_camera_init(&cfg);
  }

  // 3rd Attempt: Lower XCLK clock
  if (err != ESP_OK && !s_software_jpeg) {
    Serial.println("[CAM] Retrying with 10MHz clock...");
    esp_camera_deinit();
    delay(100);
    cfg.xclk_freq_hz = 10000000;
    err = esp_camera_init(&cfg);
  }

  if (err != ESP_OK) {
    Serial.printf("[CAM] Init FAILED: 0x%x\n", err);
    Serial.println("  --> Check camera ribbon cable (is it inserted firmly and straight?)");
    Serial.println("  --> Ensure 5V power is stable.");
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_framesize(s, CAM_FRAME_SIZE);
    Serial.printf("[CAM] Detected Sensor PID: 0x%02X\n", s->id.PID);
    if (s->id.PID == OV2640_PID) {
      Serial.println("[CAM] Sensor type: OV2640 (Hardware JPEG OK)");
    } else if (s->id.PID == OV3660_PID) {
      Serial.println("[CAM] Sensor type: OV3660 (Hardware JPEG OK)");
    } else {
      Serial.println("[CAM] Sensor type: Non-standard/OV7670 (Using Software JPEG)");
    }
    s->set_vflip(s, 1);
    s->set_hmirror(s, 0);

    // ── Image Quality & Clarity Enhancements (OV2640 ISP) ──
    s->set_brightness(s, 1);     // -2 to 2 (bright & clear)
    s->set_contrast(s, 1);       // -2 to 2 (crisp edges)
    s->set_saturation(s, 0);     // -2 to 2 (natural colors)
    s->set_whitebal(s, 1);       // Auto White Balance ON
    s->set_awb_gain(s, 1);       // AWB Gain ON
    s->set_wb_mode(s, 0);        // Auto WB
    s->set_exposure_ctrl(s, 1);  // Auto Exposure ON
    s->set_aec2(s, 1);           // Advanced AEC (dynamic light adaptation)
    s->set_gain_ctrl(s, 1);      // Auto Gain ON
    s->set_gainceiling(s, (gainceiling_t)2); // Reduces grain/noise in dim light
    s->set_bpc(s, 1);            // Black Pixel Correction ON (clears dark sensor noise)
    s->set_wpc(s, 1);            // White Pixel Correction ON (clears bright specks)
    s->set_raw_gma(s, 1);        // Gamma Correction ON (richer tone)
    s->set_lenc(s, 1);           // Lens Shading Correction ON (removes dark corners)
    s->set_dcw(s, 1);            // Advanced Denoise ON
  }
  Serial.println("[CAM] Camera OK (Enhanced Quality Active)");
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

    uint8_t *jpg_buf = NULL;
    size_t jpg_len = 0;
    bool need_free = false;

    if (fb->format == PIXFORMAT_JPEG) {
      jpg_buf = fb->buf;
      jpg_len = fb->len;
    } else {
      // Software JPEG conversion for RGB/YUV sensors
      bool ok = frame2jpg(fb, CAM_JPEG_QUALITY, &jpg_buf, &jpg_len);
      need_free = true;
      if (!ok) {
        Serial.println("[CAM] JPEG conversion failed");
        esp_camera_fb_return(fb);
        res = ESP_FAIL;
        break;
      }
    }

    snprintf(partBuf, sizeof(partBuf),
             "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
             (unsigned int)jpg_len);
    res = httpd_resp_send_chunk(req, partBuf, strlen(partBuf));
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)jpg_buf, jpg_len);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, "\r\n", 2);

    if (need_free && jpg_buf) {
      free(jpg_buf);
    }
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
    Serial.println("\n[WiFi] FAILED — Check 2.4GHz hotspot & config.h credentials.");
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n╔══════════════════════════════════════╗");
  Serial.println("║  ARK-5 ESP32-CAM Stream (Dual-Mode)  ║");
  Serial.println("╚══════════════════════════════════════╝");

  if (!initCamera()) {
    Serial.println("[FATAL] Camera init failed.");
    return;
  }
  setupWiFi();
  startStreamServer();
  Serial.println("[READY] MJPEG stream active.");
}

void loop() {
  delay(1000);
}
