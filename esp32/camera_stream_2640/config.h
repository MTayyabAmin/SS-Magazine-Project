#pragma once

// ──────────────────────────────────────────────────────────────────────────────
//  ARK-5 ESP32-CAM RHYX-M21-45 (OV2640 sensor) — MJPEG Stream Configuration
//  Camera sirf video stream karta hai. CV laptop par chalti hai.
//  NOTE: OV2640 max resolution UXGA (1600x1200) hai — OV3660 (QXGA) se kam,
//  lekin VGA/SVGA/QVGA jaise streaming sizes dono sensors support karte hain.
// ──────────────────────────────────────────────────────────────────────────────

// Router credentials (same network as laptop + robot ESP32)
#define WIFI_SSID "unknown"
#define WIFI_PASSWORD "muhammadasad"

// ── Fixed Static IP (Har dafa exact same IP rahega) ──────────────────────────
// 1 = Fixed IP (Recommended), 0 = DHCP (har restart par random IP)
#define USE_STATIC_IP      1
#define STATIC_IP_ADDR     192, 168, 137, 20   // Robot 2 Camera (Fixed)
#define GATEWAY_IP_ADDR    192, 168, 137, 1    // Laptop 2.4GHz Hotspot Gateway
#define SUBNET_MASK        255, 255, 255, 0
#define DNS_IP_ADDR        192, 168, 137, 1

// mDNS hostname -> Browser ya Python mein: http://robot2-cam.local:81/stream
#define MDNS_HOSTNAME      "robot2-cam"

// MJPEG HTTP stream port
#define STREAM_PORT 81

// Camera frame size (smaller = lower latency over WiFi)
// Options: FRAMESIZE_QVGA (320x240), FRAMESIZE_VGA (640x480), FRAMESIZE_SVGA (800x600)
// FIX: heat issue — VGA continuous encoding se ISP/encoder garam ho raha
// tha. QVGA + thodi zyada compression se encoding load kaafi kam hota
// hai. Ye setting OV3660 aur OV2640 (RHYX-M21-45) dono ke liye same
// generic hai — sensor-specific nahi.
#define CAM_FRAME_SIZE FRAMESIZE_VGA
#define CAM_JPEG_QUALITY 8 // 0-63, lower = crisper quality (10 = sharp HD)
#define CAM_FPS_TARGET 15

// OV3660 on ESP32-CAM (AI-Thinker compatible pinout)
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
