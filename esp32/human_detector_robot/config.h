#pragma once

// ──────────────────────────────────────────────────────────────────────────────
//  ARK-5 Human Detector Robot — ESP32 DevKit Configuration
//  Robot: motors + MPU6050 + ultrasonic + WiFi STA (router) + SoftAP (sensing)
// ──────────────────────────────────────────────────────────────────────────────

// ── Robot Identity (1 or 2) ──────────────────────────────────────────────────
// Robot 1 flash karne ke liye 1 rakhein. Robot 2 flash karne ke liye 2 kar dein!
#define ROBOT_ID           1

// ── Router (STA mode) — laptop se commands/telemetry ─────────────────────────
#define STA_SSID           "unknown"
#define STA_PASSWORD       "muhammadasad"

// ── Network IP Configuration ──────────────────────────────────────────────────
// 0 = DHCP (Windows Hotspot ke liye 100% required), 1 = Static IP
#define USE_STATIC_IP      0
#if ROBOT_ID == 2
  #define STATIC_IP_ADDR   192, 168, 137, 21   // Robot 2 Brain (Fixed)
#else
  #define STATIC_IP_ADDR   192, 168, 137, 11   // Robot 1 Brain (Fixed)
#endif
#define GATEWAY_IP_ADDR    192, 168, 137, 1    // Laptop 2.4GHz Hotspot Gateway
#define SUBNET_MASK        255, 255, 255, 0
#define DNS_IP_ADDR        192, 168, 137, 1

// ── UDP ports & SoftAP (Auto-configured based on ROBOT_ID) ────────────────────
#if ROBOT_ID == 2
  #define UDP_COMMAND_PORT   4220
  #define UDP_TELEMETRY_PORT 4221
  #define SENSE_AP_SSID      "Robot_2"
#else
  #define UDP_COMMAND_PORT   4210
  #define UDP_TELEMETRY_PORT 4211
  #define SENSE_AP_SSID      "Robot_1"
#endif
#define SENSE_AP_PASSWORD  "muhammadasad"
#define SENSE_AP_CHANNEL   6
#define SENSE_AP_MAX_CONN  4

// ── L298N motor pins ─────────────────────────────────────────────────────────
#define PIN_ENA   25
#define PIN_IN1   26
#define PIN_IN2   27
#define PIN_ENB   33
#define PIN_IN3   32
#define PIN_IN4   14

// ── MPU6050 I2C ───────────────────────────────────────────────────────────────
#define PIN_SDA   21
#define PIN_SCL   22

// ── HC-SR04 x2 (VCC = 5V from L298N — NOT ESP32 3.3V pin!) ───────────────────
// Tum 2 sensors use kar rahe ho — in mein se 2 wire karo:
//   Setup A (recommended): FRONT + LEFT  → T par left seedha, right rotate se
//   Setup B:               LEFT + RIGHT   → T par dono seedhe, cruise ke liye robot samne dekhe
//
// #1 FRONT (samne)
#define PIN_US_F_TRIG  18
#define PIN_US_F_ECHO  19
// #2 LEFT (90° left)
#define PIN_US_L_TRIG  5   // DevKit D5
#define PIN_US_L_ECHO  15  // DevKit D15 (tumhari wiring)
// #3 RIGHT (90° right) — 2 sensors ho to wire mat karo, US_USE_RIGHT = 0
#define PIN_US_R_TRIG  16
#define PIN_US_R_ECHO  4
#define US_USE_RIGHT   0   // 1 = teen sensors, 0 = sirf FRONT + LEFT (recommended)

#define US_STOP_CM   100
#define US_MIN_CM    25
#define US_MAX_CM    400

// ── Robot geometry ────────────────────────────────────────────────────────────
#define WHEEL_BASE_M   0.20f
#define WHEEL_RADIUS_M 0.033f

// ── PWM ───────────────────────────────────────────────────────────────────────
#define PWM_FREQ_HZ    1000
#define PWM_RESOLUTION 8
#define PWM_CHANNEL_L  0
#define PWM_CHANNEL_R  1
#define MAX_PWM        200
#define MIN_PWM        60

// ── Safety & timing ───────────────────────────────────────────────────────────
#define CMD_TIMEOUT_MS        500
#define TELEMETRY_INTERVAL_MS 100
#define US_SAMPLE_INTERVAL_MS 80

// ── Heading PID ───────────────────────────────────────────────────────────────
#define HEADING_KP   2.0f
#define HEADING_KI   0.05f
#define IMU_ALPHA    0.98f
