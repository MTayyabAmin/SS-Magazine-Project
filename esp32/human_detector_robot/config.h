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
// FIX (sensor role mapping, 2026-09) — FIRMWARE LEVEL ONLY. Koi wire nahi
// badla, physical wiring aur wiring/WIRING_GUIDE.txt bilkul same hain —
// sirf firmware ke role-labels galat the. Aapke 2 sensors PHYSICALLY:
//   GPIO 5 / GPIO15 par laga sensor  → FRONT (samne)
//   GPIO18 / GPIO19 par laga sensor  → RIGHT (90° right)
//   GPIO16 / GPIO4                   → LEFT slot, koi sensor nahi
// Pehle firmware ne 18/19 ko FRONT aur 5/15 ko LEFT mana tha, is liye
// "front" sonar asal mein side par tha aur "left" sonar samne dekh raha
// tha — wall-stop, corridor centering, map aur A* sab galat frame par bane
// the.
//
// NOTE: wiring/WIRING_GUIDE.txt ke role-labels (FRONT = 18/19, LEFT = 5/15)
// is fix ke liye chhue nahi gaye — guide wiring document karta hai, role
// (kaun sa pin kis taraf dekhta hai) yeh config.h define karta hai. Role
// ke liye yeh file authoritative hai.
//
// Python side dist_left_cm = -1 aane par corridor centering khud hi band
// ho jaati hai (dono readings > 0 chahiye) — yeh theek hai.
//
// #1 FRONT (samne)
#define PIN_US_F_TRIG  5   // DevKit D5
#define PIN_US_F_ECHO  15  // DevKit D15 (tumhari wiring)
// #2 RIGHT (90° right) — yeh 2 sensors mein se doosra sensor hai
#define PIN_US_R_TRIG  18
#define PIN_US_R_ECHO  19
// #3 LEFT (90° left) — reserved, abhi koi sensor nahi
#define PIN_US_L_TRIG  16
#define PIN_US_L_ECHO  4
#define US_USE_LEFT    0   // 0 = koi left sensor nahi (dist_left_cm = -1)
#define US_USE_RIGHT   1   // 1 = FRONT + RIGHT par 2 sensors lage hain

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

// ── Yaw: gyro-only integration + stationary gyro-bias estimator ───────────────
// FIX (yaw corruption, 2026-09): pehle yaw = IMU_ALPHA * gyro + (1-ALPHA) *
// atan2f(-ay, ax) hota tha. Accel heading ek TILT quantity hai — robot ke
// accelerate/hone, ramp ya bump par wo 180° tak ulta ho jaata tha aur 0.02
// weight ke bawajood yaw ko seconds mein 100°+ hila deta tha (log mein
// continuous sonar sweep dekh kar bhi yaw 160° → -175° ho gaya). Ab yaw
// sirf gyro se integrate hota hai; gyro ka zero-rate offset ek "stationary
// gate" ke saath estimate kiya jaata hai:
//   * boot par robot stationary hota hai → pehle YAW_BIAS_BOOTSTRAP_MS
//     samples ka mean = initial bias,
//   * phir jab bhi command zero ho (nahi chal raha, ghoom nahi raha) bahut
//     dheere (YAW_BIAS_TRACK_ALPHA) bias ko track karte hain.
#define YAW_BIAS_BOOTSTRAP_MS   1000    // stationary ms for the first bias
#define YAW_BIAS_TRACK_ALPHA    0.001f  // slow bias correction per sample
#define YAW_BIAS_STATIONARY_V   0.01f   // |cmdV| below this = not translating
#define YAW_BIAS_STATIONARY_W   0.05f   // |cmdOmega| below this = not turning
