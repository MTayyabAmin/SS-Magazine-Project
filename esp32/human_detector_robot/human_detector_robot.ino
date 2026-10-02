/*
 * ARK-5 Human Detector Robot — ESP32 DevKit Controller
 *
 * Features:
 *   - L298N dual motor drive
 *   - MPU6050 yaw telemetry
 *   - HC-SR04 wall detection (stop @ 1m)
 *   - WiFi STA → home router (UDP commands/telemetry to laptop)
 *   - WiFi SoftAP "HumanSense" → human sensing hotspot
 *
 * Flash: esp32/robot_controller/human_detector_robot.ino
 * Wiring: wiring/WIRING_GUIDE.txt
 */

#include "config.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <Wire.h>

WiFiUDP udpCmd;
WiFiUDP udpTelem;

volatile float        cmdV          = 0.0f;
volatile float        cmdOmega      = 0.0f;
volatile float        cmdTheta      = 0.0f;
volatile bool         useAbsHeading = false;
volatile unsigned long lastCmdMs    = 0;
volatile bool         cmdAllowCreep = false;
volatile uint32_t     cmdSeq        = 0;

float         yawRad       = 0.0f;
float         headingInteg = 0.0f;
unsigned long lastImuUs    = 0;

// FIX (yaw corruption): gyro-only yaw + stationary bias estimator state.
// See the YAW_BIAS_* block in config.h for why accel heading was dropped.
float         gyroBiasDps  = 0.0f;   // current zero-rate bias estimate (dps)
float         biasAccumDps = 0.0f;   // bootstrap: sum of stationary samples
uint16_t      biasSamples  = 0;      // bootstrap: sample count
unsigned long biasStartMs  = 0;      // when the bootstrap window began (0 = not started)
bool          biasReady    = false;  // bootstrap mean computed at least once

int           distFrontCm  = -1;
int           distLeftCm   = -1;
int           distRightCm  = -1;
int           senseRssi    = 0;
unsigned long lastUsMs       = 0;

// Signed PWM duty last written to each H-bridge channel (set by setMotor(),
// reported on the 1 Hz serial line and in the pwm_l/pwm_r telemetry keys).
// Debug aid: if both show 200 but only one side turns, the fault is wiring/
// L298N, not firmware.
int           pwmLeftApplied  = 0;
int           pwmRightApplied = 0;

static const uint8_t MPU_ADDR    = 0x68;
static const uint8_t REG_PWR     = 0x6B;
static const uint8_t REG_ACCEL_X = 0x3B;
static const uint8_t REG_GYRO_X  = 0x43;

// ── MPU6050 ───────────────────────────────────────────────────────────────────

static void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static int16_t mpuRead16(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)2, (uint8_t)true);
  return (Wire.read() << 8) | Wire.read();
}

static bool mpuInit() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  mpuWrite(REG_PWR, 0x00);
  delay(100);
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("[IMU] MPU6050 NOT found — check wiring!");
    return false;
  }
  Serial.println("[IMU] MPU6050 OK");
  return true;
}

// FIX (yaw corruption, 2026-09): gyro-only integration + stationary bias
// estimator. Accelerometer heading atan2f(-ay, ax) bilkul drop kar diya gaya
// hai — wo tilt par depend karta tha aur yaw ko seconds mein 100°+ ulta kar
// deta tha (see YAW_BIAS_* in config.h).
static void updateYaw() {
  unsigned long nowUs = micros();
  if (lastImuUs == 0) { lastImuUs = nowUs; return; }
  float dt = (nowUs - lastImuUs) / 1e6f;
  lastImuUs = nowUs;
  if (dt <= 0.0f || dt > 0.5f) return;

  int16_t gzRaw = mpuRead16(REG_GYRO_X + 4);
  float gzDps = (float)gzRaw / 131.0f;

  // Stationary gate: jab tak robot ko translate/rotate nahi kaha ja raha,
  // gyro ka output sirf zero-rate offset (bias) hona chahiye — is window ka
  // use karke bias estimate karte hain.
  bool stationary = (fabsf(cmdV) <= YAW_BIAS_STATIONARY_V) &&
                    (fabsf(cmdOmega) <= YAW_BIAS_STATIONARY_W);

  if (stationary) {
    if (!biasReady) {
      if (biasStartMs == 0) biasStartMs = millis();
      biasAccumDps += gzDps;
      biasSamples++;
      if ((millis() - biasStartMs) >= YAW_BIAS_BOOTSTRAP_MS) {
        gyroBiasDps = biasAccumDps / (float)biasSamples;
        biasReady   = true;
      }
    } else {
      // Bahut dheere drift track karte hain (har sample par 0.001).
      gyroBiasDps += YAW_BIAS_TRACK_ALPHA * (gzDps - gyroBiasDps);
    }
  } else if (!biasReady) {
    // Stationary window toot gayi bootstrap ke beech — dobara se start karo
    // taake purane (jagah/pose ke) samples ka mean na ban jaaye.
    biasStartMs  = 0;
    biasAccumDps = 0.0f;
    biasSamples  = 0;
  }

  yawRad += (gzDps - gyroBiasDps) * DEG_TO_RAD * dt;
  while (yawRad >  (float)M_PI) yawRad -= 2.0f * (float)M_PI;
  while (yawRad < -(float)M_PI) yawRad += 2.0f * (float)M_PI;
}

// ── HC-SR04 x2 (VCC=5V, TRIG=GPIO 3.3V OK, ECHO needs divider — see wiring guide) ──

static int readUltrasonicCm(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration <= 0) return -1;
  int cm = (int)(duration * 0.0343f / 2.0f);
  if (cm < 2 || cm > US_MAX_CM) return -1;
  return cm;
}

static void updateUltrasonic() {
  unsigned long now = millis();
  if (now - lastUsMs < US_SAMPLE_INTERVAL_MS) return;
  lastUsMs = now;

  // Sensor roles (FIX in config.h): FRONT = GPIO5/15, RIGHT = GPIO18/19.
  // Left sensor nahi laga → US_USE_LEFT 0 → distLeftCm hamesha -1 rahega
  // (Python corridor centering ko wahi -1 band kar deta hai).
  distFrontCm = readUltrasonicCm(PIN_US_F_TRIG, PIN_US_F_ECHO);
#if US_USE_LEFT
  delayMicroseconds(500);
  distLeftCm  = readUltrasonicCm(PIN_US_L_TRIG, PIN_US_L_ECHO);
#else
  distLeftCm  = -1;
#endif
#if US_USE_RIGHT
  delayMicroseconds(500);
  distRightCm = readUltrasonicCm(PIN_US_R_TRIG, PIN_US_R_ECHO);
#else
  distRightCm = -1;
#endif
}

// ── Motors (LEDC PWM Compatibility for Core 2.x & 3.x) ────────────────────────

static inline void pwmSetup(uint8_t pin, uint8_t channel) {
#if defined(ESP_ARDUINO_VERSION) && ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcAttach(pin, PWM_FREQ_HZ, PWM_RESOLUTION);
#else
  ledcSetup(channel, PWM_FREQ_HZ, PWM_RESOLUTION);
  ledcAttachPin(pin, channel);
#endif
}

static inline void pwmWrite(uint8_t pin, uint8_t channel, uint32_t val) {
#if defined(ESP_ARDUINO_VERSION) && ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcWrite(pin, val);
#else
  ledcWrite(channel, val);
#endif
}

// Writes one L298N channel and RETURNS the signed duty actually written
// (after constrain) so drive() can report it as pwm_l/pwm_r telemetry.
static int setMotor(uint8_t pinEn, uint8_t channel, uint8_t in1, uint8_t in2, int speed) {
  speed = constrain(speed, -MAX_PWM, MAX_PWM);
  if (speed > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
    pwmWrite(pinEn, channel, (uint32_t)speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);
    pwmWrite(pinEn, channel, (uint32_t)(-speed));
  } else {
    digitalWrite(in1, LOW);  digitalWrite(in2, LOW);
    pwmWrite(pinEn, channel, 0);
  }
  return speed;
}

static void applyDeadzone(int &speed) {
  if (speed == 0) return;
  int sign = (speed > 0) ? 1 : -1;
  int mag = abs(speed);
  if (mag < MIN_PWM) mag = MIN_PWM;
  speed = sign * mag;
}

// Mixes commanded linear + angular velocity into per-side PWM duty.
//
// MAX_LINEAR_V_MS is the full-scale speed: v == MAX_LINEAR_V_MS saturates a
// side to MAX_PWM. It used to be 0.5f, but the laptop speed-floor policy
// (project.yaml: every non-zero linear speed >= 0.8 m/s) made that fatal —
// 0.8/0.5*200 = 320, so BOTH sides clamped to MAX_PWM and omega could no
// longer differentiate them: every moving command drove straight, only
// v=0 in-place turns could still steer.
// At 1.0f, v=0.8 gives duty 160/200 and omega keeps its share of the mixer.
//
// TURNS (PIVOT_TURNS, config.h): any omega != 0 stops ONE side and runs the
// other at PIVOT_TURN_V_MS (1.0f = full scale = MAX_PWM) instead of the
// classic differential — a turn is always "0 on one side, max speed on the
// other", so it can never stall in the deadzone. v and omega only choose the
// direction:
//     omega > 0 (CCW), v >= 0 : left 0, right +side
//     omega > 0 (CCW), v <  0 : left -side, right 0
//     omega < 0 (CW),  v >= 0 : left +side, right 0
//     omega < 0 (CW),  v <  0 : left 0, right -side
// v == 0 (scan / recovery rotations) defaults to the forward pair, so a
// pivot always translates along an arc of radius WHEEL_BASE_M/2 — the
// laptop keeps commanding v=0 there, so dead-reckoning will NOT integrate
// that drift (see tentative_map.update_pose).
static void drive(float v, float omega) {
  const float MAX_LINEAR_V_MS = 1.0f;
  float vLeft  = v - omega * (WHEEL_BASE_M / 2.0f);
  float vRight = v + omega * (WHEEL_BASE_M / 2.0f);

#if PIVOT_TURNS
  if (omega != 0.0f) {
    // Full-power pivot: moving side pinned to PIVOT_TURN_V_MS regardless of
    // |v| / |omega| (PIVOT_TURN_V_MS == MAX_LINEAR_V_MS → duty MAX_PWM).
    const float side = PIVOT_TURN_V_MS;
    bool move_forward = (v >= 0.0f);
    if (omega > 0.0f) {            // CCW / left turn
      vLeft  = move_forward ? 0.0f : -side;
      vRight = move_forward ? side  : 0.0f;
    } else {                        // CW / right turn
      vLeft  = move_forward ? side  : 0.0f;
      vRight = move_forward ? 0.0f  : -side;
    }
  }
#endif

  int pwmLeft  = (int)(vLeft  / MAX_LINEAR_V_MS * (float)MAX_PWM);
  int pwmRight = (int)(vRight / MAX_LINEAR_V_MS * (float)MAX_PWM);
  applyDeadzone(pwmLeft);
  applyDeadzone(pwmRight);
  pwmLeftApplied  = setMotor(PIN_ENA, PWM_CHANNEL_L, PIN_IN1, PIN_IN2, pwmLeft);
  pwmRightApplied = setMotor(PIN_ENB, PWM_CHANNEL_R, PIN_IN3, PIN_IN4, pwmRight);
}

static void stopMotors() {
  pwmLeftApplied  = setMotor(PIN_ENA, PWM_CHANNEL_L, PIN_IN1, PIN_IN2, 0);
  pwmRightApplied = setMotor(PIN_ENB, PWM_CHANNEL_R, PIN_IN3, PIN_IN4, 0);
}

// ── WiFi sensing RSSI (Passive human RF attenuation sensing) ──────────────────

static void updateSenseRssi() {
  static unsigned long lastCheckMs = 0;
  unsigned long now = millis();
  if (now - lastCheckMs < 1000) return;
  lastCheckMs = now;

  // Passive RSSI tracking: human body absorbs 2.4 GHz signal between robot and laptop
  // 0ms delay, zero channel hopping (does NOT drop WiFi)
  if (WiFi.status() == WL_CONNECTED) {
    senseRssi = WiFi.RSSI();
  } else {
    senseRssi = -100;
  }
}

// ── UDP ───────────────────────────────────────────────────────────────────────

static void handleCommand(const char *payload, int len) {
  JsonDocument doc;
  if (deserializeJson(doc, payload, len)) return;

  cmdV     = doc["v"]     | 0.0f;
  cmdOmega = doc["omega"] | 0.0f;
  if (doc.containsKey("theta")) {
    cmdTheta      = doc["theta"].as<float>();
    useAbsHeading = true;
  } else {
    useAbsHeading = false;
  }
  cmdSeq    = doc["seq"] | cmdSeq;
  cmdAllowCreep = doc["allow_creep"] | false;
  lastCmdMs = millis();
}

static void pollCommands() {
  int packetSize = udpCmd.parsePacket();
  if (packetSize > 0) {
    char buf[256];
    int len = udpCmd.read(buf, sizeof(buf) - 1);
    if (len > 0) {
      buf[len] = '\0';
      handleCommand(buf, len);
    }
  }
}

static void sendTelemetry(IPAddress remoteIP, uint16_t remotePort) {
  JsonDocument doc;
  doc["robot_id"]  = ROBOT_ID;
  doc["yaw"]       = yawRad * RAD_TO_DEG;
  doc["v"]         = cmdV;
  doc["omega"]     = cmdOmega;
  doc["seq"]       = cmdSeq;
  doc["dist_cm"]      = distFrontCm;   // backward compatible = front
  doc["dist_front_cm"]= distFrontCm;
  doc["dist_left_cm"] = distLeftCm;
  doc["dist_right_cm"]= distRightCm;
  doc["sense_rssi"]   = senseRssi;
  doc["wall_near"]    = (distFrontCm > 0 && distFrontCm <= US_STOP_CM);
  // Signed duty actually written to each H-bridge channel — lets the laptop
  // compare "what we commanded" against "what the driver got".
  doc["pwm_l"]        = pwmLeftApplied;
  doc["pwm_r"]        = pwmRightApplied;

  udpTelem.beginPacket(remoteIP, remotePort);
  serializeJson(doc, udpTelem);
  udpTelem.println();
  udpTelem.endPacket();
}

// ── WiFi STA setup (Connects to existing Hotspot/Router like ESP32-CAM) ───────

static void setupWiFi() {
  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_STA);   // Pure Station mode — apna wifi on nahi karega, sirf connect hoga
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

  // STA to laptop hotspot (same as ESP32-CAM)
  WiFi.begin(STA_SSID, STA_PASSWORD);
  Serial.printf("[WiFi] Connecting to '%s' ", STA_SSID);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    WiFi.begin(STA_SSID, STA_PASSWORD);
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[WiFi] Connected to '%s'! IP: %s\n", STA_SSID, WiFi.localIP().toString().c_str());
    Serial.printf("[WiFi] Fixed UDP Port: Command=%d, Telemetry=%d\n", UDP_COMMAND_PORT, UDP_TELEMETRY_PORT);
  } else {
    Serial.println("\n[WiFi] Connection FAILED — check STA_SSID/STA_PASSWORD in config.h.");
  }
}

// ── Setup / Loop ──────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n╔══════════════════════════════════════╗");
  Serial.println("║  ARK-5 Human Detector Robot          ║");
  Serial.println("║  ESP32 DevKit Controller             ║");
  Serial.println("╚══════════════════════════════════════╝");

  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  // Sensor pin modes follow the same US_USE_* guards as updateUltrasonic()
  // (left slot reserved/unused by default — see config.h FIX note).
  pinMode(PIN_US_F_TRIG, OUTPUT);
  pinMode(PIN_US_F_ECHO, INPUT);
#if US_USE_LEFT
  pinMode(PIN_US_L_TRIG, OUTPUT);
  pinMode(PIN_US_L_ECHO, INPUT);
#endif
#if US_USE_RIGHT
  pinMode(PIN_US_R_TRIG, OUTPUT);
  pinMode(PIN_US_R_ECHO, INPUT);
#endif

  pwmSetup(PIN_ENA, PWM_CHANNEL_L);
  pwmSetup(PIN_ENB, PWM_CHANNEL_R);
  stopMotors();

  mpuInit();
  setupWiFi();

  udpCmd.begin(UDP_COMMAND_PORT);
  udpTelem.begin(UDP_TELEMETRY_PORT);
  Serial.printf("[UDP]  Command port  : %d\n", UDP_COMMAND_PORT);
  Serial.printf("[UDP]  Telemetry port: %d\n", UDP_TELEMETRY_PORT);

  lastCmdMs = millis();
  lastImuUs = micros();
#if DEBUG_STRAIGHT
  // Loud banner: wheels start spinning the moment power is applied.
  Serial.println("[DEBUG_STRAIGHT] ON — driving ALL four wheels straight at "
                 "DEBUG_STRAIGHT_V, ignoring UDP/wall/timeout. Type 's' to stop.");
#endif
  Serial.println("[READY] Waiting for laptop controller...");
}

void loop() {
  updateYaw();
  updateUltrasonic();
  updateSenseRssi();
  pollCommands();

  unsigned long now = millis();

#if DEBUG_STRAIGHT
  // ── DEBUG: force straight drive — ALL four wheels, no conditions ──────────
  // Bypasses UDP cmd, heading PID, CMD_TIMEOUT, tooClose, wallNear and the
  // allow_creep gate (see config.h). omega = 0 → vLeft == vRight → both
  // H-bridge channels get identical duty.
  // 's' in the serial monitor = emergency stop (stays off until reset).
  static bool debugStopped = false;
  if (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == 's' || c == 'S') {
      debugStopped = true;
      stopMotors();
      Serial.println("[DEBUG_STRAIGHT] 's' received — MOTORS STOPPED (reset to resume).");
    }
  }
  if (!debugStopped) {
    drive(DEBUG_STRAIGHT_V, 0.0f);
  }
#else
  bool wallNear = (distFrontCm > 0 && distFrontCm <= US_STOP_CM);
  bool tooClose = (distFrontCm > 0 && distFrontCm < US_MIN_CM);

  if ((now - lastCmdMs) > CMD_TIMEOUT_MS) {
    stopMotors();
    headingInteg = 0.0f;
  } else if (tooClose && cmdV > 0) {
    // Hard safety — never drive forward into wall closer than 25cm
    stopMotors();
  } else if (wallNear && !cmdAllowCreep) {
    stopMotors();
  } else {
    float v     = cmdV;
    float omega = cmdOmega;

    if (useAbsHeading) {
      float err = cmdTheta - yawRad;
      while (err >  (float)M_PI) err -= 2.0f * (float)M_PI;
      while (err < -(float)M_PI) err += 2.0f * (float)M_PI;
      headingInteg += err * HEADING_KI * 0.01f;
      headingInteg  = constrain(headingInteg, -1.0f, 1.0f);
      omega += HEADING_KP * err + headingInteg;
    }
    drive(v, omega);
  }
#endif

  static unsigned long lastTelemMs = 0;
  if ((now - lastTelemMs) >= TELEMETRY_INTERVAL_MS) {
    lastTelemMs = now;
    IPAddress targetIP = udpCmd.remoteIP();
    if (targetIP == IPAddress(0, 0, 0, 0)) {
      targetIP = IPAddress(GATEWAY_IP_ADDR); // Laptop hotspot IP (192.168.137.1)
    }
    // Send telemetry to configured UDP_TELEMETRY_PORT (4211 / 4221)
    sendTelemetry(targetIP, UDP_TELEMETRY_PORT);
  }

  // Periodic clean serial monitor status (1 Hz) for direct hardware monitoring
  static unsigned long lastSerialMs = 0;
  if ((now - lastSerialMs) >= 1000) {
    lastSerialMs = now;
    // mState is derived from the APPLIED duty (not from cmdV/cmdOmega) so it
    // reflects what the L298N channels actually received — in DEBUG_STRAIGHT
    // mode cmdV/cmdOmega still hold the last laptop packet and would lie.
    int dutyL = abs(pwmLeftApplied), dutyR = abs(pwmRightApplied);
    const char* mState;
    if (dutyL == 0 && dutyR == 0)      mState = "STOP";
    else if (pwmLeftApplied > 0 && pwmRightApplied > 0) mState = "FWD";
    else if (pwmLeftApplied < 0 && pwmRightApplied < 0) mState = "REV";
    else                               mState = "TURN";
    // Sensor roles: FRONT + RIGHT (config.h FIX) — left slot is reserved and
    // would always print -1, so it is not shown.
    // "PWM L/R" is the asymmetry detector: both lines must be equal for a
    // straight drive; if L=200 but the left wheels don't spin, it's wiring.
    Serial.printf("[ROBOT #%d] Motors: %-4s (v=%.2f, w=%.2f) | PWM L=%3d R=%3d | Sonar Front: %3dcm, Right: %3dcm | MPU Yaw: %+5.1f deg | Bias: %+5.2fdps%s\n",
                  ROBOT_ID, mState, cmdV, cmdOmega, pwmLeftApplied, pwmRightApplied,
                  distFrontCm, distRightCm,
                  yawRad * RAD_TO_DEG, gyroBiasDps, biasReady ? "" : " (boot)");
  }

  yield();
}
