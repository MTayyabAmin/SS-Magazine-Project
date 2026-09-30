/*
 * ARK-5 Step 1 — MPU6050 verify (Serial Monitor)
 * Wiring: GPIO21=SDA, GPIO22=SCL, 3.3V, GND
 * Flash: esp32/tests/step1_mpu6050_verify.ino
 *
 * Yaw follows the SAME method as the main firmware
 * (human_detector_robot/human_detector_robot.ino): gyro-only integration
 * with a stationary gyro-bias calibration. Accelerometer heading is
 * deliberately NOT blended in — it is a tilt quantity and used to flip the
 * yaw by 100°+ whenever the board was bumped (see YAW_BIAS_* notes in
 * esp32/human_detector_robot/config.h).
 *
 * Keep the board still for the first second so the bias can calibrate
 * (printed as "[CALIBRATING...]" then "[OK]").
 */
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define MPU_ADDR 0x68
#define YAW_BIAS_BOOTSTRAP_MS 1000   // hold still this long for calibration

unsigned long lastPrintTime = 0;
float yawRad = 0.0f;
float gyroBiasDps = 0.0f;   // zero-rate bias estimate (dps)
float biasAccum = 0.0f;
int biasSamples = 0;
unsigned long biasStart = 0;
bool biasReady = false;

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n╔══════════════════════════════════════╗");
  Serial.println("║  ARK-5 Step 1 — MPU6050 Verify       ║");
  Serial.println("╚══════════════════════════════════════╝");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  mpuWrite(0x6B, 0x00);
  delay(100);

  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("[FAIL] MPU6050 not found at 0x68 — check wiring!");
    while (true) delay(1000);
  }
  Serial.println("[OK] MPU6050 found. KEEP STILL for 1s (gyro bias cal), then rotate board — yaw should change.\n");
  Serial.println("Sample | ax     ay     az     | gx     gy     gz     | Yaw(deg)  Bias(dps)");
  Serial.println("-------|------------------------|------------------------|-------------------");
}

void loop() {
  if (millis() - lastPrintTime < 200) return;
  float dt = (millis() - lastPrintTime) / 1000.0f;
  lastPrintTime = millis();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint16_t)MPU_ADDR, (uint8_t)14, true);

  int16_t axr = (Wire.read() << 8) | Wire.read();
  int16_t ayr = (Wire.read() << 8) | Wire.read();
  int16_t azr = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  int16_t gxr = (Wire.read() << 8) | Wire.read();
  int16_t gyr = (Wire.read() << 8) | Wire.read();
  int16_t gzr = (Wire.read() << 8) | Wire.read();

  float ax = axr / 16384.0f, ay = ayr / 16384.0f, az = azr / 16384.0f;
  float gx = gxr / 131.0f, gy = gyr / 131.0f, gz = gzr / 131.0f;

  if (!biasReady) {
    // First second while the board is still → mean gyro rate = bias.
    if (biasStart == 0) biasStart = millis();
    biasAccum += gz;
    biasSamples++;
    if (millis() - biasStart >= YAW_BIAS_BOOTSTRAP_MS) {
      gyroBiasDps = biasAccum / (float)biasSamples;
      biasReady = true;
      Serial.printf("[OK] Gyro bias calibrated: %+.2f dps — now rotate the board.\n\n", gyroBiasDps);
    }
  } else {
    yawRad += (gz - gyroBiasDps) * DEG_TO_RAD * dt;
    while (yawRad >  (float)M_PI) yawRad -= 2.0f * (float)M_PI;
    while (yawRad < -(float)M_PI) yawRad += 2.0f * (float)M_PI;
  }

  Serial.printf("%6lu | %5.2f %5.2f %5.2f | %5.1f %5.1f %5.1f | %8.1f  %+6.2f%s\n",
                millis() / 200, ax, ay, az, gx, gy, gz,
                yawRad * RAD_TO_DEG, gyroBiasDps,
                biasReady ? "" : "  [CALIBRATING...]");
}
