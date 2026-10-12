#include <Arduino.h>
/* ╔══════════════════════════════════════════════════════════════════╗
 * ║       ARK-5  Step 4 — Dedicated L298N Motor Test                ║
 * ║   Tests 4x DC Motors via L298N Dual H-Bridge Driver             ║
 * ╠══════════════════════════════════════════════════════════════════╣
 * ║  WIRING CHECKLIST:                                               ║
 * ║  Left Motor  : ENA=GPIO 25 | IN1=GPIO 26 | IN2=GPIO 27          ║
 * ║  Right Motor : ENB=GPIO 33 | IN3=GPIO 32 | IN4=GPIO 14          ║
 * ║  Power       : 12V Battery -> L298N 12V & GND                   ║
 * ║  COMMON GND  : L298N GND MUST be connected to ESP32 GND!        ║
 * ╠══════════════════════════════════════════════════════════════════╣
 * ║  TEST SEQUENCE:                                                 ║
 * ║   1. FRONT (Forward)  : 3 Seconds                               ║
 * ║   2. PAUSE            : 1.5 Seconds                             ║
 * ║   3. BACK (Backward)  : 3 Seconds                               ║
 * ║   4. PAUSE            : 1.5 Seconds                             ║
 * ║   5. LEFT (Turn Left) : 3 Seconds                               ║
 * ║   6. PAUSE            : 1.5 Seconds                             ║
 * ║   7. RIGHT (Turn Right: 3 Seconds                               ║
 * ║   8. PAUSE            : 1.5 Seconds                             ║
 * ║   9. LEFT CHANNEL ONLY: 3 Seconds   <- isolates ENA/IN1/IN2     ║
 * ║  10. PAUSE            : 1.5 Seconds                             ║
 * ║  11. RIGHT CH. ONLY   : 3 Seconds   <- isolates ENB/IN3/IN4     ║
 * ║  12. PAUSE            : 2 Seconds -> Repeats automatically      ║
 * ║                                                                  ║
 * ║  SERIAL COMMANDS (115200 baud):                                  ║
 * ║   'f' -> Forward 3s                                             ║
 * ║   'b' -> Backward 3s                                             ║
 * ║   'l' -> Left Turn 3s                                            ║
 * ║   'r' -> Right Turn 3s                                           ║
 * ║   '1' -> LEFT channel ONLY 3s  (2 wheels, ENA=25/IN1=26/IN2=27)  ║
 * ║   '2' -> RIGHT channel ONLY 3s (2 wheels, ENB=33/IN3=32/IN4=14)  ║
 * ║   's' -> STOP immediately                                        ║
 * ║   'a' -> Resume Automatic Cycle (incl. channel-isolation steps)  ║
 * ║                                                                  ║
 * ║  USE '1' / '2' WHEN ONE SIDE DOESN'T SPIN:                      ║
 * ║   run this sketch with NO laptop attached. If '1' leaves the     ║
 * ║   left wheels dead too, the fault is L298N channel A / ENA       ║
 * ║   jumper / IN1-IN2 wires / 12V feed — not the main firmware.     ║
 * ╚══════════════════════════════════════════════════════════════════╝
 */

// ── Pin Definitions ──────────────────────────────────────────────────
#define PIN_ENA   25  // Left Motor Speed (PWM)
#define PIN_IN1   26  // Left Motor Direction 1
#define PIN_IN2   27  // Left Motor Direction 2

#define PIN_ENB   33  // Right Motor Speed (PWM)
#define PIN_IN3   32  // Right Motor Direction 1
#define PIN_IN4   14  // Right Motor Direction 2

// ── PWM Settings ─────────────────────────────────────────────────────
#define PWM_FREQ      1000
#define PWM_RES       8     // 8-bit resolution (0 - 255)
#define PWM_CH_L      0
#define PWM_CH_R      1
#define MOTOR_SPEED   180   // Test speed (0-255) — good torque for floor/air

// ── LEDC PWM Compatibility for ESP32 Core 2.x and Core 3.x ────────────
static inline void pwmSetup(uint8_t pin, uint8_t channel, uint32_t freq, uint8_t res) {
#if defined(ESP_ARDUINO_VERSION) && ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcAttach(pin, freq, res);
#else
  ledcSetup(channel, freq, res);
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

// ── Low-Level Motor Primitives ───────────────────────────────────────
void motorsStop() {
  pwmWrite(PIN_ENA, PWM_CH_L, 0);
  pwmWrite(PIN_ENB, PWM_CH_R, 0);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
}

void motorsForward(uint8_t spd) {
  // Left Forward
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  // Right Forward
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);
  pwmWrite(PIN_ENA, PWM_CH_L, spd);
  pwmWrite(PIN_ENB, PWM_CH_R, spd);
}

void motorsBackward(uint8_t spd) {
  // Left Backward
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);
  // Right Backward
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, HIGH);
  pwmWrite(PIN_ENA, PWM_CH_L, spd);
  pwmWrite(PIN_ENB, PWM_CH_R, spd);
}

void motorsTurnLeft(uint8_t spd) {
  // Left Reverse, Right Forward (In-Place Pivot Left)
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);
  pwmWrite(PIN_ENA, PWM_CH_L, spd);
  pwmWrite(PIN_ENB, PWM_CH_R, spd);
}

void motorsTurnRight(uint8_t spd) {
  // Left Forward, Right Reverse (In-Place Pivot Right)
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, HIGH);
  pwmWrite(PIN_ENA, PWM_CH_L, spd);
  pwmWrite(PIN_ENB, PWM_CH_R, spd);
}

// ── Single-channel isolation (dead-side diagnosis) ─────────────────────
// Drives ONLY one H-bridge channel so a side that never moves can be
// traced to the driver/wiring instead of the main firmware.
void motorsLeftOnly(uint8_t spd) {
  digitalWrite(PIN_IN1, HIGH);   // left forward
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);    // right idle (both IN low = brake)
  digitalWrite(PIN_IN4, LOW);
  pwmWrite(PIN_ENA, PWM_CH_L, spd);
  pwmWrite(PIN_ENB, PWM_CH_R, 0);
}

void motorsRightOnly(uint8_t spd) {
  digitalWrite(PIN_IN1, LOW);    // left idle
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, HIGH);   // right forward
  digitalWrite(PIN_IN4, LOW);
  pwmWrite(PIN_ENA, PWM_CH_L, 0);
  pwmWrite(PIN_ENB, PWM_CH_R, spd);
}

// ── Timed Movement with Live Second-by-Second Countdown ──────────────
void executeTimedAction(const char* name, void (*motorFunc)(uint8_t), int durationSec) {
  Serial.println();
  Serial.println(F("=================================================="));
  Serial.printf(  "  >>> ACTION: %s (Duration: %d seconds) <<<\n", name, durationSec);
  Serial.println(F("=================================================="));

  motorFunc(MOTOR_SPEED);

  for (int s = durationSec; s > 0; s--) {
    Serial.printf("  [RUNNING] %s ... %d sec remaining\n", name, s);
    delay(1000);
  }

  motorsStop();
  Serial.printf("  [STOPPED] %s complete.\n", name);
}

void executePause(float pauseSec) {
  motorsStop();
  Serial.printf("  [PAUSE] Holding for %.1f sec ...\n", pauseSec);
  delay((unsigned long)(pauseSec * 1000));
}

// ── Full 4-Direction Test Cycle ──────────────────────────────────────
void runFullCycle() {
  Serial.println(F("\n##################################################"));
  Serial.println(F("  STARTING FULL MOTOR TEST CYCLE"));
  Serial.println(F("  FRONT -> BACK -> LEFT -> RIGHT -> LEFT-ONLY -> RIGHT-ONLY"));
  Serial.println(F("##################################################"));

  // 1. FRONT (3 seconds)
  executeTimedAction("1. FRONT (FORWARD)", motorsForward, 3);
  executePause(1.5);

  // 2. BACK (3 seconds)
  executeTimedAction("2. BACK (BACKWARD)", motorsBackward, 3);
  executePause(1.5);

  // 3. LEFT (3 seconds)
  executeTimedAction("3. LEFT TURN", motorsTurnLeft, 3);
  executePause(1.5);

  // 4. RIGHT (3 seconds)
  executeTimedAction("4. RIGHT TURN", motorsTurnRight, 3);
  executePause(1.5);

  // 5. LEFT CHANNEL ONLY (3 seconds) — isolates ENA/IN1/IN2
  executeTimedAction("5. LEFT CHANNEL ONLY", motorsLeftOnly, 3);
  executePause(1.5);

  // 6. RIGHT CHANNEL ONLY (3 seconds) — isolates ENB/IN3/IN4
  executeTimedAction("6. RIGHT CHANNEL ONLY", motorsRightOnly, 3);
  executePause(2.0);

  Serial.println(F("\n[CYCLE FINISHED] All 4 directions + both channels tested!"));
  Serial.println(F("If the LEFT wheels never move in step 5 -> L298N channel A /"));
  Serial.println(F("ENA jumper / IN1-IN2 / 12V feed, NOT the main firmware."));
  Serial.println(F("Next cycle starts in 3 seconds (or type 's' to stop)...\n"));
  delay(3000);
}

// ── Setup & Loop ─────────────────────────────────────────────────────
bool autoCycle = true;

void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println(F("\n╔══════════════════════════════════════════════════╗"));
  Serial.println(F("║   ARK-5  Step 4: Dedicated Motor Verification   ║"));
  Serial.println(F("╚══════════════════════════════════════════════════╝"));
  Serial.println(F("Pin Configuration:"));
  Serial.printf( "  Left Motors  : ENA=%d, IN1=%d, IN2=%d\n", PIN_ENA, PIN_IN1, PIN_IN2);
  Serial.printf( "  Right Motors : ENB=%d, IN3=%d, IN4=%d\n", PIN_ENB, PIN_IN3, PIN_IN4);
  Serial.println(F("Safety Warning: Robot ko stand/box par rakhein taake wheels hawa mein hon!"));
  Serial.println(F("Commands: 'f'=Fwd, 'b'=Back, 'l'=Left, 'r'=Right, '1'=Left channel only,\n          '2'=Right channel only, 's'=Stop, 'a'=Auto Cycle\n"));

  // Configure Direction GPIOs
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);

  // Configure PWM (Core 2.x & Core 3.x compatible)
  pwmSetup(PIN_ENA, PWM_CH_L, PWM_FREQ, PWM_RES);
  pwmSetup(PIN_ENB, PWM_CH_R, PWM_FREQ, PWM_RES);

  motorsStop();
  Serial.println(F("[INIT OK] Motors initialized and stopped. Starting in 3s..."));
  delay(3000);
}

void loop() {
  // Check for User Serial input commands
  if (Serial.available() > 0) {
    char cmd = (char)Serial.read();
    while (Serial.available()) Serial.read(); // Clear buffer

    if (cmd == 's' || cmd == 'S') {
      motorsStop();
      autoCycle = false;
      Serial.println(F("\n[CMD] EMERGENCY STOPPED. Auto-cycle paused. Type 'a' to resume."));
      return;
    } else if (cmd == 'a' || cmd == 'A') {
      autoCycle = true;
      Serial.println(F("\n[CMD] Resuming Automatic Cycle."));
    } else if (cmd == 'f' || cmd == 'F') {
      autoCycle = false;
      executeTimedAction("MANUAL FORWARD", motorsForward, 3);
      return;
    } else if (cmd == 'b' || cmd == 'B') {
      autoCycle = false;
      executeTimedAction("MANUAL BACKWARD", motorsBackward, 3);
      return;
    } else if (cmd == 'l' || cmd == 'L') {
      autoCycle = false;
      executeTimedAction("MANUAL LEFT TURN", motorsTurnLeft, 3);
      return;
    } else if (cmd == 'r' || cmd == 'R') {
      autoCycle = false;
      executeTimedAction("MANUAL RIGHT TURN", motorsTurnRight, 3);
      return;
    } else if (cmd == '1') {
      autoCycle = false;
      executeTimedAction("MANUAL LEFT CHANNEL ONLY (ENA/IN1/IN2)", motorsLeftOnly, 3);
      return;
    } else if (cmd == '2') {
      autoCycle = false;
      executeTimedAction("MANUAL RIGHT CHANNEL ONLY (ENB/IN3/IN4)", motorsRightOnly, 3);
      return;
    }
  }

  if (autoCycle) {
    runFullCycle();
  } else {
    delay(100);
  }
}
