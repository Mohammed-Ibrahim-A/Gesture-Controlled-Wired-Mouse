// AeroClick v6 - ESP32-C3 air mouse over USB serial (no Wi-Fi / Bluetooth needed)
// Board: ESP32C3 Dev Module, USB CDC On Boot: Enabled
//
// Plug the board into the laptop, CLOSE the Arduino Serial Monitor, run the Python receiver.
//
// FIRST RUN : open Serial Monitor (115200). Keep the glove still while it calibrates,
//             then rotate RIGHT and DOWN when asked (skipped if already saved).
// RECALIBRATE: hold BOTH buttons for 1.5 s (keep the glove still).
// RE-LEARN DIRECTIONS: hold BOTH buttons while powering on.
// Cursor moves only while your wrist rotates. To reposition, hold the right
// button 2 s (cursor lock), move your hand, hold again to unlock.

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <math.h>

// ===================== PINS =====================
#define SDA_PIN        8
#define SCL_PIN        9
#define LEFT_BUTTON    4      // left click / double = scroll mode
#define RIGHT_BUTTON   3      // right click / hold 2 s = cursor lock

// ===================== TUNING =====================
#define SENSITIVITY        14.0f   // pixels per degree of wrist rotation (slow speed). Lower = slower
#define ACCEL_GAIN         2.0f    // extra gain for fast movement (0 = perfectly linear)
#define ACCEL_REF          150.0f  // deg/s at which full acceleration is reached
#define MOVE_DEADZONE      0.8f    // deg/s ignored once moving
#define MOVE_START         3.5f    // deg/s needed to start moving from rest
#define STILL_RATE         2.0f    // deg/s below which the hand counts as stopped
#define STILL_TIME_MS      120     // how long below STILL_RATE before output is zeroed
#define STILL_RANGE        1.5f    // max spread (deg/s) in a window to call the hand "still"
#define ZUPT_SAMPLES       50      // window length (50 x 5 ms = 250 ms)
#define ZUPT_MAX_STEP      3.0f    // max gyro-offset correction per window (deg/s)
#define BIAS_BLEND         0.30f   // how fast the gyro offset follows while still
#define RATE_SMOOTH        0.5f    // light smoothing of the rate (1 = none)

#define SENSOR_INTERVAL_MS 5
#define CURSOR_INTERVAL    8       // ms between cursor packets (USB is fast)
#define MAX_PACKET_MOVE    120

#define SCROLL_DEG_PER_NOTCH 3.0f  // degrees of rotation per scroll notch
#define SCROLL_MAX         5
#define SCROLL_INTERVAL    50
#define SCROLL_DIRECTION   -1      // flip to 1 if scrolling feels backwards

#define RELEASE_FREEZE_MS  120     // cursor held after a button release
#define BUTTON_DEBOUNCE    35
#define DOUBLE_CLICK_TIME  300
#define LOCK_HOLD_TIME     2000
#define RECAL_HOLD_TIME    1500
#define BOOT_CAL_MS        2500
#define RECAL_MS           2000

#define GYRO_LSB           65.5f   // +-500 deg/s range
#define DEBUG_PRINT        0       // set 1 to print motion info (ignored by the receiver)

// ===================== TYPES (keep above all functions) =====================
struct Btn {
  uint8_t  pin;
  bool     stable;     // true = released (pull-up)
  bool     lastRaw;
  uint32_t lastChange;
  uint32_t pressedAt;
  bool     consumed;
};

// ===================== GLOBALS =====================
static Preferences prefs;

static uint8_t  mpuAddr = 0x68;
static int      sensorFails = 0;
static uint32_t lastSensorUs = 0;

// gyro state
static float gyroBias[3] = {0, 0, 0};
static float zSum[3], zMin[3], zMax[3];
static int   zN = 0;
static bool  still = true;
static uint32_t belowSince = 0;
static float rHf = 0, rVf = 0;      // smoothed rates along cursor axes (deg/s)
static float lastMotion = 0;        // for debug

// direction mapping (learned by the wizard): cursor rate = sign * gyro[axis]
static int axisH = 1, signH = 1;    // + = cursor right
static int axisV = 0, signV = 1;    // + = cursor down

static bool scrollMode   = false;
static bool cursorLocked = false;

static float accX = 0, accY = 0, scrollAcc = 0;
static uint32_t lastCursorMs = 0, lastScrollMs = 0, lastStatusMs = 0;
static uint32_t freezeUntil = 0;

static Btn bL = {LEFT_BUTTON,  true, true, 0, 0, false};
static Btn bR = {RIGHT_BUTTON, true, true, 0, 0, false};
static uint8_t  lClicks = 0;
static uint32_t lLastRelease = 0;
static bool     rLongFired = false;
static uint32_t comboStart = 0;
static bool     comboFired = false;

// ===================== OUTPUT TO LAPTOP (one text line per message) =====================
static void sendMsg(const char *msg) {
  Serial.println(msg);
}

static void sendStatus() {
  char b[16];
  snprintf(b, sizeof(b), "S,%d", scrollMode ? 1 : 0);   sendMsg(b);
  snprintf(b, sizeof(b), "K,%d", cursorLocked ? 1 : 0); sendMsg(b);
}

// ===================== MPU6050 =====================
static bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool probeAddr(uint8_t a) {
  Wire.beginTransmission(a);
  return Wire.endTransmission() == 0;
}

static bool mpuConfigure() {
  if (!mpuWrite(0x6B, 0x01)) return false;   // wake, gyro PLL clock
  delay(100);
  mpuWrite(0x1A, 0x03);                      // low-pass filter ~44 Hz
  mpuWrite(0x19, 0x00);                      // 1 kHz internal sample rate
  mpuWrite(0x1B, 0x08);                      // gyro +-500 deg/s
  mpuWrite(0x1C, 0x00);                      // accel +-2 g (unused)
  delay(50);
  return true;
}

static bool findMpu() {
  const uint8_t pairs[][2] = { {SDA_PIN, SCL_PIN}, {SCL_PIN, SDA_PIN}, {7, 8}, {8, 7} };
  for (size_t i = 0; i < sizeof(pairs) / sizeof(pairs[0]); i++) {
    Wire.end();
    if (!Wire.begin(pairs[i][0], pairs[i][1])) continue;
    Wire.setTimeOut(20);
    Wire.setClock(400000);
    const uint8_t addrs[2] = {0x68, 0x69};
    for (int a = 0; a < 2; a++) {
      if (probeAddr(addrs[a])) {
        mpuAddr = addrs[a];
        Serial.printf("MPU6050 found: SDA=GPIO%d SCL=GPIO%d addr=0x%02X\n",
                      pairs[i][0], pairs[i][1], mpuAddr);
        return mpuConfigure();
      }
    }
  }
  return false;
}

// gyro rates in deg/s (raw, bias NOT removed)
static bool readGyro(float w[3]) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x43);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)mpuAddr, (uint8_t)6) != 6) return false;
  for (int i = 0; i < 3; i++) {
    int16_t hi = Wire.read();
    int16_t lo = Wire.read();
    int16_t raw = (int16_t)((hi << 8) | (lo & 0xFF));
    w[i] = raw / GYRO_LSB;
  }
  return true;
}

// ===================== BIAS CALIBRATION =====================
static void resetZupt() {
  for (int i = 0; i < 3; i++) { zSum[i] = 0; zMin[i] = 1e9f; zMax[i] = -1e9f; }
  zN = 0;
}

static void calibrateBias(uint32_t holdMs) {
  Serial.println();
  Serial.println("=== CALIBRATING: keep the glove STILL ===");
  float sum[3] = {0, 0, 0}, mn[3], mx[3];
  int n = 0;
  for (int i = 0; i < 3; i++) { mn[i] = 1e9f; mx[i] = -1e9f; }
  uint32_t overall = millis(), start = millis();

  while ((millis() - start) < holdMs && (millis() - overall) < 20000) {
    float w[3];
    if (!readGyro(w)) { delay(5); continue; }
    bool wild = false;
    for (int i = 0; i < 3; i++) {
      sum[i] += w[i];
      if (w[i] < mn[i]) mn[i] = w[i];
      if (w[i] > mx[i]) mx[i] = w[i];
      if ((mx[i] - mn[i]) > 6.0f) wild = true;
    }
    n++;
    if (wild) {                                  // moved: restart the still-timer
      for (int i = 0; i < 3; i++) { sum[i] = 0; mn[i] = 1e9f; mx[i] = -1e9f; }
      n = 0;
      start = millis();
    }
    delay(4);
  }

  if (n < 50) {
    Serial.println("Calibration failed (glove kept moving). It will self-correct while still.");
    return;
  }
  for (int i = 0; i < 3; i++) gyroBias[i] = sum[i] / n;

  still = true;
  belowSince = 0;
  rHf = rVf = 0;
  accX = accY = scrollAcc = 0;
  resetZupt();
  lastSensorUs = micros();
  Serial.printf("Calibration complete. Gyro offset: %.2f %.2f %.2f deg/s\n",
                gyroBias[0], gyroBias[1], gyroBias[2]);
}

// ===================== MOTION PROCESSING =====================
static bool inputFrozen() {
  return (!bL.stable) || (!bR.stable) || (lClicks > 0) || (millis() < freezeUntil);
}

static float softDeadzone(float r) {
  float a = fabsf(r);
  if (a <= MOVE_DEADZONE) return 0.0f;
  a -= MOVE_DEADZONE;
  return (r < 0) ? -a : a;
}

static bool sensorStep() {
  uint32_t nowUs = micros();
  if ((nowUs - lastSensorUs) < (uint32_t)SENSOR_INTERVAL_MS * 1000UL) return false;

  float w[3];
  if (!readGyro(w)) {
    if (++sensorFails > 200) { mpuConfigure(); sensorFails = 0; }
    return false;
  }
  sensorFails = 0;

  float dt = (nowUs - lastSensorUs) * 1e-6f;
  lastSensorUs = nowUs;
  if (dt > 0.05f) dt = 0.05f;

  // ---- zero-velocity detection: keeps the gyro offset correct at all times ----
  for (int i = 0; i < 3; i++) {
    zSum[i] += w[i];
    if (w[i] < zMin[i]) zMin[i] = w[i];
    if (w[i] > zMax[i]) zMax[i] = w[i];
  }
  if (++zN >= ZUPT_SAMPLES) {
    bool quiet = true;
    float mean[3];
    for (int i = 0; i < 3; i++) {
      mean[i] = zSum[i] / zN;
      if ((zMax[i] - zMin[i]) > STILL_RANGE) quiet = false;
      if (fabsf(mean[i] - gyroBias[i]) > ZUPT_MAX_STEP) quiet = false;
    }
    if (quiet) {
      for (int i = 0; i < 3; i++) gyroBias[i] += BIAS_BLEND * (mean[i] - gyroBias[i]);
      still = true;
      belowSince = 0;
    }
    resetZupt();
  }

  // ---- bias-corrected rates ----
  float c[3];
  float m = 0;
  for (int i = 0; i < 3; i++) {
    c[i] = w[i] - gyroBias[i];
    if (fabsf(c[i]) > m) m = fabsf(c[i]);
  }
  lastMotion = m;

  // ---- still / moving state ----
  if (still) {
    if (m > MOVE_START) { still = false; belowSince = 0; }
  } else {
    if (m < STILL_RATE) {
      if (belowSince == 0) belowSince = millis();
      else if ((millis() - belowSince) >= STILL_TIME_MS) { still = true; belowSince = 0; }
    } else {
      belowSince = 0;
    }
  }

  // ---- cursor-axis rates ----
  float rh = signH * c[axisH];
  float rv = signV * c[axisV];
  rHf += RATE_SMOOTH * (rh - rHf);
  rVf += RATE_SMOOTH * (rv - rVf);

  if (still || cursorLocked || inputFrozen()) {
    accX = accY = 0;
    if (!scrollMode) scrollAcc = 0;
    if (still) rHf = rVf = 0;
    return true;
  }

  float eH = softDeadzone(rHf);
  float eV = softDeadzone(rVf);

  if (scrollMode) {
    scrollAcc += (eV * dt) / SCROLL_DEG_PER_NOTCH;   // + = tilted down
  } else {
    float em = sqrtf(eH * eH + eV * eV);
    float k = em / ACCEL_REF;
    if (k > 1.0f) k = 1.0f;
    float gain = SENSITIVITY * (1.0f + ACCEL_GAIN * k);   // pixels per degree
    accX += eH * gain * dt;
    accY += eV * gain * dt;
  }
  return true;
}

// ===================== DIRECTION WIZARD (gyro based) =====================
static bool captureGesture(const char *msg, int &axis, int &sign) {
  Serial.println(msg);
  uint32_t t0 = millis();
  uint32_t last = micros();
  bool tracking = false;
  float ang[3] = {0, 0, 0};
  uint32_t trackStart = 0, quietStart = 0;

  while ((millis() - t0) < 30000) {
    delay(4);
    float w[3];
    if (!readGyro(w)) continue;
    uint32_t nowUs = micros();
    float dt = (nowUs - last) * 1e-6f;
    last = nowUs;
    if (dt > 0.05f) dt = 0.05f;

    float c[3], m = 0;
    for (int i = 0; i < 3; i++) {
      c[i] = w[i] - gyroBias[i];
      if (fabsf(c[i]) > m) m = fabsf(c[i]);
    }

    if (!tracking && m > 40.0f) {
      tracking = true;
      trackStart = millis();
      quietStart = 0;
      ang[0] = ang[1] = ang[2] = 0;
    }
    if (tracking) {
      for (int i = 0; i < 3; i++) ang[i] += c[i] * dt;
      if (m < 15.0f) { if (quietStart == 0) quietStart = millis(); }
      else quietStart = 0;

      bool ended = (quietStart != 0 && (millis() - quietStart) > 250) ||
                   ((millis() - trackStart) > 2500);
      if (ended) {
        int best = 0;
        for (int i = 1; i < 3; i++) if (fabsf(ang[i]) > fabsf(ang[best])) best = i;
        if (fabsf(ang[best]) >= 12.0f) {
          axis = best;
          sign = (ang[best] > 0) ? 1 : -1;
          Serial.printf("  got it (axis %d, %.0f deg)\n", best, ang[best]);
          return true;
        }
        Serial.println("  movement too small. Tilt more, then hold.");
        tracking = false;
      }
    }
  }
  Serial.println("  timed out.");
  return false;
}

static bool runAxisWizard() {
  Serial.println();
  Serial.println("=== DIRECTION SETUP ===");
  for (int attempt = 0; attempt < 3; attempt++) {
    int aH, sH, aV, sV;
    if (!captureGesture(">> Tilt your hand to the RIGHT, then HOLD it there...", aH, sH)) return false;
    Serial.println("   Return to your normal position and wait...");
    delay(3000);
    if (!captureGesture(">> Tilt DOWN (fingertips toward the floor), then HOLD it there...", aV, sV)) return false;
    Serial.println("   Return to your normal position and wait...");
    delay(2500);
    if (aH == aV) {
      Serial.println("Both gestures hit the same axis. Rotate straight RIGHT, then straight DOWN. Retrying...");
      continue;
    }
    axisH = aH; signH = sH;
    axisV = aV; signV = sV;
    prefs.begin("aeroclick", false);
    prefs.putInt("gaH", axisH); prefs.putInt("gsH", signH);
    prefs.putInt("gaV", axisV); prefs.putInt("gsV", signV);
    prefs.putInt("gmap", 1);
    prefs.end();
    Serial.println("Directions saved.");
    return true;
  }
  return false;
}

static bool loadMapping() {
  prefs.begin("aeroclick", true);
  bool ok = prefs.getInt("gmap", 0) == 1;
  if (ok) {
    axisH = prefs.getInt("gaH", axisH); signH = prefs.getInt("gsH", signH);
    axisV = prefs.getInt("gaV", axisV); signV = prefs.getInt("gsV", signV);
  }
  prefs.end();
  return ok;
}

// ===================== BUTTONS =====================
static int updateBtn(Btn &b) {
  bool raw = digitalRead(b.pin);
  uint32_t now = millis();
  if (raw != b.lastRaw) { b.lastRaw = raw; b.lastChange = now; }
  if ((now - b.lastChange) >= BUTTON_DEBOUNCE && raw != b.stable) {
    b.stable = raw;
    if (!b.stable) { b.pressedAt = now; return 1; }
    return -1;
  }
  return 0;
}

static void processButtons() {
  int eL = updateBtn(bL);
  int eR = updateBtn(bR);
  uint32_t now = millis();

  if (eL == 1) bL.consumed = false;
  if (eR == 1) { bR.consumed = false; rLongFired = false; }
  if (eL == -1 || eR == -1) freezeUntil = now + RELEASE_FREEZE_MS;

  // Both buttons held = recalibrate (and swallow the individual clicks)
  bool bothDown = !bL.stable && !bR.stable;
  if (bothDown) {
    bL.consumed = true;
    bR.consumed = true;
    if (comboStart == 0) comboStart = now;
    if (!comboFired && (now - comboStart) >= RECAL_HOLD_TIME) {
      comboFired = true;
      calibrateBias(RECAL_MS);
    }
  } else {
    comboStart = 0;
    comboFired = false;
  }

  // Left: single = click, double = scroll mode
  if (eL == -1 && !bL.consumed) { lClicks++; lLastRelease = now; }
  if (lClicks > 0 && bL.stable && (now - lLastRelease) > DOUBLE_CLICK_TIME) {
    if (lClicks == 1) {
      sendMsg("L");
    } else {
      scrollMode = !scrollMode;
      accX = accY = scrollAcc = 0;
      sendStatus();
    }
    lClicks = 0;
    freezeUntil = millis() + RELEASE_FREEZE_MS;
  }

  // Right: single = click, hold 2 s = cursor lock
  if (!bR.stable && !bR.consumed && !rLongFired && (now - bR.pressedAt) >= LOCK_HOLD_TIME) {
    rLongFired = true;
    cursorLocked = !cursorLocked;
    accX = accY = scrollAcc = 0;
    sendStatus();
  }
  if (eR == -1 && !bR.consumed && !rLongFired) {
    sendMsg("R");
  }
}

// ===================== OUTPUT =====================
static void sendCursor() {
  uint32_t now = millis();
  if ((now - lastCursorMs) < CURSOR_INTERVAL) return;
  lastCursorMs = now;

  if (cursorLocked || inputFrozen()) { accX = accY = scrollAcc = 0; return; }

  if (scrollMode) {
    if ((now - lastScrollMs) < SCROLL_INTERVAL) return;
    int n = (int)scrollAcc;
    if (n == 0) return;
    lastScrollMs = now;
    scrollAcc -= n;
    if (n > SCROLL_MAX) n = SCROLL_MAX;
    if (n < -SCROLL_MAX) n = -SCROLL_MAX;
    char b[16];
    snprintf(b, sizeof(b), "W,%d", n * SCROLL_DIRECTION);
    sendMsg(b);
    return;
  }

  int ix = (int)accX;  accX -= ix;
  int iy = (int)accY;  accY -= iy;
  if (ix > MAX_PACKET_MOVE) ix = MAX_PACKET_MOVE;
  if (ix < -MAX_PACKET_MOVE) ix = -MAX_PACKET_MOVE;
  if (iy > MAX_PACKET_MOVE) iy = MAX_PACKET_MOVE;
  if (iy < -MAX_PACKET_MOVE) iy = -MAX_PACKET_MOVE;
  if (ix != 0 || iy != 0) {
    char b[24];
    snprintf(b, sizeof(b), "M,%d,%d", ix, iy);
    sendMsg(b);
  }
}

// ===================== SETUP / LOOP =====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("==========================================");
  Serial.println("      AEROCLICK v6  (USB serial mouse)");
  Serial.println("==========================================");

  pinMode(LEFT_BUTTON, INPUT_PULLUP);
  pinMode(RIGHT_BUTTON, INPUT_PULLUP);
  delay(30);
  bool forceWizard = (digitalRead(LEFT_BUTTON) == LOW && digitalRead(RIGHT_BUTTON) == LOW);

  Wire.begin(SDA_PIN, SCL_PIN);
  while (!findMpu()) {
    Serial.println("MPU6050 not found. Check SDA/SCL, VCC (3.3 V) and GND. Retrying...");
    delay(2000);
  }

  Serial.println("Calibrating in 2 s: rest the glove and keep it STILL.");
  delay(2000);
  resetZupt();
  lastSensorUs = micros();
  calibrateBias(BOOT_CAL_MS);

  bool mapped = loadMapping();
  if (!mapped || forceWizard) {
    if (!runAxisWizard()) {
      Serial.println("Direction setup not finished: using defaults. It will ask again next boot.");
    }
    still = true;
    resetZupt();
  }

  bL.stable = bL.lastRaw = digitalRead(LEFT_BUTTON);
  bR.stable = bR.lastRaw = digitalRead(RIGHT_BUTTON);
  lastSensorUs = micros();

  Serial.println();
  Serial.println("READY - close the Serial Monitor and start the Python receiver");
  sendStatus();
}

void loop() {
  processButtons();
  sensorStep();
  sendCursor();

  uint32_t now = millis();
  if ((now - lastStatusMs) > 3000) {
    lastStatusMs = now;
    sendStatus();
  }

#if DEBUG_PRINT
  static uint32_t lastPrint = 0;
  if ((now - lastPrint) > 500) {
    lastPrint = now;
    Serial.printf("rate %.2f dps | %s\n", lastMotion, still ? "STILL" : "MOVING");
  }
#endif

  delay(1);
}