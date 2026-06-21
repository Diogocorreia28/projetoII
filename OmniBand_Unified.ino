// =====================================================================
// OmniBand_Unified.ino
// =====================================================================
// Fully integrated firmware combining:
//   1. Simple threshold-based gesture detection (codigo_funciona_cima.txt)
//   2. Full state machine: wake + clap context + 4 gestures (Menu_bonito_palmas.ino)
//   3. ML classifier inference via Edge Impulse (teste_ml_gesto_unico.ino)
//
// How it works:
//   - Normal operation: full state machine (IDLE → CONTEXT → GESTURE → RESULT)
//   - In IDLE state, simple threshold gestures (Cima/Baixo) also trigger quick actions
//   - Press BtnA briefly: recalibrate sensors
//   - Press and hold BtnA (>2s): run ML diagnostic test (capture → classify → display)
// =====================================================================

#include <M5Unified.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <math.h>
#include "BMI088.h"
#include <correia_2k-project-1_inferencing.h>

// =====================================================================
// WiFi & Server Configuration
// =====================================================================
const char* WIFI_SSID = "CasaLt33";
const char* WIFI_PASS = "luisdiogo";
const char* SERVER_URL = "http://192.168.1.233:5000/api/trigger";

// =====================================================================
// I2C Pins
// =====================================================================
const uint8_t I2C_PRIMARY_SDA   = 33;
const uint8_t I2C_PRIMARY_SCL   = 32;
const uint8_t I2C_FALLBACK_SDA  = 32;
const uint8_t I2C_FALLBACK_SCL  = 33;

// =====================================================================
// BMI088 IMU Objects (with fallback candidates)
// =====================================================================
Bmi088Accel accelDefault(Wire, 0x19);
Bmi088Gyro  gyroDefault (Wire, 0x69);
Bmi088Accel accelAlt    (Wire, 0x18);
Bmi088Gyro  gyroAlt     (Wire, 0x68);

Bmi088Accel* accel = &accelDefault;
Bmi088Gyro*  gyro  = &gyroDefault;

float ax0 = 0, ay0 = 0, az0 = 0;

// =====================================================================
// Microphone pin
// =====================================================================
const uint8_t MIC_ANALOG_PIN = 36;

// =====================================================================
// MICROPHONE — Adaptive Clap Detection Constants
// =====================================================================
const int AUDIO_PULSE_THRESHOLD = 650;
const int MIC_VALID_AVG_MIN = 30;
const int MIC_VALID_AVG_MAX = 3900;
const float CLAP_RATIO_THRESHOLD = 2.8f;
const int   MIN_CLAP_ABSOLUTE    = 600;
const int   NOISE_FLOOR_CAP      = 900;
const float NOISE_RISE_ALPHA     = 0.12f;
const float NOISE_FALL_ALPHA     = 0.025f;

const unsigned long MIC_SAMPLE_MS = 40;
const unsigned long MIC_SAMPLE_INTERVAL_MS = 40;
const unsigned long MIC_SETTLE_AFTER_WAKE_MS = 700;
const unsigned long CLAP_DEBOUNCE_MS = 200;
const unsigned long CLAP_GAP_MS = 2500;
const unsigned long CONTEXT_WINDOW_MS = 7000;
const unsigned long CONTEXT_WAIT_TIMEOUT_MS = 10000;
const int MAX_CONTEXT_CLAPS = 3;

// =====================================================================
// GESTURE — IMU Thresholds
// =====================================================================
// SIMPLE mode thresholds (always-on quick gestures)
const float SIMPLE_CIMA_THRESHOLD_AZ = 6.0f;
const float SIMPLE_CIMA_THRESHOLD_MAG = 7.0f;
const float SIMPLE_BAIXO_THRESHOLD_AZ = -6.0f;
const float SIMPLE_BAIXO_THRESHOLD_MAG = 7.0f;
const unsigned long SIMPLE_COOLDOWN_MS = 1500;

// FULL mode thresholds
const float WAKE_ACCEL_DELTA_THRESHOLD = 3.0f;
const float WAKE_GYRO_THRESHOLD_DPS = 150.0f;
const unsigned long WAKE_COOLDOWN_MS = 1200;
const int WAKE_CONFIRM_SAMPLES = 2;

const float GESTURE_ACCEL_DELTA_THRESHOLD = 5.0f;
const float GESTURE_AXIS_DELTA_THRESHOLD = 4.5f;
const float ROTATION_THRESHOLD_DPS = 250.0f;
const float RAD_TO_DEG_PER_SEC = 57.2957795f;
const unsigned long GESTURE_WINDOW_MS = 9000;
const unsigned long GESTURE_COOLDOWN_MS = 1500;
const unsigned long RESULT_RETURN_MS = 3000;
const bool INVERT_ROTATION_GESTURES = false;

// =====================================================================
// ML test constants
// =====================================================================
const unsigned long PRE_CAPTURE_MS = 1000;
const float ML_CONFIDENCE_OK = 0.65f;
const unsigned long BTN_HOLD_MS = 2000;  // Hold button for 2s to enter ML test

// =====================================================================
// State Variables
// =====================================================================
bool imuReady = false;
bool micReady = false;
int micThreshold = AUDIO_PULSE_THRESHOLD;
float adaptiveNoiseFloor = 350.0f;
unsigned long lastPlotterAt = 0;

// Full state machine
unsigned long lastWakeAt = 0;
unsigned long lastGestureAt = 0;
unsigned long lastImuSerialAt = 0;
unsigned long lastMicSampleAt = 0;
unsigned long stateStartedAt = 0;
unsigned long firstClapAt = 0;
unsigned long lastClapAt = 0;
unsigned long returnToIdleAt = 0;
unsigned long btnPressStartedAt = 0;
bool btnHeld = false;

int clapCount = 0;
int activeClaps = 0;
int imuFaultCount = 0;
int wakeHitCount = 0;
const char* activeRoom = "sem contexto";

// Simple mode
unsigned long simpleLastSend = 0;

enum AppState {
  STATE_IDLE,
  STATE_CONTEXT,
  STATE_GESTURE,
  STATE_RESULT,
  STATE_ML_TEST     // ML diagnostic mode
};

enum GestureId {
  GESTURE_NONE,
  GESTURE_UP,
  GESTURE_DOWN,
  GESTURE_ROTATE_OUT,
  GESTURE_ROTATE_IN
};

struct GestureInfo {
  GestureId id;
  const char* payload;
  const char* title;
  const char* action;
};

struct ImuReading {
  float ax;
  float ay;
  float az;
  float mag;
  float gxDps;
  float gyDps;
  float gzDps;
};

struct MicStats {
  int minValue;
  int maxValue;
  int peakToPeak;
  int average;
  bool loud;
  bool suspicious;
};

AppState appState = STATE_IDLE;
ImuReading previousReading;
bool hasPreviousReading = false;

// =====================================================================
// Helper Functions
// =====================================================================

GestureInfo gestureInfo(GestureId id) {
  switch (id) {
    case GESTURE_UP: return {GESTURE_UP, "Cima", "Cima", "Aumentar/subir"};
    case GESTURE_DOWN: return {GESTURE_DOWN, "Baixo", "Baixo", "Diminuir/descer"};
    case GESTURE_ROTATE_OUT: return {GESTURE_ROTATE_OUT, "RodarFora", "Rodar fora", "Ligar/acender"};
    case GESTURE_ROTATE_IN: return {GESTURE_ROTATE_IN, "RodarDentro", "Rodar dentro", "Desligar/apagar"};
    default: return {GESTURE_NONE, "", "Sem gesto", "A espera"};
  }
}

const char* roomFromClaps(int pulses) {
  if (pulses <= 1) return "corredor";
  if (pulses == 2) return "sala";
  return "quarto";
}

// =====================================================================
// IMU Initialization
// =====================================================================

bool beginImuCandidate(uint8_t sda, uint8_t scl,
                       Bmi088Accel& accelCandidate, Bmi088Gyro& gyroCandidate,
                       const char* name) {
  Wire.end(); delay(40);
  Wire.begin(sda, scl); Wire.setClock(400000);
  if (accelCandidate.begin() < 0) return false;
  if (gyroCandidate.begin() < 0) return false;
  accel = &accelCandidate; gyro = &gyroCandidate;
  Serial.printf("[imu] BMI088 OK (%s) SDA=G%d SCL=G%d\n", name, sda, scl);
  return true;
}

bool initImu() {
  if (beginImuCandidate(I2C_PRIMARY_SDA,  I2C_PRIMARY_SCL,  accelDefault, gyroDefault, "0x19/0x69")) return true;
  if (beginImuCandidate(I2C_PRIMARY_SDA,  I2C_PRIMARY_SCL,  accelAlt,     gyroAlt,     "0x18/0x68")) return true;
  if (beginImuCandidate(I2C_FALLBACK_SDA, I2C_FALLBACK_SCL, accelDefault, gyroDefault, "0x19/0x69 fallback")) return true;
  if (beginImuCandidate(I2C_FALLBACK_SDA, I2C_FALLBACK_SCL, accelAlt,     gyroAlt,     "0x18/0x68 fallback")) return true;
  return false;
}

// =====================================================================
// Calibration
// =====================================================================

void calibrateZero() {
  const int n = 50;
  float sx = 0, sy = 0, sz = 0;
  for (int i = 0; i < n; i++) {
    accel->readSensor();
    sx += accel->getAccelX_mss();
    sy += accel->getAccelY_mss();
    sz += accel->getAccelZ_mss();
    delay(20);
  }
  ax0 = sx / n;
  ay0 = sy / n;
  az0 = sz / n;
}

// =====================================================================
// WiFi
// =====================================================================

bool connectWiFi(unsigned long timeoutMs = 12000) {
  if (WiFi.status() == WL_CONNECTED) return true;
  if (String(WIFI_SSID).length() == 0) return false;
  WiFi.mode(WIFI_STA); WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("[wifi] A ligar");
  unsigned long startedAt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startedAt < timeoutMs) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  return WiFi.status() == WL_CONNECTED;
}

// =====================================================================
// Mic Sampling
// =====================================================================

MicStats sampleMic(unsigned long durationMs = MIC_SAMPLE_MS) {
  MicStats stats = {4095, 0, 0, 0, false, false};
  long sum = 0; int samples = 0;
  unsigned long startedAt = millis();
  while (millis() - startedAt < durationMs) {
    int analogValue = analogRead(MIC_ANALOG_PIN);
    stats.minValue = min(stats.minValue, analogValue);
    stats.maxValue = max(stats.maxValue, analogValue);
    sum += analogValue; samples++;
    delayMicroseconds(650);
  }
  if (samples > 0) stats.average = sum / samples;
  stats.peakToPeak = stats.maxValue - stats.minValue;
  stats.suspicious = stats.average < MIC_VALID_AVG_MIN || stats.average > MIC_VALID_AVG_MAX;
  stats.loud = stats.peakToPeak >= micThreshold && !stats.suspicious;
  return stats;
}

void calibrateMic() {
  MicStats mic = sampleMic(500);
  adaptiveNoiseFloor = constrain((float)mic.peakToPeak * 1.2f, 200.0f, (float)NOISE_FLOOR_CAP);
  micThreshold = max(MIN_CLAP_ABSOLUTE, (int)(adaptiveNoiseFloor * CLAP_RATIO_THRESHOLD));
  micReady = mic.average >= MIC_VALID_AVG_MIN && mic.average <= MIC_VALID_AVG_MAX;
  Serial.printf("[mic] avg=%d noiseFloor=%.0f threshold=%d ready=%d\n", mic.average, adaptiveNoiseFloor, micThreshold, micReady);
}

// =====================================================================
// IMU Reading
// =====================================================================

bool readImu(ImuReading& r) {
  accel->readSensor(); gyro->readSensor();
  r.ax = accel->getAccelX_mss() - ax0;
  r.ay = accel->getAccelY_mss() - ay0;
  r.az = accel->getAccelZ_mss() - az0;
  r.mag = sqrtf(r.ax * r.ax + r.ay * r.ay + r.az * r.az);
  r.gxDps = gyro->getGyroX_rads() * RAD_TO_DEG_PER_SEC;
  r.gyDps = gyro->getGyroY_rads() * RAD_TO_DEG_PER_SEC;
  r.gzDps = gyro->getGyroZ_rads() * RAD_TO_DEG_PER_SEC;
  bool valid = isfinite(r.ax) && isfinite(r.ay) && isfinite(r.az) && isfinite(r.gxDps) && isfinite(r.gyDps) && isfinite(r.gzDps);
  if (!valid) { imuFaultCount++; return false; }
  imuFaultCount = 0; return true;
}

float dominantRotationDps(const ImuReading& r) {
  float rotation = r.gxDps;
  if (fabsf(r.gyDps) > fabsf(rotation)) rotation = r.gyDps;
  if (fabsf(r.gzDps) > fabsf(rotation)) rotation = r.gzDps;
  return rotation;
}

float gyroMagnitudeDps(const ImuReading& r) {
  return sqrtf(r.gxDps * r.gxDps + r.gyDps * r.gyDps + r.gzDps * r.gzDps);
}

float accelDeltaFromPrevious(const ImuReading& r) {
  if (!hasPreviousReading) return 0.0f;
  float dx = r.ax - previousReading.ax, dy = r.ay - previousReading.ay, dz = r.az - previousReading.az;
  return sqrtf(dx * dx + dy * dy + dz * dz);
}

float accelZDeltaFromPrevious(const ImuReading& r) {
  return hasPreviousReading ? r.az - previousReading.az : 0.0f;
}

void rememberImuReading(const ImuReading& r) {
  previousReading = r; hasPreviousReading = true;
}

// =====================================================================
// Gesture Detection
// =====================================================================

bool detectWake(const ImuReading& r, float accelDelta, float gyroAbs) {
  if (millis() - lastWakeAt < WAKE_COOLDOWN_MS) return false;
  wakeHitCount = (accelDelta > WAKE_ACCEL_DELTA_THRESHOLD || gyroAbs > WAKE_GYRO_THRESHOLD_DPS) ? wakeHitCount + 1 : 0;
  return wakeHitCount >= WAKE_CONFIRM_SAMPLES;
}

GestureId detectGesture(const ImuReading& r, float accelDelta, float accelZDelta) {
  if (millis() - lastGestureAt < GESTURE_COOLDOWN_MS) return GESTURE_NONE;
  if (accelDelta > GESTURE_ACCEL_DELTA_THRESHOLD && accelZDelta > GESTURE_AXIS_DELTA_THRESHOLD) return GESTURE_UP;
  if (accelDelta > GESTURE_ACCEL_DELTA_THRESHOLD && accelZDelta < -GESTURE_AXIS_DELTA_THRESHOLD) return GESTURE_DOWN;
  float rotation = dominantRotationDps(r);
  if (fabsf(rotation) > ROTATION_THRESHOLD_DPS) {
    bool rotateOut = INVERT_ROTATION_GESTURES ? !(rotation > 0) : (rotation > 0);
    return rotateOut ? GESTURE_ROTATE_OUT : GESTURE_ROTATE_IN;
  }
  return GESTURE_NONE;
}

// SIMPLE mode detection: quick threshold-based up/down from codigo_funciona_cima.txt
bool detectSimpleUp(const ImuReading& r) {
  return (r.az > SIMPLE_CIMA_THRESHOLD_AZ && r.mag > SIMPLE_CIMA_THRESHOLD_MAG);
}

bool detectSimpleDown(const ImuReading& r) {
  return (r.az < SIMPLE_BAIXO_THRESHOLD_AZ && r.mag > SIMPLE_BAIXO_THRESHOLD_MAG);
}

// =====================================================================
// HTTP POST
// =====================================================================

bool postSimpleGesture(const char* gesture) {
  if (WiFi.status() != WL_CONNECTED) {
    if (!connectWiFi(5000)) return false;
  }
  HTTPClient http;
  http.begin(SERVER_URL);
  http.addHeader("Content-Type", "application/json");
  String payload = String("{\"gesto\":\"") + gesture + "\"}";
  int code = http.POST(payload);
  Serial.print("Payload enviado: ");
  Serial.println(payload);
  Serial.print("HTTP code: ");
  Serial.println(code);
  http.end();
  return code > 0 && code < 400;
}

bool postFullGesture(const GestureInfo& info, int& httpCode) {
  httpCode = -1;
  if (!connectWiFi(5000)) return false;
  HTTPClient http;
  http.begin(SERVER_URL);
  http.addHeader("Content-Type", "application/json");
  String payload = String("{\"gesto\":\"") + info.payload
    + "\",\"contexto\":\"" + activeRoom
    + "\",\"palmas\":" + activeClaps
    + ",\"acao\":\"" + info.action + "\"}";
  httpCode = http.POST(payload);
  Serial.println("[http] POST enviado");
  http.end();
  return httpCode > 0 && httpCode < 400;
}

// =====================================================================
// E-Paper UI
// =====================================================================

const int SCR_W = 200;
const int SCR_H = 200;

void epdFullMode() { M5.Display.setEpdMode(epd_text); }
void epdFastMode() { M5.Display.setEpdMode(epd_fastest); }

void drawTopBar(const char* leftText, const char* rightText = "") {
  M5.Display.fillRect(0, 0, SCR_W, 22, TFT_BLACK);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(6, 4);
  M5.Display.print(leftText);
  if (rightText[0] != '\0') {
    M5.Display.setTextSize(1);
    int textW = strlen(rightText) * 6;
    M5.Display.setCursor(SCR_W - textW - 6, 7);
    M5.Display.print(rightText);
  }
}

void drawFooterHint(const char* text) {
  M5.Display.drawLine(0, 178, SCR_W, 178, TFT_BLACK);
  M5.Display.setTextColor(TFT_BLACK, TFT_WHITE);
  M5.Display.setTextSize(1);
  M5.Display.setCursor(6, 185);
  M5.Display.print(text);
}

// --- Icons ---
void iconArrowUp(int x, int y, bool inverted) {
  uint16_t fg = inverted ? TFT_WHITE : TFT_BLACK;
  M5.Display.fillTriangle(x + 9, y + 2, x + 3, y + 11, x + 15, y + 11, fg);
  M5.Display.fillRect(x + 7, y + 11, 4, 5, fg);
}

void iconArrowDown(int x, int y, bool inverted) {
  uint16_t fg = inverted ? TFT_WHITE : TFT_BLACK;
  M5.Display.fillTriangle(x + 9, y + 16, x + 3, y + 7, x + 15, y + 7, fg);
  M5.Display.fillRect(x + 7, y + 2, 4, 5, fg);
}

void iconRotateOut(int x, int y, bool inverted) {
  uint16_t fg = inverted ? TFT_WHITE : TFT_BLACK;
  M5.Display.drawArc(x + 9, y + 9, 5, 7, 40, 320, fg);
  M5.Display.fillTriangle(x + 16, y + 3, x + 16, y + 9, x + 10, y + 5, fg);
}

void iconRotateIn(int x, int y, bool inverted) {
  uint16_t fg = inverted ? TFT_WHITE : TFT_BLACK;
  M5.Display.drawArc(x + 9, y + 9, 5, 7, 220, 500, fg);
  M5.Display.fillTriangle(x + 2, y + 3, x + 2, y + 9, x + 8, y + 5, fg);
}

void drawGestureIcon(GestureId id, int x, int y, bool inverted) {
  M5.Display.drawRoundRect(x, y, 18, 18, 2, inverted ? TFT_WHITE : TFT_BLACK);
  switch (id) {
    case GESTURE_UP: iconArrowUp(x, y, inverted); break;
    case GESTURE_DOWN: iconArrowDown(x, y, inverted); break;
    case GESTURE_ROTATE_OUT: iconRotateOut(x, y, inverted); break;
    case GESTURE_ROTATE_IN: iconRotateIn(x, y, inverted); break;
    default: break;
  }
}

void drawGestureRow(int y, GestureId id, const char* name, const char* action, bool highlight) {
  const int rowH = 26;
  const int x0 = 6;
  const int rowW = SCR_W - 12;
  if (highlight) {
    M5.Display.fillRoundRect(x0, y, rowW, rowH, 3, TFT_BLACK);
  } else {
    M5.Display.drawRoundRect(x0, y, rowW, rowH, 3, TFT_BLACK);
  }
  drawGestureIcon(id, x0 + 5, y + 4, highlight);
  M5.Display.setTextColor(highlight ? TFT_WHITE : TFT_BLACK, highlight ? TFT_BLACK : TFT_WHITE);
  M5.Display.setTextSize(1);
  M5.Display.setCursor(x0 + 30, y + 4);
  M5.Display.print(name);
  M5.Display.setCursor(x0 + 30, y + 15);
  M5.Display.print(action);
}

// --- Listen indicator ---
const int LISTEN_DOT_Y = 190;
const int LISTEN_DOT_X0 = 150;
const int LISTEN_DOT_GAP = 12;
const int LISTEN_DOT_R = 3;
bool listenIndicatorActive = false;
unsigned long lastListenBlinkAt = 0;
const unsigned long LISTEN_BLINK_MS = 700;
int listenIndicatorPhase = 0;

void resetListenIndicator() {
  listenIndicatorActive = false; listenIndicatorPhase = 0; lastListenBlinkAt = 0;
}

void drawListenIndicatorFrame() {
  for (int i = 0; i < 3; i++) {
    int cx = LISTEN_DOT_X0 + i * LISTEN_DOT_GAP;
    M5.Display.drawCircle(cx, LISTEN_DOT_Y, LISTEN_DOT_R, TFT_BLACK);
  }
}

void tickListenIndicator() {
  if (appState != STATE_GESTURE) { listenIndicatorActive = false; return; }
  unsigned long now = millis();
  if (listenIndicatorActive && now - lastListenBlinkAt < LISTEN_BLINK_MS) return;
  lastListenBlinkAt = now;
  listenIndicatorActive = true;
  listenIndicatorPhase = (listenIndicatorPhase + 1) % 4;
  int x0 = LISTEN_DOT_X0 - LISTEN_DOT_R - 1;
  int w = LISTEN_DOT_GAP * 2 + LISTEN_DOT_R * 2 + 2;
  int y0 = LISTEN_DOT_Y - LISTEN_DOT_R - 1;
  int h = LISTEN_DOT_R * 2 + 2;
  epdFastMode();
  M5.Display.setClipRect(x0, y0, w, h);
  M5.Display.fillRect(x0, y0, w, h, TFT_WHITE);
  for (int i = 0; i < 3; i++) {
    int cx = LISTEN_DOT_X0 + i * LISTEN_DOT_GAP;
    bool filled = i < listenIndicatorPhase;
    if (filled) M5.Display.fillCircle(cx, LISTEN_DOT_Y, LISTEN_DOT_R, TFT_BLACK);
    else M5.Display.drawCircle(cx, LISTEN_DOT_Y, LISTEN_DOT_R, TFT_BLACK);
  }
  M5.Display.clearClipRect();
  epdFullMode();
}

// --- Screen drawing functions ---

void drawBoot(const char* title, const char* line1 = "", const char* line2 = "") {
  epdFullMode();
  M5.Display.fillScreen(TFT_WHITE);
  drawTopBar(title);
  M5.Display.setTextColor(TFT_BLACK, TFT_WHITE);
  M5.Display.setTextSize(1);
  M5.Display.setCursor(8, 34);
  M5.Display.println(line1);
  M5.Display.setCursor(8, 50);
  M5.Display.println(line2);
}

void drawIdle(const char* status = "Mexe o pulso") {
  epdFullMode();
  M5.Display.fillScreen(TFT_WHITE);
  drawTopBar("OmniBand");

  M5.Display.setTextColor(TFT_BLACK, TFT_WHITE);
  M5.Display.setTextSize(1);
  M5.Display.setCursor(8, 30);
  M5.Display.print(WiFi.status() == WL_CONNECTED ? "WiFi: ligado" : "WiFi: sem ligacao");

  M5.Display.setCursor(8, 44);
  M5.Display.print("Estado: ");
  M5.Display.print(status);

  M5.Display.drawLine(0, 60, SCR_W, 60, TFT_BLACK);

  M5.Display.setCursor(8, 70);
  M5.Display.println("Como usar:");
  M5.Display.setCursor(8, 86);  M5.Display.println("1. Mexe o pulso");
  M5.Display.setCursor(8, 100); M5.Display.println("2. Palmas: 1=corredor");
  M5.Display.setCursor(8, 114); M5.Display.println("   2=sala  3=quarto");
  M5.Display.setCursor(8, 128); M5.Display.println("3. Faz o gesto");

  M5.Display.drawLine(0, 145, SCR_W, 145, TFT_BLACK);
  M5.Display.setCursor(8, 155);
  M5.Display.println("Btn: recalibrar");

  drawFooterHint("Segura btn 2s: teste ML");
}

void drawContext(const char* status) {
  epdFullMode();
  M5.Display.fillScreen(TFT_WHITE);
  drawTopBar("A ouvir palmas");

  M5.Display.setTextColor(TFT_BLACK, TFT_WHITE);
  M5.Display.setTextSize(1);
  M5.Display.setCursor(8, 30);
  M5.Display.print(status);

  const int boxSize = 36;
  const int gap = 10;
  const int totalW = boxSize * 3 + gap * 2;
  const int startX = (SCR_W - totalW) / 2;
  const int boxY = 50;
  const char* labels[3] = {"cor", "sala", "qto"};

  for (int i = 0; i < 3; i++) {
    int x = startX + i * (boxSize + gap);
    bool filled = i < clapCount;
    if (filled) M5.Display.fillRoundRect(x, boxY, boxSize, boxSize, 4, TFT_BLACK);
    else M5.Display.drawRoundRect(x, boxY, boxSize, boxSize, 4, TFT_BLACK);
    M5.Display.setTextColor(filled ? TFT_WHITE : TFT_BLACK, filled ? TFT_BLACK : TFT_WHITE);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(x + boxSize / 2 - 6, boxY + 8);
    M5.Display.print(i + 1);
    M5.Display.setTextSize(1);
    M5.Display.setCursor(x + 4, boxY + boxSize + 4);
    M5.Display.print(labels[i]);
  }

  M5.Display.setTextColor(TFT_BLACK, TFT_WHITE);
  M5.Display.setCursor(8, 110);
  M5.Display.print("1 corredor  2 sala  3 quarto");

  M5.Display.drawLine(0, 130, SCR_W, 130, TFT_BLACK);
  M5.Display.setCursor(8, 140);
  M5.Display.print("Limiar mic: ");
  M5.Display.println(micThreshold);

  drawFooterHint("Fica quieto entre palmas");
}

void drawGestureWait() {
  epdFullMode();
  M5.Display.fillScreen(TFT_WHITE);

  char rightLabel[24];
  snprintf(rightLabel, sizeof(rightLabel), "%d palma%s", activeClaps, activeClaps == 1 ? "" : "s");
  String roomUpper = String(activeRoom);
  roomUpper.toUpperCase();
  drawTopBar(roomUpper.c_str(), rightLabel);

  int y = 28;
  const int rowGap = 5;
  drawGestureRow(y, GESTURE_UP, "CIMA", "aumentar", false); y += 26 + rowGap;
  drawGestureRow(y, GESTURE_DOWN, "BAIXO", "diminuir", false); y += 26 + rowGap;
  drawGestureRow(y, GESTURE_ROTATE_OUT, "ROD. FORA", "ligar", false); y += 26 + rowGap;
  drawGestureRow(y, GESTURE_ROTATE_IN, "ROD. DENTRO", "desligar", false);

  drawListenIndicatorFrame();
  resetListenIndicator();
  drawFooterHint("A espera do gesto...");
}

void drawGestureResult(const GestureInfo& info, bool sent, int httpCode) {
  epdFullMode();
  M5.Display.fillScreen(TFT_WHITE);
  drawTopBar(sent ? "Enviado" : "Falhou");

  M5.Display.setTextColor(TFT_BLACK, TFT_WHITE);
  M5.Display.setTextSize(1);
  M5.Display.setCursor(8, 30);
  M5.Display.print("Contexto: ");
  M5.Display.println(activeRoom);

  M5.Display.drawRoundRect(8, 46, SCR_W - 16, 40, 4, TFT_BLACK);
  drawGestureIcon(info.id, 16, 57, false);
  M5.Display.setCursor(42, 56);
  M5.Display.setTextSize(1);
  M5.Display.print(info.title);
  M5.Display.setCursor(42, 70);
  M5.Display.print(info.action);

  M5.Display.setCursor(8, 96);
  if (sent) M5.Display.println("Servidor recebeu o comando");
  else if (WiFi.status() != WL_CONNECTED) M5.Display.println("Falha: WiFi desligado");
  else M5.Display.println("Falha no envio HTTP");

  M5.Display.setCursor(8, 112);
  M5.Display.print("HTTP: ");
  M5.Display.println(httpCode);

  drawFooterHint("A voltar ao inicio...");
}

// =====================================================================
// ML Test Screen (from teste_ml_gesto_unico.ino)
// =====================================================================

void drawMsg(const char* title, const char* line1, const char* line2) {
  M5.Display.setEpdMode(epd_fastest);
  M5.Display.fillScreen(TFT_WHITE);
  M5.Display.fillRect(0, 0, 200, 22, TFT_BLACK);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(6, 4);
  M5.Display.print(title);
  M5.Display.setTextColor(TFT_BLACK, TFT_WHITE);
  M5.Display.setTextSize(1);
  if (line1) { M5.Display.setCursor(8, 40); M5.Display.println(line1); }
  if (line2) { M5.Display.setCursor(8, 56); M5.Display.println(line2); }
}

void drawMLResult(ei_impulse_result_t& result, const char* bestLabel, float bestScore,
                  float peakAccel, float peakGyro) {
  M5.Display.setEpdMode(epd_text);
  M5.Display.fillScreen(TFT_WHITE);

  bool confident = bestScore >= ML_CONFIDENCE_OK;
  M5.Display.fillRect(0, 0, 200, 22, confident ? TFT_BLACK : TFT_WHITE);
  M5.Display.setTextColor(confident ? TFT_WHITE : TFT_BLACK,
                           confident ? TFT_BLACK : TFT_WHITE);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(6, 4);
  M5.Display.print(confident ? "ML OK" : "ML baixo");

  M5.Display.fillRoundRect(8, 30, 184, 50, 6, TFT_BLACK);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.setTextSize(3);
  M5.Display.setCursor(20, 44);
  M5.Display.print(bestLabel);
  M5.Display.setTextSize(1);
  M5.Display.setCursor(20, 68);
  M5.Display.printf("conf: %.0f%%", bestScore * 100.0f);

  M5.Display.setTextColor(TFT_BLACK, TFT_WHITE);
  M5.Display.setTextSize(1);
  M5.Display.setCursor(8, 92);
  M5.Display.println("Scores ML:");
  for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    int y = 106 + i * 16;
    float score = result.classification[i].value;
    const char* lbl = result.classification[i].label;
    int barLen = (int)(score * 130.0f);
    M5.Display.fillRect(8, y + 8, barLen, 6, TFT_BLACK);
    M5.Display.drawRect(8, y + 8, 130, 6, TFT_BLACK);
    M5.Display.setCursor(144, y + 4);
    M5.Display.printf("%s %.2f", lbl, score);
  }

  int diagY = 108 + EI_CLASSIFIER_LABEL_COUNT * 16 + 8;
  M5.Display.drawLine(0, diagY, 200, diagY, TFT_BLACK);
  M5.Display.setCursor(8, diagY + 6);
  M5.Display.printf("pico accel: %.2f m/s2", peakAccel);
  M5.Display.setCursor(8, diagY + 20);
  M5.Display.printf("pico gyro:  %.1f dps", peakGyro);
  M5.Display.setCursor(8, diagY + 36);
  M5.Display.println("A voltar...");
}

// =====================================================================
// State Machine Functions
// =====================================================================

void goIdle(const char* status) {
  appState = STATE_IDLE; stateStartedAt = millis();
  firstClapAt = 0; lastClapAt = 0; lastMicSampleAt = 0; clapCount = 0; activeClaps = 0; wakeHitCount = 0;
  activeRoom = "sem contexto"; returnToIdleAt = 0; drawIdle(status);
}

void startContext() {
  appState = STATE_CONTEXT; stateStartedAt = millis();
  firstClapAt = 0; lastClapAt = 0; lastMicSampleAt = 0; clapCount = 0; wakeHitCount = 0; lastWakeAt = millis();
  Serial.println("\n[wake] ---> ACORDOU! A ouvir palmas...");
  drawContext(micReady ? "A ouvir palmas" : "Mic suspeito");
}

void finalizeContext() {
  if (clapCount <= 0) { goIdle("Sem palmas"); return; }
  activeClaps = clapCount; activeRoom = roomFromClaps(clapCount);
  appState = STATE_GESTURE; stateStartedAt = millis(); lastGestureAt = 0;
  Serial.printf("\n[context] FECHADO: palmas=%d room=%s\n", activeClaps, activeRoom);
  drawGestureWait();
}

void printPlotter(int p2p, int avg, int threshold) {
  if (millis() - lastPlotterAt < 50) return;
  lastPlotterAt = millis();
  Serial.print("Volume_P2P:"); Serial.print(p2p);
  Serial.print(",Media_Base:"); Serial.print(avg);
  Serial.print(",Limiar_Adapt:"); Serial.println(threshold);
}

void processContext() {
  unsigned long now = millis();

  if (clapCount == 0 && now - stateStartedAt > CONTEXT_WAIT_TIMEOUT_MS) { goIdle("Sem palmas"); return; }

  if (clapCount > 0) {
    if (now - lastClapAt > CLAP_GAP_MS) { finalizeContext(); return; }
    if (now - firstClapAt > CONTEXT_WINDOW_MS) { finalizeContext(); return; }
  }

  if (now - stateStartedAt < MIC_SETTLE_AFTER_WAKE_MS) return;
  if (now - lastMicSampleAt < MIC_SAMPLE_INTERVAL_MS) return;
  lastMicSampleAt = now;

  MicStats mic = sampleMic(MIC_SAMPLE_MS);
  int dynamicThreshold = max(MIN_CLAP_ABSOLUTE, (int)(adaptiveNoiseFloor * CLAP_RATIO_THRESHOLD));
  micThreshold = dynamicThreshold;
  printPlotter(mic.peakToPeak, mic.average, dynamicThreshold);

  bool isClap = mic.peakToPeak >= dynamicThreshold && !mic.suspicious;

  if (!isClap && !mic.suspicious) {
    float alpha = (mic.peakToPeak > adaptiveNoiseFloor) ? NOISE_RISE_ALPHA : NOISE_FALL_ALPHA;
    adaptiveNoiseFloor = alpha * (float)mic.peakToPeak + (1.0f - alpha) * adaptiveNoiseFloor;
    adaptiveNoiseFloor = constrain(adaptiveNoiseFloor, 0.0f, (float)NOISE_FLOOR_CAP);
  }

  if (mic.suspicious) return;

  if (isClap) {
    if (now - lastClapAt < CLAP_DEBOUNCE_MS) return;
    if (clapCount == 0) firstClapAt = now;
    lastClapAt = now;
    clapCount = min(clapCount + 1, MAX_CONTEXT_CLAPS);
    Serial.printf("\n PALMA CONFIRMADA: %d\n", clapCount);
    if (clapCount >= MAX_CONTEXT_CLAPS) finalizeContext();
  }
}

void processGesture(const ImuReading& reading, float accelDelta, float accelZDelta) {
  if (millis() - stateStartedAt > GESTURE_WINDOW_MS) { goIdle("Sem gesto"); return; }
  GestureId gestureId = detectGesture(reading, accelDelta, accelZDelta);
  if (gestureId == GESTURE_NONE) return;
  lastGestureAt = millis();
  GestureInfo info = gestureInfo(gestureId);
  Serial.printf("[gesture] Gesto enviado: %s\n", info.payload);
  drawBoot("Enviar", info.title, activeRoom);
  int httpCode = -1;
  bool sent = postFullGesture(info, httpCode);
  drawGestureResult(info, sent, httpCode);
  appState = STATE_RESULT; returnToIdleAt = millis() + RESULT_RETURN_MS;
}

void recoverImuIfNeeded() {
  if (imuFaultCount < 3) return;
  imuReady = initImu();
  if (imuReady) { calibrateZero(); goIdle("IMU recuperado"); }
}

void maybeReturnToIdle() {
  if (appState != STATE_RESULT || returnToIdleAt == 0) return;
  if (millis() >= returnToIdleAt) goIdle("Mexe o pulso");
}

void printImuStatus(const ImuReading& r, float accelDelta, float gyroAbs) {
  if (appState == STATE_CONTEXT) return;
  if (millis() - lastImuSerialAt < 1000) return;
  lastImuSerialAt = millis();
  Serial.printf("[imu] dA=%.2f gAbs=%.1f\n", accelDelta, gyroAbs);
}

// =====================================================================
// ML Test — from teste_ml_gesto_unico.ino, adapted to run on demand
// =====================================================================

void runMLDiagnostic() {
  appState = STATE_ML_TEST;

  drawMsg("Teste ML", "Prepara...", "Vai comecar!");
  Serial.println("\n===== ML Diagnostic Test =====");
  Serial.printf("  Labels: %d\n", EI_CLASSIFIER_LABEL_COUNT);
  for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    Serial.printf("  [%d] %s\n", i, ei_classifier_inferencing_categories[i]);
  }

  // Wait and prompt user
  drawMsg("PREPARA", "Aguarda... NAO faças", "o gesto ainda.");
  delay(PRE_CAPTURE_MS);

  drawMsg("JA!", "Faz o gesto", "UMA VEZ, agora!");
  Serial.println("[ml] JA! -> recolha a comecar.");

  // Capture 1s at 100Hz (same as original ML test)
  const int numSamples = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE / 6;
  static float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];

  float peakAccel = 0.0f, peakGyro = 0.0f;

  for (int i = 0; i < numSamples; i++) {
    unsigned long t0 = millis();

    accel->readSensor();
    gyro->readSensor();

    int base = i * 6;
    float ax = accel->getAccelX_mss();
    float ay = accel->getAccelY_mss();
    float az = accel->getAccelZ_mss();
    float gx = gyro->getGyroX_rads() * RAD_TO_DEG_PER_SEC;
    float gy = gyro->getGyroY_rads() * RAD_TO_DEG_PER_SEC;
    float gz = gyro->getGyroZ_rads() * RAD_TO_DEG_PER_SEC;

    buffer[base + 0] = ax;
    buffer[base + 1] = ay;
    buffer[base + 2] = az;
    buffer[base + 3] = gx;
    buffer[base + 4] = gy;
    buffer[base + 5] = gz;

    float mag = sqrtf(ax*ax + ay*ay + az*az);
    if (mag > peakAccel) peakAccel = mag;
    float gmag = sqrtf(gx*gx + gy*gy + gz*gz);
    if (gmag > peakGyro) peakGyro = gmag;

    long remaining = 10L - (long)(millis() - t0);
    if (remaining > 0) delay(remaining);
  }

  Serial.println("[ok] Recolha concluida (1s).");
  Serial.printf("  Pico accel: %.3f m/s2  |  Pico gyro: %.1f dps\n", peakAccel, peakGyro);

  // Run inference
  drawMsg("A pensar...", "A classificar o gesto", nullptr);
  Serial.println("[ml] A correr classificador...");

  signal_t signal;
  int sigErr = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
  if (sigErr != 0) {
    drawMsg("ERRO", "signal_from_buffer falhou", nullptr);
    Serial.printf("[ml] ERRO signal_from_buffer: %d\n", sigErr);
    delay(3000);
    goIdle("ML erro");
    return;
  }

  ei_impulse_result_t result = {0};
  EI_IMPULSE_ERROR status = run_classifier(&signal, &result, false);
  if (status != EI_IMPULSE_OK) {
    drawMsg("ERRO", "run_classifier falhou", nullptr);
    Serial.printf("[ml] ERRO run_classifier: %d\n", (int)status);
    delay(3000);
    goIdle("ML erro");
    return;
  }

  // Parse results
  float bestScore = 0.0f;
  int   bestIdx   = 0;
  Serial.println("[ml] Resultados:");
  for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    float s = result.classification[i].value;
    const char* l = result.classification[i].label;
    Serial.printf("     %-10s %.4f  %s\n", l, s,
                  s >= ML_CONFIDENCE_OK ? "<-- ACIMA LIMIAR" : "");
    if (s > bestScore) { bestScore = s; bestIdx = i; }
  }

  if (peakAccel < 5.0f) {
    Serial.println("[AVISO] Pico de accel muito baixo (<5 m/s2).");
  }
  if (bestScore < ML_CONFIDENCE_OK) {
    Serial.printf("[AVISO] Confianca baixa (%.2f).\n", bestScore);
  }

  const char* bestLabel = result.classification[bestIdx].label;
  Serial.printf("\n>>> RESULTADO ML: %s (%.0f%%) <<<\n\n", bestLabel, bestScore * 100.0f);

  drawMLResult(result, bestLabel, bestScore, peakAccel, peakGyro);

  // Show result for 5 seconds, then return to idle
  delay(5000);
  goIdle("ML completo");
}

// =====================================================================
// Button Handling
// =====================================================================

void handleButton() {
  M5.update();

  if (M5.BtnA.wasHold()) {
    // Button was held for >= 500ms (default hold threshold)
    if (!btnHeld && appState == STATE_IDLE) {
      btnHeld = true;
      Serial.println("[btn] Hold detectado! A iniciar teste ML...");
      runMLDiagnostic();
    }
    return;
  }

  if (M5.BtnA.wasPressed()) {
    // Short press — recalibrate (only in IDLE or RESULT)
    if (appState == STATE_IDLE || appState == STATE_RESULT) {
      Serial.println("\n[btn] Recalibrar sensores");
      calibrateZero();
      calibrateMic();
      goIdle("Sensores calibrados");
    }
    return;
  }

  // Reset hold state when button is released
  if (btnHeld && !M5.BtnA.isPressed()) {
    btnHeld = false;
  }
}

// =====================================================================
// setup()
// =====================================================================

void setup() {
  auto cfg = M5.config(); cfg.clear_display = true; M5.begin(cfg);
  Serial.begin(115200); delay(500); M5.Display.setRotation(0);
  if (M5.Display.isEPD()) M5.Display.setColorDepth(8);
  epdFullMode();

  pinMode(MIC_ANALOG_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(MIC_ANALOG_PIN, ADC_11db);

  // Configure button hold time (long press detection)
  M5.BtnA.setHoldThresh(2000);  // 2 seconds for hold

  drawBoot("OmniBand", "A iniciar IMU...", "");
  imuReady = initImu();
  if (!imuReady) {
    drawBoot("ERRO", "IMU nao encontrado!", "Verifica I2C.");
    while (true) delay(1000);
  }

  drawBoot("OmniBand", "A calibrar...", "");
  calibrateZero();
  calibrateMic();
  connectWiFi();

  goIdle(WiFi.status() == WL_CONNECTED ? "Pronto" : "Sem WiFi");
}

// =====================================================================
// loop() — Integrated main loop
// =====================================================================

void loop() {
  handleButton();

  // If in ML test mode, do nothing (ML runs in its own blocking flow)
  if (appState == STATE_ML_TEST) {
    delay(100);
    return;
  }

  maybeReturnToIdle();

  ImuReading reading;
  if (!readImu(reading)) { recoverImuIfNeeded(); delay(50); return; }

  float accelDelta = accelDeltaFromPrevious(reading);
  float accelZDelta = accelZDeltaFromPrevious(reading);
  float gyroAbs = gyroMagnitudeDps(reading);

  printImuStatus(reading, accelDelta, gyroAbs);

  // ===================================================================
  // SIMPLE MODE INTEGRATION:
  // In IDLE state, also check for simple threshold gestures.
  // If detected, send quick command without context.
  // This integrates codigo_funciona_cima.txt into the same loop.
  // ===================================================================
  if (appState == STATE_IDLE) {
    if (millis() - simpleLastSend > SIMPLE_COOLDOWN_MS) {
      if (detectSimpleUp(reading)) {
        Serial.println("Gesto Cima (simples) detetado");
        if (postSimpleGesture("Cima")) {
          simpleLastSend = millis();
        }
      } else if (detectSimpleDown(reading)) {
        Serial.println("Gesto Baixo (simples) detetado");
        if (postSimpleGesture("Baixo")) {
          simpleLastSend = millis();
        }
      }
    }

    // Also check for wake gesture (full mode)
    if (detectWake(reading, accelDelta, gyroAbs)) {
      startContext();
    }
  }

  // ===================================================================
  // FULL STATE MACHINE
  // ===================================================================
  switch (appState) {
    case STATE_CONTEXT:
      processContext();
      break;
    case STATE_GESTURE:
      processGesture(reading, accelDelta, accelZDelta);
      break;
    case STATE_RESULT:
    default:
      break;
  }

  rememberImuReading(reading);

  // UI indicator
  tickListenIndicator();

  // Dynamic delay
  if (appState == STATE_CONTEXT) {
    delay(2);
  } else {
    delay(30);
  }
}