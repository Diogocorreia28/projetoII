// =====================================================================
// teste_ml_gesto_unico.ino
// =====================================================================
// Versão SIMPLIFICADA para um único teste rápido:
//   1. Carrega o sketch -> o dispositivo calibra e espera 1s
//   2. Ecrã mostra "JA!" -> faz o gesto UMA VEZ nesse instante
//   3. Ele recolhe 1s de dados, classifica, e mostra logo o resultado
//      (cima / baixo / parado) no ecrã e no Serial Monitor
//   4. FIM. Não há botões, não há loop de novos testes.
//      Para repetir o teste, volta a carregar o sketch (reset).
// =====================================================================

#include <M5Unified.h>
#include <Wire.h>
#include "BMI088.h"
#include <correia_2k-project-1_inferencing.h>

// --- Pinos I2C (mesmos do sketch principal) ---
const uint8_t I2C_PRIMARY_SDA  = 33;
const uint8_t I2C_PRIMARY_SCL  = 32;
const uint8_t I2C_FALLBACK_SDA = 32;
const uint8_t I2C_FALLBACK_SCL = 33;

// Tempo de preparação antes do "JA!" (não faças o gesto durante este tempo)
const unsigned long PRE_CAPTURE_MS = 1000;

// Confiança mínima — só para colorir o resultado
const float CONFIDENCE_OK = 0.65f;

// --- BMI088 ---
Bmi088Accel accelDefault(Wire, 0x19);
Bmi088Gyro  gyroDefault (Wire, 0x69);
Bmi088Accel accelAlt    (Wire, 0x18);
Bmi088Gyro  gyroAlt     (Wire, 0x68);

Bmi088Accel* accel = &accelDefault;
Bmi088Gyro*  gyro  = &gyroDefault;

// =====================================================================
// IMU — init e candidatos de endereço/pinos
// =====================================================================
bool beginImuCandidate(uint8_t sda, uint8_t scl,
                       Bmi088Accel& a, Bmi088Gyro& g, const char* tag) {
  Wire.end(); delay(40);
  Wire.begin(sda, scl); Wire.setClock(400000);
  if (a.begin() < 0 || g.begin() < 0) return false;
  accel = &a; gyro = &g;
  Serial.printf("[imu] BMI088 OK (%s) SDA=G%d SCL=G%d\n", tag, sda, scl);
  return true;
}

bool initImu() {
  if (beginImuCandidate(I2C_PRIMARY_SDA,  I2C_PRIMARY_SCL,  accelDefault, gyroDefault, "0x19/0x69")) return true;
  if (beginImuCandidate(I2C_PRIMARY_SDA,  I2C_PRIMARY_SCL,  accelAlt,     gyroAlt,     "0x18/0x68")) return true;
  if (beginImuCandidate(I2C_FALLBACK_SDA, I2C_FALLBACK_SCL, accelDefault, gyroDefault, "0x19/0x69-fb")) return true;
  if (beginImuCandidate(I2C_FALLBACK_SDA, I2C_FALLBACK_SCL, accelAlt,     gyroAlt,     "0x18/0x68-fb")) return true;
  return false;
}

// =====================================================================
// UI — CoreInk (e-paper 200×200)
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

void drawResult(ei_impulse_result_t& result, const char* bestLabel, float bestScore,
                float peakAccel, float peakGyro) {
  M5.Display.setEpdMode(epd_text);
  M5.Display.fillScreen(TFT_WHITE);

  bool confident = bestScore >= CONFIDENCE_OK;
  M5.Display.fillRect(0, 0, 200, 22, confident ? TFT_BLACK : TFT_WHITE);
  M5.Display.setTextColor(confident ? TFT_WHITE : TFT_BLACK,
                           confident ? TFT_BLACK : TFT_WHITE);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(6, 4);
  M5.Display.print(confident ? "OK" : "?? baixo");

  // Resultado em destaque (grande, no centro)
  M5.Display.fillRoundRect(8, 30, 184, 50, 6, TFT_BLACK);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.setTextSize(3);
  M5.Display.setCursor(20, 44);
  M5.Display.print(bestLabel);
  M5.Display.setTextSize(1);
  M5.Display.setCursor(20, 68);
  M5.Display.printf("confianca: %.0f%%", bestScore * 100.0f);

  // Scores de todas as labels
  M5.Display.setTextColor(TFT_BLACK, TFT_WHITE);
  M5.Display.setTextSize(1);
  M5.Display.setCursor(8, 92);
  M5.Display.println("Scores:");
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
  M5.Display.println("Reset para repetir o teste.");
}

// =====================================================================
// Setup — faz TUDO uma vez: calibra, avisa, recolhe, classifica, mostra
// =====================================================================
void setup() {
  auto cfg = M5.config();
  cfg.clear_display = true;
  M5.begin(cfg);
  Serial.begin(115200);
  delay(500);

  M5.Display.setRotation(0);
  if (M5.Display.isEPD()) M5.Display.setColorDepth(8);
  M5.Display.setEpdMode(epd_text);

  Serial.println("\n========================================");
  Serial.println("  Teste ML (gesto unico) — correia_2k-project-1");
  Serial.println("========================================");
  Serial.printf("  Labels: %d\n", EI_CLASSIFIER_LABEL_COUNT);
  for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    Serial.printf("  [%d] %s\n", i, ei_classifier_inferencing_categories[i]);
  }

  // --- Iniciar IMU ---
  drawMsg("Teste ML", "A iniciar IMU...", nullptr);
  if (!initImu()) {
    drawMsg("ERRO", "IMU nao encontrado!", "Verifica cabos I2C.");
    Serial.println("[ERRO] IMU nao encontrado. Verifica cabos I2C.");
    while (true) delay(1000);
  }

  // --- Preparação: 1s de aviso ANTES do gesto ---
  drawMsg("PREPARA", "Aguarda... NAO faças", "o gesto ainda.");
  Serial.println("[info] Prepara o pulso. Gesto comeca quando aparecer JA!");
  delay(PRE_CAPTURE_MS);

  // --- Sinal único: JA! Faz o gesto agora ---
  drawMsg("JA!", "Faz o gesto", "UMA VEZ, agora!");
  Serial.println("[info] JA! -> recolha a comecar.");

  // --- Recolha de 1s a 100Hz ---
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
    float gx = gyro->getGyroX_rads() * RAD_TO_DEG;
    float gy = gyro->getGyroY_rads() * RAD_TO_DEG;
    float gz = gyro->getGyroZ_rads() * RAD_TO_DEG;

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

  // --- Inferência ---
  drawMsg("A pensar...", "A classificar o gesto", nullptr);
  Serial.println("[ml] A correr classificador...");

  signal_t signal;
  int sigErr = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
  if (sigErr != 0) {
    drawMsg("ERRO", "signal_from_buffer falhou", nullptr);
    Serial.printf("[ml] ERRO signal_from_buffer: %d\n", sigErr);
    return;
  }

  ei_impulse_result_t result = {0};
  EI_IMPULSE_ERROR status = run_classifier(&signal, &result, false);
  if (status != EI_IMPULSE_OK) {
    drawMsg("ERRO", "run_classifier falhou", nullptr);
    Serial.printf("[ml] ERRO run_classifier: %d\n", (int)status);
    return;
  }

  // --- Resultados ---
  float bestScore = 0.0f;
  int   bestIdx   = 0;
  Serial.println("[ml] Resultados:");
  for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    float s = result.classification[i].value;
    const char* l = result.classification[i].label;
    Serial.printf("     %-10s %.4f  %s\n", l, s,
                  s >= CONFIDENCE_OK ? "<-- ACIMA LIMIAR" : "");
    if (s > bestScore) { bestScore = s; bestIdx = i; }
  }

  if (peakAccel < 5.0f) {
    Serial.println("[AVISO] Pico de accel muito baixo (<5 m/s2). Sensor pode nao estar a ler bem.");
  }
  if (bestScore < CONFIDENCE_OK) {
    Serial.printf("[AVISO] Confianca baixa (%.2f).\n", bestScore);
  }

  const char* bestLabel = result.classification[bestIdx].label;
  Serial.printf("\n>>> RESULTADO: %s (%.0f%%) <<<\n\n", bestLabel, bestScore * 100.0f);

  drawResult(result, bestLabel, bestScore, peakAccel, peakGyro);
}

// =====================================================================
// Loop — não faz nada; é um teste de UM ÚNICO gesto.
// Para repetir, faz reset ao dispositivo (botão de reset ou desliga/liga).
// =====================================================================
void loop() {
  delay(1000);
}
