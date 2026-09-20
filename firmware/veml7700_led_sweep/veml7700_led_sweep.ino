/*
 * Caracterizacion de LED con VEML7700  -  BARRIDO AUTOMATICO 0->100->0
 *
 * Comandos por Serial (115200 baud, terminar con Enter):
 *     RUN           ->  barrido completo 0->100->0 con paso SWEEP_STEP%
 *                       (etiqueta automaticamente u0..u100 y d100..d0)
 *     STOP          ->  aborta el sweep al final del nivel actual
 *     50  | u50 | d50  ->  un solo nivel (con prefijo opcional u/d para histeresis)
 *
 * Cada nivel:  setPWM  ->  estabilizacion STABILIZE_MS  ->  N_SAMPLES medidas
 * El LED queda apagado al terminar el sweep.
 *
 * CSV: timestamp_ms,burst_id,sample_idx,brightness,pct,pwm,lux,als_raw,white_raw
 *
 * Hardware: LED en pin 3 con resistencia limitadora. VEML7700 + LCD en I2C
 *           con pull-ups externos (montaje validado), Wire a 50 kHz.
 */
#include <Wire.h>
#include "Adafruit_VEML7700.h"
#include <LiquidCrystal_I2C.h>

Adafruit_VEML7700 veml = Adafruit_VEML7700();
LiquidCrystal_I2C lcd(0x27, 16, 2);   // cambia a 0x3F si tu LCD usa esa

// ---------- Parametros del experimento ----------
const uint8_t  LED_PIN      = 3;
const uint16_t N_SAMPLES    = 30;
const uint16_t INTERVAL_MS  = 120;
const uint16_t STABILIZE_MS = 3000;
const uint8_t  SWEEP_STEP   = 10;     // paso del barrido en %  (10 -> 22 niveles totales)

uint32_t burstId = 0;
String   lastLabel = "NA";
int      currentPct = 0;
int      currentPwm = 0;
float    lastAvg = NAN;
unsigned long lastIdleRefresh = 0;
bool     abortSweep = false;

void lcdLine(uint8_t row, const String &s) {
  String t = s;
  while (t.length() < 16) t += ' ';
  if (t.length() > 16) t = t.substring(0, 16);
  lcd.setCursor(0, row);
  lcd.print(t);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  pinMode(LED_PIN, OUTPUT);
  analogWrite(LED_PIN, 0);

  Wire.begin();
  Wire.setClock(50000);

  lcd.init();
  lcd.backlight();
  lcdLine(0, "Iniciando...");

  if (!veml.begin()) {
    Serial.println("# ERROR: VEML7700 no detectado (revisa I2C/pull-ups/0x10)");
    lcdLine(0, "VEML no detect.");
    lcdLine(1, "Revisa I2C/pullup");
    while (1) { delay(100); }
  }
  veml.setGain(VEML7700_GAIN_1_8);
  veml.setIntegrationTime(VEML7700_IT_100MS);
  delay(300);

  Serial.println("timestamp_ms,burst_id,sample_idx,brightness,pct,pwm,lux,als_raw,white_raw");
  Serial.println("# Comandos: RUN (sweep 0->100->0), STOP (abortar), o un % suelto (ej. 50, u30, d70).");

  lcdLine(0, "Listo");
  lcdLine(1, "RUN o % manual");
}

// ---------- Verifica si llego un STOP por serial (no bloquea) ----------
bool serialHasStop() {
  while (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s.equalsIgnoreCase("STOP") || s.equalsIgnoreCase("ABORT")) return true;
  }
  return false;
}

// ---------- Un nivel completo: PWM -> estabiliza -> rafaga ----------
void runLevel(int pct, char dir) {
  int pwm = (int)round(pct * 255.0 / 100.0);
  String label = String(pct);
  if (dir != '\0') label = String(dir) + label;

  analogWrite(LED_PIN, pwm);
  currentPct = pct;
  currentPwm = pwm;
  lastLabel  = label;

  int seconds = STABILIZE_MS / 1000;
  for (int s = seconds; s > 0; s--) {
    lcdLine(0, "PWM:" + String(pwm) + " (" + label + ")");
    lcdLine(1, "Estabiliza " + String(s) + "s");
    Serial.print("# Estabilizando "); Serial.print(s); Serial.println(" s");
    delay(1000);
  }
  runBurst(label, pct, pwm);
}

// ---------- Barrido automatico 0->100->0 ----------
void runSweep() {
  abortSweep = false;
  Serial.print("# === Sweep iniciado (paso ");
  Serial.print(SWEEP_STEP); Serial.println("%) ===");

  // Subida: 0, STEP, 2*STEP, ..., 100
  for (int p = 0; p <= 100; p += SWEEP_STEP) {
    runLevel(p, 'u');
    if (serialHasStop()) { abortSweep = true; break; }
  }

  // Bajada: 100, 100-STEP, ..., 0   (endpoints repetidos a proposito: histeresis)
  if (!abortSweep) {
    for (int p = 100; p >= 0; p -= SWEEP_STEP) {
      runLevel(p, 'd');
      if (serialHasStop()) { abortSweep = true; break; }
    }
  }

  analogWrite(LED_PIN, 0);
  currentPct = 0; currentPwm = 0;
  Serial.println(abortSweep ? "# === Sweep ABORTADO. LED apagado. ==="
                            : "# === Sweep terminado. LED apagado. ===");
  lcdLine(0, abortSweep ? "Sweep abortado" : "Sweep terminado");
  lcdLine(1, "RUN o % manual");
}

// ---------- Comando manual de un solo nivel ----------
void runManual(const String &cmd) {
  char dir = '\0';
  String num = cmd;
  if (num.length() > 0 && (num[0]=='u'||num[0]=='U'||num[0]=='d'||num[0]=='D')) {
    dir = (num[0]=='u'||num[0]=='U') ? 'u' : 'd';
    num = num.substring(1);
  }
  float pct = num.toFloat();
  pct = constrain(pct, 0.0f, 100.0f);
  runLevel((int)pct, dir);
  Serial.println("# Rafaga manual terminada.");
}

void loop() {
  // Indicador en vivo al esperar
  if (millis() - lastIdleRefresh >= 500) {
    float lux = veml.readLux(VEML_LUX_CORRECTED_NOWAIT);
    lcdLine(0, "Lux:" + String(lux, 1) + " " + String(currentPct) + "%");
    if (!isnan(lastAvg))
      lcdLine(1, "Avg:" + String(lastAvg, 1) + " (" + lastLabel + ")");
    else
      lcdLine(1, "RUN o % manual");
    lastIdleRefresh = millis();
  }

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() == 0) return;

    if (cmd.equalsIgnoreCase("RUN") || cmd.equalsIgnoreCase("SWEEP")) {
      runSweep();
    } else if (cmd.equalsIgnoreCase("STOP") || cmd.equalsIgnoreCase("ABORT")) {
      Serial.println("# (no hay sweep en curso)");
    } else {
      runManual(cmd);
    }
    lastIdleRefresh = 0;
  }
}

void runBurst(const String &label, int pct, int pwm) {
  burstId++;
  double sum = 0.0;
  uint32_t scheduled = millis();

  for (uint16_t i = 0; i < N_SAMPLES; i++) {
    while ((int32_t)(millis() - scheduled) < 0) { ; }

    uint32_t ts    = millis();
    float    lux   = veml.readLux(VEML_LUX_CORRECTED_NOWAIT);
    uint16_t als   = veml.readALS();
    uint16_t white = veml.readWhite();
    sum += lux;

    Serial.print(ts);      Serial.print(',');
    Serial.print(burstId); Serial.print(',');
    Serial.print(i);       Serial.print(',');
    Serial.print(label);   Serial.print(',');
    Serial.print(pct);     Serial.print(',');
    Serial.print(pwm);     Serial.print(',');
    Serial.print(lux, 4);  Serial.print(',');
    Serial.print(als);     Serial.print(',');
    Serial.println(white);

    lcdLine(0, label + "  " + String(i + 1) + "/" + String(N_SAMPLES));
    lcdLine(1, "Lux:" + String(lux, 1) + " PWM:" + String(pwm));

    scheduled += INTERVAL_MS;
  }
  lastAvg = (float)(sum / N_SAMPLES);
  lcdLine(0, label + " listo");
  lcdLine(1, "Avg:" + String(lastAvg, 1));
}
