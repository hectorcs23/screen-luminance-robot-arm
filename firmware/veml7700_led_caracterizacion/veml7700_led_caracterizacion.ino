/*
 * Caracterizacion de LED con VEML7700  (Arduino UNO R4 WiFi)
 *
 * LED en pin 3 (PWM). Envia el porcentaje (0..100) por Serial + Enter:
 *     50      -> PWM = 128, ejecuta una rafaga
 *     u50     -> igual, etiquetado como "subiendo" (para histeresis)
 *     d50     -> igual, etiquetado como "bajando"
 *
 * Flujo de cada comando:
 *   1) Mapea %  ->  PWM (0..255) y aplica analogWrite() al LED.
 *   2) Espera STABILIZE_MS (cuenta regresiva en LCD + Serial).
 *   3) Toma N_SAMPLES mediciones espaciadas INTERVAL_MS y las imprime en CSV.
 *   4) Deja el LED encendido al ultimo nivel hasta el siguiente comando.
 *
 * CSV: timestamp_ms,burst_id,sample_idx,brightness,pct,pwm,lux,als_raw,white_raw
 *      (la columna `brightness` es la etiqueta tal cual: "50", "u50", "d50"...)
 *
 * Librerias: Adafruit_VEML7700 | LiquidCrystal_I2C | Wire
 * Cableado: SDA/SCL compartidos por VEML7700 y LCD con pull-ups externos;
 *           LED en pin 3 con su resistencia limitadora a GND.
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
const uint16_t STABILIZE_MS = 3000;   // espera tras cambiar el PWM (estabilizacion termica)

uint32_t burstId = 0;
String   lastLabel = "NA";
int      currentPct = 0;
int      currentPwm = 0;
float    lastAvg = NAN;
unsigned long lastIdleRefresh = 0;

// Linea completa de 16 chars (sin parpadeo, sin lcd.clear)
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
  analogWrite(LED_PIN, 0);   // arranca con el LED apagado

  Wire.begin();
  Wire.setClock(50000);      // 50 kHz: tolerante al ruido del bus compartido

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
  Serial.println("# LED en pin 3. Envia % (0..100) + Enter. Prefija u/d para histeresis (u50, d50).");

  lcdLine(0, "Listo");
  lcdLine(1, "Envia % (0-100)");
}

void loop() {
  // Indicador en vivo mientras espera
  if (millis() - lastIdleRefresh >= 500) {
    float lux = veml.readLux(VEML_LUX_CORRECTED_NOWAIT);
    lcdLine(0, "Lux:" + String(lux, 1) + " " + String(currentPct) + "%");
    if (!isnan(lastAvg))
      lcdLine(1, "Avg:" + String(lastAvg, 1) + " (" + lastLabel + ")");
    else
      lcdLine(1, "Envia % (0-100)");
    lastIdleRefresh = millis();
  }

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() == 0) return;

    // Parsea prefijo u/d opcional y el numero
    char dir = '\0';
    String num = cmd;
    if (num.length() > 0 && (num[0] == 'u' || num[0] == 'U' || num[0] == 'd' || num[0] == 'D')) {
      dir = (num[0] == 'u' || num[0] == 'U') ? 'u' : 'd';
      num = num.substring(1);
    }
    float pct = num.toFloat();
    pct = constrain(pct, 0.0f, 100.0f);
    int pwm = (int)round(pct * 255.0 / 100.0);

    String label = String((int)pct);
    if (dir != '\0') label = String(dir) + label;

    analogWrite(LED_PIN, pwm);
    currentPct = (int)pct;
    currentPwm = pwm;
    lastLabel  = label;

    // Cuenta regresiva de estabilizacion
    int seconds = STABILIZE_MS / 1000;
    for (int s = seconds; s > 0; s--) {
      lcdLine(0, "PWM:" + String(pwm) + " (" + label + ")");
      lcdLine(1, "Estabiliza " + String(s) + "s");
      Serial.print("# Estabilizando "); Serial.print(s); Serial.println(" s");
      delay(1000);
    }

    runBurst(label, (int)pct, pwm);
    Serial.println("# Rafaga terminada. Envia siguiente %.");
    lastIdleRefresh = 0;   // refresca LCD ya
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
