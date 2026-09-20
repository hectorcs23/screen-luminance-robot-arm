/*
 * VEML7700 - Logger de luminancia/iluminancia de pantalla  +  indicador LCD 16x2
 *
 * Flujo:
 *   1) Espera una "etiqueta de brillo" por serial (ej. "50" + Enter).
 *   2) Toma N_SAMPLES mediciones espaciadas INTERVAL_MS.
 *   3) Vuelve a esperar (ahí ajustas el brillo).  -> indefinidamente.
 *
 * LCD (16x2, I2C en el MISMO bus que el sensor):
 *   - Esperando : Lux en vivo + último promedio/etiqueta.
 *   - Midiendo  : etiqueta + progreso (i/N) + lux instantáneo.
 *   - Al acabar : promedio de la ráfaga.
 *
 * Salida CSV por serial a 115200 baud (idéntica al logger sin LCD).
 * Librerías: Adafruit_VEML7700 | LiquidCrystal_I2C | Wire
 *
 * Cableado (montaje validado): SDA/SCL compartidos por VEML7700 y LCD,
 * con resistencias pull-up externas (~4.7 kΩ) a 3.3/5 V y masa común.
 */
#include <Wire.h>
#include "Adafruit_VEML7700.h"
#include <LiquidCrystal_I2C.h>

Adafruit_VEML7700 veml = Adafruit_VEML7700();
LiquidCrystal_I2C lcd(0x27, 16, 2);   // cambia a 0x3F si tu LCD no responde

// ---------- Parámetros del experimento ----------
const uint16_t N_SAMPLES   = 30;    // mediciones por ráfaga
const uint16_t INTERVAL_MS = 120;   // espaciado entre mediciones (>= tiempo de integración)

const unsigned long IDLE_REFRESH_MS = 400;   // refresco del lux en vivo al esperar

uint32_t burstId = 0;
String   brightnessLabel = "NA";
float    lastAvg = NAN;             // promedio de la última ráfaga (para el LCD)
unsigned long lastIdleRefresh = 0;

// ---------- Utilidad: escribir una línea completa de 16 chars (sin parpadeo) ----------
void lcdLine(uint8_t row, const String &s) {
  String t = s;
  while (t.length() < 16) t += ' ';   // rellena para borrar lo anterior
  if (t.length() > 16) t = t.substring(0, 16);
  lcd.setCursor(0, row);
  lcd.print(t);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  Wire.begin();
  Wire.setClock(50000);   // 50 kHz: tolerante a ruido / bus compartido (como tu montaje)

  lcd.init();
  lcd.backlight();
  lcdLine(0, "Iniciando...");

  if (!veml.begin()) {
    Serial.println("# ERROR: no se detecta el VEML7700 (revisa I2C / pull-ups / 0x10)");
    lcdLine(0, "VEML no detect.");
    lcdLine(1, "Revisa I2C/pullup");
    while (1) { delay(100); }
  }

  // Ajustes FIJOS -> todas las ráfagas comparables entre sí.
  // Ganancia baja + 100 ms: buen punto de partida para pantalla a corta distancia.
  veml.setGain(VEML7700_GAIN_1_8);
  veml.setIntegrationTime(VEML7700_IT_100MS);
  delay(300);

  // Encabezado CSV (una sola vez: primera fila del archivo).
  Serial.println("timestamp_ms,burst_id,sample_idx,brightness,lux,als_raw,white_raw");
  Serial.println("# Listo. Envia una etiqueta de brillo (ej. 50) + Enter para disparar una rafaga.");

  lcdLine(0, "Listo");
  lcdLine(1, "Envia etiqueta");
}

void loop() {
  // --- Indicador en vivo mientras espera ---
  if (millis() - lastIdleRefresh >= IDLE_REFRESH_MS) {
    float lux = veml.readLux(VEML_LUX_CORRECTED_NOWAIT);
    lcdLine(0, "Lux:" + String(lux, 1));
    if (!isnan(lastAvg))
      lcdLine(1, "B:" + brightnessLabel + " Avg:" + String(lastAvg, 0));
    else
      lcdLine(1, "Envia etiqueta");
    lastIdleRefresh = millis();
  }

  // --- Comando por serial: dispara una ráfaga ---
  if (Serial.available()) {
    brightnessLabel = Serial.readStringUntil('\n');
    brightnessLabel.trim();
    if (brightnessLabel.length() == 0) brightnessLabel = "NA";
    runBurst();
    Serial.println("# Rafaga terminada. Ajusta el brillo y envia la siguiente etiqueta.");
    lastIdleRefresh = 0;   // fuerza refresco inmediato del indicador
  }
}

void runBurst() {
  burstId++;
  double   sum = 0.0;
  uint32_t scheduled = millis();

  for (uint16_t i = 0; i < N_SAMPLES; i++) {
    // Espera activa hasta el instante programado -> espaciado constante.
    while ((int32_t)(millis() - scheduled) < 0) { ; }

    uint32_t ts    = millis();
    float    lux   = veml.readLux(VEML_LUX_CORRECTED_NOWAIT);
    uint16_t als   = veml.readALS();
    uint16_t white = veml.readWhite();
    sum += lux;

    // --- CSV por serial (sin cambios) ---
    Serial.print(ts);              Serial.print(',');
    Serial.print(burstId);         Serial.print(',');
    Serial.print(i);               Serial.print(',');
    Serial.print(brightnessLabel); Serial.print(',');
    Serial.print(lux, 4);          Serial.print(',');
    Serial.print(als);             Serial.print(',');
    Serial.println(white);

    // --- Indicador de progreso en LCD ---
    lcdLine(0, "B:" + brightnessLabel + "  " + String(i + 1) + "/" + String(N_SAMPLES));
    lcdLine(1, "Lux:" + String(lux, 1));

    scheduled += INTERVAL_MS;
  }

  lastAvg = (float)(sum / N_SAMPLES);

  // Resultado de la ráfaga en LCD
  lcdLine(0, "B:" + brightnessLabel + " listo");
  lcdLine(1, "Avg Lux:" + String(lastAvg, 1));
}
