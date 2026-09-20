/*
 * VEML7700 - Logger de luminancia/iluminancia de pantalla
 *
 * Flujo:
 *   1) Espera que llegue una "etiqueta de brillo" por serial (ej. "50" + Enter).
 *   2) Toma N_SAMPLES mediciones espaciadas INTERVAL_MS.
 *   3) Vuelve a esperar (ahí ajustas el brillo del dispositivo).  -> indefinidamente.
 *
 * Salida: lineas CSV por el puerto serial a 115200 baud.
 * Librería: Adafruit_VEML7700 (Sketch > Include Library > Manage Libraries).
 */
#include <Wire.h>
#include "Adafruit_VEML7700.h"

Adafruit_VEML7700 veml = Adafruit_VEML7700();

// ---------- Parámetros del experimento ----------
const uint16_t N_SAMPLES   = 30;    // mediciones por ráfaga
const uint16_t INTERVAL_MS = 120;   // espaciado entre mediciones (DEBE ser >= tiempo de integración)

uint32_t burstId = 0;
String   brightnessLabel = "NA";

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  if (!veml.begin()) {
    Serial.println("# ERROR: no se detecta el VEML7700 (revisa cableado I2C / dirección 0x10)");
    while (1) { delay(100); }
  }

  // Ajustes FIJOS -> todas las ráfagas son comparables entre sí.
  // Ganancia baja + 100 ms es un buen punto de partida para una pantalla a corta distancia.
  // Si saturas (als_raw ~ 65535) baja la ganancia o el tiempo de integración.
  // Si te quedas corto de señal, súbelos (y sube INTERVAL_MS en consecuencia).
  veml.setGain(VEML7700_GAIN_1_8);
  veml.setIntegrationTime(VEML7700_IT_100MS);
  delay(300);  // estabilización tras configurar

  // Encabezado CSV (una sola vez: será la primera fila del archivo).
  Serial.println("timestamp_ms,burst_id,sample_idx,brightness,lux,als_raw,white_raw");
  Serial.println("# Listo. Envia una etiqueta de brillo (ej. 50) + Enter para disparar una rafaga.");
}

void loop() {
  if (Serial.available()) {
    brightnessLabel = Serial.readStringUntil('\n');
    brightnessLabel.trim();
    if (brightnessLabel.length() == 0) brightnessLabel = "NA";
    runBurst();
    Serial.println("# Rafaga terminada. Ajusta el brillo y envia la siguiente etiqueta.");
  }
}

void runBurst() {
  burstId++;
  uint32_t scheduled = millis();

  for (uint16_t i = 0; i < N_SAMPLES; i++) {
    // Espera activa hasta el instante programado -> espaciado constante,
    // sin importar cuánto tarde la lectura I2C (siempre que tarde < INTERVAL_MS).
    while ((int32_t)(millis() - scheduled) < 0) { ; }

    uint32_t ts    = millis();
    // _NOWAIT: la librería NO añade su propia espera de un tiempo de integración,
    // así la cadencia la controla nuestro planificador (los 120 ms).
    float    lux   = veml.readLux(VEML_LUX_CORRECTED_NOWAIT);
    uint16_t als   = veml.readALS();    // cuentas crudas del canal ALS
    uint16_t white = veml.readWhite();  // cuentas crudas del canal blanco

    Serial.print(ts);              Serial.print(',');
    Serial.print(burstId);         Serial.print(',');
    Serial.print(i);               Serial.print(',');
    Serial.print(brightnessLabel); Serial.print(',');
    Serial.print(lux, 4);          Serial.print(',');
    Serial.print(als);             Serial.print(',');
    Serial.println(white);

    scheduled += INTERVAL_MS;
  }
}
