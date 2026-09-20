/*
 * Prueba de funcionamiento del VEML7700 — acceso directo a registros.
 *
 * Usa lecturas con STOP (endTransmission(true)) para evitar el repeated-start
 * que falla en tu placa, así que NO depende de la librería Adafruit.
 * Auto-ajusta la ganancia para no saturar (ALS->65535) ni quedarse en cero,
 * y convierte a lux. Tapa e ilumina el sensor: el lux debe cambiar.
 *
 * Si funciona, ya tienes un driver mínimo: lux = ALS * resolucion(ganancia).
 */
#include <Wire.h>

#define VEML       0x10
#define REG_CONF   0x00
#define REG_ALS    0x04
#define REG_WHITE  0x05
#define REG_ID     0x07
#define NO_DATA    0xEEEE   // marcador: la lectura no devolvio bytes

// Ganancias a tiempo de integracion 100 ms, de MAS a MENOS sensible:
const uint16_t gainBits[4] = {0x01,   0x00,   0x03,   0x02};    // x2, x1, x1/4, x1/8
const float    gainRes[4]  = {0.0288, 0.0576, 0.2304, 0.4608};  // lux por cuenta
const char*    gainName[4] = {"x2",   "x1",   "x1/4", "x1/8"};
int gainIdx = 1;            // arranca en x1

float luxMin = 1e9, luxMax = -1;

void writeReg(uint8_t reg, uint16_t v) {
  Wire.beginTransmission(VEML);
  Wire.write(reg);
  Wire.write(v & 0xFF);     // VEML7700: 16 bits, byte bajo primero
  Wire.write(v >> 8);
  Wire.endTransmission();
}

uint16_t readReg(uint8_t reg) {
  Wire.beginTransmission(VEML);
  Wire.write(reg);
  Wire.endTransmission(true);                 // STOP en vez de repeated-start
  Wire.requestFrom((uint8_t)VEML, (uint8_t)2);
  if (Wire.available() < 2) return NO_DATA;
  uint16_t lo = Wire.read(), hi = Wire.read();
  return (hi << 8) | lo;
}

void applyGain() {
  writeReg(REG_CONF, gainBits[gainIdx] << 11); // IT=100 ms (bits en 0), encendido (SD=0)
  delay(150);                                  // espera > tiempo de integracion
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Wire.begin();                 // ESP32/ESP8266/RP2040: Wire.begin(SDA, SCL) con TUS pines
  Wire.setClock(100000);        // 100 kHz: mas tolerante

  applyGain();
  uint16_t cfg = readReg(REG_CONF);
  uint16_t id  = readReg(REG_ID);

  Serial.println(F("======== Prueba VEML7700 ========"));
  Serial.print(F("CONFIG releido: 0x")); Serial.println(cfg, HEX);
  Serial.print(F("ID (reg 0x07):  0x")); Serial.println(id, HEX);
  if (cfg == NO_DATA || id == NO_DATA) {
    Serial.println(F(">> SIN lectura de registros: revisa I2C / baja Wire.setClock a 10000."));
  } else {
    Serial.println(F(">> Comunicacion OK. Tapa e ilumina el sensor: el ALS y el Lux deben cambiar."));
  }
  Serial.println();
}

void loop() {
  uint16_t als = readReg(REG_ALS);
  if (als == NO_DATA) { Serial.println(F("Lectura fallida")); delay(500); return; }

  // Auto-rango: si satura baja sensibilidad; si casi cero, sube.
  if (als > 60000 && gainIdx < 3) { gainIdx++; applyGain(); return; }
  if (als < 80    && gainIdx > 0) { gainIdx--; applyGain(); return; }

  uint16_t white = readReg(REG_WHITE);
  float lux = als * gainRes[gainIdx];
  if (lux < luxMin) luxMin = lux;
  if (lux > luxMax) luxMax = lux;

  Serial.print(F("Ganancia ")); Serial.print(gainName[gainIdx]);
  Serial.print(F(" | ALS "));   Serial.print(als);
  Serial.print(F(" | White ")); Serial.print(white);
  Serial.print(F(" | Lux "));   Serial.print(lux, 3);
  Serial.print(F("  [rango visto: ")); Serial.print(luxMin, 2);
  Serial.print(F(" .. "));             Serial.print(luxMax, 2);
  Serial.println(F(" lux]"));

  delay(500);
}
