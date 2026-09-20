/*
  ============================================================
  brazo_serial_USB.ino   (Arduino UNO R4 WiFi)  -  SIN WiFi
  ============================================================
  Version SOLO por cable USB / Serial, para usar con la app en
  modo "Serial". NO usa WiFiS3 ni servidor TCP, asi se evita el
  pico de corriente del ESP32 que hundia el riel de 5V y corrompia
  la LCD / reiniciaba la placa.

  Compatible con la app (brazo_ctk_v3.py) en modo Serial a 9600 baud.

  Comandos reconocidos (por Serial):
    s1:<deg>,s2:<deg>   -> mueve servos (0..180)
    SCAN:<n>            -> toma n muestras y devuelve promedio
        Respuesta:  SCAN_RESULT:lux_promedio,white_promedio
    THRESH:<low>,<high> -> ajusta umbrales de los LEDs (lux)
        Respuesta:  OK THRESH low,high
    PING               -> responde "PONG"

  LEDs (umbral sobre Lux):
    verde    -> low <= lux <= high
    amarillo -> lux < low
    rojo     -> lux > high

  CONEXIONES:
    A4 (SDA) -> SDA  (VEML7700 y LCD en paralelo)
    A5 (SCL) -> SCL
    5V / GND -> ambos dispositivos
    Servo1   -> D9        Servo2 -> D10
    LED verde -> D2   LED amarillo -> D3   LED rojo -> D4

  LIBRERIAS:
    Adafruit VEML7700 | LiquidCrystal I2C | Wire | Servo
  ============================================================
*/

#include <Wire.h>
#include <Adafruit_VEML7700.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

// ==========================
// Servos
// ==========================
Servo servo1;
Servo servo2;
const int pinServo1 = 9;
const int pinServo2 = 10;

int ultimoS1 = 90;
int ultimoS2 = 90;

// ==========================
// LEDs de umbral
// ==========================
const int pinLedVerde    = 2;
const int pinLedAmarillo = 3;
const int pinLedRojo     = 4;

float luxLow  = 50.0;    // por debajo -> amarillo
float luxHigh = 500.0;   // por encima -> rojo  (entre ambos -> verde)

// ==========================
// Sensor y LCD
// ==========================
Adafruit_VEML7700  veml;
LiquidCrystal_I2C  lcd(0x27, 16, 2);   // cambiar a 0x3F si no responde

bool vemlOk = false;

// ==========================
// Temporizacion
// ==========================
unsigned long ultimaLectura = 0;
const unsigned long INTERVALO_MS = 500;
bool midiendo = false;

unsigned long ultimoIntentoVeml = 0;
const unsigned long REINTENTO_VEML_MS = 2000;

unsigned long ultimoReinitLCD = 0;
const unsigned long REINIT_LCD_MS = 2000;

unsigned long ultimoComandoMs = 0;        // ultimo comando recibido
const unsigned long IDLE_MS = 300;        // mantenimiento solo si brazo quieto

const uint8_t VEML_ADDR = 0x10;   // direccion I2C del VEML7700

// ==========================
// Re-inicializa la LCD (recupera de corrupcion por ruido I2C)
// ==========================
void reinitLCD()
{
  lcd.init();
  lcd.backlight();
}

// ==========================
// Init del VEML7700 (sin bloquear)
// ==========================
bool initVeml()
{
  if (veml.begin()) {
    veml.setGain(VEML7700_GAIN_1);
    veml.setIntegrationTime(VEML7700_IT_100MS);
    return true;
  }
  return false;
}

// ==========================
// El VEML sigue presente en el bus I2C?
// ==========================
bool vemlPresente()
{
  Wire.beginTransmission(VEML_ADDR);
  return (Wire.endTransmission() == 0);
}

// ==========================
// LEDs segun nivel de luminancia
// ==========================
void actualizarLeds(float lux)
{
  digitalWrite(pinLedVerde,    (lux >= luxLow && lux <= luxHigh) ? HIGH : LOW);
  digitalWrite(pinLedAmarillo, (lux < luxLow)                    ? HIGH : LOW);
  digitalWrite(pinLedRojo,     (lux > luxHigh)                   ? HIGH : LOW);
}

// ==========================
// Mostrar en LCD lectura actual + servos
// ==========================
void mostrarSensor()
{
  lcd.setCursor(0, 0);
  if (vemlOk) {
    float lux = veml.readLux();
    actualizarLeds(lux);
    lcd.print("Lux:");
    lcd.print(lux, 1);
    lcd.print("       ");
  } else {
    lcd.print("Sin sensor     ");
  }

  lcd.setCursor(0, 1);
  lcd.print("S1:");
  lcd.print(ultimoS1);
  lcd.print(" S2:");
  lcd.print(ultimoS2);
  lcd.print("    ");
}

// ==========================
// Tomar n mediciones y devolver promedio
// ==========================
void ejecutarScan(int n, int delayMs)
{
  if (n < 1)   n = 1;
  if (n > 200) n = 200;
  if (delayMs < 0)    delayMs = 0;
  if (delayMs > 2000) delayMs = 2000;

  if (!vemlOk) {
    Serial.println("SCAN_RESULT:0,0");   // sin sensor: no hay lectura valida
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Sin sensor");
    return;
  }

  midiendo = true;

  double sumLux   = 0.0;
  double sumWhite = 0.0;
  int    usados   = 0;
  int    skip     = (n >= 5) ? 1 : 0;   // descartar primera muestra (warm-up)

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Midiendo...");

  for (int i = 0; i < n; i++)
  {
    float lx = veml.readLux();
    float wh = veml.readWhite();

    if (i >= skip) {
      sumLux   += lx;
      sumWhite += wh;
      usados++;
    }

    // Manda cada muestra cruda de iluminancia a la PC
    Serial.print("SAMPLE:");
    Serial.println(lx, 4);

    lcd.setCursor(0, 1);
    lcd.print("Muestra ");
    lcd.print(i + 1);
    lcd.print("/");
    lcd.print(n);
    lcd.print("   ");

    delay(delayMs);
  }

  double avgLux   = sumLux   / (double)usados;
  double avgWhite = sumWhite / (double)usados;

  actualizarLeds((float)avgLux);

  // Responder a la PC
  Serial.print("SCAN_RESULT:");
  Serial.print(avgLux, 4);
  Serial.print(",");
  Serial.println(avgWhite, 4);

  // Mostrar resultado en LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Avg Lux:");
  lcd.print(avgLux, 1);
  lcd.setCursor(0, 1);
  lcd.print("Avg W:  ");
  lcd.print(avgWhite, 1);

  midiendo = false;
  ultimaLectura = millis();
}

// ==========================
// Procesar comando serial
// ==========================
void procesarComando(String cmd)
{
  cmd.trim();
  if (cmd.length() == 0) return;

  // -- PING --------------------------------------------------
  if (cmd.equalsIgnoreCase("PING")) {
    Serial.println("PONG");
    return;
  }

  // -- SCAN:<n>[,<delay_ms>] ---------------------------------
  if (cmd.startsWith("SCAN:")) {
    String rest = cmd.substring(5);
    int coma = rest.indexOf(',');
    int n, delayMs = 120;   // default 120 ms entre muestras
    if (coma == -1) {
      n = rest.toInt();
    } else {
      n = rest.substring(0, coma).toInt();
      delayMs = rest.substring(coma + 1).toInt();
    }
    ejecutarScan(n, delayMs);
    return;
  }

  // -- THRESH:<low>,<high> -----------------------------------
  if (cmd.startsWith("THRESH:")) {
    String rest = cmd.substring(7);
    int coma = rest.indexOf(',');
    if (coma == -1) {
      Serial.println("ERR: formato THRESH invalido");
      return;
    }
    float lo = rest.substring(0, coma).toFloat();
    float hi = rest.substring(coma + 1).toFloat();
    if (hi < lo) { float t = lo; lo = hi; hi = t; }
    luxLow  = lo;
    luxHigh = hi;
    Serial.print("OK THRESH ");
    Serial.print(luxLow, 2);
    Serial.print(",");
    Serial.println(luxHigh, 2);
    return;
  }

  // -- Servos ------------------------------------------------
  int idx1 = cmd.indexOf("s1:");
  int idx2 = cmd.indexOf("s2:");

  if (idx1 == -1 || idx2 == -1) {
    Serial.print("ERR: comando no reconocido: ");
    Serial.println(cmd);
    return;
  }

  int commaPos = cmd.indexOf(',', idx1);
  if (commaPos == -1) {
    Serial.println("ERR: formato invalido");
    return;
  }

  int s1 = cmd.substring(idx1 + 3, commaPos).toInt();
  int s2 = cmd.substring(idx2 + 3).toInt();

  s1 = constrain(s1, 0, 180);
  s2 = constrain(s2, 0, 180);

  servo1.write(s1);
  servo2.write(s2);
  ultimoS1 = s1;
  ultimoS2 = s2;
  // Sin respuesta: la app no la lee y acumularla satura el buffer USB (lag).
}

// ==========================
// Setup
// ==========================
void setup()
{
  Serial.begin(9600);
  Serial.setTimeout(50);  // no esperar ~1s por un comando partido
  Wire.begin();
  Wire.setClock(50000);   // 50 kHz: mas tolerante a ruido / cables largos

  pinMode(pinLedVerde,    OUTPUT);
  pinMode(pinLedAmarillo, OUTPUT);
  pinMode(pinLedRojo,     OUTPUT);
  digitalWrite(pinLedVerde,    LOW);
  digitalWrite(pinLedAmarillo, LOW);
  digitalWrite(pinLedRojo,     LOW);

  reinitLCD();
  lcd.setCursor(0, 0);
  lcd.print("Iniciando...");

  // Init del VEML sin bloquear: si falla, sigue y reintenta en el loop
  for (int i = 0; i < 5 && !vemlOk; i++) {
    vemlOk = initVeml();
    if (!vemlOk) delay(200);
  }
  if (!vemlOk) {
    Serial.println("ERR: VEML7700 no encontrado (sigo sin sensor)");
    lcd.setCursor(0, 1);
    lcd.print("Sin sensor     ");
  }

  servo1.attach(pinServo1);
  servo2.attach(pinServo2);
  servo1.write(ultimoS1);
  servo2.write(ultimoS2);

  delay(800);
  lcd.clear();

  Serial.println("Listo. Esperando comandos (Serial)...");
  ultimaLectura     = millis();
  ultimoIntentoVeml = millis();
  ultimoReinitLCD   = millis();
}

// ==========================
// Loop principal
// ==========================
void loop()
{
  // 1) PRIMERO el serial: vaciar TODOS los comandos en cola (hasta un tope)
  //    para que el servo vaya al objetivo mas reciente, sin backlog/lag.
  int procesados = 0;
  while (Serial.available() && procesados < 30) {
    String cmd = Serial.readStringUntil('\n');
    procesarComando(cmd);
    ultimoComandoMs = millis();
    procesados++;
  }

  unsigned long ahora = millis();
  bool idle = (ahora - ultimoComandoMs >= IDLE_MS);

  // 2) Mantenimiento (LCD/sensor) SOLO cuando el brazo esta quieto, para no
  //    robarle tiempo al serial durante una rutina (evita el lag).
  if (!midiendo && idle) {
    if (ahora - ultimoReinitLCD >= REINIT_LCD_MS) {
      reinitLCD();
      ultimoReinitLCD = ahora;
    }

    if (ahora - ultimoIntentoVeml >= REINTENTO_VEML_MS) {
      if (vemlOk && !vemlPresente()) {
        vemlOk = false;
      }
      if (!vemlOk) {
        vemlOk = initVeml();
      }
      ultimoIntentoVeml = ahora;
    }

    if (ahora - ultimaLectura >= INTERVALO_MS) {
      mostrarSensor();
      ultimaLectura = ahora;
    }
  }
}
