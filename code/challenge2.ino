/*
  ================== Sistema de Alerta Temprana de Talud (ESP32) - Challenge 2 ==================
  
  Requisitos Challenge 2:
   - WiFiManager para configuración de red
   - Servidor web embebido con acceso restringido a WLAN local
   - Multihilo para mediciones (FreeRTOS)
   - Tablero de control con historial y control de alarmas
   - Fusión de al menos 3 señales independientes
  
  Conexiones Hardware (MANTIENE LOS MISMOS PINES):
  I2C: SDA=21, SCL=22 | LCD: 0x27/0x3F | MPU6050: 0x68/0x69
  Vibración: GPIO34 | Lluvia: A0=GPIO36, D0=GPIO4 | Suelo: GPIO39
  DS18B20: GPIO5 | LEDs: 13,12,14,27 | Buzzer: GPIO25
  Motores: AIN1=33, AIN2=32, PWMA=18, BIN1=19, BIN2=23, PWMB=26
*/

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFiManager.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>

// ==================== Tipos y utilidades base ====================
struct AdcStats { 
  int minv; 
  int maxv; 
  float avg; 
};

static inline float clampf(float x, float lo, float hi) { 
  return (x < lo) ? lo : (x > hi) ? hi : x; 
}

// ==================== Pines ====================
#define SDA_PIN 21
#define SCL_PIN 22

#define LCD_ADDR1 0x27
#define LCD_ADDR2 0x3F

// Sensores
const int PIN_VIBRATION = 34; // SIG del modulo de vibracion
const int PIN_RAIN_A    = 36; // lluvia analogica
const int PIN_RAIN_D    = 4;  // lluvia digital (comparador)
const int PIN_SOIL_A    = 39; // YL-100 AO
const int PIN_DS18B20   = 5;

// YL-100 calibracion (ajustalo en campo)
const int SOIL_ADC_DRY  = 3000; // seco (RAW)
const int SOIL_ADC_WET  = 1300; // saturado (RAW)

// LEDs y buzzer - Cambiamos nombres para evitar conflictos
const int LED_VERDE   = 13;
const int LED_AMARILLO = 12;
const int LED_NARANJA = 14;
const int LED_ROJO    = 27;
const int PIN_BUZZER = 25;

// ==================== Motores (Puente H) ====================
const int AIN1 = 33;
const int AIN2 = 32;
const int PWMA = 18; // PWM canal A

const int BIN1 = 19;
const int BIN2 = 23;
const int PWMB = 26;  // PWM canal B 

// ==================== Objetos globales ====================
LiquidCrystal_I2C lcd(LCD_ADDR1, 16, 2);
Adafruit_MPU6050 mpu;
OneWire oneWire(PIN_DS18B20);
DallasTemperature ds(&oneWire);
WiFiManager wm;
WebServer server(80);

// ==================== Variables compartidas (protegidas por semáforos) ====================
SemaphoreHandle_t dataMutex;
volatile bool alarmaActiva = false;
volatile bool alarmaAcknowledged = false;

struct SensorData {
  float inclinacion = NAN;
  float vibracion_per_min = 0.0f;
  float temperatura = NAN;
  float humedad_suelo = NAN;
  int lluvia_raw = 0;
  float score_total = 0.0f;
  int nivel_alerta = 0;
  unsigned long timestamp = 0;
  bool vibContinuous = false;
  float tempGradient = 0.0f;
} currentData;

// Historial (últimas 50 lecturas)
const int HISTORY_SIZE = 50;
SensorData history[HISTORY_SIZE];
int historyIndex = 0;

// Estado de conexión
String wifiSSID = "";
String localIP = "";

// ==================== Flags de disponibilidad ====================
bool i2c_ok = false;
bool lcd_ok = false; 
bool mpu_ok = false; 
bool ds_ok = false;
bool rain_ana_ok = false; 
bool soil_ok = false;
bool lcd_ok_once = false;

// ==================== Estado de sensores ====================
volatile unsigned long vibPulses = 0;
volatile unsigned long lastVibMicros = 0;
unsigned long lastVibCalc = 0;
const unsigned long VIB_WINDOW_MS = 10000;
const unsigned long VIB_CONT_MS = 5000;
const bool VIB_ACTIVE_LOW = true;
unsigned long vibActiveStart = 0;

// Variables de lluvia y temperatura
unsigned long rainHighStart = 0;
bool rainHighPersist = false;
float lastTempC = NAN;
unsigned long lastTempMs = 0;
float tempGradientC_per_min = 0.0f;
// ==================== Umbrales y constantes ====================
const int RAIN_ADC_NORMAL_MAX   = 800;   // < 200 en 10 bits
const int RAIN_ADC_PRECAU_MAX   = 2400;  // 200-600 en 10 bits
const int RAIN_TORRENCIAL_MIN   = 2400;  // >600 en 10 bits

// YL-100 (en %)
const float SOIL_NORMAL_MAX      = 40.0f; // 0-40%
const float SOIL_PRECAU_MAX      = 70.0f; // 40-70% ; >70% saturado

// Vibracion (eventos/min)
const float VIB_NORMAL_MAX       = 2.0f;  // 0-2
const float VIB_PRECAU_MAX       = 5.0f;  // 3-5 ; >5 alerta

// Temperatura (°C, gradiente °C/min)
const float TEMP_NORMAL_MIN      = 10.0f; // 10-30°C estable (y gradiente <=2)
const float TEMP_NORMAL_MAX      = 30.0f;
const float TEMP_PRECAU_MIN      = 5.0f;  // <10°C precaucion; <5°C emergencia
const float TEMP_GRAD_PRECAU     = 2.0f;  // >2 °C/min precaucion
const float TEMP_GRAD_ALERTA     = 5.0f;  // >5 °C/min alarma

// Inclinacion (criterio razonable)
const float INC_PRECAU_MIN       = 2.0f;
const float INC_ALERTA_MIN       = 5.0f;

// ==================== Declaraciones de funciones ====================
// Tareas FreeRTOS
void sensorTask(void *parameter);
void webServerTask(void *parameter);
void displayTask(void *parameter);

// Funciones de sensores (mantienen misma lógica)
AdcStats readAdcStats(int pin, int samples = 32, int us_between = 200);
bool analogLooksWired(const AdcStats& s, int spreadMax, int avgMin, int avgMax);
void beep(uint16_t f, uint16_t ms);
void beepPattern(uint8_t nivel);
void setLEDs(int n);
bool i2cHas(uint8_t addr);
void scanI2C();
void motorA_set(int speedPct);
void motorB_set(int speedPct);
void motors_stop();
void simulate_quake(uint8_t level);
void detectHardware();
void printStatus();

// Funciones de lectura de sensores
float leerInclinacionDeg();
float leerVibracionPerMin();
float leerTemperaturaC();
float leerHumedadSueloPct();
int leerLluviaRaw();

// Funciones de scoring
float scoreInclinacion(float inc);
float scoreVibracion(float ev_min, bool cont);
float scoreLluvia(int adc);
float scoreSuelo(float pct);
float scoreTemperatura(float T, float dTdt);
float calcularRiesgoFusion(float sc_inc, float sc_vib, float sc_llu, float sc_suelo, float sc_temp);
int nivelPorScore(float s);

// Funciones de red y web server
void setupWiFi();
void setupWebServer();
void handleRoot();
void handleData();
void handleHistory();
void handleAckAlarm();
void handleResetAlarm();
String generateDashboard();
void updateHistory();

// Funciones de pantalla
void drawMetrics();
void drawAlert();
void drawWiFiInfo();

// ==================== Implementacion de funciones ====================

AdcStats readAdcStats(int pin, int samples, int us_between) {
  int mn = 4095, mx = 0; 
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    int v = analogRead(pin);
    if (v < mn) mn = v;
    if (v > mx) mx = v;
    sum += v;
    delayMicroseconds(us_between);
  }
  AdcStats s{ mn, mx, (float)sum / samples };
  return s;
}

bool analogLooksWired(const AdcStats& s, int spreadMax, int avgMin, int avgMax) {
  int spread = s.maxv - s.minv;
  if (spread > spreadMax) return false;
  if (s.avg < avgMin || s.avg > avgMax) return false;
  return true;
}

// ==================== Utilidades de salida ====================
void beep(uint16_t f, uint16_t ms) { 
  tone(PIN_BUZZER, f, ms); 
}

void beepPattern(uint8_t nivel) {
  static unsigned long t0 = 0; 
  unsigned long now = millis();
  switch(nivel) {
    case 0: break;
    case 1: if(now - t0 > 3000) { t0 = now; beep(1200, 100); } break;
    case 2: if(now - t0 > 650) { t0 = now; beep(1500, 120); delay(80); beep(1500, 120); } break;
    case 3: if(now - t0 > 300) { t0 = now; beep(2000, 180); } break;
  }
}

void setLEDs(int n) { 
  digitalWrite(LED_VERDE, n == 0); 
  digitalWrite(LED_AMARILLO, n == 1); 
  digitalWrite(LED_NARANJA, n == 2); 
  digitalWrite(LED_ROJO, n == 3); 
}

// ==================== I2C Helpers ====================
bool i2cHas(uint8_t addr) { 
  Wire.beginTransmission(addr); 
  return (Wire.endTransmission() == 0); 
}

void scanI2C() {
  Serial.println("\n[I2C] Escaneando...");
  int n = 0; 
  for(uint8_t a = 1; a < 127; a++) { 
    Wire.beginTransmission(a); 
    if(Wire.endTransmission() == 0) { 
      Serial.print("  - 0x"); 
      if(a < 16) Serial.print('0'); 
      Serial.println(a, HEX); 
      n++; 
    } 
  }
  if(!n) Serial.println("  (sin dispositivos)");
  i2c_ok = (n > 0);
}

// ==================== Motores: control con puente H ====================
void motorA_set(int speedPct) { // -100..100
  speedPct = constrain(speedPct, -100, 100);
  int pwm = map(abs(speedPct), 0, 100, 0, 255);
  if (speedPct >= 0) { 
    digitalWrite(AIN1, HIGH); 
    digitalWrite(AIN2, LOW); 
  } else { 
    digitalWrite(AIN1, LOW);  
    digitalWrite(AIN2, HIGH); 
  }
  ledcWrite(PWMA, pwm);
}

void motorB_set(int speedPct) { // -100..100
  speedPct = constrain(speedPct, -100, 100);
  int pwm = map(abs(speedPct), 0, 100, 0, 255);
  if (speedPct >= 0) { 
    digitalWrite(BIN1, HIGH); 
    digitalWrite(BIN2, LOW); 
  } else { 
    digitalWrite(BIN1, LOW);  
    digitalWrite(BIN2, HIGH); 
  }
  ledcWrite(PWMB, pwm);
}

void motors_stop() { 
  motorA_set(0); 
  motorB_set(0); 
}

void simulate_quake(uint8_t level) { // 1=leve, 2=fuerte (5 s)
  unsigned long t0 = millis();
  while(millis() - t0 < 5000) {
    int s = (level == 1) ? 35 : 80;
    motorA_set(s); 
    motorB_set(-s); 
    delay(150);
    motorA_set(-s); 
    motorB_set(s); 
    delay(150);
  }
  motors_stop();
}

// ==================== Deteccion de hardware ====================
void printStatus() {
  Serial.println("\n=== ESTADO ===");
  Serial.printf("I2C    : %s\n", i2c_ok ? "OK" : "NO");
  Serial.printf("LCD    : %s\n", lcd_ok ? "OK" : "NO");
  Serial.printf("MPU    : %s\n", mpu_ok ? "OK" : "NO");
  Serial.printf("DS18B20: %s\n", ds_ok ? "OK" : "NO");
  Serial.printf("RAIN_A : %s (%s)\n", rain_ana_ok ? "OK" : "NA", rain_ana_ok ? "wired" : "floating?");
  Serial.printf("SOIL_A : %s (%s)\n", soil_ok ? "OK" : "NA", soil_ok ? "wired" : "floating?");
  Serial.println("==============\n");
}

void detectHardware() {
  // I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); // mas compatible
  scanI2C();

  // LCD
  lcd_ok = false;
  if(i2cHas(LCD_ADDR1)) { 
    lcd = LiquidCrystal_I2C(LCD_ADDR1, 16, 2); 
    lcd.init(); 
    lcd.backlight(); 
    lcd_ok = true; 
  } else if(i2cHas(LCD_ADDR2)) { 
    lcd = LiquidCrystal_I2C(LCD_ADDR2, 16, 2); 
    lcd.init(); 
    lcd.backlight(); 
    lcd_ok = true; 
  }
  if(lcd_ok) { 
    lcd.clear(); 
    lcd.setCursor(0, 0); 
    lcd.print("Talud ESP32"); 
    lcd.setCursor(0, 1); 
    lcd.print("Inicializando"); 
    lcd_ok_once = true; 
  }

  // MPU6050
  mpu_ok = false;
  if(i2cHas(0x68) && mpu.begin(0x68)) {
    mpu_ok = true;
  } else if(i2cHas(0x69) && mpu.begin(0x69)) {
    mpu_ok = true;
  }
  
  if(mpu_ok) {
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  } else {
    Serial.println("[WARN] MPU6050 NO detectado.");
  }

  // DS18B20
  ds.begin(); 
  ds_ok = (ds.getDeviceCount() > 0);
  if(!ds_ok) Serial.println("[WARN] DS18B20 NO detectado.");

  // Vibracion - ISR para contar pulsos
  pinMode(PIN_VIBRATION, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_VIBRATION), []() {
    unsigned long now = micros();
    if(now - lastVibMicros > 15000) { 
      vibPulses++; 
      lastVibMicros = now; 
    }
  }, CHANGE);

  // Lluvia D0 sin pull interno (si no hay modulo, flota)
  pinMode(PIN_RAIN_D, INPUT);

  // ADCs
  analogReadResolution(12);
  AdcStats rstats = readAdcStats(PIN_RAIN_A, 32, 200);
  AdcStats sstats = readAdcStats(PIN_SOIL_A, 32, 200);
  int d0 = digitalRead(PIN_RAIN_D);
  bool d0_defined = (d0 == LOW || d0 == HIGH);
  bool rain_adc_ok = analogLooksWired(rstats, 1200, 100, 3800);
  rain_ana_ok = (d0_defined && rain_adc_ok);
  soil_ok = analogLooksWired(sstats, 800, 200, 3600);

  // LEDs, buzzer
  pinMode(LED_VERDE, OUTPUT); 
  pinMode(LED_AMARILLO, OUTPUT); 
  pinMode(LED_NARANJA, OUTPUT); 
  pinMode(LED_ROJO, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  // Motores PWM - Compatible con ESP32 Core 3.x
  pinMode(AIN1, OUTPUT); 
  pinMode(AIN2, OUTPUT); 
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); 
  pinMode(BIN2, OUTPUT); 
  pinMode(PWMB, OUTPUT);
  
  // Configurar PWM para motores (nueva API ESP32 Core 3.x)
  ledcAttach(PWMA, 20000, 8);  // pin, frecuencia, resolucion
  ledcAttach(PWMB, 20000, 8);  // pin, frecuencia, resolucion
  motors_stop();

  printStatus();
  if(lcd_ok) { 
    lcd.clear(); 
    lcd.setCursor(0, 0); 
    lcd.print("Sistema listo!"); 
    lcd.setCursor(0, 1); 
    lcd.print("Uni Sabana"); 
  }
}

// ==================== Lecturas seguras ====================
float leerInclinacionDeg() {
  if(!mpu_ok) return NAN;
  sensors_event_t a, g, t; 
  mpu.getEvent(&a, &g, &t);
  float ax = a.acceleration.x, ay = a.acceleration.y, az = a.acceleration.z;
  float mag = sqrt(ax*ax + ay*ay + az*az);
  if (mag < 0.5f) return NAN; // datos absurdos
  float roll  = atan2(ay, az) * 180.0/PI;
  float pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0/PI;
  return max(fabs(roll), fabs(pitch));
}

float leerVibracionPerMin() {
  static float lastValue = 0.0f;
  unsigned long now = millis();
  if(now - lastVibCalc >= VIB_WINDOW_MS) {
    noInterrupts(); 
    unsigned long p = vibPulses; 
    vibPulses = 0; 
    interrupts();
    lastValue = (float)p * (60000.0f / (float)VIB_WINDOW_MS);
    lastVibCalc = now;
  }
  return lastValue;
}

float leerTemperaturaC() {
  if(!ds_ok) return NAN;
  ds.requestTemperatures();
  float t = ds.getTempCByIndex(0);
  if(t == DEVICE_DISCONNECTED_C) return NAN;

  unsigned long now = millis();
  if(!isnan(lastTempC) && lastTempMs > 0) {
    float dt_min = (now - lastTempMs) / 60000.0f;
    if (dt_min > 0.001f) {
      tempGradientC_per_min = fabs(t - lastTempC) / dt_min;
    }
  }
  lastTempC = t;
  lastTempMs = now;
  return t;
}

float leerHumedadSueloPct() {
  if(!soil_ok) return NAN;
  int adc = analogRead(PIN_SOIL_A);
  adc = constrain(adc, min(SOIL_ADC_WET, SOIL_ADC_DRY), max(SOIL_ADC_WET, SOIL_ADC_DRY));
  float pct = 100.0f * (float)(SOIL_ADC_DRY - adc) / (float)(SOIL_ADC_DRY - SOIL_ADC_WET);
  return clampf(pct, 0.0f, 100.0f);
}

int leerLluviaRaw() {
  if(!rain_ana_ok) return -1;
  int d = digitalRead(PIN_RAIN_D);
  int adc = analogRead(PIN_RAIN_A);
  // Persistencia torrencial: si adc >= torrencial, inicia/continua contador
  unsigned long now = millis();
  if (adc >= RAIN_TORRENCIAL_MIN) {
    if (rainHighStart == 0) rainHighStart = now;
    if (now - rainHighStart >= 30UL * 60UL * 1000UL) rainHighPersist = true; // >30 min
  } else {
    rainHighStart = 0;
    rainHighPersist = false;
  }
  if (d == HIGH && adc < (RAIN_ADC_NORMAL_MAX / 2)) adc = 0;
  return adc;
}

// ==================== Scoring conforme a umbrales ====================
// Mapea cada sensor a score 0..100 segun los umbrales dados
float scoreInclinacion(float inc) {
  if(isnan(inc)) return 0.0f;
  if(inc >= INC_ALERTA_MIN) return 100.0f;
  if(inc >= INC_PRECAU_MIN) return 60.0f + (inc - INC_PRECAU_MIN) * (40.0f / (INC_ALERTA_MIN - INC_PRECAU_MIN));
  return inc * (20.0f / INC_PRECAU_MIN); // suave antes de precaucion
}

float scoreVibracion(float ev_min, bool cont) {
  if(cont) return 100.0f;
  if(ev_min > VIB_PRECAU_MAX) return 100.0f;
  if(ev_min > VIB_NORMAL_MAX) return 60.0f + (ev_min - VIB_NORMAL_MAX) * (40.0f / (VIB_PRECAU_MAX - VIB_NORMAL_MAX + 0.0001f));
  return ev_min * (30.0f / (VIB_NORMAL_MAX + 0.0001f));
}

float scoreLluvia(int adc) {
  if(adc < 0) return 0.0f; // NA
  if(adc >= RAIN_TORRENCIAL_MIN) return rainHighPersist ? 100.0f : 80.0f; // >600 (torrencial) persiste => 100
  if(adc > RAIN_ADC_NORMAL_MAX) return 60.0f + (adc - RAIN_ADC_NORMAL_MAX) * (20.0f / (RAIN_ADC_PRECAU_MAX - RAIN_ADC_NORMAL_MAX));
  return (float)adc * (20.0f / RAIN_ADC_NORMAL_MAX);
}

float scoreSuelo(float pct) {
  if(isnan(pct)) return 0.0f;
  if(pct > SOIL_PRECAU_MAX) return 100.0f; // >70%
  if(pct > SOIL_NORMAL_MAX) return 60.0f + (pct - SOIL_NORMAL_MAX) * (40.0f / (SOIL_PRECAU_MAX - SOIL_NORMAL_MAX));
  return pct * (30.0f / SOIL_NORMAL_MAX);
}

float scoreTemperatura(float T, float dTdt) {
  if(isnan(T)) return 0.0f;
  float base = 0.0f;
  if(T < TEMP_PRECAU_MIN) base = (T < 5.0f) ? 80.0f : 60.0f; // <5°C fuerte, <10°C moderado
  else if(T > TEMP_NORMAL_MAX) base = 40.0f;
  else base = 0.0f;
  if(dTdt > TEMP_GRAD_ALERTA) return clampf(80.0f + (dTdt - TEMP_GRAD_ALERTA) * 5.0f, 80.0f, 100.0f);
  if(dTdt > TEMP_GRAD_PRECAU) base = max(base, 60.0f + (dTdt - TEMP_GRAD_PRECAU) * 5.0f);
  return clampf(base, 0.0f, 100.0f);
}

// Fusion (pesos + sinergias)
float calcularRiesgoFusion(float sc_inc, float sc_vib, float sc_llu, float sc_suelo, float sc_temp) {
  // Pesos (puedes ajustarlos)
  const float w_inc = 0.35f, w_vib = 0.25f, w_sue = 0.20f, w_llu = 0.15f, w_tmp = 0.05f;
  float base = sc_inc * w_inc + sc_vib * w_vib + sc_suelo * w_sue + sc_llu * w_llu + sc_temp * w_tmp;

  // Sinergias:
  float f = 1.0f;
  if (sc_inc > 60 && sc_vib > 60) f *= 1.5f;    // inclinacion + vibracion
  if (sc_llu > 60 && sc_suelo > 60) f *= 1.5f;  // lluvia + suelo saturado
  return clampf(base * f, 0.0f, 100.0f);
}

int nivelPorScore(float s) { 
  if(s >= 76) return 3; 
  if(s >= 51) return 2; 
  if(s >= 26) return 1; 
  return 0; 
}

// ==================== Pantalla (todo a la vez) ====================
void drawMetrics() {
  if(!lcd_ok) return;
  char l1[17], l2[17];
  
  if(xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50))) {
    float I = isnan(currentData.inclinacion) ? 0.0f : currentData.inclinacion;
    int V = (int)roundf(currentData.vibracion_per_min);
    int L = (currentData.lluvia_raw < 0) ? -1 : currentData.lluvia_raw;
    int H = isnan(currentData.humedad_suelo) ? -1 : (int)roundf(currentData.humedad_suelo);
    int T = isnan(currentData.temperatura) ? -1000 : (int)roundf(currentData.temperatura);
    bool incNaN = isnan(currentData.inclinacion);
    xSemaphoreGive(dataMutex);
    
    // Línea 1
    if (incNaN) snprintf(l1, sizeof(l1), "I:NA   V:%3d", V);
    else snprintf(l1, sizeof(l1), "I:%4.1f V:%3d", I, V);

    // Línea 2
    char lbuf[6];
    if (L < 0) strcpy(lbuf, " NA ");
    else snprintf(lbuf, sizeof(lbuf), "%4d", L);
    if (H < 0 && T == -1000) snprintf(l2, sizeof(l2), "L:%s  H:NA T:NA", lbuf);
    else if(H < 0) snprintf(l2, sizeof(l2), "L:%s  H:NA T:%2d", lbuf, T);
    else if(T == -1000) snprintf(l2, sizeof(l2), "L:%s  H:%2d T:NA", lbuf, H);
    else snprintf(l2, sizeof(l2), "L:%s  H:%2d T:%2d", lbuf, H, T);
    
    lcd.clear();
    lcd.setCursor(0, 0); 
    lcd.print(l1);
    lcd.setCursor(0, 1); 
    lcd.print(l2);
  }
}

void drawAlert() {
  if(!lcd_ok) return;
  lcd.clear();
  lcd.setCursor(0, 0); 
  lcd.print("ALERTA DE");
  lcd.setCursor(0, 1); 
  lcd.print("DESLIZAMIENTO");
}

void drawWiFiInfo() {
  if(!lcd_ok) return;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("WiFi: " + wifiSSID.substring(0, 10));
  lcd.setCursor(0, 1);
  lcd.print("IP:" + localIP);
}

// ==================== Funciones WiFi y Web Server ====================

void setupWiFi() {
  if(lcd_ok) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Config WiFi...");
  }
  
  Serial.println("Iniciando configuración WiFi...");
  
  // Configuraciones más robusta para WiFiManager
  wm.setDebugOutput(true);
  wm.setConfigPortalBlocking(true); // Cambiar a bloqueante para mayor estabilidad
  wm.setConfigPortalTimeout(300); // 5 minutos timeout
  wm.setConnectTimeout(30); // 30 segundos para conectar
  wm.setAPStaticIPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
  
  // Reinicia configuraciones WiFi si es necesario
  // wm.resetSettings();
  
  Serial.println("Intentando autoconexión...");
  
  // Intentar conexión automática primero
  if(wm.autoConnect("TaludESP32-Config", "12345678")) {
    wifiSSID = WiFi.SSID();
    localIP = WiFi.localIP().toString();
    Serial.println("WiFi conectado exitosamente!");
    Serial.println("SSID: " + wifiSSID);
    Serial.println("IP: " + localIP);
    
    if(lcd_ok) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("WiFi OK");
      lcd.setCursor(0, 1);
      lcd.print(localIP);
      delay(3000);
    }
  } else {
    Serial.println("ERROR: No se pudo establecer conexión WiFi");
    Serial.println("El portal de configuración debería estar activo en:");
    Serial.println("SSID: TaludESP32-Config");
    Serial.println("Password: 12345678");
    Serial.println("IP: 192.168.4.1");
    
    if(lcd_ok) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Config Portal");
      lcd.setCursor(0, 1);
      lcd.print("192.168.4.1");
    }
    
    // Reiniciar después de timeout del portal
    delay(5000);
    Serial.println("Reiniciando ESP32...");
    ESP.restart();
  }
}

void setupWebServer() {
  // Solo configurar servidor si WiFi está conectado
  if(WiFi.status() == WL_CONNECTED) {
    server.on("/", HTTP_GET, handleRoot);
    server.on("/data", HTTP_GET, handleData);
    server.on("/history", HTTP_GET, handleHistory);
    server.on("/ack", HTTP_POST, handleAckAlarm);
    server.on("/reset", HTTP_POST, handleResetAlarm);
    
    server.begin();
    Serial.println("Servidor web iniciado en: http://" + WiFi.localIP().toString());
  } else {
    Serial.println("WiFi no conectado - Servidor web no iniciado");
  }
}

void handleRoot() {
  // Verificar acceso desde red local solamente
  IPAddress clientIP = server.client().remoteIP();
  IPAddress localNet = WiFi.localIP();
  if(clientIP[0] != localNet[0] || clientIP[1] != localNet[1] || clientIP[2] != localNet[2]) {
    server.send(403, "text/plain", "Acceso denegado - Solo red local");
    return;
  }
  
  server.send(200, "text/html", generateDashboard());
}

void handleData() {
  DynamicJsonDocument doc(512);
  
  if(xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100))) {
    doc["inclinacion"] = isnan(currentData.inclinacion) ? 0 : currentData.inclinacion;
    doc["vibracion"] = currentData.vibracion_per_min;
    doc["temperatura"] = isnan(currentData.temperatura) ? 0 : currentData.temperatura;
    doc["humedad_suelo"] = isnan(currentData.humedad_suelo) ? 0 : currentData.humedad_suelo;
    doc["lluvia"] = currentData.lluvia_raw;
    doc["score"] = currentData.score_total;
    doc["nivel"] = currentData.nivel_alerta;
    doc["alarma_activa"] = alarmaActiva;
    doc["timestamp"] = currentData.timestamp;
    xSemaphoreGive(dataMutex);
  }
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleHistory() {
  DynamicJsonDocument doc(4096);
  JsonArray array = doc.createNestedArray("history");
  
  for(int i = 0; i < HISTORY_SIZE; i++) {
    int idx = (historyIndex + i) % HISTORY_SIZE;
    if(history[idx].timestamp > 0) {
      JsonObject obj = array.createNestedObject();
      obj["inclinacion"] = isnan(history[idx].inclinacion) ? 0 : history[idx].inclinacion;
      obj["vibracion"] = history[idx].vibracion_per_min;
      obj["temperatura"] = isnan(history[idx].temperatura) ? 0 : history[idx].temperatura;
      obj["humedad_suelo"] = isnan(history[idx].humedad_suelo) ? 0 : history[idx].humedad_suelo;
      obj["lluvia"] = history[idx].lluvia_raw;
      obj["score"] = history[idx].score_total;
      obj["nivel"] = history[idx].nivel_alerta;
      obj["timestamp"] = history[idx].timestamp;
    }
  }
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleAckAlarm() {
  if(xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100))) {
    alarmaAcknowledged = true;
    xSemaphoreGive(dataMutex);
  }
  server.send(200, "text/plain", "Alarma reconocida");
}

void handleResetAlarm() {
  if(xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100))) {
    alarmaActiva = false;
    alarmaAcknowledged = false;
    xSemaphoreGive(dataMutex);
  }
  server.send(200, "text/plain", "Alarma reiniciada");
}

String generateDashboard() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<title>Sistema Alerta Talud - UoSabana</title>";
  html += "<style>";
  html += "body{font-family:Arial,sans-serif;margin:20px;background:#f5f5f5}";
  html += ".header{background:#2c3e50;color:white;padding:15px;text-align:center;border-radius:8px}";
  html += ".container{display:grid;grid-template-columns:1fr 1fr;gap:20px;margin-top:20px}";
  html += ".card{background:white;padding:20px;border-radius:8px;box-shadow:0 2px 5px rgba(0,0,0,0.1)}";
  html += ".metric{display:flex;justify-content:space-between;margin:10px 0;padding:8px;border-radius:4px}";
  html += ".metric.normal{background:#d4edda}";
  html += ".metric.precaucion{background:#fff3cd}";
  html += ".metric.alerta{background:#f8d7da}";
  html += ".value{font-weight:bold}";
  html += ".status{padding:10px;text-align:center;border-radius:8px;margin:10px 0}";
  html += ".status.normal{background:#28a745;color:white}";
  html += ".status.precaucion{background:#ffc107;color:black}";
  html += ".status.alerta{background:#dc3545;color:white}";
  html += ".controls{text-align:center}";
  html += ".btn{padding:10px 20px;margin:5px;border:none;border-radius:4px;cursor:pointer}";
  html += ".btn.ack{background:#28a745;color:white}";
  html += ".btn.reset{background:#dc3545;color:white}";
  html += "@media (max-width:768px){.container{grid-template-columns:1fr}}";
  html += "@keyframes blink{0%{opacity:1}50%{opacity:0.5}100%{opacity:1}}";
  html += ".alarm{background:red;color:white;padding:10px;margin:10px 0;text-align:center;border-radius:8px;animation:blink 1s infinite}";
  html += "</style></head><body>";
  
  html += "<div class='header'>";
  html += "<h1>🏔️ Sistema Alerta Temprana Talud</h1>";
  html += "<p>Universidad de la Sabana - Sabana Centro, Cundinamarca</p>";
  html += "</div>";
  
  html += "<div class='container'>";
  html += "<div class='card'>";
  html += "<h3>📊 Métricas en Tiempo Real</h3>";
  html += "<div id='metrics'></div>";
  html += "<div id='status'></div>";
  html += "</div>";
  
  html += "<div class='card'>";
  html += "<h3>⚠️ Control de Alarmas</h3>";
  html += "<div class='controls'>";
  html += "<button class='btn ack' onclick='ackAlarm()'>Reconocer Alarma</button>";
  html += "<button class='btn reset' onclick='resetAlarm()'>Reiniciar Sistema</button>";
  html += "</div>";
  html += "<h3>📈 Historial Reciente</h3>";
  html += "<div id='history' style='max-height:300px;overflow-y:auto'></div>";
  html += "</div>";
  html += "</div>";

  html += "<script>";
  html += "function updateData(){";
  html += "fetch('/data').then(r=>r.json()).then(d=>{";
  html += "let html='';";
  html += "let nivel=['Normal','Precaucion','Alerta','Emergencia'];";
  html += "let css=['normal','precaucion','alerta','alerta'];";
  html += "html+='<div class=\"metric '+css[d.nivel]+'\"><span>Inclinacion:</span><span class=\"value\">'+d.inclinacion.toFixed(1)+'°</span></div>';";
  html += "html+='<div class=\"metric '+css[Math.min(Math.floor(d.vibracion/3),3)]+'\"><span>Vibracion:</span><span class=\"value\">'+d.vibracion.toFixed(1)+'/min</span></div>';";
  html += "html+='<div class=\"metric '+css[Math.min(Math.floor(d.temperatura<10?2:0),3)]+'\"><span>Temperatura:</span><span class=\"value\">'+d.temperatura.toFixed(1)+'°C</span></div>';";
  html += "html+='<div class=\"metric '+css[Math.min(Math.floor(d.humedad_suelo/35),3)]+'\"><span>Humedad Suelo:</span><span class=\"value\">'+d.humedad_suelo.toFixed(1)+'%</span></div>';";
  html += "html+='<div class=\"metric '+css[Math.min(Math.floor(d.lluvia/1000),3)]+'\"><span>Lluvia:</span><span class=\"value\">'+d.lluvia+'</span></div>';";
  html += "document.getElementById('metrics').innerHTML=html;";
  html += "let statusHtml='<div class=\"status '+css[d.nivel]+'\"><h3>Estado: '+nivel[d.nivel]+'</h3><p>Puntuacion Total: '+d.score.toFixed(1)+'</p></div>';";
  html += "if(d.alarma_activa) statusHtml+='<div class=\"alarm\">🚨 ALARMA ACTIVA 🚨</div>';";
  html += "document.getElementById('status').innerHTML=statusHtml;";
  html += "})}";

  html += "function updateHistory(){";
  html += "fetch('/history').then(r=>r.json()).then(d=>{";
  html += "let html='<table style=\"width:100%;font-size:12px\"><tr><th>Tiempo</th><th>Inc</th><th>Vib</th><th>Temp</th><th>Score</th></tr>';";
  html += "d.history.slice(-10).reverse().forEach(h=>{";
  html += "let t=new Date(h.timestamp).toLocaleTimeString();";
  html += "html+='<tr><td>'+t+'</td><td>'+h.inclinacion.toFixed(1)+'</td><td>'+h.vibracion.toFixed(1)+'</td><td>'+h.temperatura.toFixed(1)+'</td><td>'+h.score.toFixed(1)+'</td></tr>';";
  html += "});";
  html += "html+='</table>';";
  html += "document.getElementById('history').innerHTML=html;";
  html += "})}";

  html += "function ackAlarm(){fetch('/ack',{method:'POST'}).then(()=>updateData())}";
  html += "function resetAlarm(){fetch('/reset',{method:'POST'}).then(()=>updateData())}";
  html += "setInterval(updateData,2000);";
  html += "setInterval(updateHistory,10000);";
  html += "updateData();";
  html += "updateHistory();";
  html += "</script>";
  html += "</body></html>";
  
  return html;
}

void updateHistory() {
  if(xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100))) {
    history[historyIndex] = currentData;
    historyIndex = (historyIndex + 1) % HISTORY_SIZE;
    xSemaphoreGive(dataMutex);
  }
}

// ==================== Setup / Loop ====================
void setup() {
  Serial.begin(115200);
  delay(500);
  
  // Inicializar mutex para protección de datos
  dataMutex = xSemaphoreCreateMutex();
  
  // Detección de hardware inicial
  detectHardware();
  
  // Configurar WiFi con WiFiManager
  setupWiFi();
  
  // Configurar servidor web
  setupWebServer();
  
  // Crear tareas FreeRTOS
  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(webServerTask, "WebTask", 4096, NULL, 1, NULL, 1);  
  xTaskCreatePinnedToCore(displayTask, "DisplayTask", 2048, NULL, 1, NULL, 1);
  
  Serial.println("Sistema iniciado - Tareas creadas");
  if(localIP != "") {
    Serial.println("Tablero disponible en: http://" + localIP);
  }
}

void loop() {
  // Comandos por Serial mantenidos
  if(Serial.available()) {
    char c = Serial.read();
    switch(c) {
      case 'd': case 'D': 
        Serial.println("\n[CMD] Re-detectar hardware..."); 
        detectHardware(); 
        break;
      case '0': 
        motors_stop(); 
        Serial.println("[M] Motores STOP"); 
        break;
      case '1': 
        Serial.println("[M] Temblor leve (5s)"); 
        simulate_quake(1); 
        break;
      case '2': 
        Serial.println("[M] Temblor fuerte (5s)"); 
        simulate_quake(2); 
        break;
      case 'r': case 'R':
        if(xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100))) {
          alarmaAcknowledged = true;
          xSemaphoreGive(dataMutex);
          Serial.println("[CMD] Alarma reconocida");
        }
        break;
      case 'w': case 'W':
        Serial.println("[CMD] Reseteando configuración WiFi...");
        wm.resetSettings();
        delay(1000);
        Serial.println("[CMD] Reiniciando ESP32...");
        ESP.restart();
        break;
    }
  }
  
  // Manejar WiFiManager si está en modo portal
  wm.process();
  
  delay(100);
}

// ==================== Tareas FreeRTOS ====================

void sensorTask(void *parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1000); // 1 segundo
  
  for(;;) {
    // Leer sensores
    float inclin = leerInclinacionDeg();
    float vib_min = leerVibracionPerMin();
    float temp = leerTemperaturaC();  
    float suelo = leerHumedadSueloPct();
    int lluvia = leerLluviaRaw();
    
    // Calcular vibración continua
    bool vibCont = false;
    int lv = digitalRead(PIN_VIBRATION);
    bool active = VIB_ACTIVE_LOW ? (lv == LOW) : (lv == HIGH);
    if(active) {
      if(vibActiveStart == 0) vibActiveStart = millis();
      vibCont = (millis() - vibActiveStart >= VIB_CONT_MS);
    } else {
      vibActiveStart = 0;
    }
    
    // Calcular gradiente de temperatura
    float tempGrad = 0.0f;
    if(!isnan(temp) && !isnan(lastTempC) && lastTempMs > 0) {
      float dt_min = (millis() - lastTempMs) / 60000.0f;
      if (dt_min > 0.001f) {
        tempGrad = fabs(temp - lastTempC) / dt_min;
      }
    }
    lastTempC = temp;
    lastTempMs = millis();
    
    // Calcular scores y fusión
    float sc_inc = scoreInclinacion(inclin);
    float sc_vib = scoreVibracion(vib_min, vibCont);
    float sc_llu = scoreLluvia(lluvia);
    float sc_suelo = scoreSuelo(suelo);
    float sc_temp = scoreTemperatura(temp, tempGrad);
    float score = calcularRiesgoFusion(sc_inc, sc_vib, sc_llu, sc_suelo, sc_temp);
    int nivel = nivelPorScore(score);
    
    // Actualizar LEDs y buzzer
    setLEDs(nivel);
    if(!alarmaAcknowledged) beepPattern(nivel);
    
    // Actualizar datos compartidos
    if(xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100))) {
      currentData.inclinacion = inclin;
      currentData.vibracion_per_min = vib_min;
      currentData.temperatura = temp;
      currentData.humedad_suelo = suelo;
      currentData.lluvia_raw = lluvia;
      currentData.score_total = score;
      currentData.nivel_alerta = nivel;
      currentData.vibContinuous = vibCont;
      currentData.tempGradient = tempGrad;
      currentData.timestamp = millis();
      
      // Control de alarma
      if(nivel >= 2 && !alarmaActiva) {
        alarmaActiva = true;
        alarmaAcknowledged = false;
      }
      
      xSemaphoreGive(dataMutex);
    }
    
    // Actualizar historial cada 30 segundos
    static unsigned long lastHistUpdate = 0;
    if(millis() - lastHistUpdate > 30000) {
      updateHistory();
      lastHistUpdate = millis();
    }
    
    // Log cada 5 segundos
    static unsigned long lastLog = 0;
    if(millis() - lastLog > 5000) {
      Serial.printf("I=%.1f V=%.1f%s L=%d H=%.1f T=%.1f Score=%.1f Nivel=%d\n",
        isnan(inclin) ? 0.0f : inclin, vib_min, vibCont ? "(CONT)" : "",
        lluvia, isnan(suelo) ? 0.0f : suelo, isnan(temp) ? 0.0f : temp, score, nivel);
      lastLog = millis();
    }
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void webServerTask(void *parameter) {
  for(;;) {
    // Solo manejar el servidor si WiFi está conectado
    if(WiFi.status() == WL_CONNECTED) {
      server.handleClient();
    } else {
      // Si se desconectó, intentar reconectar cada 30 segundos
      static unsigned long lastReconnect = 0;
      if(millis() - lastReconnect > 30000) {
        Serial.println("WiFi desconectado, intentando reconectar...");
        WiFi.reconnect();
        lastReconnect = millis();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void displayTask(void *parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(500); // 0.5 segundos
  
  for(;;) {
    if(xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50))) {
      bool showAlert = (currentData.nivel_alerta >= 2 && !alarmaAcknowledged);
      xSemaphoreGive(dataMutex);
      
      if(showAlert) {
        drawAlert();
        vTaskDelay(pdMS_TO_TICKS(2000));
      } else {
        // Verificar estado de WiFi
        if(WiFi.status() != WL_CONNECTED) {
          if(lcd_ok) {
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Config Portal");
            lcd.setCursor(0, 1);
            lcd.print("192.168.4.1");
          }
        } else if(localIP.length() > 0) {
          drawMetrics();
        } else {
          drawWiFiInfo();
        }
      }
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}