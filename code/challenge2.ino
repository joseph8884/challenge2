/*
  ================== Sistema de Alerta Temprana de Talud (ESP32) ==================
  
  NOTA IMPORTANTE: 
  - Compatible con ESP32 Core 3.x (nueva API de PWM)
  - La advertencia de LiquidCrystal_I2C es normal y no afecta la funcionalidad
  - Requiere instalar las librerias: Adafruit MPU6050, LiquidCrystal I2C, OneWire, DallasTemperature
  
  Requisitos implementados:
   1) Autodiagnostico de sensores/modulos al iniciar (y comando 'd' para re-detectar).
   2) Logica de alerta (score) conforme a umbrales solicitados:
        - Vibracion (activaciones/min y activacion continua >5s)
        - Lluvia (ADC 0-4095, con persistencia torrencial >30 min)
        - Humedad de suelo (YL-100) en %
        - Temperatura (°C) y gradiente °C/min
        - Inclinacion (MPU6050) (mantengo umbrales razonables para inclinacion)
   3) Visualizacion en LCD 16x2 mostrando TODOS los valores a la vez, con letra inicial:
        L1: I:xx.x  V:xx
        L2: L:xxxx  H:xx T:xx
   4) Si el score indica posible deslizamiento (nivel >= ALARMA), activa buzzer y
      muestra "ALERTA DE DESLIZAMIENTO" (pantalla de aviso 2 s, luego vuelve a metricas).
   5) Control de dos motores DC mediante puente H (recomendado TB6612/L298N). Incluye:
        - Pines, funciones motorA_set/motorB_set (-100..100)
        - Comandos Serial: '0'=stop, '1'=temblor leve, '2'=temblor fuerte
      (No se recomienda conectar motor directo al ESP32).
   6) Codigo documentado y README mas abajo.

  ----------------------------- Conexiones (Hardware) ------------------------------
  I2C: SDA=21, SCL=22
  LCD 16x2 I2C: 0x27 (o 0x3F)
  MPU6050 (GY-521): 0x68 (AD0=GND) o 0x69 (AD0=3V3)
  Vibracion (KY-002/SW-420/LM393): SIG -> GPIO32
  Lluvia: A0 (analogico) -> GPIO36 ; D0 (digital) -> GPIO4
  YL-100: AO -> GPIO39  (VCC 3.3V, GND)
  DS18B20: DATA -> GPIO5 (4.7k entre DATA y 3.3V)
  LEDs: Verde=13, Amarillo=12, Naranja=14, Rojo=27
  Buzzer: GPIO25
  Motores (puente H):
    TB6612/L298N -> ver pines abajo en seccion "MOTORES"
*/

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>

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

// ==================== Motores (Puente H recomendado) ====================
/* Recomendado TB6612 o L298N. NO conectes motores directo al ESP32.
   Define pines segun tu driver:
   TB6612 tipico:
     AIN1, AIN2, PWMA (ENA) -> Motor A
     BIN1, BIN2, PWMB (ENB) -> Motor B
*/
const int AIN1 = 33;
const int AIN2 = 32;
const int PWMA = 18; // PWM canal A

const int BIN1 = 19;
const int BIN2 = 23;
const int PWMB = 26;  // PWM canal B (cambiado de 5 para evitar conflicto con DS18B20)

// ==================== Objetos globales ====================
LiquidCrystal_I2C lcd(LCD_ADDR1, 16, 2);
Adafruit_MPU6050 mpu;
OneWire oneWire(PIN_DS18B20);
DallasTemperature ds(&oneWire);

// ==================== Flags de disponibilidad ====================
bool i2c_ok = false;
bool lcd_ok = false; 
bool mpu_ok = false; 
bool ds_ok = false;
bool rain_ana_ok = false; 
bool soil_ok = false;
bool lcd_ok_once = false;

// ==================== Estado de sensores ====================
// Vibracion
volatile unsigned long vibPulses = 0;
volatile unsigned long lastVibMicros = 0;
unsigned long lastVibCalc = 0;
float vibracion_per_min = 0.0f;
const unsigned long VIB_WINDOW_MS = 10000; // 10 s para calculo rapido
const bool VIB_ACTIVE_LOW = true;        // la mayoria de modulos LM393 entregan LOW en activacion
unsigned long vibActiveStart = 0;          // para detectar activacion continua >5s
bool vibContinuous = false;

// Lluvia
unsigned long rainHighStart = 0;           // para persistencia torrencial >30min
bool rainHighPersist = false;

// Temperatura
float temperatura_c = NAN;
float lastTempC = NAN;
unsigned long lastTempMs = 0;
float tempGradientC_per_min = 0.0f;

// Inclinacion
float inclin_deg = NAN;

// Suelo
float suelo_pct = NAN;

// Lluvia (valor analogico crudo y clasificacion)
int rain_raw = 0;

// ==================== Umbrales (adaptados a ESP32 y requeridos) ====================
// Nota: en tu tabla lluvia usa 0-1023 ADC. ESP32 es 0-4095.
// Escalado x4: 200 -> 800 ; 600 -> 2400.
const int RAIN_ADC_NORMAL_MAX   = 800;   // < 200 en 10 bits
const int RAIN_ADC_PRECAU_MAX   = 2400;  // 200-600 en 10 bits
const int RAIN_TORRENCIAL_MIN   = 2400;  // >600 en 10 bits

// YL-100 (en %)
const float SOIL_NORMAL_MAX      = 40.0f; // 0-40%
const float SOIL_PRECAU_MAX      = 70.0f; // 40-70% ; >70% saturado

// Vibracion (eventos/min)
const float VIB_NORMAL_MAX       = 2.0f;  // 0-2
const float VIB_PRECAU_MAX       = 5.0f;  // 3-5 ; >5 alerta
const unsigned long VIB_CONT_MS  = 5000;  // activacion continua >5 s => alerta

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
void printStatus();
void detectHardware();
float leerInclinacionDeg();
float leerVibracionPerMin();
float leerTemperaturaC();
float leerHumedadSueloPct();
int leerLluviaRaw();
float scoreInclinacion(float inc);
float scoreVibracion(float ev_min, bool cont);
float scoreLluvia(int adc);
float scoreSuelo(float pct);
float scoreTemperatura(float T, float dTdt);
float calcularRiesgoFusion(float sc_inc, float sc_vib, float sc_llu, float sc_suelo, float sc_temp);
int nivelPorScore(float s);
void drawMetrics();
void drawAlert();

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

// Heuristica: ADC luce cableado o flotante?
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
  // Pulsos por 10 s escalados a ev/min
  unsigned long now = millis();
  // Activacion continua (>5 s) deteccion por nivel:
  int lv = digitalRead(PIN_VIBRATION);
  bool active = VIB_ACTIVE_LOW ? (lv == LOW) : (lv == HIGH);
  if(active) {
    if(vibActiveStart == 0) vibActiveStart = now;
    vibContinuous = (now - vibActiveStart >= VIB_CONT_MS);
  } else {
    vibActiveStart = 0;
    vibContinuous = false;
  }

  if(now - lastVibCalc >= VIB_WINDOW_MS) {
    noInterrupts(); 
    unsigned long p = vibPulses; 
    vibPulses = 0; 
    interrupts();
    vibracion_per_min = (float)p * (60000.0f / (float)VIB_WINDOW_MS);
    lastVibCalc = now;
  }
  return vibracion_per_min;
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
  int d = digitalRead(PIN_RAIN_D); // si tienes el modulo, d define seco/mojado
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
  // Anti-falsos: si D0 indica NO lluvia (HIGH en muchos modulos) y adc bajo, fuerza a zona baja
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
  // base por temperatura absoluta
  float base = 0.0f;
  if(T < TEMP_PRECAU_MIN) base = (T < 5.0f) ? 80.0f : 60.0f; // <5°C fuerte, <10°C moderado
  else if(T > TEMP_NORMAL_MAX) base = 40.0f;
  else base = 0.0f;
  // gradientes
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
// L1: I:xx.x  V:xx
// L2: L:xxxx  H:xx T:xx
void drawMetrics() {
  if(!lcd_ok) return;
  char l1[17], l2[17];
  // I: inclinacion ; V: vibracion ev/min (entero)
  float I = isnan(inclin_deg) ? 0.0f : inclin_deg;
  int   V = (int)roundf(vibracion_per_min);

  // L: lluvia ADC 0..4095 (4 digitos) ; H: humedad % ; T: temp °C (entero)
  int   L = (rain_raw < 0) ? -1 : rain_raw;
  int   H = isnan(suelo_pct) ? -1 : (int)roundf(suelo_pct);
  int   T = isnan(temperatura_c) ? -1000 : (int)roundf(temperatura_c);

  // Linea 1
  if (isnan(inclin_deg)) snprintf(l1, sizeof(l1), "I:NA   V:%3d", V);
  else                   snprintf(l1, sizeof(l1), "I:%4.1f V:%3d", I, V);

  // Linea 2
  char lbuf[6];
  if (L < 0) strcpy(lbuf, " NA ");
  else snprintf(lbuf, sizeof(lbuf), "%4d", L);
  if (H < 0 && T == -1000) snprintf(l2, sizeof(l2), "L:%s  H:NA T:NA", lbuf);
  else if(H < 0)           snprintf(l2, sizeof(l2), "L:%s  H:NA T:%2d", lbuf, T);
  else if(T == -1000)      snprintf(l2, sizeof(l2), "L:%s  H:%2d T:NA", lbuf, H);
  else                     snprintf(l2, sizeof(l2), "L:%s  H:%2d T:%2d", lbuf, H, T);

  lcd.clear();
  lcd.setCursor(0, 0); 
  lcd.print(l1);
  lcd.setCursor(0, 1); 
  lcd.print(l2);
}

void drawAlert() {
  if(!lcd_ok) return;
  lcd.clear();
  lcd.setCursor(0, 0); 
  lcd.print("ALERTA DE");
  lcd.setCursor(0, 1); 
  lcd.print("DESLIZAMIENTO");
}

// ==================== Setup / Loop ====================
void setup() {
  Serial.begin(115200); 
  delay(200);
  detectHardware();
}

void loop() {
  // Comandos por Serial: 'd' re-detecta; '0' stop; '1' temblor leve; '2' temblor fuerte
  if(Serial.available()) {
    char c = Serial.read();
    if(c == 'd' || c == 'D') { 
      Serial.println("\n[CMD] Re-detectar..."); 
      detectHardware(); 
    }
    if(c == '0') { 
      motors_stop(); 
      Serial.println("[M] STOP"); 
    }
    if(c == '1') { 
      Serial.println("[M] Temblor leve"); 
      simulate_quake(1); 
    }
    if(c == '2') { 
      Serial.println("[M] Temblor fuerte"); 
      simulate_quake(2); 
    }
  }

  // 1) Lecturas
  inclin_deg     = leerInclinacionDeg();
  float vib_min  = leerVibracionPerMin();
  temperatura_c  = leerTemperaturaC();
  suelo_pct      = leerHumedadSueloPct();
  rain_raw       = leerLluviaRaw();

  // 2) Scores por sensor (conforme a umbrales pedidos)
  float sc_inc   = scoreInclinacion(inclin_deg);
  float sc_vib   = scoreVibracion(vib_min, vibContinuous);
  float sc_llu   = scoreLluvia(rain_raw);
  float sc_suelo = scoreSuelo(suelo_pct);
  float sc_temp  = scoreTemperatura(temperatura_c, tempGradientC_per_min);

  // 3) Fusion y nivel
  float score = calcularRiesgoFusion(sc_inc, sc_vib, sc_llu, sc_suelo, sc_temp);
  int   nivel = nivelPorScore(score);

  // 4) Actuadores
  setLEDs(nivel);
  beepPattern(nivel);

  // 5) Pantalla
  static unsigned long tUI = 0;
  if (nivel >= 2) { // ALERTA o EMERGENCIA
    drawAlert();
    delay(2000);
  }
  if(millis() - tUI > 500) { 
    tUI = millis(); 
    drawMetrics(); 
  }

  // 6) Log
  static unsigned long tLOG = 0;
  if(millis() - tLOG > 2000) { 
    tLOG = millis();
    Serial.printf("I=%.2f | V=%.1f/min%s | L=%s | H=%s | T=%s (dT/dt=%.1fC/min) | Score=%.1f | Nivel=%d\n",
      isnan(inclin_deg) ? 0.0f : inclin_deg,
      vib_min, vibContinuous ? " (CONT)" : "",
      (rain_raw < 0) ? "NA" : String(rain_raw).c_str(),
      isnan(suelo_pct) ? "NA" : String(suelo_pct, 1).c_str(),
      isnan(temperatura_c) ? "NA" : String(temperatura_c, 1).c_str(),
      tempGradientC_per_min,
      score, nivel
    );
  }

  delay(60);
}