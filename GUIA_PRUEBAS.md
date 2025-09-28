# 🏔️ Guía de Pruebas - Sistema Alerta Temprana Talud Challenge 2

## ⚠️ IMPORTANTE - Errores Corregidos
- **HTML Dashboard**: Corregido problema de comillas y caracteres especiales
- **Variables duplicadas**: Eliminada duplicación de VIB_CONT_MS
- **Thread-safety**: Mejorado acceso a datos compartidos con mutex
- **Compilación**: Compatible con ESP32 Core 3.x

## 📋 Preparación Inicial

### 1. Librería Requeridas 
- WiFiManager
- ArduinoJson 
- Adafruit MPU6050
- LiquidCrystal I2C (warning normal, funciona bien)
- OneWire, DallasTemperature

### 2. Hardware Conectado 
- LCD I2C (0x27), MPU6050, DS18B20
- Sensores: Vibración=GPIO34, Lluvia=GPIO36, Suelo=GPIO39
- LEDs: Verde=13, Amarillo=12, Naranja=14, Rojo=27
- Buzzer=GPIO25, Motores según configuración

## 🚀 Proceso de Prueba

### Paso 1: Compilación y Carga
1. **Abrir Arduino IDE** con ESP32 Core 3.x
2. **Verificar/Compilar** - debe compilar sin errores
3. **Subir al ESP32** - esperar "Hard resetting via RTS pin..."
4. **Abrir Serial Monitor** a 115200 baud

### Paso 2: Primera Conexión WiFi
1. **Al encender primera vez** verás en LCD "Config Portal 192.168.4.1"  
2. **Conectar tu teléfono/PC** a red WiFi "TaludESP32-Config"
3. **Abrir navegador** en `192.168.4.1`
4. **Escanear y conectar** a tu red WiFi local
5. **LCD mostrará "WiFi OK"** y la IP asignada

### Paso 3: Acceso al Tablero Web
1. **Anotar la IP** mostrada en LCD (ej: 192.168.1.100)
2. **Desde cualquier dispositivo en la misma red WiFi** abrir navegador
3. **Ir a** `http://[IP_DEL_ESP32]`
4. **¡Verás el tablero de control!** 🎉

## 🧪 Pruebas de Funcionalidad

### Monitor Serial (115200 baud)
```
Comandos disponibles:
'd' = Re-detectar hardware
'0' = Parar motores  
'1' = Simulación temblor leve (5s)
'2' = Simulación temblor fuerte (5s)
'r' = Reconocer alarma manualmente
```

### Verificar Multihilo
- **LCD actualiza** cada 0.5s sin bloqueos
- **Sensores leen** cada 1s (log cada 5s en Serial)  
- **Web responde** instantáneamente
- **Historial se actualiza** cada 30s

### Tablero Web - Funciones
- **📊 Tiempo real**: Métricas cada 2 segundos
- **📈 Historial**: Últimas 10 lecturas cada 10s  
- **⚠️ Reconocer Alarma**: Silencia buzzers
- **🔄 Reiniciar Sistema**: Reset completo

## 🔧 Simulación de Alertas

### Generar Alerta Rápida:
1. **Inclinar MPU6050** >5° manualmente
2. **Tocar sensor vibración** repetidamente
3. **Humedecer sensor suelo** YL-100
4. **Observar**: LEDs rojos, buzzer, LCD "ALERTA"

### Control desde Web:
1. **Estado en tiempo real** con colores indicativos
2. **Botón "Reconocer Alarma"** para silenciar
3. **Historial tabular** con timestamp

## 🌐 Seguridad y Características

- ✅ **WiFiManager**: Portal cautivo para configuración
- ✅ **Acceso restringido**: Solo misma red WiFi local
- ✅ **Sin MQTT**: Servidor embebido directo  
- ✅ **FreeRTOS**: 3 tareas independientes sin bloqueos
- ✅ **Thread-safe**: Semáforos protegen datos compartidos
- ✅ **Responsive**: Web funciona en móviles

## 📱 Flujo de Uso Normal

1. **Encender ESP32** → Auto-conecta a WiFi conocida
2. **LCD muestra IP** → http://192.168.1.X disponible  
3. **Monitoreo continuo** → Sensores cada 1s, web cada 2s
4. **Si hay riesgo** → LEDs + buzzer + web "🚨 ALARMA ACTIVA"
5. **Reconocer alarma** → Web o Serial 'r' → Silencia audio
6. **Historial persistente** → Últimas 50 lecturas en memoria

## 🆘 Troubleshooting WiFi

### Problema: "Connection Refused" en Portal de Configuración

**Síntomas:**
- LCD muestra "Config Portal 192.168.4.1"
- Puerto abierto en escaneo pero no se puede acceder
- Serial Monitor se queda en boot logs

**Soluciones paso a paso:**

1. **Reset completo de WiFi:**
   ```
   Enviar 'w' por Serial Monitor → Resetea credenciales WiFi y reinicia
   ```

2. **Verificar conexión al portal:**
   - **SSID**: `TaludESP32-Config`
   - **Password**: `12345678` (agregado para mayor seguridad)
   - **IP**: `192.168.4.1`
   - **Timeout**: 5 minutos (ampliado)

3. **Si sigue sin funcionar:**
   - Desconectar ESP32 físicamente 10 segundos
   - Reconectar y esperar hasta ver "Portal de configuración activo"
   - Usar diferentes dispositivos (teléfono/PC) para conectar

4. **Verificar en Serial Monitor:**
   ```
   Iniciando configuración WiFi...
   Intentando autoconexión...
   Portal de configuración activo en:
   SSID: TaludESP32-Config
   Password: 12345678
   IP: 192.168.4.1
   ```

### Comandos Serial Actualizados
```
'd' = Re-detectar hardware
'0' = Parar motores  
'1' = Simulación temblor leve (5s)
'2' = Simulación temblor fuerte (5s)
'r' = Reconocer alarma manualmente
'w' = RESET WiFi + reiniciar (NUEVO)
```

**No compila**: Verificar ESP32 Core 3.x, todas las librerías instaladas
**WiFi no conecta**: Usar comando 'w' para reset completo
**Portal "Connection Refused"**: 
  - Verificar password: 12345678
  - Esperar 30 segundos después del reinicio
  - Usar diferentes navegadores/dispositivos
**Web no carga**: Estar en misma red, verificar IP en LCD
**Sensores NaN**: Normal si no están físicamente conectados
**Alarma no silencia**: Usar botón web o comando Serial 'r'

## 💡 Tips de Demostración

- **Usar teléfono móvil** para mostrar web responsive
- **Inclinar físicamente** el ESP32 para demo inclinación  
- **Mostrar historial** refrescándose automáticamente
- **Demo comandos Serial** para control motores
- **Explicar multihilo** viendo LCD + web simultáneos
