/*
 * ident_altura.ino — blimp_definitivo/ident_altura
 * ============================================================
 * Controlador PROPORCIONAL de altura para identificación.
 * Sensor: VL53L1X (ToF)
 * Actuador: 2 motores + servo vectorial (embrague mecánico)
 *
 * PROTOCOLO TCP:
 *   MATLAB → ESP32 : "modo,setpoint_mm,K,mapeo\n"
 *                     modo: 0=STOP  1=Proporcional cerrado
 *                     mapeo: 0=directo  1=interpolado [50-255]
 *   ESP32  → MATLAB: "millis,dist_mm,sp_mm,e_m,u_k,pwm,emb\n"
 * ============================================================
 */

#include <WiFi.h>
#include <Wire.h>
#include <VL53L1X.h>

// ── Pines ────────────────────────────────────────────────────
const int motorIzqPin = 4;
const int motorDerPin = 5;
const int servoPin    = 2;
const int sdaPin      = 6;
const int sclPin      = 7;

// ── WiFi ─────────────────────────────────────────────────────
const char* ssid     = "iPhone de HP";
const char* password = "Contra12";
WiFiServer server(80);

// ── Sensor ToF ───────────────────────────────────────────────
VL53L1X sensorTof;
bool    tof_ok = false;

// ── Servo vectorial ──────────────────────────────────────────
const int SERVO_DESPEGAR   = 2000;
const int SERVO_ATERRIZAR  = 400;
const unsigned long TRANSIT_TIME_MS = 350;

// ── Límites PWM ──────────────────────────────────────────────
const float U_MAX     = 255.0f;
const int   PWM_MIN_Z = 50;
const int   PWM_MAX_Z = 255;

// ── Filtro EMA y zona muerta ─────────────────────────────────
const float ERR_DEADBAND_M = 0.015f;
const float ALPHA_EMA      = 0.4f;
float dist_filtrada_m = 0.0f;
bool  primer_muestra  = true;

// ── Estado controlador ───────────────────────────────────────
int   modo         = 0;
bool  mapeo_activo = false;
float sp_mm        = 1000.0f;
float K_coef       = 1.0f;     // Ganancia proporcional

int   offset_izq   = 10;
int   offset_der   = 10;

uint16_t last_valid_dist    = 0;
int      error_sensor_count = 0;
const int MAX_SENSOR_ERRORS = 5;

// ── Embrague ─────────────────────────────────────────────────
enum EstadoEmbrague { EMB_IDLE, EMB_MOVING, EMB_ON };
EstadoEmbrague estadoEmbrague  = EMB_IDLE;
int            direccionActual = SERVO_DESPEGAR;
unsigned long  t_transit_ini   = 0;

// ── Temporización ────────────────────────────────────────────
const unsigned long T_MS = 500;
unsigned long last_ctrl_ms     = 0;
bool          modo_inicializado = false;

// ── Prototipos ───────────────────────────────────────────────
bool leerToF(uint16_t& d);
void loopPropAltura(WiFiClient& c);
bool leerComando(WiFiClient& c);
void enviarTelemetria(WiFiClient& c, uint16_t dist, float sp_tel, float e_m, float u_k, int pwm);
int  mapearPWM(float u_abs, bool usar_mapeo);
void resetCtrl();

// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Wire.begin(sdaPin, sclPin);

  pinMode(motorIzqPin, OUTPUT); pinMode(motorDerPin, OUTPUT);
  analogWrite(motorIzqPin, 0); analogWrite(motorDerPin, 0);

  ledcAttach(servoPin, 50, 14);
  ledcWrite(servoPin, SERVO_DESPEGAR);

  for (int i = 1; i <= 3 && !tof_ok; i++) {
    delay(100);
    sensorTof.setTimeout(500);
    if (sensorTof.init()) {
      sensorTof.setDistanceMode(VL53L1X::Long);
      sensorTof.setMeasurementTimingBudget(50000);
      sensorTof.startContinuous(50);
      delay(200);
      tof_ok = true;
      Serial.printf("[ToF] OK (intento %d)\n", i);
    } else {
      Serial.printf("[ToF] Intento %d fallido\n", i);
    }
  }
  if (!tof_ok) Serial.println("[ToF] ERROR");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("[WiFi] Conectando");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.printf("\n[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
  server.begin();
  Serial.println("[Server] Puerto 80 listo — ident_altura");
}

// ─────────────────────────────────────────────────────────────
void loop() {
  WiFiClient client = server.available();
  if (!client) return;

  client.setNoDelay(true);
  Serial.println("[TCP] Cliente conectado");
  modo = 0; modo_inicializado = false; mapeo_activo = false;
  resetCtrl();
  analogWrite(motorIzqPin, 0); analogWrite(motorDerPin, 0);
  ledcWrite(servoPin, SERVO_DESPEGAR);

  while (client.connected()) {
    leerComando(client);
    if (millis() - last_ctrl_ms >= T_MS) {
      last_ctrl_ms = millis();
      if (modo == 1) {
        loopPropAltura(client);
      } else {
        analogWrite(motorIzqPin, 0); analogWrite(motorDerPin, 0);
        ledcWrite(servoPin, SERVO_DESPEGAR);
        estadoEmbrague = EMB_IDLE;
        uint16_t dist = 0; leerToF(dist);
        enviarTelemetria(client, dist ? dist : 8190, sp_mm, 0.0f, 0.0f, 0);
      }
    }
  }

  analogWrite(motorIzqPin, 0); analogWrite(motorDerPin, 0);
  ledcWrite(servoPin, SERVO_DESPEGAR);
  resetCtrl();
  Serial.println("[TCP] Cliente desconectado");
}

// ── leerComando — "modo,setpoint_mm,K,mapeo\n" ──────────────
bool leerComando(WiFiClient& client) {
  if (client.available() > 0) {
    String ultimo = client.readStringUntil('\n');
    ultimo.trim();
    client.flush();
    if (ultimo.length() == 0) return false;

    int c1 = ultimo.indexOf(',');
    int c2 = ultimo.indexOf(',', c1+1);
    int c3 = ultimo.indexOf(',', c2+1);
    int c4 = ultimo.indexOf(',', c3+1);
    int c5 = ultimo.indexOf(',', c4+1);
    
    if (c1 < 0 || c3 < 0) return false;

    int   nuevo_modo  = ultimo.substring(0,    c1).toInt();
    float nuevo_sp    = ultimo.substring(c1+1, c2).toFloat();
    float nuevo_K     = ultimo.substring(c2+1, c3).toFloat();
    bool  nuevo_mapeo = ultimo.substring(c3+1, c4 > 0 ? c4 : ultimo.length()).toInt() != 0;
    
    if (c4 > 0) {
        offset_izq = ultimo.substring(c4+1, c5 > 0 ? c5 : ultimo.length()).toInt();
        if (c5 > 0) {
            offset_der = ultimo.substring(c5+1).toInt();
        }
    }

    if (nuevo_modo != modo) {
      resetCtrl(); modo = nuevo_modo; modo_inicializado = false;
      Serial.printf("[MODO] → %d\n", modo);
    }
    sp_mm        = nuevo_sp;
    K_coef       = nuevo_K;
    mapeo_activo = nuevo_mapeo;
    return true;
  }
  return false;
}

// ── leerToF ──────────────────────────────────────────────────
bool leerToF(uint16_t& dist_out) {
  if (!tof_ok) { error_sensor_count = MAX_SENSOR_ERRORS; return false; }
  uint16_t dr = sensorTof.read();
  if (sensorTof.timeoutOccurred() || dr == 0 || dr > 6000) {
    error_sensor_count++; dist_out = last_valid_dist; return false;
  }
  if (last_valid_dist > 0 && abs((int)dr - (int)last_valid_dist) > 500) {
    error_sensor_count++; dist_out = last_valid_dist; return false;
  }
  error_sensor_count = 0; last_valid_dist = dr; dist_out = dr;
  return true;
}

// ── resetCtrl ────────────────────────────────────────────────
void resetCtrl() {
  error_sensor_count = 0; estadoEmbrague = EMB_IDLE;
  direccionActual = SERVO_DESPEGAR;
  primer_muestra = true; dist_filtrada_m = 0.0f;
}

// ── mapearPWM ─────────────────────────────────────────────────
int mapearPWM(float u_abs) {
  if (u_abs < 1.0f) return 0;
  if (u_abs > U_MAX) u_abs = U_MAX;
  
  // Mapeo Lineal (Interpolación) para eliminar la onda cuadrada
  int pwm = map((int)u_abs, 1, (int)U_MAX, PWM_MIN_Z, PWM_MAX_Z);
  return pwm;
}

// ── loopPropAltura ───────────────────────────────────────────
void loopPropAltura(WiFiClient& client) {
  if (!modo_inicializado) {
    resetCtrl(); modo_inicializado = true;
    Serial.printf("[P] SP=%.0fmm K=%.4f Mapeo=%s\n",
                  sp_mm, K_coef, mapeo_activo?"SI":"NO");
  }

  uint16_t dist_mm_raw = 0;
  bool ok = leerToF(dist_mm_raw);
  if (!ok && error_sensor_count >= MAX_SENSOR_ERRORS) {
    analogWrite(motorIzqPin, 0); analogWrite(motorDerPin, 0);
    ledcWrite(servoPin, SERVO_DESPEGAR); estadoEmbrague = EMB_IDLE;
    enviarTelemetria(client, 8190, sp_mm, 0.0f, 0.0f, 0);
    return;
  }

  uint16_t dist_mm    = dist_mm_raw ? dist_mm_raw : last_valid_dist;
  float    dist_m_raw = (float)dist_mm / 1000.0f;
  if (primer_muestra) { dist_filtrada_m = dist_m_raw; primer_muestra = false; }
  else { dist_filtrada_m = ALPHA_EMA * dist_m_raw + (1.0f - ALPHA_EMA) * dist_filtrada_m; }

  float sp_m = sp_mm / 1000.0f;
  float e_k  = sp_m - dist_filtrada_m;

  if (fabsf(e_k) < ERR_DEADBAND_M) {
    analogWrite(motorIzqPin, 0); analogWrite(motorDerPin, 0);
    enviarTelemetria(client, dist_mm, sp_mm, e_k, 0.0f, 0);
    return;
  }

  // Proporcional puro: u = K * e
  float u_k     = K_coef * e_k;
  float u_k_sat = constrain(u_k, -U_MAX, U_MAX);

  // u>=0 (subir) → SERVO_DESPEGAR | u<0 (bajar) → SERVO_ATERRIZAR
  int   nueva_dir = (u_k_sat >= 0.0f) ? SERVO_DESPEGAR : SERVO_ATERRIZAR;
  float u_abs     = fabsf(u_k_sat);
  int   pwm_cmd   = mapeo_activo ? mapearPWM(u_abs) : (int)constrain(u_abs, 0.0f, (float)U_MAX);
  int   pwm_izq   = pwm_cmd;
  int   pwm_der   = pwm_cmd;
  if (!mapeo_activo && pwm_cmd > 0) {
    pwm_izq = constrain(pwm_cmd + offset_izq, offset_izq, 255);
    pwm_der = constrain(pwm_cmd + offset_der, offset_der, 255);
  }

  switch (estadoEmbrague) {
    case EMB_IDLE:
      if (nueva_dir == direccionActual) {
        estadoEmbrague = EMB_ON;
        analogWrite(motorIzqPin, pwm_izq); analogWrite(motorDerPin, pwm_der);
        enviarTelemetria(client, dist_mm, sp_mm, e_k, u_k_sat, pwm_cmd);
      } else {
        direccionActual = nueva_dir; ledcWrite(servoPin, direccionActual);
        t_transit_ini = millis(); estadoEmbrague = EMB_MOVING;
        analogWrite(motorIzqPin, 0); analogWrite(motorDerPin, 0);
        enviarTelemetria(client, dist_mm, sp_mm, e_k, u_k_sat, 0);
      }
      break;
    case EMB_MOVING:
      analogWrite(motorIzqPin, 0); analogWrite(motorDerPin, 0);
      if (millis() - t_transit_ini >= TRANSIT_TIME_MS) estadoEmbrague = EMB_ON;
      enviarTelemetria(client, dist_mm, sp_mm, e_k, u_k_sat, 0);
      break;
    case EMB_ON:
      if (nueva_dir != direccionActual) {
        analogWrite(motorIzqPin, 0); analogWrite(motorDerPin, 0);
        direccionActual = nueva_dir; ledcWrite(servoPin, direccionActual);
        t_transit_ini = millis(); estadoEmbrague = EMB_MOVING;
        enviarTelemetria(client, dist_mm, sp_mm, e_k, u_k_sat, 0);
        break;
      }
      analogWrite(motorIzqPin, pwm_izq); analogWrite(motorDerPin, pwm_der);
      enviarTelemetria(client, dist_mm, sp_mm, e_k, u_k_sat, pwm_cmd);
      Serial.printf("[P] e=%.4f u=%.2f pwm=%d %s\n",
                    e_k, u_k_sat, pwm_cmd, mapeo_activo?"[MAP]":"[DIR]");
      break;
  }
}

// ── enviarTelemetria ─────────────────────────────────────────
void enviarTelemetria(WiFiClient& client, uint16_t dist, float sp_tel,
                      float e_m, float u_k, int pwm) {
  client.printf("%lu,%d,%.0f,%.4f,%.3f,%d,%d\n",
    millis(), dist, sp_tel, e_m, u_k, pwm, (int)estadoEmbrague);
}
