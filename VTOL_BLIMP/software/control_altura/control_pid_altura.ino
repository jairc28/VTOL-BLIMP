#include <WiFi.h>
#include <Wire.h>
#include <VL53L1X.h>

const int motorIzqPin = 4;
const int motorDerPin = 5;
const int servoPin    = 2;
const int sdaPin      = 6;
const int sclPin      = 7;

const char* ssid     = "iPhone de HP";
const char* password = "Contra12";

WiFiServer server(80);
VL53L1X    sensorTof;
bool       tof_ok = false;

const int SERVO_DESPEGAR  = 2000;
const int SERVO_ATERRIZAR = 400;
const int SERVO_NEUTRAL   = 1600;
const unsigned long TRANSIT_TIME_MS = 350;

const float U_MAX     = 255.0f;
const int   PWM_MIN_Z = 45;
const int   PWM_MAX_Z = 255;

const float ERR_DEADBAND_M = 0.015f;
const float DIST_MIN_M     = 0.10f;

const float ALPHA_EMA = 0.4f;
float dist_filtrada_m = 0.0f;
bool  primer_muestra  = true;

int   modo  = 0;
float sp_mm = 1000.0f;
float sp_mm_ant = 1000.0f;  // para detectar cambio de setpoint

float K_coef  = 50.0f;
float K1_coef = 30.0f;
float Kd_coef = 10.0f;

// u[k] = u[k-1] + K*e[k] - K1*e[k-1] + Kd*e[k-2]
float u_prev = 0.0f;
float e_k1   = 0.0f;
float e_k2   = 0.0f;

uint16_t last_valid_dist    = 0;
int      error_sensor_count = 0;
const int MAX_SENSOR_ERRORS = 5;

enum EstadoEmbrague { EMB_IDLE, EMB_MOVING, EMB_ON };
EstadoEmbrague estadoEmbrague  = EMB_IDLE;
int            direccionActual = SERVO_NEUTRAL;
unsigned long  t_transit_ini   = 0;

const unsigned long T_MS = 500;
unsigned long last_pid_ms    = 0;
bool          modo_inicializado = false;

void  resetPID();
bool  leerToF(uint16_t& dist_out);
void  loopControlAltura(WiFiClient& c);
bool  leerComando(WiFiClient& c);
void  enviarTelemetria(WiFiClient& c, uint16_t dist, float e_m, float u_k, int pwm);
int   mapearPWM(float u_abs, int dir);

void setup() {
  Serial.begin(115200);
  Wire.begin(sdaPin, sclPin);

  pinMode(motorIzqPin, OUTPUT);
  pinMode(motorDerPin, OUTPUT);
  analogWrite(motorIzqPin, 0);
  analogWrite(motorDerPin, 0);

  ledcAttach(servoPin, 50, 14);
  ledcWrite(servoPin, SERVO_NEUTRAL);

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
  if (!tof_ok) Serial.println("[ToF] ERROR — SDA=6 SCL=7");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("[WiFi] Conectando");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.printf("\n[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
  server.begin();
  Serial.println("[Server] Puerto 80 listo");
}

void loop() {
  WiFiClient client = server.available();
  if (!client) return;

  client.setNoDelay(true);
  Serial.println("[TCP] Cliente conectado");
  modo = 0;
  modo_inicializado = false;
  resetPID();
  analogWrite(motorIzqPin, 0);
  analogWrite(motorDerPin, 0);
  ledcWrite(servoPin, SERVO_NEUTRAL);

  while (client.connected()) {
    leerComando(client);

    if (millis() - last_pid_ms >= T_MS) {
      last_pid_ms = millis();

      if (modo == 1) {
        loopControlAltura(client);
      } else {
        analogWrite(motorIzqPin, 0);
        analogWrite(motorDerPin, 0);
        ledcWrite(servoPin, SERVO_NEUTRAL);
        estadoEmbrague = EMB_IDLE;
        uint16_t dist = 0;
        leerToF(dist);
        enviarTelemetria(client, dist ? dist : 8190, 0.0f, 0.0f, 0);
      }
    }
  }

  analogWrite(motorIzqPin, 0);
  analogWrite(motorDerPin, 0);
  ledcWrite(servoPin, SERVO_NEUTRAL);
  resetPID();
  Serial.println("[TCP] Cliente desconectado");
}

// Protocolo MATLAB->ESP32: "modo,setpoint_mm,K,K1,Kd\n"
bool leerComando(WiFiClient& client) {
  if (!client.available()) return false;

  String ultimo = "", acum = "";
  while (client.available()) {
    char c = client.read();
    if (c == '\n') {
      acum.trim();
      if (acum.length()) ultimo = acum;
      acum = "";
    } else { acum += c; }
  }
  acum.trim();
  if (acum.length()) ultimo = acum;
  if (!ultimo.length()) return false;

  int c1 = ultimo.indexOf(',');
  int c2 = ultimo.indexOf(',', c1+1);
  int c3 = ultimo.indexOf(',', c2+1);
  int c4 = ultimo.indexOf(',', c3+1);
  if (c1 < 0 || c4 < 0) return false;

  int   nuevo_modo = ultimo.substring(0,    c1).toInt();
  float nuevo_sp   = ultimo.substring(c1+1, c2).toFloat();
  float nuevo_K    = ultimo.substring(c2+1, c3).toFloat();
  float nuevo_K1   = ultimo.substring(c3+1, c4).toFloat();
  float nuevo_Kd   = ultimo.substring(c4+1    ).toFloat();

  if (nuevo_modo != modo) {
    resetPID();
    analogWrite(motorIzqPin, 0);
    analogWrite(motorDerPin, 0);
    modo = nuevo_modo;
    modo_inicializado = false;
    Serial.printf("[MODO] -> %d\n", modo);
  }

  // Si el setpoint cambia mas de 1m, resetear PID para arrancar limpio
  if (fabsf(nuevo_sp - sp_mm) > 1000.0f) {
    resetPID();
    Serial.printf("[SP] Cambio > 1m detectado (%.0f->%.0f), reset PID\n", sp_mm, nuevo_sp);
  }

  sp_mm   = nuevo_sp;
  K_coef  = nuevo_K;
  K1_coef = nuevo_K1;
  Kd_coef = nuevo_Kd;
  return true;
}

bool leerToF(uint16_t& dist_out) {
  if (!tof_ok) { error_sensor_count = MAX_SENSOR_ERRORS; return false; }
  uint16_t dr = sensorTof.read();
  if (sensorTof.timeoutOccurred() || dr == 0 || dr > 6000) {
    error_sensor_count++;
    dist_out = last_valid_dist;
    return false;
  }
  if (last_valid_dist > 0 && abs((int)dr - (int)last_valid_dist) > 500) {
    error_sensor_count++;
    dist_out = last_valid_dist;
    return false;
  }
  error_sensor_count = 0;
  last_valid_dist = dr;
  dist_out = dr;
  return true;
}

void resetPID() {
  u_prev = 0.0f; e_k1 = 0.0f; e_k2 = 0.0f;
  error_sensor_count = 0;
  estadoEmbrague  = EMB_IDLE;
  direccionActual = SERVO_NEUTRAL;
  primer_muestra  = true;
  dist_filtrada_m = 0.0f;
}

int mapearPWM(float u_abs, int dir) {
  // Elegir el offset dependiendo de la dirección
  int offset = (dir == SERVO_DESPEGAR) ? 45 : 40;
  
  // Suma directa del offset
  int pwm = (int)u_abs + offset; 
  
  // Límite máximo de seguridad
  if (pwm > 255) pwm = 255;
  
  return pwm;
}
// Control ejecutado cada 500ms
// u[k] = u[k-1] + K*e[k] - K1*e[k-1] + Kd*e[k-2]
// e[k] = sp_m - dist_m  (metros)
void loopControlAltura(WiFiClient& client) {

  if (!modo_inicializado) {
    resetPID();
    modo_inicializado = true;
    Serial.printf("[CTRL] SP=%.0fmm K=%.3f K1=%.3f Kd=%.3f\n",
                  sp_mm, K_coef, K1_coef, Kd_coef);
  }

  uint16_t dist_mm_raw = 0;
  bool ok = leerToF(dist_mm_raw);

  if (!ok && error_sensor_count >= MAX_SENSOR_ERRORS) {
    analogWrite(motorIzqPin, 0);
    analogWrite(motorDerPin, 0);
    ledcWrite(servoPin, SERVO_NEUTRAL);
    estadoEmbrague = EMB_IDLE;
    e_k2 = e_k1 = 0.0f;
    Serial.println("[WARN] Sensor no disponible");
    enviarTelemetria(client, 8190, 0.0f, 0.0f, 0);
    return;
  }

  uint16_t dist_mm = dist_mm_raw ? dist_mm_raw : last_valid_dist;

  float dist_m_raw = (float)dist_mm / 1000.0f;
  if (primer_muestra) {
    dist_filtrada_m = dist_m_raw;
    primer_muestra  = false;
  } else {
    dist_filtrada_m = ALPHA_EMA * dist_m_raw + (1.0f - ALPHA_EMA) * dist_filtrada_m;
  }

  float sp_m = sp_mm / 1000.0f;

  // Anti-choque: blimp muy cerca del piso -> forzar subida
  if (dist_filtrada_m < DIST_MIN_M) {
    if (direccionActual != SERVO_DESPEGAR) {
      ledcWrite(servoPin, SERVO_DESPEGAR);
      direccionActual = SERVO_DESPEGAR;
      t_transit_ini   = millis();
      estadoEmbrague  = EMB_MOVING;
    }
    if (estadoEmbrague == EMB_ON ||
       (estadoEmbrague == EMB_MOVING && millis()-t_transit_ini >= TRANSIT_TIME_MS)) {
      estadoEmbrague = EMB_ON;
      analogWrite(motorIzqPin, PWM_MAX_Z);
      analogWrite(motorDerPin, PWM_MAX_Z);
    }
    e_k2 = e_k1;
    e_k1 = sp_m - dist_filtrada_m;
    enviarTelemetria(client, dist_mm, e_k1, U_MAX, PWM_MAX_Z);
    return;
  }

  float e_k = sp_m - dist_filtrada_m;

  // Zona muerta +/-15mm
 // Zona muerta +/-15mm
  if (fabsf(e_k) < ERR_DEADBAND_M) {
    analogWrite(motorIzqPin, 40); // Se quedan recibiendo 40 físicamente
    analogWrite(motorDerPin, 40);
    u_prev = 0.0f;
    e_k2 = e_k1;
    e_k1 = e_k;
    enviarTelemetria(client, dist_mm, e_k, 0.0f, 40); // La gráfica verde mostrará 40, la naranja 0
    return;
  }
  // u_prev se guarda SIN saturar para que el siguiente paso no rebote
  // Solo se satura la salida al actuador
// Ecuacion de diferencias
  float u_k = u_prev + K_coef*e_k - K1_coef*e_k1 + Kd_coef*e_k2;

  // Anti-windup: saturar PRIMERO y guardar el valor SATURADO.
  float u_k_sat = constrain(u_k, -U_MAX, U_MAX);
  u_prev = u_k_sat; // <--- AHORA SÍ, ESTO SALVARÁ EL VUELO

  e_k2 = e_k1;
  e_k1 = e_k;
  Serial.printf("[PID] dist=%.3f sp=%.3f e=%.4f u=%.2f\n",
                dist_filtrada_m, sp_m, e_k, u_k_sat);

  // u>0 -> SUBIR, u<0 -> BAJAR
  int   nueva_dir = (u_k_sat >= 0.0f) ? SERVO_DESPEGAR : SERVO_ATERRIZAR;
  float u_abs     = fabsf(u_k_sat);
  int   pwm_cmd   = mapearPWM(u_abs, nueva_dir);

  // Maquina de estados del embrague (servo necesita 350ms para girar)
  switch (estadoEmbrague) {

    case EMB_IDLE:
      direccionActual = nueva_dir;
      ledcWrite(servoPin, direccionActual);
      t_transit_ini  = millis();
      estadoEmbrague = EMB_MOVING;
      analogWrite(motorIzqPin, 0);
      analogWrite(motorDerPin, 0);
      enviarTelemetria(client, dist_mm, e_k, u_k_sat, 0);
      break;

    case EMB_MOVING:
      analogWrite(motorIzqPin, 0);
      analogWrite(motorDerPin, 0);
      if (millis() - t_transit_ini >= TRANSIT_TIME_MS)
        estadoEmbrague = EMB_ON;
      enviarTelemetria(client, dist_mm, e_k, u_k_sat, 0);
      break;

    case EMB_ON:
      if (nueva_dir != direccionActual) {
        analogWrite(motorIzqPin, 0);
        analogWrite(motorDerPin, 0);
        direccionActual = nueva_dir;
        ledcWrite(servoPin, direccionActual);
        t_transit_ini  = millis();
        estadoEmbrague = EMB_MOVING;
        enviarTelemetria(client, dist_mm, e_k, u_k_sat, 0);
        break;
      }
      analogWrite(motorIzqPin, pwm_cmd);
      analogWrite(motorDerPin, pwm_cmd);
      enviarTelemetria(client, dist_mm, e_k, u_k_sat, pwm_cmd);
      Serial.printf("[ACT] %s PWM=%d\n",
        nueva_dir == SERVO_DESPEGAR ? "SUBIR" : "BAJAR", pwm_cmd);
      break;
  }
}

// Protocolo ESP32->MATLAB: "millis,dist_mm,sp_mm,e_m,u_k,pwm,emb\n"
void enviarTelemetria(WiFiClient& client,
                      uint16_t dist, float e_m, float u_k, int pwm) {
  client.printf("%lu,%d,%.0f,%.4f,%.3f,%d,%d\n",
    millis(), dist, sp_mm, e_m, u_k, pwm, (int)estadoEmbrague);
}
