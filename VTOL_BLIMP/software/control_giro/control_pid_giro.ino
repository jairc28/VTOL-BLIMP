/*
 * control_pid_giro.ino — blimp_definitivo/control_giro
 * ============================================================
 * Control PID discreto de Yaw con opción de mapeo de PWM.
 * IMU: MPU6050 (calibración automática en setup, 5 s)
 *
 * PROTOCOLO TCP:
 *   MATLAB → ESP32 : "modo,sp_deg,K,K1,Kd,mapeo\n"
 *                     modo: 0=STOP 1=PID 2=Proporcional 3=LazoAbierto
 *                     mapeo: 0=directo  1=interpolado [50-255]
 *   ESP32  → MATLAB: "millis,yaw,sp,e,u_k,pwm_izq,pwm_der,omega,mapeo\n"
 *
 * Modo 3 — lazo abierto (OL):
 *   MATLAB → "3,pwm,t_pulso_s,t_descanso_s,0"
 * ============================================================
 */

#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ── Pines ────────────────────────────────────────────────────
const int motorIzqPin = 4;
const int motorDerPin = 5;
const int servoPin    = 2;
const int sdaPin      = 6;
const int sclPin      = 7;
const int SERVO_HORIZONTAL = 1200;

// ── WiFi ─────────────────────────────────────────────────────
const char* ssid     = "iPhone de HP";
const char* password = "Contra12";
WiFiServer server(80);

// ── IMU ──────────────────────────────────────────────────────
Adafruit_MPU6050 mpu;
bool  mpu_ok          = false;
float gyroZ_offset    = 0.0f;
float yaw_angle       = 0.0f;
float omega_z_dps     = 0.0f;
unsigned long t_ultimo_imu_us = 0;

// ── Límites PWM ──────────────────────────────────────────────
const float U_MAX     = 255.0f;
const int   PWM_MIN_Z = 50;
const int   PWM_MAX_Z = 255;

// ── Zona muerta ──────────────────────────────────────────────
const float ERR_DEADBAND_DEG = 2.0f;

// ── Estado PID ───────────────────────────────────────────────
int   modo         = 0;
bool  mapeo_activo = false;

float sp_deg        = 0.0f;
float ol_pwm        = 50.0f;
float ol_t_pulso    = 20.0f;
float ol_t_descanso = 30.0f;
const float K_A  = -91.072f, K1_A = -180.304f, Kd_A = -89.242f;
const float K_B  =  3.00f, K1_B =  5.93f, Kd_B =  2.93f;
const float COEF_TOL = 0.01f;

float K_coef  = K_A, K1_coef = K1_A, Kd_coef = Kd_A;
float u_prev = 0.0f, e_k1 = 0.0f, e_k2 = 0.0f;

int           ol_fase        = 4;
unsigned long ol_fase_ini_ms = 0;

const unsigned long T_MS = 50;
unsigned long last_pid_ms      = 0;
bool          modo_inicializado = false;

// ── Prototipos ───────────────────────────────────────────────
void  calibrarMPU();
void  actualizarIMU();
float wrapAngle(float a);
void  resetPID();
void  mapearActuadores(float u_k, bool usar_mapeo, int& izq, int& der);
void  loopPID(WiFiClient& c);
void  loopProporcional(WiFiClient& c);
void  loopLazoAbierto(WiFiClient& c);
bool  leerComando(WiFiClient& c);
void  enviarTelemetria(WiFiClient& c, float yaw, float sp, float e,
                       float u_k, int izq, int der, float omega);

// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Wire.begin(sdaPin, sclPin);

  pinMode(motorIzqPin, OUTPUT); pinMode(motorDerPin, OUTPUT);
  analogWrite(motorIzqPin, 0); analogWrite(motorDerPin, 0);

  ledcAttach(servoPin, 50, 14);
  ledcWrite(servoPin, SERVO_HORIZONTAL);

  if (mpu.begin()) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
    calibrarMPU();
  } else {
    Serial.println("[MPU] ERROR — verificar I2C");
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("[WiFi] Conectando");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.printf("\n[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
  server.begin();
  Serial.println("[Server] Puerto 80 listo");
}

// ─────────────────────────────────────────────────────────────
void calibrarMPU() {
  Serial.println("[MPU] Calibrando 5s — NO mover.");
  float acc = 0.0f;
  const int N = 500;
  for (int i = 0; i < N; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    acc += g.gyro.z;
    delay(10);
    if (i % 100 == 0) Serial.printf("  %d%%\n", i*100/N);
  }
  gyroZ_offset = acc / (float)N;
  mpu_ok = true; yaw_angle = 0.0f; t_ultimo_imu_us = 0;
  Serial.printf("[MPU] OK gyroZ_offset=%.5f rad/s\n", gyroZ_offset);
}

// ─────────────────────────────────────────────────────────────
void actualizarIMU() {
  if (!mpu_ok) return;
  unsigned long ahora_us = micros();
  
  float dt = (t_ultimo_imu_us == 0) ? 0.05f :
             (float)(ahora_us - t_ultimo_imu_us) / 1e6f;
  if (dt < 0.001f || dt > 0.5f) dt = 0.05f;
  t_ultimo_imu_us = ahora_us;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float gz = g.gyro.z - gyroZ_offset;
  omega_z_dps = gz * (180.0f / PI);

  if (fabsf(gz) < 10.0f) {
    float nuevo = yaw_angle + gz * dt * (180.0f / PI);
    while (nuevo >  180.0f) nuevo -= 360.0f;
    while (nuevo < -180.0f) nuevo += 360.0f;
    yaw_angle = nuevo;
  }
}

// ─────────────────────────────────────────────────────────────
void loop() {
  actualizarIMU();
  WiFiClient client = server.available();
  if (!client) { delay(10); return; }

  client.setNoDelay(true);
  Serial.println("[TCP] Cliente conectado");
  modo = 0; modo_inicializado = false; ol_fase = 4; mapeo_activo = false;
  resetPID();
  analogWrite(motorIzqPin, 0); analogWrite(motorDerPin, 0);
  ledcWrite(servoPin, SERVO_HORIZONTAL);

  while (client.connected()) {
    leerComando(client);
    if (millis() - last_pid_ms >= T_MS) {
      actualizarIMU();
      last_pid_ms = millis();
      switch (modo) {
        case 1: loopPID(client);          break;
        case 2: loopProporcional(client); break;
        case 3: loopLazoAbierto(client);  break;
        default:
          analogWrite(motorIzqPin, 0); analogWrite(motorDerPin, 0);
          ledcWrite(servoPin, SERVO_HORIZONTAL);
          enviarTelemetria(client, yaw_angle, sp_deg, 0.0f, 0.0f, 0, 0, omega_z_dps);
          break;
      }
    }
    delay(1);
  }

  analogWrite(motorIzqPin, 0); analogWrite(motorDerPin, 0);
  ledcWrite(servoPin, SERVO_HORIZONTAL);
  resetPID(); ol_fase = 4;
  Serial.println("[TCP] Cliente desconectado");
}

// ── leerComando — "modo,sp_deg,K,K1,Kd,mapeo\n" ─────────────
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
    if (c1 < 0 || c4 < 0) return false;

    int   nuevo_modo  = ultimo.substring(0,    c1).toInt();
    float campo2      = ultimo.substring(c1+1, c2).toFloat();
    float campo3      = ultimo.substring(c2+1, c3).toFloat();
    float campo4      = ultimo.substring(c3+1, c4).toFloat();
    float campo5      = (c5 > 0) ? ultimo.substring(c4+1, c5).toFloat()
                                  : ultimo.substring(c4+1).toFloat();
    bool  nuevo_mapeo = (c5 > 0) ? (ultimo.substring(c5+1).toInt() != 0) : mapeo_activo;

    if (nuevo_modo != modo) {
      resetPID(); modo = nuevo_modo; modo_inicializado = false; ol_fase = 4;
      Serial.printf("[MODO] → %d\n", modo);
    }
    mapeo_activo = nuevo_mapeo;

    if (modo == 3) {
      ol_pwm = campo2; ol_t_pulso = campo3; ol_t_descanso = campo4;
    } else {
      sp_deg = campo2;
      if (fabsf(campo3 - K_coef)  > COEF_TOL ||
          fabsf(campo4 - K1_coef) > COEF_TOL ||
          fabsf(campo5 - Kd_coef) > COEF_TOL) {
        K_coef = campo3; K1_coef = campo4; Kd_coef = campo5;
        Serial.printf("[COEF] K=%.4f K1=%.4f Kd=%.4f\n", K_coef, K1_coef, Kd_coef);
      }
    }
    return true;
  }
  return false;
}

// ── Utilidades ───────────────────────────────────────────────
float wrapAngle(float a) {
  while (a >  180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

void resetPID() {
  u_prev = 0.0f; e_k1 = 0.0f; e_k2 = 0.0f;
  ledcWrite(servoPin, SERVO_HORIZONTAL);
}

void mapearActuadores(float u_k, int& pwm_izq, int& pwm_der) {
  float u_abs = fabsf(u_k);
  int   pwm   = (int)constrain(u_abs, 0.0f, (float)U_MAX);
  if      (u_k >  0.5f) { pwm_izq = pwm; pwm_der = 0;   }
  else if (u_k < -0.5f) { pwm_izq = 0;   pwm_der = pwm; }
  else                  { pwm_izq = 0;   pwm_der = 0;   }
}

// ── loopPID ───────────────────────────────────────────────────
void loopPID(WiFiClient& client) {
  if (!modo_inicializado) {
    resetPID(); modo_inicializado = true;
    Serial.printf("[PID] SP=%.1f° K=%.4f K1=%.4f Kd=%.4f Mapeo=%s\n",
                  sp_deg, K_coef, K1_coef, Kd_coef, mapeo_activo?"SI":"NO");
  }
  ledcWrite(servoPin, SERVO_HORIZONTAL);
  float e_k = wrapAngle(sp_deg - yaw_angle);

  if (fabsf(e_k) < ERR_DEADBAND_DEG) {
    analogWrite(motorIzqPin, 0); analogWrite(motorDerPin, 0);
    u_prev = 0.0f; e_k2 = e_k1; e_k1 = e_k;
    enviarTelemetria(client, yaw_angle, sp_deg, e_k, 0.0f, 0, 0, omega_z_dps);
    return;
  }

  float u_k     = u_prev + K_coef*e_k - K1_coef*e_k1 + Kd_coef*e_k2;
  float u_k_sat = constrain(u_k, -U_MAX, U_MAX);
  u_prev = u_k_sat;   // Anti-windup: valor saturado
  e_k2 = e_k1; e_k1 = e_k;

  int pwm_izq, pwm_der;
  mapearActuadores(u_k_sat, pwm_izq, pwm_der);
  analogWrite(motorIzqPin, pwm_izq); analogWrite(motorDerPin, pwm_der);
  enviarTelemetria(client, yaw_angle, sp_deg, e_k, u_k_sat, pwm_izq, pwm_der, omega_z_dps);
  Serial.printf("[PID] e=%.2f u=%.2f I=%d D=%d\n", e_k, u_k_sat, pwm_izq, pwm_der);
}

// ── loopProporcional ─────────────────────────────────────────
void loopProporcional(WiFiClient& client) {
  if (!modo_inicializado) {
    resetPID(); modo_inicializado = true;
    Serial.printf("[P] SP=%.1f° K=%.4f\n", sp_deg, K_coef);
  }
  ledcWrite(servoPin, SERVO_HORIZONTAL);
  float e_k     = wrapAngle(sp_deg - yaw_angle);
  float u_k     = K_coef * e_k;
  float u_k_sat = constrain(u_k, -U_MAX, U_MAX);

  int pwm_izq, pwm_der;
  if (fabsf(e_k) < ERR_DEADBAND_DEG) { pwm_izq = pwm_der = 0; u_k_sat = 0.0f; }
  else { mapearActuadores(u_k_sat, pwm_izq, pwm_der); }

  analogWrite(motorIzqPin, pwm_izq); analogWrite(motorDerPin, pwm_der);
  enviarTelemetria(client, yaw_angle, sp_deg, e_k, u_k_sat, pwm_izq, pwm_der, omega_z_dps);
}

// ── loopLazoAbierto ──────────────────────────────────────────
void loopLazoAbierto(WiFiClient& client) {
  if (!modo_inicializado) {
    ol_fase = 0; ol_fase_ini_ms = millis(); modo_inicializado = true;
    Serial.printf("[OL] PWM=%.0f Pulso=%.0fs Descanso=%.0fs\n",
                  ol_pwm, ol_t_pulso, ol_t_descanso);
  }
  ledcWrite(servoPin, SERVO_HORIZONTAL);

  unsigned long dur = millis() - ol_fase_ini_ms;
  unsigned long t_p = (unsigned long)(ol_t_pulso    * 1000.0f);
  unsigned long t_d = (unsigned long)(ol_t_descanso * 1000.0f);

  bool avanzar = false;
  switch (ol_fase) {
    case 0: avanzar = (dur >= t_p); break;
    case 1: avanzar = (dur >= t_d); break;
    case 2: avanzar = (dur >= t_p); break;
    case 3: avanzar = (dur >= t_d); break;
  }
  if (avanzar && ol_fase < 4) { ol_fase++; ol_fase_ini_ms = millis(); }

  int   pwm_izq = 0, pwm_der = 0;
  float u_k     = 0.0f;
  int   pwm_cmd = constrain((int)ol_pwm, 0, PWM_MAX_Z);
  switch (ol_fase) {
    case 0: pwm_izq = pwm_cmd; u_k =  ol_pwm; break;
    case 1: break;
    case 2: pwm_der = pwm_cmd; u_k = -ol_pwm; break;
    case 3: break;
    case 4: break;
  }
  analogWrite(motorIzqPin, pwm_izq); analogWrite(motorDerPin, pwm_der);
  enviarTelemetria(client, yaw_angle, ol_pwm, (float)ol_fase, u_k,
                   pwm_izq, pwm_der, omega_z_dps);
}

// ── enviarTelemetria ─────────────────────────────────────────
void enviarTelemetria(WiFiClient& client, float yaw, float sp, float e,
                      float u_k, int izq, int der, float omega) {
  client.printf("%lu,%.2f,%.2f,%.2f,%.3f,%d,%d,%.3f,%d\n",
    millis(), yaw, sp, e, u_k, izq, der, omega, (int)mapeo_activo);
}
