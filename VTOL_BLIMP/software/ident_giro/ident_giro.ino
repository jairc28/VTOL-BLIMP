/*
 * ident_giro.ino — blimp_definitivo/ident_giro
 * ============================================================
 * Identificación de giro (Yaw) — dos modos:
 *   Modo 1: Lazo ABIERTO  — pulsos alternos con N ciclos
 *   Modo 2: Lazo CERRADO  — control proporcional K
 *
 * IMU: MPU6050
 * Giro positivo → Motor IZQ | Giro negativo → Motor DER
 *
 * PROTOCOLO TCP:
 *   MATLAB → ESP32 :
 *     Modo 0 (STOP):      "0,0,0,0,0\n"
 *     Modo 1 (LA):        "1,pwm,t_on_s,t_off_s,n_ciclos\n"
 *     Modo 2 (LC K):      "2,sp_deg,K,mapeo,0\n"
 *                          mapeo: 0=directo 1=interpolado
 *   ESP32 → MATLAB:
 *     "millis,yaw,sp_o_pwm,e_o_fase,u_k,pwm_izq,pwm_der,omega,fase\n"
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
const int SERVO_HORIZONTAL = 1300;

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

// ── Zona muerta (lazo cerrado) ───────────────────────────────
const float ERR_DEADBAND_DEG = 2.0f;

// ── Estado general ───────────────────────────────────────────
int   modo         = 0;
bool  mapeo_activo = false;
bool  modo_inicializado = false;

// ── Parámetros Modo 1 (Lazo Abierto) ────────────────────────
float ol_pwm        = 50.0f;  // PWM de prueba configurable
float ol_t_on_s     = 20.0f;  // 20s motor encendido
float ol_t_off_s    = 20.0f;  // 20s reposo
int   ol_n_ciclos   = 3;      // número de ciclos (IZQ ON -> OFF -> DER ON -> OFF)

// Máquina de estados OL:
// fase 0: IZQ ON | fase 1: REPOSO | fase 2: DER ON | fase 3: REPOSO
// Se repite ol_n_ciclos veces, luego fase=99 (DONE)
int           ol_fase        = 99;
int           ol_ciclo_actual = 0;
unsigned long ol_fase_ini_ms = 0;

// ── Parámetros Modo 2 (Proporcional K) ──────────────────────
float sp_deg   = 90.0f;   // Setpoint por defecto a 90 grados
float K_coef   = -0.5f;   // Ganancias típicas: -0.5 o -0.7

// ── Temporización ────────────────────────────────────────────
const unsigned long T_MS = 50;   // 20 Hz
unsigned long last_ctrl_ms = 0;

// ── Prototipos ───────────────────────────────────────────────
void  calibrarMPU();
void  actualizarIMU();
float wrapAngle(float a);
void  mapearActuadores(float u_k, bool usar_mapeo, int& izq, int& der);
void  loopLazoAbierto(WiFiClient& c);
void  loopProporcional(WiFiClient& c);
bool  leerComando(WiFiClient& c);
void  enviarTelemetria(WiFiClient& c, float yaw, float sp_o_pwm,
                       float e_o_fase, float u_k, int izq, int der,
                       float omega, int fase);
void  stopMotores();

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
  Serial.println("[Server] Puerto 80 listo — ident_giro");
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
  modo = 0; modo_inicializado = false; mapeo_activo = false;
  ol_fase = 99; ol_ciclo_actual = 0;
  stopMotores();

  while (client.connected()) {
    leerComando(client);
    if (millis() - last_ctrl_ms >= T_MS) {
      actualizarIMU();
      last_ctrl_ms = millis();
      switch (modo) {
        case 1: loopLazoAbierto(client);  break;
        case 2: loopProporcional(client); break;
        default:
          stopMotores();
          enviarTelemetria(client, yaw_angle, 0.0f, 0.0f, 0.0f, 0, 0, omega_z_dps, 99);
          break;
      }
    }
    delay(1);
  }

  stopMotores();
  Serial.println("[TCP] Cliente desconectado");
}

// ── leerComando ──────────────────────────────────────────────
// Modo 1: "1,pwm,t_on_s,t_off_s,n_ciclos\n"
// Modo 2: "2,sp_deg,K,mapeo,0\n"
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
    if (c1 < 0 || c4 < 0) return false;

    int   nuevo_modo = ultimo.substring(0,    c1).toInt();
    float f2         = ultimo.substring(c1+1, c2).toFloat();
    float f3         = ultimo.substring(c2+1, c3).toFloat();
    float f4         = ultimo.substring(c3+1, c4).toFloat();
    float f5         = ultimo.substring(c4+1).toFloat();

    if (nuevo_modo != modo) {
      modo = nuevo_modo; modo_inicializado = false;
      ol_fase = 99; ol_ciclo_actual = 0;
      stopMotores();
      Serial.printf("[MODO] → %d\n", modo);
    }

    if (modo == 1) {
      ol_pwm      = f2;
      ol_t_on_s   = f3;
      ol_t_off_s  = f4;
      ol_n_ciclos = (int)f5;
    } else if (modo == 2) {
      sp_deg       = f2;
      K_coef       = f3;
      mapeo_activo = ((int)f4 != 0);
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

void stopMotores() {
  analogWrite(motorIzqPin, 0); analogWrite(motorDerPin, 0);
  ledcWrite(servoPin, SERVO_HORIZONTAL);
}

void mapearActuadores(float u_k, int& pwm_izq, int& pwm_der) {
  float u_abs = fabsf(u_k);
  int   pwm   = (int)constrain(u_abs, 0.0f, (float)U_MAX);
  if      (u_k >  0.5f) { pwm_izq = pwm; pwm_der = 0;   }
  else if (u_k < -0.5f) { pwm_izq = 0;   pwm_der = pwm; }
  else                  { pwm_izq = 0;   pwm_der = 0;   }
}

// ── loopLazoAbierto — Modo 1 ─────────────────────────────────
// Fases: 0=IZQ_ON, 1=REPOSO, 2=DER_ON, 3=REPOSO → repite n_ciclos
void loopLazoAbierto(WiFiClient& client) {
  if (!modo_inicializado) {
    ol_fase = 0; ol_ciclo_actual = 0;
    ol_fase_ini_ms = millis(); modo_inicializado = true;
    Serial.printf("[OL] PWM=%.0f ton=%.1fs toff=%.1fs ciclos=%d\n",
                  ol_pwm, ol_t_on_s, ol_t_off_s, ol_n_ciclos);
  }

  if (ol_fase == 99) {
    // Secuencia terminada
    stopMotores();
    enviarTelemetria(client, yaw_angle, ol_pwm, 99.0f, 0.0f, 0, 0, omega_z_dps, 99);
    return;
  }

  ledcWrite(servoPin, SERVO_HORIZONTAL);
  unsigned long dur     = millis() - ol_fase_ini_ms;
  unsigned long t_on_ms  = (unsigned long)(ol_t_on_s  * 1000.0f);
  unsigned long t_off_ms = (unsigned long)(ol_t_off_s * 1000.0f);

  // Duración de la fase actual
  unsigned long dur_fase = (ol_fase % 2 == 0) ? t_on_ms : t_off_ms;

  if (dur >= dur_fase) {
    ol_fase++;
    ol_fase_ini_ms = millis();
    if (ol_fase >= 4) {
      ol_ciclo_actual++;
      if (ol_ciclo_actual >= ol_n_ciclos) {
        ol_fase = 99;  // Secuencia completa
        stopMotores();
        Serial.println("[OL] Secuencia terminada");
        return;
      } else {
        ol_fase = 0;   // Nuevo ciclo
        Serial.printf("[OL] Ciclo %d/%d\n", ol_ciclo_actual+1, ol_n_ciclos);
      }
    }
    const char* desc[] = {"IZQ ON","REPOSO","DER ON","REPOSO"};
    if (ol_fase < 4) Serial.printf("[OL] Fase %s | Ciclo %d/%d\n",
                                   desc[ol_fase], ol_ciclo_actual+1, ol_n_ciclos);
  }

  int   pwm_izq = 0, pwm_der = 0;
  float u_k     = 0.0f;
  int   pwm_cmd = constrain((int)ol_pwm, 0, PWM_MAX_Z);

  switch (ol_fase) {
    case 0: pwm_izq = pwm_cmd; u_k =  ol_pwm; break; // IZQ ON
    case 1: break;                                      // REPOSO
    case 2: pwm_der = pwm_cmd; u_k = -ol_pwm; break; // DER ON
    case 3: break;                                      // REPOSO
  }
  analogWrite(motorIzqPin, pwm_izq); analogWrite(motorDerPin, pwm_der);
  enviarTelemetria(client, yaw_angle, ol_pwm, (float)ol_fase, u_k,
                   pwm_izq, pwm_der, omega_z_dps, ol_fase);
}

// ── loopProporcional — Modo 2 ────────────────────────────────
void loopProporcional(WiFiClient& client) {
  if (!modo_inicializado) {
    modo_inicializado = true;
    Serial.printf("[P] SP=%.1f° K=%.4f Mapeo=%s\n",
                  sp_deg, K_coef, mapeo_activo?"SI":"NO");
  }
  ledcWrite(servoPin, SERVO_HORIZONTAL);
  float e_k     = wrapAngle(sp_deg - yaw_angle);
  float u_k     = K_coef * e_k;
  float u_k_sat = constrain(u_k, -U_MAX, U_MAX);

  int pwm_izq = 0, pwm_der = 0;
  if (fabsf(e_k) < ERR_DEADBAND_DEG) { u_k_sat = 0.0f; }
  else { mapearActuadores(u_k_sat, pwm_izq, pwm_der); }

  analogWrite(motorIzqPin, pwm_izq); analogWrite(motorDerPin, pwm_der);
  enviarTelemetria(client, yaw_angle, sp_deg, e_k, u_k_sat,
                   pwm_izq, pwm_der, omega_z_dps, 0);
  Serial.printf("[P] e=%.2f u=%.2f I=%d D=%d\n", e_k, u_k_sat, pwm_izq, pwm_der);
}

// ── enviarTelemetria ─────────────────────────────────────────
void enviarTelemetria(WiFiClient& client, float yaw, float sp_o_pwm,
                      float e_o_fase, float u_k, int izq, int der,
                      float omega, int fase) {
  client.printf("%lu,%.2f,%.2f,%.2f,%.3f,%d,%d,%.3f,%d\n",
    millis(), yaw, sp_o_pwm, e_o_fase, u_k, izq, der, omega, fase);
}
