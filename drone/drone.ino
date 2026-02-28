#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// ===== WiFi SoftAP =====
static const char* AP_SSID = "C3-DRONE";
static const char* AP_PASS = "12345678";

// ===== Pins (NO CHANGE) =====
static const int PIN_M1 = 21;   // Front Left
static const int PIN_M2 = 20;   // Front Right
static const int PIN_M3 = 2;    // Rear Right
static const int PIN_M4 = 3;    // Rear Left
static const int I2C_SDA = 7;
static const int I2C_SCL = 6;

// ===== Status LED =====
static const int STATUS_LED = 8;  // ESP32-C3 SuperMini onboard LED (GPIO8)

// ===== PWM =====
static const int PWM_FREQ = 20000;
static const int PWM_RES_BITS = 8;
static const int PWM_MAX = (1 << PWM_RES_BITS) - 1;
static const int CH_M1 = 0, CH_M2 = 1, CH_M3 = 2, CH_M4 = 3;

// Coreless tune
static const int MOTOR_MIN_START = 35;
static const int MOTOR_MAX_LIMIT = 240;
static const int THR_RAMP_PER_LOOP = 3;

// Loop
static const float LOOP_HZ = 250.0f;
static const uint32_t LOOP_US = (uint32_t)(1000000.0f / LOOP_HZ);

// Filter / Expo
static const float CF_ALPHA = 0.98f;
static const float EXPO = 0.35f;

// Safety timeouts (more stable than 350ms)
static const uint32_t CMD_TIMEOUT_MS = 1000;      // throttle cut if no packets
static const uint32_t DISARM_TIMEOUT_MS = 6000;   // auto disarm if no packets

enum FlightMode { MODE_ANGLE = 0, MODE_RATE = 1 };

// PID (start point)
static float Kp_angle = 4.5f;

static float Kp_rate_rp = 0.09f;
static float Ki_rate_rp = 0.18f;
static float Kd_rate_rp = 0.0025f;

static float Kp_rate_y = 0.12f;
static float Ki_rate_y = 0.10f;
static float Kd_rate_y = 0.0f;

static const float MAX_ANGLE_DEG = 30.0f;
static const float MAX_RATE_RP = 220.0f;
static const float MAX_RATE_Y  = 180.0f;

// ===== Globals =====
Adafruit_MPU6050 mpu;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

volatile bool armed = false;
volatile bool killSwitch = false;
volatile FlightMode mode = MODE_ANGLE;
volatile uint32_t lastCmdMs = 0;

// Inputs
volatile float in_throttle = 0.0f;  // 0..1
volatile float in_roll = 0.0f;      // -1..1
volatile float in_pitch = 0.0f;     // -1..1
volatile float in_yaw = 0.0f;       // -1..1
volatile float in_thrLimit = 1.0f;  // 0.2..1

// State
static float roll_deg = 0, pitch_deg = 0;
static float gyro_bias_x = 0, gyro_bias_y = 0, gyro_bias_z = 0;

// PID
static float i_roll = 0, i_pitch = 0, i_yaw = 0;
static float last_err_r = 0, last_err_p = 0, last_err_y = 0;

// Throttle smoothing
static int thr_duty_smooth = 0;

// ===== Utils =====
static inline float clampf(float x, float a, float b) { return (x < a) ? a : (x > b) ? b : x; }
static inline float expoCurve(float x, float expo) {
  float ax = fabsf(x);
  float y = (1.0f - expo) * x + expo * x * ax * ax;
  return clampf(y, -1.0f, 1.0f);
}
static void motorWrite(int ch, int duty) { ledcWrite(ch, constrain(duty, 0, PWM_MAX)); }
static void allMotorsOff() { motorWrite(CH_M1,0); motorWrite(CH_M2,0); motorWrite(CH_M3,0); motorWrite(CH_M4,0); }

// ===== Status LED =====
static void updateStatusLED() {
  // blink while no station connected to SoftAP, solid ON when connected
  if (WiFi.softAPgetStationNum() > 0) {
    digitalWrite(STATUS_LED, HIGH);
  } else {
    static uint32_t t = 0;
    static bool s = false;
    if (millis() - t > 250) {
      t = millis();
      s = !s;
      digitalWrite(STATUS_LED, s ? HIGH : LOW);
    }
  }
}

// ===== Calibration =====
static void resetPid() {
  i_roll = i_pitch = i_yaw = 0;
  last_err_r = last_err_p = last_err_y = 0;
}
static void calibrateGyro(uint16_t samples = 800) {
  float sx = 0, sy = 0, sz = 0;
  sensors_event_t a, g, t;
  for (uint16_t i = 0; i < samples; i++) {
    mpu.getEvent(&a, &g, &t);
    sx += g.gyro.x; sy += g.gyro.y; sz += g.gyro.z;
    delay(2);
  }
  gyro_bias_x = sx / samples;
  gyro_bias_y = sy / samples;
  gyro_bias_z = sz / samples;
  resetPid();
}

// ===== Mini page (optional) =====
static const char mini_html[] PROGMEM =
  "<!doctype html><html><body style='font-family:sans-serif'>"
  "<h3>C3 DRONE</h3><p>Use ESP-Drone II app. WS endpoint: <b>/ws</b></p>"
  "</body></html>";

// ===== WebSocket Handler (ESP-Drone II protocol) =====
static void onWsEvent(AsyncWebSocket *s, AsyncWebSocketClient *c,
                      AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type != WS_EVT_DATA || len == 0) return;

  String msg; msg.reserve(len + 1);
  for (size_t i = 0; i < len; i++) msg += (char)data[i];
  lastCmdMs = millis();

  if (msg == "ARM")    { killSwitch=false; armed=true;  resetPid(); thr_duty_smooth=0; c->text("ARMED"); return; }
  if (msg == "DISARM") { armed=false; c->text("DISARMED"); return; }
  if (msg == "KILL1")  { killSwitch=true; armed=false; c->text("KILL ON"); return; }
  if (msg == "KILL0")  { killSwitch=false; c->text("KILL OFF"); return; }
  if (msg == "MODE0")  { mode=MODE_ANGLE; c->text("MODE ANGLE"); return; }
  if (msg == "MODE1")  { mode=MODE_RATE;  c->text("MODE RATE");  return; }
  if (msg == "CAL")    { armed=false; c->text("CAL..."); calibrateGyro(); c->text("CAL OK"); return; }

  // Control: C:throttle,roll,pitch,yaw,limit   (0..1000, -1000..1000, 0..1000)
  if (msg.startsWith("C:")) {
    int t, r, p, y, lim;
    if (sscanf(msg.c_str(), "C:%d,%d,%d,%d,%d", &t, &r, &p, &y, &lim) == 5) {
      float thr = clampf(t / 1000.0f, 0.0f, 1.0f);
      float rr  = clampf(r / 1000.0f, -1.0f, 1.0f);
      float pp  = clampf(p / 1000.0f, -1.0f, 1.0f);
      float yy  = clampf(y / 1000.0f, -1.0f, 1.0f);
      float ll  = clampf(lim / 1000.0f, 0.2f, 1.0f);

      in_throttle = thr;
      in_roll  = expoCurve(rr, EXPO);
      in_pitch = expoCurve(pp, EXPO);
      in_yaw   = expoCurve(yy, EXPO);
      in_thrLimit = ll;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(50);

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  if (!mpu.begin()) { while (1) delay(100); }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  ledcSetup(CH_M1, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(CH_M2, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(CH_M3, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(CH_M4, PWM_FREQ, PWM_RES_BITS);
  ledcAttachPin(PIN_M1, CH_M1);
  ledcAttachPin(PIN_M2, CH_M2);
  ledcAttachPin(PIN_M3, CH_M3);
  ledcAttachPin(PIN_M4, CH_M4);
  allMotorsOff();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req){ req->send_P(200, "text/html", mini_html); });
  server.begin();

  delay(400);
  calibrateGyro();         // keep still at boot
  lastCmdMs = millis();
}

void loop() {
  ws.cleanupClients();
  updateStatusLED();

  const uint32_t nowMs = millis();

  // Failsafe
  if (armed && (nowMs - lastCmdMs > CMD_TIMEOUT_MS)) {
    in_throttle = 0.0f; in_roll = 0.0f; in_pitch = 0.0f; in_yaw = 0.0f;
  }
  if (armed && (nowMs - lastCmdMs > DISARM_TIMEOUT_MS)) {
    armed = false;
  }
  if (killSwitch) armed = false;

  static uint32_t lastUs = micros();
  const uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastUs) < LOOP_US) return;

  float dt = (nowUs - lastUs) / 1000000.0f;
  lastUs = nowUs;
  if (dt < 0.002f) dt = 0.002f;
  if (dt > 0.02f)  dt = 0.02f;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Gyro (deg/s) bias removed
  float gx = (g.gyro.x - gyro_bias_x) * 180.0f / PI;
  float gy = (g.gyro.y - gyro_bias_y) * 180.0f / PI;
  float gz = (g.gyro.z - gyro_bias_z) * 180.0f / PI;

  // Acc angles
  float accRoll  = atan2f(a.acceleration.y, a.acceleration.z) * 180.0f / PI;
  float accPitch = atan2f(-a.acceleration.x,
                          sqrtf(a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z)) * 180.0f / PI;

  // Complementary
  roll_deg  = CF_ALPHA * (roll_deg  + gx * dt) + (1.0f - CF_ALPHA) * accRoll;
  pitch_deg = CF_ALPHA * (pitch_deg + gy * dt) + (1.0f - CF_ALPHA) * accPitch;

  // Desired rates
  float des_rate_roll = 0, des_rate_pitch = 0, des_rate_yaw = 0;

  if (mode == MODE_ANGLE) {
    float des_ang_roll  = in_roll  * MAX_ANGLE_DEG;
    float des_ang_pitch = in_pitch * MAX_ANGLE_DEG;
    des_rate_roll  = clampf((des_ang_roll  - roll_deg)  * Kp_angle, -MAX_RATE_RP, MAX_RATE_RP);
    des_rate_pitch = clampf((des_ang_pitch - pitch_deg) * Kp_angle, -MAX_RATE_RP, MAX_RATE_RP);
  } else {
    des_rate_roll  = in_roll  * MAX_RATE_RP;
    des_rate_pitch = in_pitch * MAX_RATE_RP;
  }
  des_rate_yaw = in_yaw * MAX_RATE_Y;

  // Rate PID Roll
  float err_r = des_rate_roll - gx;
  i_roll = clampf(i_roll + err_r * dt, -200.0f, 200.0f);
  float d_r = (err_r - last_err_r) / dt; last_err_r = err_r;
  float out_r = Kp_rate_rp * err_r + Ki_rate_rp * i_roll + Kd_rate_rp * d_r;

  // Rate PID Pitch
  float err_p = des_rate_pitch - gy;
  i_pitch = clampf(i_pitch + err_p * dt, -200.0f, 200.0f);
  float d_p = (err_p - last_err_p) / dt; last_err_p = err_p;
  float out_p = Kp_rate_rp * err_p + Ki_rate_rp * i_pitch + Kd_rate_rp * d_p;

  // Rate PID Yaw
  float err_y = des_rate_yaw - gz;
  i_yaw = clampf(i_yaw + err_y * dt, -200.0f, 200.0f);
  float d_y = (err_y - last_err_y) / dt; last_err_y = err_y;
  float out_y = Kp_rate_y * err_y + Ki_rate_y * i_yaw + Kd_rate_y * d_y;

  // Throttle -> duty
  float thr = clampf(in_throttle, 0.0f, 1.0f) * clampf(in_thrLimit, 0.2f, 1.0f);

  int targetDuty = 0;
  if (armed && thr > 0.001f) {
    int usable = MOTOR_MAX_LIMIT - MOTOR_MIN_START;
    targetDuty = MOTOR_MIN_START + (int)(thr * usable);
  } else {
    targetDuty = 0;
  }

  // Ramp throttle
  if (targetDuty > thr_duty_smooth) thr_duty_smooth = min(targetDuty, thr_duty_smooth + THR_RAMP_PER_LOOP);
  else                              thr_duty_smooth = max(targetDuty, thr_duty_smooth - THR_RAMP_PER_LOOP);

  float base = (float)thr_duty_smooth;

  const float yawSign = 1.0f; // if yaw reversed, set -1.0f
  float yawTerm = yawSign * out_y;

  // X-mix (M1 FL, M2 FR, M3 RR, M4 RL)
  float m1 = base - out_p + out_r - yawTerm;
  float m2 = base - out_p - out_r + yawTerm;
  float m3 = base + out_p - out_r - yawTerm;
  float m4 = base + out_p + out_r + yawTerm;

  // Headroom clamp
  float maxOut = fmaxf(fmaxf(m1, m2), fmaxf(m3, m4));
  if (maxOut > MOTOR_MAX_LIMIT) {
    float shift = maxOut - MOTOR_MAX_LIMIT;
    m1 -= shift; m2 -= shift; m3 -= shift; m4 -= shift;
  }

  m1 = clampf(m1, 0, PWM_MAX);
  m2 = clampf(m2, 0, PWM_MAX);
  m3 = clampf(m3, 0, PWM_MAX);
  m4 = clampf(m4, 0, PWM_MAX);

  if (!armed || killSwitch) {
    allMotorsOff();
  } else {
    motorWrite(CH_M1, (int)m1);
    motorWrite(CH_M2, (int)m2);
    motorWrite(CH_M3, (int)m3);
    motorWrite(CH_M4, (int)m4);
  }

  // Debug (optional)
  static uint32_t dbgMs = 0;
  if (nowMs - dbgMs > 250) {
    dbgMs = nowMs;
    char buf[80];
    snprintf(buf, sizeof(buf), "ARM:%d MODE:%c R:%.1f P:%.1f STA:%d",
             armed ? 1 : 0, (mode == MODE_ANGLE) ? 'A' : 'R',
             roll_deg, pitch_deg, WiFi.softAPgetStationNum());
    ws.textAll(buf);
  }
}
