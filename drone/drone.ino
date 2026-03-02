#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include "driver/ledc.h"

// =================== PWM CONFIG ===================
const int PWM_FREQ = 8000;                         // brushed
const ledc_timer_bit_t PWM_RES = LEDC_TIMER_8_BIT; // 0..255
const int PWM_MAX = 255;

// =================== YOUR MOTOR PINS ===================
// M1 GPIO21 → Front Left  (CW)
// M2 GPIO20 → Front Right (CCW)
// M3 GPIO10 → Back Right  (CW)
// M4 GPIO5  → Back Left   (CCW)
const int PIN_M1 = 21;
const int PIN_M2 = 20;
const int PIN_M3 = 10;
const int PIN_M4 = 5;

const ledc_channel_t CH_M1 = LEDC_CHANNEL_0;
const ledc_channel_t CH_M2 = LEDC_CHANNEL_1;
const ledc_channel_t CH_M3 = LEDC_CHANNEL_2;
const ledc_channel_t CH_M4 = LEDC_CHANNEL_3;

const ledc_timer_t LEDC_TIMER = LEDC_TIMER_0;
const ledc_mode_t  LEDC_MODE  = LEDC_LOW_SPEED_MODE;

// =================== CONTROL ===================
int   MOTOR_IDLE = 150;
const int HEADROOM = 40;

float Kp_pitch = 3.0f, Kd_pitch = 0.5f;
float Kp_roll  = 3.0f, Kd_roll  = 0.5f;
float Kp_yaw   = 0.5f;

// Complementary filter
const float ALPHA = 0.99f;

// If direction reversed, change sign to -1
const int ROLL_SIGN  = +1;
const int PITCH_SIGN = +1;

// =================== I2C (YOUR WIRING) ===================
#define I2C_SDA 7
#define I2C_SCL 6

Adafruit_MPU6050 mpu;

// =================== LiteWing / CRTP over UDP ===================
static const uint16_t UDP_RX_PORT = 2390;
static const char* AP_PASS = "12345678";
IPAddress apIP(192, 168, 43, 42);
IPAddress apGW(192, 168, 43, 42);
IPAddress apSN(255, 255, 255, 0);

WiFiUDP udp;

// Manual commands from app
float manualPitchCmd = 0; // deg
float manualRollCmd  = 0; // deg
float manualYawCmd   = 0; // deg/s

bool ActivarMotor = false;
int  baseCmd = 0; // 0..(PWM_MAX - MOTOR_IDLE - HEADROOM)

// State
float roll_f = 0, pitch_f = 0, yaw_f = 0;
float gyroZ_corr = 0;
unsigned long lastTime = 0;

// =================== PWM UTILS ===================
static inline void setMotor(ledc_channel_t ch, int duty) {
  duty = constrain(duty, 0, PWM_MAX);
  ledc_set_duty(LEDC_MODE, ch, duty);
  ledc_update_duty(LEDC_MODE, ch);
}

static void setupPWM() {
  ledc_timer_config_t timer = {};
  timer.speed_mode      = LEDC_MODE;
  timer.timer_num       = LEDC_TIMER;
  timer.duty_resolution = PWM_RES;
  timer.freq_hz         = PWM_FREQ;
  timer.clk_cfg         = LEDC_AUTO_CLK;
  ledc_timer_config(&timer);

  auto attachChannel = [](int pin, ledc_channel_t ch) {
    ledc_channel_config_t c = {};
    c.gpio_num   = (gpio_num_t)pin;
    c.speed_mode = LEDC_MODE;
    c.channel    = ch;
    c.intr_type  = LEDC_INTR_DISABLE;
    c.timer_sel  = LEDC_TIMER;
    c.duty       = 0;
    c.hpoint     = 0;
    ledc_channel_config(&c);
  };

  attachChannel(PIN_M1, CH_M1);
  attachChannel(PIN_M2, CH_M2);
  attachChannel(PIN_M3, CH_M3);
  attachChannel(PIN_M4, CH_M4);
}

// =================== CRTP HELPERS ===================
static uint8_t crtpChecksum(const uint8_t* buf, int len_without_cksum) {
  uint32_t s = 0;
  for (int i = 0; i < len_without_cksum; i++) s += buf[i];
  return (uint8_t)(s & 0xFF);
}

// Expect Commander setpoint (port=3, chan=0):
// payload: <float roll><float pitch><float yawrate><uint16 thrust>
static void handleCrtpPacket(const uint8_t* pkt, int len) {
  if (len < 3) return;
  uint8_t rx_ck = pkt[len - 1];
  uint8_t calc = crtpChecksum(pkt, len - 1);
  if (rx_ck != calc) return;

  uint8_t header = pkt[0];
  uint8_t port = header >> 4;
  uint8_t chan = header & 0x03;

  const uint8_t* payload = pkt + 1;
  int payloadLen = (len - 2);

  if (port == 3 && chan == 0 && payloadLen >= 14) {
    float rollDeg, pitchDeg, yawRate;
    uint16_t thrust;

    memcpy(&rollDeg,  payload + 0,  4);
    memcpy(&pitchDeg, payload + 4,  4);
    memcpy(&yawRate,  payload + 8,  4);
    memcpy(&thrust,   payload + 12, 2);

    // Simple ARM rule for safe test
    if (thrust < 1000) {
      ActivarMotor = false;
      baseCmd = 0;
      manualPitchCmd = manualRollCmd = manualYawCmd = 0;
      return;
    } else {
      if (!ActivarMotor) {
        roll_f = pitch_f = yaw_f = 0.0f;
        MOTOR_IDLE = 150;
      }
      ActivarMotor = true;
    }

    manualRollCmd  = constrain(rollDeg,  -15.0f, 15.0f);
    manualPitchCmd = constrain(pitchDeg, -15.0f, 15.0f);
    manualYawCmd   = constrain(yawRate,  -60.0f, 60.0f);

    const float THR_MIN = 1000.0f;
    const float THR_MAX = 60000.0f;
    float t = ((float)thrust - THR_MIN) / (THR_MAX - THR_MIN);
    t = constrain(t, 0.0f, 1.0f);

    int maxBase = PWM_MAX - MOTOR_IDLE - HEADROOM;
    baseCmd = (int)lroundf(t * maxBase);
    baseCmd = constrain(baseCmd, 0, maxBase);
  }
}

// =================== SETUP ===================
void setup() {
  Serial.begin(115200);

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  if (!mpu.begin(0x68)) {
    Serial.println("MPU6050 not found!");
    while (1) {}
  }

  setupPWM();

  // WiFi AP
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apGW, apSN);

  uint8_t mac[6];
  WiFi.softAPmacAddress(mac);
  char ssid[32];
  snprintf(ssid, sizeof(ssid), "ESP-DRONE_%02X%02X", mac[4], mac[5]);

  WiFi.softAP(ssid, AP_PASS);
  delay(200);

  udp.begin(UDP_RX_PORT);

  Serial.println("=== START OK (NO VL53L0X) ===");
  Serial.print("AP SSID: "); Serial.println(ssid);
  Serial.print("AP PASS: "); Serial.println(AP_PASS);
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
  Serial.print("UDP RX Port: "); Serial.println(UDP_RX_PORT);

  lastTime = micros();
}

// =================== LOOP ===================
void loop() {
  // ---- UDP receive ----
  int psize = udp.parsePacket();
  if (psize > 0) {
    static uint8_t buf[64];
    int n = udp.read(buf, sizeof(buf));
    if (n > 0) handleCrtpPacket(buf, n);
  }

  // ---- IMU ----
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // YOUR ORIENTATION: Y=Front, X=Right
  float ax = a.acceleration.x; // right
  float ay = a.acceleration.y; // front
  float az = a.acceleration.z; // up

  float accRoll  = atan2(ax, az) * 180.0f / PI;
  float accPitch = atan2(-ay, sqrtf(ax*ax + az*az)) * 180.0f / PI;

  // small offsets (tune later)
  accPitch -= 0.5f;
  accRoll  += 1.9f;

  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0f;
  if (dt <= 0) dt = 0.001f;
  lastTime = now;

  // gyro mapping with your axis
  float gyroRoll  = g.gyro.y * 180.0f / PI; // roll rate
  float gyroPitch = g.gyro.x * 180.0f / PI; // pitch rate
  float gyroZ     = g.gyro.z * 180.0f / PI;

  float a_total = sqrt(ax*ax + ay*ay + az*az);
  bool acc_ok = (a_total > 7.0 && a_total < 12.0) &&
                (fabs(accRoll) < 30.0 && fabs(accPitch) < 30.0);

  if (acc_ok) {
    roll_f  = ALPHA * (roll_f  + (ROLL_SIGN  * gyroRoll)  * dt) + (1 - ALPHA) * (ROLL_SIGN  * accRoll);
    pitch_f = ALPHA * (pitch_f + (PITCH_SIGN * gyroPitch) * dt) + (1 - ALPHA) * (PITCH_SIGN * accPitch);
  } else {
    roll_f  += (ROLL_SIGN  * gyroRoll)  * dt;
    pitch_f += (PITCH_SIGN * gyroPitch) * dt;
  }

  // yaw rate correction (bias tune later)
  float gyro_bias_z = -1.11f;
  gyroZ_corr = gyroZ - gyro_bias_z;
  yaw_f += gyroZ_corr * dt;
  if (yaw_f > 180) yaw_f -= 360;
  if (yaw_f < -180) yaw_f += 360;

  // trims
  float roll_trim = 3.0f;
  float pitch_trim = -3.0f;

  float roll_corr  = (roll_f  - manualRollCmd  - roll_trim)  * Kp_roll  + (ROLL_SIGN  * gyroRoll)  * Kd_roll;
  float pitch_corr = (pitch_f - manualPitchCmd - pitch_trim) * Kp_pitch + (PITCH_SIGN * gyroPitch) * Kd_pitch;
  float yaw_corr   = (manualYawCmd - gyroZ_corr) * Kp_yaw;

  // Base
  int motorBase = 0;
  int minClamp  = 0;
  if (ActivarMotor) {
    minClamp  = MOTOR_IDLE;
    motorBase = MOTOR_IDLE + baseCmd;
    motorBase = constrain(motorBase, MOTOR_IDLE, PWM_MAX - HEADROOM);
  }

  // Mix (X-frame)
  float m1 = motorBase - pitch_corr + roll_corr - yaw_corr; // Front Left
  float m2 = motorBase - pitch_corr - roll_corr + yaw_corr; // Front Right
  float m3 = motorBase + pitch_corr - roll_corr - yaw_corr; // Back Right
  float m4 = motorBase + pitch_corr + roll_corr + yaw_corr; // Back Left

  // Desaturate
  float maxOut = fmaxf(fmaxf(m1, m2), fmaxf(m3, m4));
  if (maxOut > PWM_MAX) {
    float shiftDown = maxOut - PWM_MAX;
    m1 -= shiftDown; m2 -= shiftDown; m3 -= shiftDown; m4 -= shiftDown;
  }
  float minOut = fminf(fminf(m1, m2), fminf(m3, m4));
  if (minOut < minClamp) {
    float shiftUp = minClamp - minOut;
    m1 += shiftUp; m2 += shiftUp; m3 += shiftUp; m4 += shiftUp;
    maxOut = fmaxf(fmaxf(m1, m2), fmaxf(m3, m4));
    if (maxOut > PWM_MAX) {
      float shiftDown = maxOut - PWM_MAX;
      m1 -= shiftDown; m2 -= shiftDown; m3 -= shiftDown; m4 -= shiftDown;
    }
  }

  int out1 = constrain((int)lroundf(m1), 0, PWM_MAX);
  int out2 = constrain((int)lroundf(m2), 0, PWM_MAX);
  int out3 = constrain((int)lroundf(m3), 0, PWM_MAX);
  int out4 = constrain((int)lroundf(m4), 0, PWM_MAX);

  if (ActivarMotor) {
    setMotor(CH_M1, out1);
    setMotor(CH_M2, out2);
    setMotor(CH_M3, out3);
    setMotor(CH_M4, out4);
  } else {
    setMotor(CH_M1, 0);
    setMotor(CH_M2, 0);
    setMotor(CH_M3, 0);
    setMotor(CH_M4, 0);
  }

  Serial.printf("ARM:%d base:%3d | m1:%3d m2:%3d m3:%3d m4:%3d | P:%5.1f R:%5.1f Y:%5.1f | cmdP:%5.1f cmdR:%5.1f cmdY:%5.1f\n",
                (int)ActivarMotor, motorBase, out1, out2, out3, out4,
                pitch_f, roll_f, yaw_f,
                manualPitchCmd, manualRollCmd, manualYawCmd);

  delay(1);
}
