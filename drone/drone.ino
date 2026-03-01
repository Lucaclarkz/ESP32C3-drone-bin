/*
  LiteWing-App Compatible (CRTP over UDP) - ESP32-C3 SuperMini Plus + MPU6050 Brushed Quad
  Pins (as you said):
    M1 GPIO21 Front Left
    M2 GPIO20 Front Right
    M3 GPIO10 Back Right
    M4 GPIO5  Back Left
    MPU6050: SCL GPIO6, SDA GPIO7
    LED: GPIO8  (Power-on blink, WiFi-ready solid)

  Protocol:
    - LiteWing app is based on Crazyflie/CRTP.  (CircuitDigest)
    - UDP packet: CRTP bytes + 1-byte checksum (sum mod 256). (ESP-Drone docs)
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// =================== USER CONFIG ===================
static const int PIN_M1 = 21;   // FL
static const int PIN_M2 = 20;   // FR
static const int PIN_M3 = 10;   // BR
static const int PIN_M4 = 5;    // BL

static const int I2C_SCL = 6;
static const int I2C_SDA = 7;

static const int PIN_LED = 8;

// SSID format LiteWing_xxxxxxxxxxxx, password 12345678
static const char* AP_PASS = "12345678";

// Many LiteWing/ESP-Drone examples use 192.168.43.42
static const IPAddress AP_IP(192,168,43,42);
static const IPAddress AP_GW(192,168,43,42);
static const IPAddress AP_MASK(255,255,255,0);

// UDP ports (common CRTP WiFi control)
static const uint16_t UDP_RX_PORT = 2390; // app -> drone
static const uint16_t UDP_TX_PORT = 2399; // optional

// =================== PWM (Brushed) ===================
static const int PWM_FREQ = 20000;
static const int PWM_RES_BITS = 8;
static const int PWM_MAX = (1 << PWM_RES_BITS) - 1;

static const int CH_M1 = 0;
static const int CH_M2 = 1;
static const int CH_M3 = 2;
static const int CH_M4 = 3;

static const int MOTOR_MIN_START = 35;
static const int MOTOR_MAX_LIMIT = 240;
static const int THR_RAMP_STEP   = 3;

// =================== CONTROL LOOP ===================
static const float LOOP_HZ = 250.0f;
static const uint32_t LOOP_US = (uint32_t)(1000000.0f / LOOP_HZ);

static const float CF_ALPHA = 0.98f;

// failsafe
static const uint32_t FAILSAFE_MS = 350;

// =================== MODES ===================
enum FlightMode { MODE_ANGLE = 0, MODE_RATE = 1 };
static FlightMode mode = MODE_ANGLE;

// =================== PID (START VALUES, tune later) ===================
// Angle outer loop: angle error -> desired rate
static float Kp_angle = 4.5f;

// Rate PID (deg/s)
static float Kp_rate_rp = 0.09f;
static float Ki_rate_rp = 0.18f;
static float Kd_rate_rp = 0.0025f;

static float Kp_rate_y  = 0.12f;
static float Ki_rate_y  = 0.10f;
static float Kd_rate_y  = 0.0f;

static const float MAX_ANGLE_DEG = 30.0f;
static const float MAX_RATE_RP   = 220.0f;
static const float MAX_RATE_Y    = 180.0f;

// =================== GLOBALS ===================
Adafruit_MPU6050 mpu;
WiFiUDP udp;

// attitude (deg)
static float roll_deg = 0, pitch_deg = 0;

// gyro bias (rad/s)
static float gbx=0, gby=0, gbz=0;

// rate PID integrators
static float i_r=0, i_p=0, i_y=0;
static float prev_er=0, prev_ep=0, prev_ey=0;

// throttle smoothing
static int thr_smooth = 0;

// command from app
static volatile float cmd_roll = 0;     // deg (ANGLE) or rate fraction (RATE) depending mode below
static volatile float cmd_pitch = 0;
static volatile float cmd_yawrate = 0;  // deg/s
static volatile float cmd_thrust = 0;   // 0..1

static volatile uint32_t lastPktMs = 0;

// arming logic (simple)
static bool armed = false;
static uint32_t zeroThrStartMs = 0;

// LED
static void ledBlinkBoot() {
  static uint32_t last=0; static bool s=false;
  if (millis() - last >= 350) { last = millis(); s = !s; digitalWrite(PIN_LED, s); }
}

// =================== UTILS ===================
static inline float clampf(float x, float a, float b){ return (x<a)?a:(x>b)?b:x; }

static inline uint8_t cksum_sum_mod256(const uint8_t* data, size_t n){
  uint32_t s=0; for(size_t i=0;i<n;i++) s+=data[i]; return (uint8_t)(s & 0xFF);
}

static inline uint8_t crtp_port(uint8_t h){ return (h >> 4) & 0x0F; }
static inline uint8_t crtp_chan(uint8_t h){ return h & 0x03; }

static void motorWrite(int ch, int duty){
  duty = constrain(duty, 0, PWM_MAX);
  ledcWrite(ch, duty);
}

static void motorsOff(){
  motorWrite(CH_M1,0); motorWrite(CH_M2,0); motorWrite(CH_M3,0); motorWrite(CH_M4,0);
}

// =================== IMU CAL ===================
static void calibrateGyro(uint16_t samples=900){
  float sx=0, sy=0, sz=0;
  sensors_event_t a,g,t;
  for(uint16_t i=0;i<samples;i++){
    mpu.getEvent(&a,&g,&t);
    sx += g.gyro.x; sy += g.gyro.y; sz += g.gyro.z;
    delay(2);
  }
  gbx = sx/samples; gby = sy/samples; gbz = sz/samples;

  roll_deg = 0; pitch_deg = 0;
  i_r=i_p=i_y=0; prev_er=prev_ep=prev_ey=0;
  thr_smooth = 0;
}

// =================== CRTP SETPOINT PARSE ===================
// Expect Commander setpoint: port=3, chan=0
// payload: float roll, float pitch, float yawrate, uint16 thrust
static bool handlePacket(const uint8_t* buf, size_t len){
  if(len < 2) return false;
  uint8_t rx = buf[len-1];
  uint8_t calc = cksum_sum_mod256(buf, len-1);
  if(rx != calc) return false;

  uint8_t h = buf[0];
  uint8_t port = crtp_port(h);
  uint8_t chan = crtp_chan(h);

  if(port==3 && chan==0){
    if(len < (1 + 4 + 4 + 4 + 2 + 1)) return false;
    float r,p,y; uint16_t t16;
    memcpy(&r,   &buf[1], 4);
    memcpy(&p,   &buf[5], 4);
    memcpy(&y,   &buf[9], 4);
    memcpy(&t16, &buf[13],2);

    cmd_roll = r;
    cmd_pitch = p;
    cmd_yawrate = y;
    cmd_thrust = clampf((float)t16 / 65535.0f, 0.0f, 1.0f);

    lastPktMs = millis();
    return true;
  }

  return false;
}

static void udpPoll(){
  int n = udp.parsePacket();
  if(n <= 0) return;
  uint8_t buf[64];
  int r = udp.read(buf, (int)sizeof(buf));
  if(r > 0) handlePacket(buf, (size_t)r);
}

// =================== WIFI/AP ===================
static void makeSSID(char* out, size_t outlen){
  // "LiteWing_" + MAC without ':'
  uint8_t mac[6];
  WiFi.macAddress(mac);
  snprintf(out, outlen, "LiteWing_%02X%02X%02X%02X%02X%02X",
           mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
}

// =================== SETUP ===================
void setup(){
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  Serial.begin(115200);
  delay(120);

  // IMU
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  if(!mpu.begin()){
    while(1){ ledBlinkBoot(); delay(5); }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // PWM
  ledcSetup(CH_M1, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(CH_M2, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(CH_M3, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(CH_M4, PWM_FREQ, PWM_RES_BITS);

  ledcAttachPin(PIN_M1, CH_M1);
  ledcAttachPin(PIN_M2, CH_M2);
  ledcAttachPin(PIN_M3, CH_M3);
  ledcAttachPin(PIN_M4, CH_M4);
  motorsOff();

  // WiFi AP
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);

  char ssid[40];
  makeSSID(ssid, sizeof(ssid));
  WiFi.softAP(ssid, AP_PASS);

  udp.begin(UDP_RX_PORT);

  // Boot blink during calibration
  uint32_t t0 = millis();
  while(millis()-t0 < 500){ ledBlinkBoot(); delay(5); }
  calibrateGyro();

  // WiFi ready => solid ON
  digitalWrite(PIN_LED, HIGH);

  lastPktMs = millis();

  Serial.print("SSID: "); Serial.println(ssid);
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
  Serial.printf("UDP RX: %u\n", UDP_RX_PORT);
}

// =================== MAIN LOOP ===================
void loop(){
  // receive UDP packets as fast as possible
  udpPoll();

  // failsafe
  uint32_t nowMs = millis();
  if(nowMs - lastPktMs > FAILSAFE_MS){
    armed = false;
    motorsOff();
    return;
  }

  // simple arm logic:
  // - arm when we get thrust > 0.08
  // - disarm if thrust stays near 0 for > 1s
  float thr = cmd_thrust;

  if(!armed){
    if(thr > 0.08f){
      armed = true;
      i_r=i_p=i_y=0; prev_er=prev_ep=prev_ey=0;
      thr_smooth = 0;
      zeroThrStartMs = 0;
    }else{
      motorsOff();
      return;
    }
  }else{
    if(thr < 0.02f){
      if(zeroThrStartMs==0) zeroThrStartMs = nowMs;
      if(nowMs - zeroThrStartMs > 1000){
        armed = false;
        motorsOff();
        return;
      }
    }else{
      zeroThrStartMs = 0;
    }
  }

  // fixed-rate control
  static uint32_t lastUs = micros();
  uint32_t nowUs = micros();
  if((uint32_t)(nowUs - lastUs) < LOOP_US) return;
  float dt = (nowUs - lastUs) / 1000000.0f;
  lastUs = nowUs;
  if(dt <= 0) dt = 0.004f;

  sensors_event_t a,g,t;
  mpu.getEvent(&a,&g,&t);

  // gyro deg/s with bias removed
  float gx = (g.gyro.x - gbx) * 180.0f / PI;
  float gy = (g.gyro.y - gby) * 180.0f / PI;
  float gz = (g.gyro.z - gbz) * 180.0f / PI;

  // accel angles deg
  float accRoll  = atan2f(a.acceleration.y, a.acceleration.z) * 180.0f / PI;
  float accPitch = atan2f(-a.acceleration.x,
                          sqrtf(a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z)) * 180.0f / PI;

  // complementary filter
  roll_deg  = CF_ALPHA*(roll_deg  + gx*dt) + (1.0f-CF_ALPHA)*accRoll;
  pitch_deg = CF_ALPHA*(pitch_deg + gy*dt) + (1.0f-CF_ALPHA)*accPitch;

  // desired rates
  float des_r=0, des_p=0, des_y=0;

  if(mode == MODE_ANGLE){
    float desAngR = clampf(cmd_roll,  -MAX_ANGLE_DEG, MAX_ANGLE_DEG);
    float desAngP = clampf(cmd_pitch, -MAX_ANGLE_DEG, MAX_ANGLE_DEG);
    des_r = clampf((desAngR - roll_deg ) * Kp_angle, -MAX_RATE_RP, MAX_RATE_RP);
    des_p = clampf((desAngP - pitch_deg) * Kp_angle, -MAX_RATE_RP, MAX_RATE_RP);
  }else{
    // If you switch to RATE mode later: interpret cmd_roll/cmd_pitch as deg/s directly
    des_r = clampf(cmd_roll,  -MAX_RATE_RP, MAX_RATE_RP);
    des_p = clampf(cmd_pitch, -MAX_RATE_RP, MAX_RATE_RP);
  }
  des_y = clampf(cmd_yawrate, -MAX_RATE_Y, MAX_RATE_Y);

  // Rate PID Roll
  float er = des_r - gx;
  i_r = clampf(i_r + er*dt, -200.0f, 200.0f);
  float dr = (er - prev_er)/dt; prev_er = er;
  float out_r = Kp_rate_rp*er + Ki_rate_rp*i_r + Kd_rate_rp*dr;

  // Rate PID Pitch
  float ep = des_p - gy;
  i_p = clampf(i_p + ep*dt, -200.0f, 200.0f);
  float dp = (ep - prev_ep)/dt; prev_ep = ep;
  float out_p = Kp_rate_rp*ep + Ki_rate_rp*i_p + Kd_rate_rp*dp;

  // Rate PID Yaw
  float ey = des_y - gz;
  i_y = clampf(i_y + ey*dt, -200.0f, 200.0f);
  float dy = (ey - prev_ey)/dt; prev_ey = ey;
  float out_y = Kp_rate_y*ey + Ki_rate_y*i_y + Kd_rate_y*dy;

  // Thrust -> base PWM duty
  int target = 0;
  if(thr > 0.01f){
    int usable = MOTOR_MAX_LIMIT - MOTOR_MIN_START;
    target = MOTOR_MIN_START + (int)(thr * usable);
  }else{
    target = 0;
  }

  // throttle ramp
  if(target > thr_smooth) thr_smooth = min(target, thr_smooth + THR_RAMP_STEP);
  else                   thr_smooth = max(target, thr_smooth - THR_RAMP_STEP);

  float base = (float)thr_smooth;

  // If yaw feels reversed, set yawSign = -1
  const float yawSign = 1.0f;
  float yawTerm = yawSign * out_y;

  // X mix: M1 FL, M2 FR, M3 BR, M4 BL
  float m1 = base - out_p + out_r - yawTerm;
  float m2 = base - out_p - out_r + yawTerm;
  float m3 = base + out_p - out_r - yawTerm;
  float m4 = base + out_p + out_r + yawTerm;

  // saturation protection (shift down)
  float maxOut = fmaxf(fmaxf(m1,m2), fmaxf(m3,m4));
  if(maxOut > MOTOR_MAX_LIMIT){
    float s = maxOut - MOTOR_MAX_LIMIT;
    m1-=s; m2-=s; m3-=s; m4-=s;
  }

  // clamp
  m1 = clampf(m1, 0, PWM_MAX);
  m2 = clampf(m2, 0, PWM_MAX);
  m3 = clampf(m3, 0, PWM_MAX);
  m4 = clampf(m4, 0, PWM_MAX);

  if(!armed){
    motorsOff();
  }else{
    motorWrite(CH_M1, (int)m1);
    motorWrite(CH_M2, (int)m2);
    motorWrite(CH_M3, (int)m3);
    motorWrite(CH_M4, (int)m4);
  }
}
