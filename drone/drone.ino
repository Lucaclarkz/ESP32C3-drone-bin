/*
  LiteWing App - Ready To Fly (Brushed) | ESP32-C3 SuperMini Plus + MPU6050
  FIXED for:
   - Right side weak (M2 + M3) => boost those motors
   - Instant stop on throttle release (NO 2s delay)
   - No boot motor blip (force pins LOW before PWM attach)
   - LED status on GPIO8 (boot blink, wifi ready solid)

  PINS:
    M1 GPIO21 Front Left
    M2 GPIO20 Front Right  (RIGHT)
    M3 GPIO10 Back  Right  (RIGHT)
    M4 GPIO5  Back  Left
    MPU6050: SCL GPIO6, SDA GPIO7
    LED: GPIO8
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// =================== PINS ===================
static const int PIN_M1 = 21;   // FL
static const int PIN_M2 = 20;   // FR (RIGHT)
static const int PIN_M3 = 10;   // BR (RIGHT)
static const int PIN_M4 = 5;    // BL
static const int I2C_SCL = 6;
static const int I2C_SDA = 7;
static const int PIN_LED = 8;

// =================== WIFI/AP ===================
static const char* AP_PASS = "12345678";
static const IPAddress AP_IP(192,168,43,42);
static const IPAddress AP_GW(192,168,43,42);
static const IPAddress AP_MASK(255,255,255,0);

// UDP port (LiteWing / CRTP WiFi commonly uses this)
static const uint16_t UDP_RX_PORT = 2390;

// =================== PWM ===================
static const int PWM_FREQ = 20000;
static const int PWM_RES_BITS = 8;
static const int PWM_MAX = (1 << PWM_RES_BITS) - 1;

static const int CH_M1 = 0;
static const int CH_M2 = 1;
static const int CH_M3 = 2;
static const int CH_M4 = 3;

// =================== MOTOR FIX (RIGHT SIDE WEAK M2/M3) ===================
// Per-motor start threshold + gain (RIGHT side boosted)
static int motorMin[4]      = {35, 48, 48, 35};             // M2/M3 start earlier
static float motorGain[4]   = {1.00f, 1.20f, 1.20f, 1.00f}; // M2/M3 stronger
static const int MOTOR_MAX_LIMIT = 240;                     // headroom

// =================== CONTROL LOOP ===================
static const uint32_t LOOP_US = 4000;     // 250Hz
static const float CF_ALPHA = 0.98f;

// SAFETY / INSTANT STOP
static const float THR_ON_ARM   = 0.08f;  // need >8% thrust to arm
static const float THR_OFF_STOP = 0.02f;  // <2% => instant stop
static const uint32_t FAILSAFE_MS = 220;  // faster failsafe (packet lost)

// =================== FLIGHT MODE ===================
enum FlightMode { MODE_ANGLE = 0, MODE_RATE = 1 };
static FlightMode mode = MODE_ANGLE;

// PID start values (tune later)
static float Kp_angle = 4.5f;
static float Kp_rate_rp = 0.09f, Ki_rate_rp = 0.18f, Kd_rate_rp = 0.0025f;
static float Kp_rate_y  = 0.12f, Ki_rate_y  = 0.10f, Kd_rate_y  = 0.0f;

static const float MAX_ANGLE_DEG = 30.0f;
static const float MAX_RATE_RP   = 220.0f;
static const float MAX_RATE_Y    = 180.0f;

// =================== GLOBALS ===================
Adafruit_MPU6050 mpu;
WiFiUDP udp;

static float roll_deg = 0, pitch_deg = 0;
static float rollTrim = 0.0f, pitchTrim = 0.0f;

static float gbx=0, gby=0, gbz=0;   // gyro bias rad/s

static float i_r=0, i_p=0, i_y=0;
static float prev_er=0, prev_ep=0, prev_ey=0;

static volatile float cmd_roll = 0;
static volatile float cmd_pitch = 0;
static volatile float cmd_yawrate = 0;
static volatile float cmd_thrust = 0;     // 0..1
static volatile uint32_t lastPktMs = 0;

static bool armed = false;

// =================== UTILS ===================
static inline float clampf(float x, float a, float b){ return (x<a)?a:(x>b)?b:x; }
static inline uint8_t sum_mod256(const uint8_t* data, size_t n){
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

static void ledBootBlink(){
  static uint32_t last=0; static bool s=false;
  if(millis()-last >= 250){ last=millis(); s=!s; digitalWrite(PIN_LED, s); }
}

static void resetPID(){
  i_r=i_p=i_y=0;
  prev_er=prev_ep=prev_ey=0;
}

// =================== SAFE BOOT (NO MOTOR BLIP) ===================
static void forcePinsLowBeforePWM(){
  pinMode(PIN_M1, OUTPUT); digitalWrite(PIN_M1, LOW);
  pinMode(PIN_M2, OUTPUT); digitalWrite(PIN_M2, LOW);
  pinMode(PIN_M3, OUTPUT); digitalWrite(PIN_M3, LOW);
  pinMode(PIN_M4, OUTPUT); digitalWrite(PIN_M4, LOW);
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
  gbx=sx/samples; gby=sy/samples; gbz=sz/samples;
  roll_deg=0; pitch_deg=0;
  resetPID();
}

static void calibrateLevelTrim(){
  // estimate level roll/pitch average while flat, disarmed
  float sr=0, sp=0;
  const int N=160;
  for(int i=0;i<N;i++){
    sensors_event_t a,g,t;
    mpu.getEvent(&a,&g,&t);

    float gx = (g.gyro.x - gbx) * 180.0f / PI;
    float gy = (g.gyro.y - gby) * 180.0f / PI;

    float accRoll  = atan2f(a.acceleration.y, a.acceleration.z) * 180.0f / PI;
    float accPitch = atan2f(-a.acceleration.x,
                            sqrtf(a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z)) * 180.0f / PI;

    // update filter quickly
    roll_deg  = CF_ALPHA*(roll_deg  + gx*0.004f) + (1.0f-CF_ALPHA)*accRoll;
    pitch_deg = CF_ALPHA*(pitch_deg + gy*0.004f) + (1.0f-CF_ALPHA)*accPitch;

    sr += roll_deg;
    sp += pitch_deg;
    delay(5);
  }
  rollTrim  = sr / N;
  pitchTrim = sp / N;
}

// =================== CRTP SETPOINT ===================
// Commander port=3, chan=0: float roll, float pitch, float yawrate, uint16 thrust
static bool handlePacket(const uint8_t* buf, size_t len){
  if(len < 2) return false;
  uint8_t rx = buf[len-1];
  uint8_t calc = sum_mod256(buf, len-1);
  if(rx != calc) return false;

  uint8_t h = buf[0];
  if(crtp_port(h)==3 && crtp_chan(h)==0){
    if(len < (1+4+4+4+2+1)) return false;

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

// =================== WIFI SSID ===================
static void makeSSID(char* out, size_t outlen){
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

  forcePinsLowBeforePWM();   // <<< boot blip fix

  // IMU
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  if(!mpu.begin()){
    while(1){ ledBootBlink(); delay(5); }
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

  // blink during calibration
  uint32_t t0=millis();
  while(millis()-t0 < 650){ ledBootBlink(); delay(5); }

  calibrateGyro();
  calibrateLevelTrim();

  // ready
  digitalWrite(PIN_LED, HIGH);

  lastPktMs = millis();

  Serial.print("SSID: "); Serial.println(ssid);
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
  Serial.printf("UDP RX: %u\n", UDP_RX_PORT);
}

// =================== LOOP ===================
void loop(){
  udpPoll();

  uint32_t nowMs = millis();

  // failsafe => instant stop
  if(nowMs - lastPktMs > FAILSAFE_MS){
    armed = false;
    motorsOff();
    resetPID();
    return;
  }

  float thr = cmd_thrust;

  // INSTANT STOP on throttle release (no delay)
  if(thr < THR_OFF_STOP){
    armed = false;
    motorsOff();
    resetPID();
    return;
  }

  // arm condition
  if(!armed){
    if(thr > THR_ON_ARM){
      armed = true;
      resetPID();
    }else{
      motorsOff();
      return;
    }
  }

  // fixed loop rate
  static uint32_t lastUs = micros();
  uint32_t nowUs = micros();
  if((uint32_t)(nowUs - lastUs) < LOOP_US) return;
  float dt = (nowUs - lastUs) / 1000000.0f;
  lastUs = nowUs;
  if(dt <= 0) dt = 0.004f;

  // IMU
  sensors_event_t a,g,t;
  mpu.getEvent(&a,&g,&t);

  float gx = (g.gyro.x - gbx) * 180.0f / PI;
  float gy = (g.gyro.y - gby) * 180.0f / PI;
  float gz = (g.gyro.z - gbz) * 180.0f / PI;

  float accRoll  = atan2f(a.acceleration.y, a.acceleration.z) * 180.0f / PI;
  float accPitch = atan2f(-a.acceleration.x,
                          sqrtf(a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z)) * 180.0f / PI;

  roll_deg  = CF_ALPHA*(roll_deg  + gx*dt) + (1.0f-CF_ALPHA)*accRoll;
  pitch_deg = CF_ALPHA*(pitch_deg + gy*dt) + (1.0f-CF_ALPHA)*accPitch;

  float rollLevel  = roll_deg  - rollTrim;
  float pitchLevel = pitch_deg - pitchTrim;

  // desired rates
  float des_r=0, des_p=0, des_y=0;

  if(mode == MODE_ANGLE){
    float desAngR = clampf(cmd_roll,  -MAX_ANGLE_DEG, MAX_ANGLE_DEG);
    float desAngP = clampf(cmd_pitch, -MAX_ANGLE_DEG, MAX_ANGLE_DEG);
    des_r = clampf((desAngR - rollLevel ) * Kp_angle, -MAX_RATE_RP, MAX_RATE_RP);
    des_p = clampf((desAngP - pitchLevel) * Kp_angle, -MAX_RATE_RP, MAX_RATE_RP);
  }else{
    des_r = clampf(cmd_roll,  -MAX_RATE_RP, MAX_RATE_RP);
    des_p = clampf(cmd_pitch, -MAX_RATE_RP, MAX_RATE_RP);
  }
  des_y = clampf(cmd_yawrate, -MAX_RATE_Y, MAX_RATE_Y);

  // PID roll
  float er = des_r - gx;
  i_r = clampf(i_r + er*dt, -200.0f, 200.0f);
  float dr = (er - prev_er)/dt; prev_er = er;
  float out_r = Kp_rate_rp*er + Ki_rate_rp*i_r + Kd_rate_rp*dr;

  // PID pitch
  float ep = des_p - gy;
  i_p = clampf(i_p + ep*dt, -200.0f, 200.0f);
  float dp = (ep - prev_ep)/dt; prev_ep = ep;
  float out_p = Kp_rate_rp*ep + Ki_rate_rp*i_p + Kd_rate_rp*dp;

  // PID yaw
  float ey = des_y - gz;
  i_y = clampf(i_y + ey*dt, -200.0f, 200.0f);
  float dy = (ey - prev_ey)/dt; prev_ey = ey;
  float out_y = Kp_rate_y*ey + Ki_rate_y*i_y + Kd_rate_y*dy;

  // Thrust to base PWM
  int minAvg = (motorMin[0]+motorMin[1]+motorMin[2]+motorMin[3]) / 4;
  int usable = MOTOR_MAX_LIMIT - minAvg;
  int base = minAvg + (int)(thr * usable);
  base = constrain(base, 0, MOTOR_MAX_LIMIT);

  // yaw reverse if needed
  const float yawSign = 1.0f;
  float yawTerm = yawSign * out_y;

  // Mix (X): M1 FL, M2 FR, M3 BR, M4 BL
  float m[4];
  m[0] = base - out_p + out_r - yawTerm; // M1
  m[1] = base - out_p - out_r + yawTerm; // M2
  m[2] = base + out_p - out_r - yawTerm; // M3
  m[3] = base + out_p + out_r + yawTerm; // M4

  // Apply per-motor min + gain
  for(int i=0;i<4;i++){
    if(m[i] > 0){
      if(m[i] < motorMin[i]) m[i] = motorMin[i];
      m[i] = m[i] * motorGain[i];
    }
  }

  // shift down if any exceeds limit
  float maxOut = fmaxf(fmaxf(m[0],m[1]), fmaxf(m[2],m[3]));
  if(maxOut > MOTOR_MAX_LIMIT){
    float s = maxOut - MOTOR_MAX_LIMIT;
    for(int i=0;i<4;i++) m[i] -= s;
  }

  for(int i=0;i<4;i++) m[i] = clampf(m[i], 0, PWM_MAX);

  motorWrite(CH_M1, (int)m[0]);
  motorWrite(CH_M2, (int)m[1]);
  motorWrite(CH_M3, (int)m[2]);
  motorWrite(CH_M4, (int)m[3]);
}
