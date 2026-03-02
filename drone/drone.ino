#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// ================= PINS =================
#define M1_PIN 21   // Front Left
#define M2_PIN 20   // Front Right
#define M3_PIN 10   // Back Right
#define M4_PIN 5    // Back Left
#define LED_PIN 8
#define SDA_PIN 7
#define SCL_PIN 6

// ================= PWM =================
#define PWM_FREQ 20000
#define PWM_RES 8
#define PWM_MAX 255

// Motor tuning (RIGHT side too strong -> reduce gain)
float motorGain[4] = {1.05, 0.95, 0.95, 1.00};
int motorMin[4]    = {55, 45, 45, 45};   // M1 boosted start

// ================= WIFI =================
WiFiUDP udp;
#define UDP_PORT 2390

// ================= IMU =================
Adafruit_MPU6050 mpu;
float gbx=0,gby=0,gbz=0;
float roll=0,pitch=0;
#define CF_ALPHA 0.98

// ================= CONTROL =================
float cmdRoll=0, cmdPitch=0, cmdYaw=0, cmdThr=0;
unsigned long lastPacket=0;
#define FAILSAFE 250
#define LOOP_US 4000

// ================= UTIL =================
float clampf(float x,float a,float b){return x<a?a:(x>b?b:x);}

void motorWrite(int ch,int duty){
  duty=constrain(duty,0,PWM_MAX);
  ledcWrite(ch,duty);
}
void motorsOff(){
  motorWrite(0,0); motorWrite(1,0);
  motorWrite(2,0); motorWrite(3,0);
}

// ================= IMU CAL =================
void calibrateGyro(){
  float sx=0,sy=0,sz=0;
  sensors_event_t a,g,t;
  for(int i=0;i<800;i++){
    mpu.getEvent(&a,&g,&t);
    sx+=g.gyro.x; sy+=g.gyro.y; sz+=g.gyro.z;
    delay(2);
  }
  gbx=sx/800; gby=sy/800; gbz=sz/800;
}

// ================= SETUP =================
void setup(){
  pinMode(LED_PIN,OUTPUT);
  digitalWrite(LED_PIN,LOW);

  // Prevent boot spin
  pinMode(M1_PIN,OUTPUT); digitalWrite(M1_PIN,LOW);
  pinMode(M2_PIN,OUTPUT); digitalWrite(M2_PIN,LOW);
  pinMode(M3_PIN,OUTPUT); digitalWrite(M3_PIN,LOW);
  pinMode(M4_PIN,OUTPUT); digitalWrite(M4_PIN,LOW);

  Serial.begin(115200);

  Wire.begin(SDA_PIN,SCL_PIN);
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  calibrateGyro();

  ledcSetup(0,PWM_FREQ,PWM_RES);
  ledcSetup(1,PWM_FREQ,PWM_RES);
  ledcSetup(2,PWM_FREQ,PWM_RES);
  ledcSetup(3,PWM_FREQ,PWM_RES);

  ledcAttachPin(M1_PIN,0);
  ledcAttachPin(M2_PIN,1);
  ledcAttachPin(M3_PIN,2);
  ledcAttachPin(M4_PIN,3);

  motorsOff();

  WiFi.mode(WIFI_AP);
  WiFi.softAP("LiteWing_Drone","12345678");
  udp.begin(UDP_PORT);

  digitalWrite(LED_PIN,HIGH);
}

// ================= LOOP =================
void loop(){

  // UDP read (LiteWing setpoint)
  int size=udp.parsePacket();
  if(size>0){
    uint8_t buf[32];
    udp.read(buf,32);
    memcpy(&cmdRoll,&buf[1],4);
    memcpy(&cmdPitch,&buf[5],4);
    memcpy(&cmdYaw,&buf[9],4);
    uint16_t thr16;
    memcpy(&thr16,&buf[13],2);
    cmdThr=thr16/65535.0f;
    lastPacket=millis();
  }

  // FAILSAFE
  if(millis()-lastPacket>FAILSAFE){
    motorsOff();
    return;
  }

  if(cmdThr<0.03){ motorsOff(); return; }

  static unsigned long last=micros();
  if(micros()-last<LOOP_US) return;
  float dt=(micros()-last)/1000000.0f;
  last=micros();

  sensors_event_t a,g,t;
  mpu.getEvent(&a,&g,&t);

  // === REMAP AXIS (Y front, X right) ===
  float ax = a.acceleration.y;
  float ay = -a.acceleration.x;
  float az = a.acceleration.z;

  float gx = (g.gyro.y - gbx)*180/PI;
  float gy = -(g.gyro.x - gby)*180/PI;
  float gz = (g.gyro.z - gbz)*180/PI;

  float accRoll = atan2(ay,az)*180/PI;
  float accPitch = atan2(-ax,sqrt(ay*ay+az*az))*180/PI;

  roll = CF_ALPHA*(roll+gx*dt)+(1-CF_ALPHA)*accRoll;
  pitch = CF_ALPHA*(pitch+gy*dt)+(1-CF_ALPHA)*accPitch;

  // PID simple proportional (stable base)
  float rCorr = (cmdRoll-roll)*4.0;
  float pCorr = (cmdPitch-pitch)*4.0;
  float yCorr = cmdYaw*2.0;

  int base = motorMin[0] + cmdThr*(200);

  float m[4];
  m[0]=base - pCorr + rCorr - yCorr; // M1
  m[1]=base - pCorr - rCorr + yCorr; // M2
  m[2]=base + pCorr - rCorr - yCorr; // M3
  m[3]=base + pCorr + rCorr + yCorr; // M4

  for(int i=0;i<4;i++){
    if(m[i]<motorMin[i]) m[i]=motorMin[i];
    m[i]*=motorGain[i];
    m[i]=clampf(m[i],0,255);
  }

  motorWrite(0,m[0]);
  motorWrite(1,m[1]);
  motorWrite(2,m[2]);
  motorWrite(3,m[3]);
}
