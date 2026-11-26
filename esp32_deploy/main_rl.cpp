


#include <Arduino.h>
#include <Wire.h>
#include <ctype.h>
#include <math.h>
#include "rl_policy.h"
#include <Preferences.h>
#include "web_telemetry.h"
// ============================================================================
// 1. CAU HINH CHAN ESP32 (tham khao dung config chan trong main.cpp cu)
// ============================================================================
// Dong co A (Trai) - L298N + GA25-370 Encoder pha A/B
#define ENA     14
#define IN1     27
#define IN2     26
#define ENCA_A  16  // RX2 - Day Vang  (Phase A)
#define ENCA_B  17  // TX2 - Day Xanh la (Phase B)

// Dong co B (Phai) - L298N + GA25-370 Encoder pha A/B
#define ENB     25
#define IN3     32
#define IN4     33
#define ENCB_A  18  // Day Vang  (Phase A)
#define ENCB_B  19  // Day Xanh la (Phase B)
// Luu y dau day GA25-370 (6 day):
//   Do/Trang: Nguon motor noi ra OUT1/OUT2 (hoac OUT3/OUT4) cua L298N
//   Xanh duong (VCC encoder): Noi vao 3.3V cua ESP32 (KHONG noi 5V de tranh hong GPIO)
//   Den (GND encoder): Noi GND chung voi ESP32 & L298N

// I2C cho MPU6050 (chan mac dinh ESP32: SDA=21, SCL=22)
#define IMU_SDA       21
#define IMU_SCL       22
#define MPU6050_ADDR  0x68

// ============================================================================
// 2. THONG SO DONG CO GA25-370 & HIEU CHINH CHIEU PHAN CUNG
// ============================================================================
// Encoder tu tinh tren truc dong co GA25-370 co 11 xung/vong (11 PPR).
// Khi doc 4x Quadrature (ca canh len/xuong tren ca 2 pha A & B):
//   TICKS_PER_REV = 11 * 4 * MOTOR_GEAR_RATIO
// Bang tham khao MOTOR_GEAR_RATIO cho cac ban GA25-370 pho bien:
//   - Ban ~1360 RPM :  4.4f  ->  193.6 ticks/vong (4x)
//   - Ban ~620  RPM :  9.8f  ->  431.2 ticks/vong (4x)
//   - Ban ~280-300 RPM: 21.3f -> 937.2 ticks/vong (4x)
//   - Ban ~170-210 RPM: 34.0f -> 1496.0 ticks/vong (4x)
//   - Ban ~130  RPM : 46.8f  -> 2059.2 ticks/vong (4x)  [Mac dinh]
//   - Ban ~80-100 RPM: 75.0f -> 3300.0 ticks/vong (4x)
// Co the kiem tra thuc te bang cach mo Serial Monitor (115200), bam 'T' roi
// quay tay banh xe dung 1 vong xem gia tri encA / encB thay doi bao nhieu xung.
static const float ENCODER_BASE_PPR  = 11.0f;
static const float ENCODER_QUAD_MULT = 4.0f;
static const float MOTOR_GEAR_RATIO  = 46.8f;
static const float TICKS_PER_REV     = ENCODER_BASE_PPR * ENCODER_QUAD_MULT * MOTOR_GEAR_RATIO;

// Chinh truc & dau (+/-1.0f) sao cho khop chieu gan MPU6050 & L298N tren xe:
//   - cfgPitchIsX = false: truc quay cua xe trung truc Y tren board MPU6050 (gy, ax)
//   - cfgPitchIsX = true : neu ban gan board MPU6050 xoay 90 do (gx, ay)
//   - +pitch    = xe nghieng ve phia truoc
//   - +motor    = banh quay theo chieu tien
//   - +encoder  = enc tang khi banh lan tien
// Cac gia tri duoi day la MAC DINH; co the doi truc tiep tren web (muc "Cau hinh chieu")
// va duoc luu vao NVS (giu sau khi khoi dong lai).
//   cfgPitchIsX = true : pitch quay quanh truc X cua MPU6050 (dung gx, ay)  <- mac dinh moi
//   cfgPitchIsX = false: pitch quay quanh truc Y (dung gy, ax)
static bool  cfgPitchIsX = true;
static float cfgPitchSign  = 1.0f;
static float cfgYawSign    = 1.0f;
static float cfgEncLSign   = 1.0f;
static float cfgEncRSign   = 1.0f;
static float cfgMotorLSign = 1.0f;
static float cfgMotorRSign = 1.0f;
// Che do tam khi encoder PHAI hong: dung wR = wL (mat thong tin quay/yaw -> can bang kem hon)
static bool  cfgMirrorEnc  = false;
#define PITCH_OFFSET_RAD    0.0f

// Hieu chuan cam bien trong 5 giay dau tien khi bat nguon:
//   - Trong 5s nay motor TAT hoan toan, LED tren ESP32 (GPIO 2) nhay nhanh bao hieu.
//   - 1.5s dau cho ban dat xe dung yen tai diem can bang; 3.5s tiep theo lay trung binh.
//   - AUTO_CALIB_PITCH_ZERO = 1: tu dong lay diem 0 can bang (neu giu xe trong +/-25 deg)
//     giup bu sai lech trong tam / goc gan MPU6050 ma khong can cam USB go 'Z'.
#define CALIB_DURATION_MS     5000UL
#define CALIB_SETTLE_MS       1500UL
#define AUTO_CALIB_PITCH_ZERO 1
#define CALIB_LED_PIN         2

// Che do hoat dong:
//   0 = Chi tu can bang tai cho (tat W/S/A/D, giu cmdV=0 & cmdYaw=0)
//   1 = Bat nhan lenh di chuyen qua Serial (W/S/A/D/X/+/-)  [Mac dinh]
#define ENABLE_MOVE_COMMANDS 1

// Bu vung chet PWM cua L298N (~1.8-2V sut ap BJT) + hop so GA25-370 (0..35).
// Tang len ~18..28 neu thay banh bi re ma khong nhich khi xe nghieng nhe.
#define MOTOR_DEADZONE_PWM  0

// Tu kiem tra dong co khi bat nguon (quay NGUOC -> dung -> quay XUOI -> tat roi moi calib 5s):
//   - MOTOR_SELFTEST_ENABLE = 1: bat quay test 2 chieu luc khoi dong (dat 0 de bo qua)
//   - MOTOR_SELFTEST_CMD    = 0.30f: muc PWM nhe (0.20..0.45) de banh quay test khong giat manh
//   - MOTOR_SELFTEST_MS     = 350: thoi gian chay moi chieu (ms)
#define MOTOR_SELFTEST_ENABLE 1
#define MOTOR_SELFTEST_CMD    0.30f
#define MOTOR_SELFTEST_MS     350

// Thong so vat ly & dieu khien (khop 100% voi MuJoCo env 130 RPM)
static const float WHEEL_RADIUS  = 0.0325f; // Banh 65mm -> R = 0.0325 m
static const float CONTROL_DT    = 0.02f;   // 50 Hz (20 ms)
#if ENABLE_MOVE_COMMANDS
static const float MAX_CMD_V     = 0.28f;     // m/s
static const float MAX_CMD_YAW   = 1.40f;   // rad/s
static const float MAX_ACCEL_V   = 0.45f;   // m/s^2
static const float MAX_ACCEL_YAW = 3.00f; // rad/s^2
#endif

// ============================================================================
// 3. DOC ENCODER PHA A/B 4X QUADRATURE (AN TOAN DA NHAN ESP32)
// ============================================================================
static portMUX_TYPE encAMux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE encBMux = portMUX_INITIALIZER_UNLOCKED;
static volatile long encoderA_count = 0;
static volatile long encoderB_count = 0;
static volatile uint8_t prevStateA = 0;
static volatile uint8_t prevStateB = 0;

static const int8_t QEM_LUT[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

// Khai bao prototype 1 lan de Arduino IDE / PlatformIO .ino khong nhan doi IRAM_ATTR (__COUNTER__)
void IRAM_ATTR isrEncA();
void IRAM_ATTR isrEncB();

void isrEncA() {
  portENTER_CRITICAL_ISR(&encAMux);
  uint8_t cur = ((uint8_t)digitalRead(ENCA_A) << 1) | (uint8_t)digitalRead(ENCA_B);
  encoderA_count += QEM_LUT[(prevStateA << 2) | cur];
  prevStateA = cur;
  portEXIT_CRITICAL_ISR(&encAMux);
}

void isrEncB() {
  portENTER_CRITICAL_ISR(&encBMux);
  uint8_t cur = ((uint8_t)digitalRead(ENCB_A) << 1) | (uint8_t)digitalRead(ENCB_B);
  encoderB_count += QEM_LUT[(prevStateB << 2) | cur];
  prevStateB = cur;
  portEXIT_CRITICAL_ISR(&encBMux);
}

static inline void readEncoders(long &encA, long &encB) {
  portENTER_CRITICAL(&encAMux);
  encA = encoderA_count;
  portEXIT_CRITICAL(&encAMux);
  portENTER_CRITICAL(&encBMux);
  encB = encoderB_count;
  portEXIT_CRITICAL(&encBMux);
}

static inline void resetEncoders() {
  portENTER_CRITICAL(&encAMux);
  encoderA_count = 0;
  portEXIT_CRITICAL(&encAMux);
  portENTER_CRITICAL(&encBMux);
  encoderB_count = 0;
  portEXIT_CRITICAL(&encBMux);
}

// ============================================================================
// 4. DIEU KHIEN DONG CO L298N & TU KIEM TRA LUC KHOI DONG
// ============================================================================
static void driveMotor(int pwmPin, int in1, int in2, float normCmd) {
  float mag = fabsf(normCmd);
  if (mag < 0.005f) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, 0);
    return;
  }
  if (mag > 1.0f) mag = 1.0f;

  int pwm = MOTOR_DEADZONE_PWM + (int)(mag * (float)(255 - MOTOR_DEADZONE_PWM));
  if (pwm > 255) pwm = 255;

  if (normCmd >= 0.0f) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  analogWrite(pwmPin, pwm);
}

static inline void runStartupMotorSelfTest() {
#if MOTOR_SELFTEST_ENABLE
  Serial.println("Dang kiem tra dong co: quay NGUOC -> quay XUOI...");
  long a0 = 0, b0 = 0, a1 = 0, b1 = 0;

  // 1. Quay NGUOC
  resetEncoders();
  driveMotor(ENA, IN1, IN2, -cfgMotorLSign * MOTOR_SELFTEST_CMD);
  driveMotor(ENB, IN3, IN4, -cfgMotorRSign * MOTOR_SELFTEST_CMD);
  delay(MOTOR_SELFTEST_MS);
  driveMotor(ENA, IN1, IN2, 0.0f);
  driveMotor(ENB, IN3, IN4, 0.0f);
  delay(150);
  readEncoders(a0, b0);
  long revL = (long)(cfgEncLSign * (float)a0);
  long revR = (long)(cfgEncRSign * (float)b0);
  Serial.print("  [TEST NGUOC] dEncL="); Serial.print(revL);
  Serial.print(" | dEncR="); Serial.println(revR);

  // Nghi ngan bao ve cau H L298N truoc khi dao chieu
  delay(150);

  // 2. Quay XUOI
  resetEncoders();
  driveMotor(ENA, IN1, IN2, cfgMotorLSign * MOTOR_SELFTEST_CMD);
  driveMotor(ENB, IN3, IN4, cfgMotorRSign * MOTOR_SELFTEST_CMD);
  delay(MOTOR_SELFTEST_MS);
  driveMotor(ENA, IN1, IN2, 0.0f);
  driveMotor(ENB, IN3, IN4, 0.0f);
  delay(250); // Cho xe & banh dung han truoc khi calib MPU6050 5s
  readEncoders(a1, b1);
  long fwdL = (long)(cfgEncLSign * (float)a1);
  long fwdR = (long)(cfgEncRSign * (float)b1);
  Serial.print("  [TEST XUOI]  dEncL="); Serial.print(fwdL);
  Serial.print(" | dEncR="); Serial.println(fwdR);

  if (labs(revL) <= 2 && labs(fwdL) <= 2) {
    Serial.println("  [CANH BAO] Khong thay xung encoder Trai (Kiem tra nguon 3.3V/chan 16-17 hoac day motor L)!");
  } else if (fwdL < 0 && revL > 0) {
    Serial.println("  [GOI Y] Encoder Trai nguoc chieu quay -> bam 'Encoder trai' tren web de dao dau (hoac dao day motor A).");
  }
  if (labs(revR) <= 2 && labs(fwdR) <= 2) {
    Serial.println("  [CANH BAO] Khong thay xung encoder Phai (Kiem tra nguon 3.3V/chan 18-19 hoac day motor R)! Tam thoi bat 'Mirror wR=wL' tren web.");
  } else if (fwdR < 0 && revR > 0) {
    Serial.println("  [GOI Y] Encoder Phai nguoc chieu quay -> bam 'Encoder phai' tren web de dao dau (hoac dao day motor B).");
  }
  resetEncoders();
#endif
}

// ============================================================================
// 5. MPU6050 I2C + COMPLEMENTARY FILTER (DONG BO DAU GYRO & ACCEL)
// ============================================================================
static uint8_t mpuAddr = MPU6050_ADDR;
static float gyroPitchOffset = 0.0f;
static float gyroZOffset = 0.0f;
static float pitchZeroOffset = PITCH_OFFSET_RAD;
static float pitchTrim = 0.0f;   // rad, chinh tay tren web, luu NVS, cong them vao zero
static inline float zeroTotal() { return pitchZeroOffset + pitchTrim; }
static void calibrateMpu();
static float pitchEst = 0.0f;
static float lastPitchAccel = 0.0f;
static float lastPitchRate = 0.0f;
static float lastYawRate = 0.0f;
static int i2cFailCount = 0;

static void writeMpuReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static bool readMpuReg(uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(mpuAddr, (uint8_t)1, (uint8_t)true) != 1) return false;
  val = Wire.read();
  return true;
}

static bool readMpuRaw(int16_t &ax, int16_t &ay, int16_t &az,
                       int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(mpuAddr, (uint8_t)14, (uint8_t)true) != 14) {
    return false;
  }
  ax = (int16_t)((Wire.read() << 8) | Wire.read());
  ay = (int16_t)((Wire.read() << 8) | Wire.read());
  az = (int16_t)((Wire.read() << 8) | Wire.read());
  Wire.read(); Wire.read(); // bo qua nhiet do
  gx = (int16_t)((Wire.read() << 8) | Wire.read());
  gy = (int16_t)((Wire.read() << 8) | Wire.read());
  gz = (int16_t)((Wire.read() << 8) | Wire.read());
  return true;
}

static float computeRawAccelPitch(int16_t ax, int16_t ay, int16_t az) {
  // Quy tac ban tay phai MPU6050:
  //   Quanh +Y: +gy khop voi atan2(-ax, az)
  //   Quanh +X: +gx khop voi atan2(+ay, az)
  if (cfgPitchIsX) return cfgPitchSign * atan2f((float)ay, (float)az);
  return cfgPitchSign * atan2f(-(float)ax, (float)az);
}

static int16_t selectPitchGyroRaw(int16_t gx, int16_t gy) {
  return cfgPitchIsX ? gx : gy;
}

static void initMpu6050() {
  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(400000);

  uint8_t who = 0;
  mpuAddr = 0x68;
  if (!readMpuReg(0x75, who)) {
    mpuAddr = 0x69;
    readMpuReg(0x75, who);
  }
  Serial.print("MPU6050 addr=0x"); Serial.print(mpuAddr, HEX);
  Serial.print(" WHO_AM_I=0x"); Serial.println(who, HEX);

  writeMpuReg(0x6B, 0x00); // Danh thuc MPU6050
  writeMpuReg(0x1A, 0x03); // DLPF ~44 Hz giam rung co khi
  writeMpuReg(0x1B, 0x08); // Gyro +-500 deg/s (65.5 LSB/(deg/s))
  writeMpuReg(0x1C, 0x00); // Accel +-2g
  delay(60);
  calibrateMpu();
}

// Hieu chuan MPU6050 5 giay (goi luc khoi dong & khi bam 'Calib' tren web; motor phai dang tat)
static void calibrateMpu() {
#if CALIB_LED_PIN >= 0
  pinMode(CALIB_LED_PIN, OUTPUT);
  digitalWrite(CALIB_LED_PIN, LOW);
#endif
  Serial.println("Dang calib MPU6050 trong 5 giay... Hay giu xe DUNG YEN tai diem can bang!");

  unsigned long startMs = millis();
  unsigned long lastCountdownMs = 0UL;
  unsigned long lastBlinkMs = 0UL;
  int secLeft = (int)((CALIB_DURATION_MS + 999UL) / 1000UL);
  bool ledState = false;

  double sumGp = 0.0;
  double sumGz = 0.0;
  int valid = 0;
#if AUTO_CALIB_PITCH_ZERO
  double sumPitch = 0.0;
  int validPitch = 0;
#endif

  Serial.print("  Calib con "); Serial.print(secLeft); Serial.println("s...");
  while (millis() - startMs < CALIB_DURATION_MS) {
    unsigned long elapsed = millis() - startMs;

    if (elapsed - lastBlinkMs >= 125UL) {
      lastBlinkMs = elapsed;
      ledState = !ledState;
#if CALIB_LED_PIN >= 0
      digitalWrite(CALIB_LED_PIN, ledState ? HIGH : LOW);
#endif
    }

    if (elapsed - lastCountdownMs >= 1000UL) {
      lastCountdownMs += 1000UL;
      secLeft--;
      if (secLeft > 0) {
        Serial.print("  Calib con "); Serial.print(secLeft); Serial.println("s...");
      }
    }

    // Bo qua CALIB_SETTLE_MS (1.5s) dau de tay on dinh xe, lay mau trong 3.5s con lai
    if (elapsed >= CALIB_SETTLE_MS) {
      int16_t ax, ay, az, gx, gy, gz;
      if (readMpuRaw(ax, ay, az, gx, gy, gz)) {
        sumGp += (double)selectPitchGyroRaw(gx, gy);
        sumGz += (double)gz;
        valid++;
#if AUTO_CALIB_PITCH_ZERO
        if (ax != 0 || ay != 0 || az != 0) {
          sumPitch += (double)computeRawAccelPitch(ax, ay, az);
          validPitch++;
        }
#endif
      }
    }
    delay(5);
  }

  if (valid > 0) {
    gyroPitchOffset = (float)(sumGp / (double)valid);
    gyroZOffset = (float)(sumGz / (double)valid);
    Serial.print("Calib gyro xong ("); Serial.print(valid);
    Serial.print(" mau) | gpOff="); Serial.print(gyroPitchOffset, 1);
    Serial.print(", gzOff="); Serial.println(gyroZOffset, 1);
  } else {
    Serial.println("[CANH BAO] Khong doc duoc MPU6050 trong 5s calib! Kiem tra SDA(21)/SCL(22)/VCC(3.3V).");
  }

#if AUTO_CALIB_PITCH_ZERO
  if (validPitch > 0) {
    float avgRawPitch = (float)(sumPitch / (double)validPitch);
    // Chi tu dong zero neu xe giu gan thang dung (<= 25 deg), tranh zero nham khi xe nam ngang
    if (fabsf(avgRawPitch) <= 0.436f) {
      pitchZeroOffset = avgRawPitch + PITCH_OFFSET_RAD;
      Serial.print("-> Tu dong Zero trong tam/goc can bang! pitchZeroOffset(deg)=");
      Serial.println(pitchZeroOffset * 57.29578f, 2);
    } else {
      pitchZeroOffset = PITCH_OFFSET_RAD;
      Serial.print("[CANH BAO] Xe nghieng > 25 deg luc calib (");
      Serial.print(avgRawPitch * 57.29578f, 1);
      Serial.println(" deg) -> Giu nguyen PITCH_OFFSET_RAD mac dinh.");
    }
  }
#endif

  int16_t ax, ay, az, gx, gy, gz;
  if (readMpuRaw(ax, ay, az, gx, gy, gz) && (ax != 0 || ay != 0 || az != 0)) {
    lastPitchAccel = computeRawAccelPitch(ax, ay, az) - zeroTotal();
    pitchEst = lastPitchAccel;
  }
}

// ============================================================================
// 6. TRANG THAI DIEU KHIEN & LENH SERIAL (T/Z; them W/S/A/D/X/+/- khi bat di chuyen)
// ============================================================================
#if ENABLE_MOVE_COMMANDS
static float targetSpeed = 0.24f;    // Toc do tien/lui mac dinh cho ban 130 RPM (m/s)
static float targetTurnRate = 1.20f; // Toc do re mac dinh (rad/s)
static float targetV = 0.0f;
static float targetYawRate = 0.0f;
#endif
static float cmdV = 0.0f;
static float cmdYawRate = 0.0f;
static float posErr = 0.0f;
static float yawErr = 0.0f;

static long prevEncA = 0, prevEncB = 0;
static float prevActL = 0.0f, prevActR = 0.0f;
static unsigned long lastStepUs = 0;
static bool telemetryEnabled = false;
static bool safetyTripped = false;
static bool paused = false;   // Pause tu web/Serial: tat motor, giu telemetry, 'Tiep tuc' de chay lai RL
static int uprightStableCount = 0;

static float clampf(float v, float lo, float hi) {
  if (v > hi) return hi;
  if (v < lo) return lo;
  return v;
}

static inline float slewToward(float cur, float target, float maxDelta) {
  float diff = target - cur;
  if (diff > maxDelta) return cur + maxDelta;
  if (diff < -maxDelta) return cur - maxDelta;
  return target;
}

static void resetControllerState(long encA, long encB) {
  prevEncA = encA;
  prevEncB = encB;
  prevActL = 0.0f;
  prevActR = 0.0f;
  posErr = 0.0f;
  yawErr = 0.0f;
  cmdV = 0.0f;
  cmdYawRate = 0.0f;
}

static void maybeLogTelemetry(long encA, long encB, float wL, float wR,
                              float vFwd, float actL, float actR) {
  if (!telemetryEnabled) return;
  static unsigned long lastLogMs = 0;
  unsigned long nowMs = millis();
  if (nowMs - lastLogMs < 200) return;
  lastLogMs = nowMs;
  Serial.print(safetyTripped ? "[SAFE] " : "[RUN]  ");
  Serial.print("pitch(deg)="); Serial.print(pitchEst * 57.29578f, 2);
  Serial.print(" (acc="); Serial.print(lastPitchAccel * 57.29578f, 2); Serial.print(")");
  Serial.print(" | encL/R="); Serial.print(encA); Serial.print("/"); Serial.print(encB);
  Serial.print(" | wL/R="); Serial.print(wL, 1); Serial.print("/"); Serial.print(wR, 1);
  Serial.print(" | v="); Serial.print(vFwd, 2);
  Serial.print(" | act="); Serial.print(actL, 2); Serial.print(","); Serial.println(actR, 2);
}

static void processSerialCommands(long encA, long encB) {
  while (Serial.available() > 0) {
    char command = (char)toupper(Serial.read());
    if (command == 'T') {
      telemetryEnabled = !telemetryEnabled;
      Serial.println(telemetryEnabled ? "-> BAT Telemetry (200ms)" : "-> TAT Telemetry");
    } else if (command == 'P') {
      paused = !paused;
      resetControllerState(encA, encB);
      uprightStableCount = 0;
      Serial.println(paused ? "-> PAUSE (tat motor)" : "-> TIEP TUC");
    } else if (command == 'Z') {
      pitchZeroOffset += pitchEst;
      pitchEst = 0.0f;
      resetControllerState(encA, encB);
      Serial.print("-> Da Zero goc & encoder! pitchZeroOffset(deg)=");
      Serial.println(pitchZeroOffset * 57.29578f, 2);
#if ENABLE_MOVE_COMMANDS
    } else if (command == 'W') {
      targetV = targetSpeed;
      targetYawRate = 0.0f;
      Serial.println("-> Chay TIEN");
    } else if (command == 'S') {
      targetV = -targetSpeed;
      targetYawRate = 0.0f;
      Serial.println("-> Chay LUI");
    } else if (command == 'A') {
      targetYawRate = targetTurnRate;
      Serial.println("-> Re TRAI");
    } else if (command == 'D') {
      targetYawRate = -targetTurnRate;
      Serial.println("-> Re PHAI");
    } else if (command == 'X' || command == ' ') {
      targetV = 0.0f;
      targetYawRate = 0.0f;
      Serial.println("-> DUNG CAN BANG");
    } else if (command == '+') {
      targetSpeed = clampf(targetSpeed + 0.04f, 0.10f, MAX_CMD_V);
      if (targetV > 0.01f) targetV = targetSpeed;
      else if (targetV < -0.01f) targetV = -targetSpeed;
      Serial.print("Toc do muc tieu: "); Serial.println(targetSpeed, 2);
    } else if (command == '-') {
      targetSpeed = clampf(targetSpeed - 0.04f, 0.10f, MAX_CMD_V);
      if (targetV > 0.01f) targetV = targetSpeed;
      else if (targetV < -0.01f) targetV = -targetSpeed;
      Serial.print("Toc do muc tieu: "); Serial.println(targetSpeed, 2);
#endif
    }
  }
}

// ---- Lenh tu giao dien web: trim / offset / zero / calib ----
static Preferences prefs;

static void saveTrim() {
  prefs.begin("balance", false);
  prefs.putFloat("trim", pitchTrim);
  prefs.end();
}

static void saveConfig() {
  prefs.begin("balance", false);
  prefs.putBool("axisX", cfgPitchIsX);
  prefs.putChar("ps", cfgPitchSign > 0 ? 1 : -1);
  prefs.putChar("ys", cfgYawSign > 0 ? 1 : -1);
  prefs.putChar("el", cfgEncLSign > 0 ? 1 : -1);
  prefs.putChar("er", cfgEncRSign > 0 ? 1 : -1);
  prefs.putChar("ml", cfgMotorLSign > 0 ? 1 : -1);
  prefs.putChar("mr", cfgMotorRSign > 0 ? 1 : -1);
  prefs.putBool("mirror", cfgMirrorEnc);
  prefs.end();
}

static void loadConfig() {
  prefs.begin("balance", true);
  cfgPitchIsX   = prefs.getBool("axisX", cfgPitchIsX);
  cfgPitchSign  = prefs.getChar("ps", 1) >= 0 ? 1.0f : -1.0f;
  cfgYawSign    = prefs.getChar("ys", 1) >= 0 ? 1.0f : -1.0f;
  cfgEncLSign   = prefs.getChar("el", 1) >= 0 ? 1.0f : -1.0f;
  cfgEncRSign   = prefs.getChar("er", 1) >= 0 ? 1.0f : -1.0f;
  cfgMotorLSign = prefs.getChar("ml", 1) >= 0 ? 1.0f : -1.0f;
  cfgMotorRSign = prefs.getChar("mr", 1) >= 0 ? 1.0f : -1.0f;
  cfgMirrorEnc  = prefs.getBool("mirror", false);
  prefs.end();
}

// Chay lai calib 5s voi motor tat (dung chung cho lenh Calib & khi doi truc / dau pitch)
static void runCalibNow() {
  driveMotor(ENA, IN1, IN2, 0.0f);
  driveMotor(ENB, IN3, IN4, 0.0f);
  webSetState(1);
  calibrateMpu();
  i2cFailCount = 0;
  resetEncoders();
  resetControllerState(0, 0);
  uprightStableCount = 0;
  safetyTripped = fabsf(pitchEst) > 0.785f;
  webClearCommands();            // bo cac lenh bam trong luc dang calib
  lastStepUs = micros();
}

// Doi diem zero them `delta` rad: goc hien thi dich -delta ngay lap tuc (khong cho bo loc troi)
static inline void shiftPitchZero(float delta) {
  pitchEst -= delta;
  lastPitchAccel -= delta;
}

// Tim goc pitch p* (do) tai do policy cho luc trung binh = 0 khi dung yen (van toc = 0).
// Policy co do lech co san (act != 0 o pitch = 0); bu bang trim de xe dung thang khong bi day 1 phia.
static float policyZeroPitchDeg() {
  float lo = -10.0f * 3.14159265f / 180.0f, hi = -lo;
  for (int it = 0; it < 24; it++) {
    float mid = 0.5f * (lo + hi);
    float a[RL_ACT_DIM] = {0.0f, 0.0f};
    for (int k = 0; k < 20; k++) {          // prevAct la 2 dau vao cuoi -> lap den diem co dinh
      float obs[RL_OBS_DIM] = {0, 0, mid, 0, 0, 0, 0, 0, 0, 0, a[0], a[1]};
      rl_policy_predict(obs, a);
    }
    if (0.5f * (a[0] + a[1]) < 0.0f) lo = mid; else hi = mid;   // act tang theo pitch
  }
  return 0.5f * (lo + hi) * 57.29578f;
}

// Tra ve true neu da chay calib (loop() nen bo qua chu ky hien tai vi dt/encoder da cu)
static bool processWebCommands() {
  const float degToRad = 3.14159265f / 180.0f;
  WebCmd c;
  while (webPopCommand(c)) {
    if (c.type == WCMD_TRIM) {
      float newTrim = clampf(pitchTrim + c.val * degToRad, -10.0f * degToRad, 10.0f * degToRad);
      shiftPitchZero(newTrim - pitchTrim);
      pitchTrim = newTrim;
      saveTrim();
      Serial.print("[WEB] Trim(deg)="); Serial.println(pitchTrim / degToRad, 2);
    } else if (c.type == WCMD_AUTO_TRIM) {
      if (!paused) { Serial.println("[WEB] Auto-trim: can Pause truoc"); continue; }
      float pz = policyZeroPitchDeg();                       // pitch (do) de act = 0
      float newTrim = clampf(-pz * degToRad, -10.0f * degToRad, 10.0f * degToRad);
      shiftPitchZero(newTrim - pitchTrim);
      pitchTrim = newTrim;
      saveTrim();
      Serial.print("[WEB] Auto-trim: policy zero tai pitch(deg)="); Serial.print(pz, 2);
      Serial.print(" -> trim(deg)="); Serial.println(pitchTrim / degToRad, 2);
    } else if (c.type == WCMD_RESET_TRIM) {
      shiftPitchZero(-pitchTrim);
      pitchTrim = 0.0f;
      saveTrim();
      Serial.println("[WEB] Reset trim = 0");
    } else if (c.type == WCMD_SET_OFFSET) {
      float newOff = c.val * degToRad;
      shiftPitchZero(newOff - pitchZeroOffset);
      pitchZeroOffset = newOff;
      Serial.print("[WEB] Offset(deg)="); Serial.println(c.val, 2);
    } else if (c.type == WCMD_ZERO) {
      pitchZeroOffset += pitchEst;
      pitchEst = 0.0f;
      lastPitchAccel = 0.0f;
      Serial.print("[WEB] Zero tai cho! Offset(deg)="); Serial.println(pitchZeroOffset / degToRad, 2);
    } else if (c.type == WCMD_CALIB) {
      Serial.println("[WEB] Calib lai: tat motor...");
      runCalibNow();
      return true;
    } else if (c.type == WCMD_PAUSE) {
      paused = true;
      driveMotor(ENA, IN1, IN2, 0.0f);
      driveMotor(ENB, IN3, IN4, 0.0f);
      Serial.println("[WEB] PAUSE: tat motor");
    } else if (c.type == WCMD_RESUME) {
      long a, b;
      readEncoders(a, b);
      resetControllerState(a, b);
      uprightStableCount = 0;
      paused = false;
      webSetManual(0.0f, 0.0f);
      Serial.println("[WEB] TIEP TUC");
    } else if (c.type == WCMD_CFG_AXIS) {
      cfgPitchIsX = !cfgPitchIsX;
      saveConfig();
      Serial.print("[WEB] Truc pitch = "); Serial.println(cfgPitchIsX ? "X (gx, ay)" : "Y (gy, ax)");
      runCalibNow();                 // doi truc phai calib lai gyro & zero
      return true;
    } else if (c.type == WCMD_CFG_FLIP) {
      int k = (int)c.val;
      driveMotor(ENA, IN1, IN2, 0.0f);
      driveMotor(ENB, IN3, IN4, 0.0f);
      if      (k == 0) cfgPitchSign  = -cfgPitchSign;
      else if (k == 1) cfgYawSign    = -cfgYawSign;
      else if (k == 2) cfgEncLSign   = -cfgEncLSign;
      else if (k == 3) cfgEncRSign   = -cfgEncRSign;
      else if (k == 4) cfgMotorLSign = -cfgMotorLSign;
      else if (k == 5) cfgMotorRSign = -cfgMotorRSign;
      else if (k == 6) cfgMirrorEnc  = !cfgMirrorEnc;
      saveConfig();
      Serial.print("[WEB] Dao dau cau hinh #"); Serial.println(k);
      if (k == 0) { runCalibNow(); return true; }   // doi dau pitch -> calib lai
      long a, b;
      readEncoders(a, b);
      resetControllerState(a, b);
      return true;
    } else if (c.type == WCMD_ENC_RESET) {
      resetEncoders();
      resetControllerState(0, 0);
      Serial.println("[WEB] Reset encoder");
    }
  }
  return false;
}

void setup() {
  Serial.begin(115200);

  // Doc trim da luu (NVS)
  loadConfig();
  prefs.begin("balance", true);
  pitchTrim = clampf(prefs.getFloat("trim", 0.0f), -0.175f, 0.175f);
  prefs.end();

  // 1. Khoi tao L298N
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  driveMotor(ENA, IN1, IN2, 0.0f);
  driveMotor(ENB, IN3, IN4, 0.0f);

  // 2. Khoi tao Encoder GA25-370 (4x Quadrature tren chan cu 16/17 & 18/19)
  pinMode(ENCA_A, INPUT_PULLUP); pinMode(ENCA_B, INPUT_PULLUP);
  pinMode(ENCB_A, INPUT_PULLUP); pinMode(ENCB_B, INPUT_PULLUP);
  prevStateA = ((uint8_t)digitalRead(ENCA_A) << 1) | (uint8_t)digitalRead(ENCA_B);
  prevStateB = ((uint8_t)digitalRead(ENCB_A) << 1) | (uint8_t)digitalRead(ENCB_B);
  attachInterrupt(digitalPinToInterrupt(ENCA_A), isrEncA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA_B), isrEncA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB_A), isrEncB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB_B), isrEncB, CHANGE);

  // 2b. Phat WiFi AP + web local (http://192.168.4.1) de xem thong so ngay tu luc khoi dong
  webBegin();
  webSetState(0);

  // 3. Test dong co quay nguoc & quay xuoi truoc, sau do tat dong co de calib 5s
  runStartupMotorSelfTest();

  // 4. Khoi tao & calib MPU6050 trong 5 giay (luc nay dong co da tat hoan toan)
  webSetState(1);
  initMpu6050();

  // Xoa xung encoder phat sinh trong luc test / cam giu xe calib 5s
  resetEncoders();
  resetControllerState(0, 0);

  if (fabsf(pitchEst) > 0.785f) {
    safetyTripped = true;
  }
#if CALIB_LED_PIN >= 0
  digitalWrite(CALIB_LED_PIN, safetyTripped ? LOW : HIGH);
#endif

  lastStepUs = micros();
  Serial.println("--- HE THONG RL GA25-370 SAN SANG ---");
  Serial.print("TICKS_PER_REV (4x): "); Serial.println(TICKS_PER_REV, 1);
#if ENABLE_MOVE_COMMANDS
  Serial.println("Che do: CAN BANG + DI CHUYEN | Lenh: W/S/A/D/X/+/-, T(Debug), Z(Zero)");
#else
  Serial.println("Che do: CHI TU CAN BANG TAI CHO | Lenh: T(Debug), Z(Zero)");
#endif
}

void loop() {
  // Doc snapshot encoder an toan
  long encA, encB;
  readEncoders(encA, encB);

  processSerialCommands(encA, encB);
  if (processWebCommands()) return;

  unsigned long nowUs = micros();
  if (nowUs - lastStepUs < (unsigned long)(CONTROL_DT * 1e6f)) return;
  float dt = (nowUs - lastStepUs) * 1e-6f;
  if (dt > 0.05f || dt <= 0.0f) dt = CONTROL_DT;
  lastStepUs = nowUs;

  // Doc MPU6050 & cap nhat bo loc bu (Complementary Filter)
  int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  if (readMpuRaw(ax, ay, az, gx, gy, gz)) {
    i2cFailCount = 0;
    const float degToRad = 3.14159265f / 180.0f;
    float gpRaw = (float)selectPitchGyroRaw(gx, gy);
    lastPitchRate = cfgPitchSign * ((gpRaw - gyroPitchOffset) / 65.5f) * degToRad;
    lastYawRate   = cfgYawSign   * (((float)gz - gyroZOffset) / 65.5f) * degToRad;
    if (ax != 0 || ay != 0 || az != 0) {
      lastPitchAccel = computeRawAccelPitch(ax, ay, az) - zeroTotal();
      pitchEst = 0.98f * (pitchEst + lastPitchRate * dt) + 0.02f * lastPitchAccel;
    } else {
      pitchEst += lastPitchRate * dt;
    }
  } else {
    i2cFailCount++;
    pitchEst += lastPitchRate * dt;
  }

  float pitchRate = lastPitchRate;
  float yawRate = lastYawRate;

  // Tinh van toc goc banh xe (rad/s) tu GA25-370 encoder 4x
  float radPerTick = (2.0f * 3.14159265f) / TICKS_PER_REV;
  float wL = cfgEncLSign * ((float)(encA - prevEncA) * radPerTick) / dt;
  float wR = cfgEncRSign * ((float)(encB - prevEncB) * radPerTick) / dt;
  if (cfgMirrorEnc) wR = wL;   // encoder phai hong -> mirror tu encoder trai
  prevEncA = encA;
  prevEncB = encB;

  // Bu van toc nghieng than xe de lay dung van toc tinh tien mat dat
  float vFwd = (0.5f * (wL + wR) + pitchRate) * WHEEL_RADIUS;

  // Dong goi thong so cho web telemetry (chay o core 0, khong chan vong dieu khien)
  auto publishWeb = [&](float actL, float actR) {
    WebTelemetry t;
    const float radToDeg = 57.29578f;
    t.pitch     = pitchEst * radToDeg;
    t.pitchAcc  = lastPitchAccel * radToDeg;
    t.pitchRate = pitchRate * radToDeg;
    t.yawRate   = yawRate * radToDeg;
    t.ax = (float)ax / 16384.0f; t.ay = (float)ay / 16384.0f; t.az = (float)az / 16384.0f;
    t.gx = (float)gx / 65.5f;    t.gy = (float)gy / 65.5f;    t.gz = (float)gz / 65.5f;
    t.wL = wL; t.wR = wR; t.vFwd = vFwd;
    t.actL = actL; t.actR = actR;
    t.encL = (long)(cfgEncLSign * (float)encA); t.encR = (long)(cfgEncRSign * (float)encB);
    t.pitchZero = pitchZeroOffset * radToDeg;
    t.trim = pitchTrim * radToDeg;
    t.cfg[0] = cfgPitchIsX ? 1 : 0;
    t.cfg[1] = (int8_t)cfgPitchSign;  t.cfg[2] = (int8_t)cfgYawSign;
    t.cfg[3] = (int8_t)cfgEncLSign;   t.cfg[4] = (int8_t)cfgEncRSign;
    t.cfg[5] = (int8_t)cfgMotorLSign; t.cfg[6] = (int8_t)cfgMotorRSign;
    t.cfg[7] = cfgMirrorEnc ? 1 : 0;
    t.dtMs = dt * 1000.0f;
    t.state = paused ? 4 : (safetyTripped ? 3 : 2);
    t.i2cFail = (uint8_t)(i2cFailCount > 255 ? 255 : i2cFailCount);
    webPublish(t);
  };

  // Kiem tra an toan co tre (cat > 45 deg hoac mat IMU; tu khoi phuc khi dung thang < 12 deg trong 0.5s)
  if (fabsf(pitchEst) > 0.785f || i2cFailCount > 5) {
    safetyTripped = true;
    uprightStableCount = 0;
  } else if (safetyTripped) {
    if (fabsf(pitchEst) < 0.21f && i2cFailCount == 0) {
      uprightStableCount++;
      if (uprightStableCount >= 25) {
        safetyTripped = false;
        resetControllerState(encA, encB);
        Serial.println("-> Xe da thang, KICH HOAT LAI RL!");
      }
    } else {
      uprightStableCount = 0;
    }
  }

#if CALIB_LED_PIN >= 0
  digitalWrite(CALIB_LED_PIN, safetyTripped ? LOW : HIGH);
#endif

  if (safetyTripped || paused) {
    // Khi PAUSE co the chay tay tung banh tu web (deadman 400 ms); con lai motor tat
    float mL = 0.0f, mR = 0.0f;
    if (paused) webGetManual(mL, mR);
    mL = clampf(cfgMotorLSign * mL, -1.0f, 1.0f);
    mR = clampf(cfgMotorRSign * mR, -1.0f, 1.0f);
    driveMotor(ENA, IN1, IN2, mL);
    driveMotor(ENB, IN3, IN4, mR);
    resetControllerState(encA, encB);
    publishWeb(mL, mR);
    maybeLogTelemetry(encA, encB, wL, wR, vFwd, 0.0f, 0.0f);
    return;
  }

#if ENABLE_MOVE_COMMANDS
  // Slew-rate ramp cho lenh tien/lui & re
  cmdV = slewToward(cmdV, clampf(targetV, -MAX_CMD_V, MAX_CMD_V), MAX_ACCEL_V * dt);
  cmdYawRate = slewToward(cmdYawRate, clampf(targetYawRate, -MAX_CMD_YAW, MAX_CMD_YAW), MAX_ACCEL_YAW * dt);
#else
  // Chi giu can bang tai cho (khong nhan lenh di chuyen)
  cmdV = 0.0f;
  cmdYawRate = 0.0f;
#endif

  // Tich phan giu vi tri/huong khi dung yen; xa muot khi dang di chuyen
  if (fabsf(cmdV) < 1e-3f && fabsf(vFwd) < 0.15f) {
    posErr = clampf(posErr + vFwd * dt, -0.5f, 0.5f);
  } else {
    posErr *= 0.85f;
  }

  if (fabsf(cmdYawRate) < 1e-3f && fabsf(yawRate) < 0.30f) {
    yawErr = clampf(yawErr + yawRate * dt, -1.0f, 1.0f);
  } else {
    yawErr *= 0.85f;
  }

  // Chay suy luan mang noron RL (12 -> 128 -> 128 -> 2)
  float rawObs[RL_OBS_DIM] = {
    posErr, vFwd, pitchEst, pitchRate, yawErr, yawRate, wL, wR,
    cmdV, cmdYawRate, prevActL, prevActR
  };
  float action[RL_ACT_DIM];
  rl_policy_predict(rawObs, action);

  driveMotor(ENA, IN1, IN2, cfgMotorLSign * action[0]);
  driveMotor(ENB, IN3, IN4, cfgMotorRSign * action[1]);
  prevActL = action[0];
  prevActR = action[1];

  publishWeb(clampf(cfgMotorLSign * action[0], -1.0f, 1.0f), clampf(cfgMotorRSign * action[1], -1.0f, 1.0f));
  maybeLogTelemetry(encA, encB, wL, wR, vFwd, action[0], action[1]);
}
