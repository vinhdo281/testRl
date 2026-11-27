// ============================================================================
// XE TU CAN BANG 2 BANH - ESP32 + L298N + GA25-370 (130 RPM) + MPU6050 - MANG RL (PPO)
//
//   main_rl.cpp     : phan cung (motor, encoder, MPU6050), calib, NVS, lenh Serial/web, vong lap
//   controller.h    : loi dieu khien (uoc luong goc, vector quan sat, mang RL, an toan)
//                     -> khong phu thuoc Arduino, da chay vong kin voi MuJoCo tren PC
//   web_telemetry.h : WiFi AP "XeCanBang" (mk 12345678) + trang web http://192.168.4.1
//   rl_policy.h     : trong so mang (sinh boi export_esp32.py)
//
// Trinh tu khi bat nguon:
//   1. Phat WiFi + web
//   2. Tu kiem tra motor: quay NGUOC -> XUOI ~0.35 s moi chieu (de banh quay tu do neu duoc)
//   3. Calib 5 s (LED nhay nhanh): GIU XE DUNG YEN
//        - bias con quay: do duoc o MOI tu the (nam cung duoc)
//        - diem 0: chi lay khi xe dung trong +-25 deg (va luu NVS); nam nghieng -> dung diem 0 da luu
//   4. Cho dung thang (LED nhay cham): dua xe ve gan thang (< 8 deg), giu yen 0.3 s -> RL chay (LED sang)
//
// Serial 115200: T telemetry | P tam dung | Z zero tai cho | C calib | R khoi dong lai
//                W/S tien/lui | A/D re (bi bo khi mirror bat) | X dung | +/- toc do
// ============================================================================
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <ctype.h>
#include "controller.h"
#include "web_telemetry.h"

using ctl::Controller;
using ctl::clampf;
using ctl::D2R;
using ctl::R2D;

// ============================================================================
// 1. CHAN ESP32 (khop day hien tai tren xe - KHONG theo SO_DO_NOI_DAY.md / ban .ino cu)
// ============================================================================
#define ENA      14   // Motor trai (A): PWM
#define IN1      27
#define IN2      26
#define ENB      25   // Motor phai (B): PWM
#define IN3      32
#define IN4      33
#define ENCA_A   16   // Encoder trai: day vang (pha A)
#define ENCA_B   17   //               day xanh la (pha B)
#define ENCB_A   18   // Encoder phai: day vang (pha A)
#define ENCB_B   19   //               day xanh la (pha B)  <- dang ho mach -> mirror
#define IMU_SDA  21
#define IMU_SCL  22
#define LED_PIN  2

// ============================================================================
// 2. TUY CHON
// ============================================================================
#define ENABLE_MOVE_COMMANDS   1       // 1: nhan W/S/A/D/X/+/- qua Serial
#define MOTOR_DEADZONE_PWM     0       // bu vung chet L298N (0..40), 0 = giong mo phong
#define MOTOR_SELFTEST_ENABLE  1
#define MOTOR_SELFTEST_CMD     0.30f
#define MOTOR_SELFTEST_MS      350
#define CALIB_MS               5000UL
#define CALIB_SETTLE_MS        1500UL  // bo 1.5 s dau (tay dang dat xe)
#define ZERO_MAX_RAD           0.436f  // chi lay diem 0 khi xe dung trong +-25 deg
#define CALIB_STILL_DPS        2.0f    // do lech chuan gyro > 2 deg/s -> xe bi rung luc calib
#define IMU_PERIOD_US          4000UL  // doc MPU6050 250 Hz
#define CTRL_DIV               5       // dieu khien 50 Hz = 250 / 5

static Controller C;
static Preferences prefs;
static bool zeroSaved = false;

// ============================================================================
// 3. ENCODER 4x (ngat tren ca 2 pha)
// ============================================================================
static portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;
static volatile long encCountL = 0, encCountR = 0;
static volatile uint8_t encPrevL = 0, encPrevR = 0;
static const int8_t QEM_LUT[16] = { 0, -1, 1, 0,  1, 0, 0, -1,  -1, 0, 0, 1,  0, 1, -1, 0 };

void IRAM_ATTR isrEncL();
void IRAM_ATTR isrEncR();
void isrEncL() {
  uint8_t cur = ((uint8_t)digitalRead(ENCA_A) << 1) | (uint8_t)digitalRead(ENCA_B);
  portENTER_CRITICAL_ISR(&encMux);
  encCountL += QEM_LUT[(encPrevL << 2) | cur];
  encPrevL = cur;
  portEXIT_CRITICAL_ISR(&encMux);
}
void isrEncR() {
  uint8_t cur = ((uint8_t)digitalRead(ENCB_A) << 1) | (uint8_t)digitalRead(ENCB_B);
  portENTER_CRITICAL_ISR(&encMux);
  encCountR += QEM_LUT[(encPrevR << 2) | cur];
  encPrevR = cur;
  portEXIT_CRITICAL_ISR(&encMux);
}
static void readEncoders(long &l, long &r) {
  portENTER_CRITICAL(&encMux);
  l = encCountL; r = encCountR;
  portEXIT_CRITICAL(&encMux);
}
static void resetEncoders() {
  portENTER_CRITICAL(&encMux);
  encCountL = 0; encCountR = 0;
  portEXIT_CRITICAL(&encMux);
}
static void setupEncoders() {
  pinMode(ENCA_A, INPUT_PULLUP); pinMode(ENCA_B, INPUT_PULLUP);
  pinMode(ENCB_A, INPUT_PULLUP); pinMode(ENCB_B, INPUT_PULLUP);
  encPrevL = ((uint8_t)digitalRead(ENCA_A) << 1) | (uint8_t)digitalRead(ENCA_B);
  encPrevR = ((uint8_t)digitalRead(ENCB_A) << 1) | (uint8_t)digitalRead(ENCB_B);
  attachInterrupt(digitalPinToInterrupt(ENCA_A), isrEncL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA_B), isrEncL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB_A), isrEncR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB_B), isrEncR, CHANGE);
}

// ============================================================================
// 4. MOTOR L298N (lenh -1..1 da nhan dau motor)
// ============================================================================
static void driveMotor(int pwmPin, int in1, int in2, float cmd) {
  float mag = fabsf(cmd);
  if (mag < 0.005f) {
    digitalWrite(in1, LOW); digitalWrite(in2, LOW);
    analogWrite(pwmPin, 0);
    return;
  }
  if (mag > 1.0f) mag = 1.0f;
  int pwm = MOTOR_DEADZONE_PWM + (int)(mag * (float)(255 - MOTOR_DEADZONE_PWM));
  if (pwm > 255) pwm = 255;
  digitalWrite(in1, cmd > 0.0f ? HIGH : LOW);
  digitalWrite(in2, cmd > 0.0f ? LOW : HIGH);
  analogWrite(pwmPin, pwm);
}
static void motors(float l, float r) { driveMotor(ENA, IN1, IN2, l); driveMotor(ENB, IN3, IN4, r); }
static void motorsOff() { motors(0.0f, 0.0f); }

// ============================================================================
// 5. MPU6050 (I2C 400 kHz, DLPF 44 Hz, gyro +-500 deg/s, accel +-2 g)
// ============================================================================
static uint8_t mpuAddr = 0x68;
static int16_t imuRaw[6] = {0};    // ax ay az gx gy gz (lan doc gan nhat)
static uint32_t i2cFailTotal = 0;

static void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(mpuAddr); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}
static bool mpuRead8(uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(mpuAddr); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(mpuAddr, (uint8_t)1, (uint8_t)true) != 1) return false;
  val = Wire.read();
  return true;
}
static bool mpuReadRaw(int16_t v[6]) {
  Wire.beginTransmission(mpuAddr); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(mpuAddr, (uint8_t)14, (uint8_t)true) != 14) return false;
  uint8_t b[14];
  for (int i = 0; i < 14; i++) b[i] = Wire.read();
  v[0] = (int16_t)((b[0] << 8) | b[1]);   v[1] = (int16_t)((b[2] << 8) | b[3]);   v[2] = (int16_t)((b[4] << 8) | b[5]);
  v[3] = (int16_t)((b[8] << 8) | b[9]);   v[4] = (int16_t)((b[10] << 8) | b[11]); v[5] = (int16_t)((b[12] << 8) | b[13]);
  return !(v[0] == 0 && v[1] == 0 && v[2] == 0);   // toan 0 = loi doc
}
static void initMpu() {
  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(400000);
  uint8_t who = 0;
  mpuAddr = 0x68;
  if (!mpuRead8(0x75, who)) { mpuAddr = 0x69; mpuRead8(0x75, who); }
  Serial.printf("MPU6050 addr=0x%02X WHO_AM_I=0x%02X\n", mpuAddr, who);
  mpuWrite(0x6B, 0x00);   // danh thuc
  mpuWrite(0x1A, 0x03);   // DLPF ~44 Hz
  mpuWrite(0x1B, 0x08);   // gyro +-500 deg/s
  mpuWrite(0x1C, 0x00);   // accel +-2 g
  delay(60);
}

// ============================================================================
// 6. NVS (namespace & khoa giu nguyen nhu ban cu -> khong mat cau hinh da luu)
// ============================================================================
static void saveConfig() {
  prefs.begin("balance", false);
  prefs.putBool("axisX", C.cfg.pitchIsX);
  prefs.putChar("ps", C.cfg.pitchSign); prefs.putChar("ys", C.cfg.yawSign);
  prefs.putChar("el", C.cfg.encLSign);  prefs.putChar("er", C.cfg.encRSign);
  prefs.putChar("ml", C.cfg.motLSign);  prefs.putChar("mr", C.cfg.motRSign);
  prefs.putBool("mirrorR", C.cfg.mirrorR);
  prefs.end();
}
static void saveTrim() { prefs.begin("balance", false); prefs.putFloat("trim", C.trim); prefs.end(); }
static void saveZero() { prefs.begin("balance", false); prefs.putFloat("zero", C.zero); prefs.end(); zeroSaved = true; }
static void loadAll() {
  prefs.begin("balance", true);
  C.cfg.pitchIsX  = prefs.getBool("axisX", C.cfg.pitchIsX);
  C.cfg.pitchSign = prefs.getChar("ps", 1) >= 0 ? 1 : -1;
  C.cfg.yawSign   = prefs.getChar("ys", 1) >= 0 ? 1 : -1;
  C.cfg.encLSign  = prefs.getChar("el", 1) >= 0 ? 1 : -1;
  C.cfg.encRSign  = prefs.getChar("er", 1) >= 0 ? 1 : -1;
  C.cfg.motLSign  = prefs.getChar("ml", 1) >= 0 ? 1 : -1;
  C.cfg.motRSign  = prefs.getChar("mr", 1) >= 0 ? 1 : -1;
  C.cfg.mirrorR   = prefs.getBool("mirrorR", C.cfg.mirrorR);
  C.trim = clampf(prefs.getFloat("trim", 0.0f), -10.0f * D2R, 10.0f * D2R);
  zeroSaved = prefs.isKey("zero");
  C.zero = clampf(prefs.getFloat("zero", 0.0f), -ZERO_MAX_RAD, ZERO_MAX_RAD);
  prefs.end();
}

// ============================================================================
// 7. CALIB (motor tat): bias gyro o moi tu the, diem 0 chi khi xe gan thang
// ============================================================================
static uint32_t nextImuUs = 0, lastImuUs = 0, lastCtrlUs = 0;
static int imuTick = 0;

static void calibrate() {
  motorsOff();
  webSetState(ctl::ST_CALIB);
  Serial.println("Calib MPU6050 5 s: GIU XE DUNG YEN (dung thang o diem can bang de lay lai diem 0)...");
  double sP = 0, sP2 = 0, sZ = 0, sAcc = 0;
  int n = 0;
  uint32_t t0 = millis(), lastBlink = 0;
  int lastSec = -1;
  bool led = false;
  while (millis() - t0 < CALIB_MS) {
    uint32_t el = millis() - t0;
    if (el - lastBlink >= 125) { lastBlink = el; led = !led; digitalWrite(LED_PIN, led); }
    int sec = (int)((CALIB_MS - el + 999) / 1000);
    if (sec != lastSec) { lastSec = sec; Serial.printf("  con %d s...\n", sec); }
    if (el >= CALIB_SETTLE_MS) {
      int16_t v[6];
      if (mpuReadRaw(v)) {
        double gp = (double)C.pitchGyroRaw(v[3], v[4]);
        sP += gp; sP2 += gp * gp; sZ += v[5];
        sAcc += C.accelPitchRaw(v[0], v[1], v[2]);
        n++;
      }
    }
    delay(4);
  }
  if (n < 50) {
    Serial.println("[LOI] Khong doc duoc MPU6050 khi calib! Kiem tra SDA 21 / SCL 22 / VCC 3.3V.");
  } else {
    float bP = (float)(sP / n), bZ = (float)(sZ / n);
    float sd = sqrtf(fmaxf(0.0f, (float)(sP2 / n) - bP * bP)) / ctl::GYRO_LSB;
    C.gyroBiasP = bP; C.gyroBiasZ = bZ;
    Serial.printf("Bias gyro: pitch=%.1f z=%.1f LSB (%d mau, rung %.2f deg/s)\n", bP, bZ, n, sd);
    if (sd > CALIB_STILL_DPS) Serial.println("  [CANH BAO] Xe bi rung/di chuyen luc calib -> bias co the sai, nen calib lai.");
    float accP = (float)(sAcc / n);
    if (fabsf(accP) <= ZERO_MAX_RAD) {
      C.zero = accP;
      saveZero();
      Serial.printf("Diem 0 moi = %.2f deg (da luu NVS)\n", C.zero * R2D);
    } else {
      Serial.printf("Xe nghieng %.1f deg (> 25) -> khong lay diem 0, dung diem 0 %s = %.2f deg\n",
                    accP * R2D, zeroSaved ? "da luu" : "mac dinh", C.zero * R2D);
    }
  }
  C.restartFusion();
  resetEncoders();
  C.rebase(0, 0);
  C.trip(ctl::WHY_WAIT);
  webClearCommands();
  uint32_t now = micros();
  nextImuUs = now; lastImuUs = now; lastCtrlUs = now; imuTick = 0;
}

// ============================================================================
// 8. TU KIEM TRA MOTOR LUC BAT NGUON
// ============================================================================
static void motorSelfTest() {
#if MOTOR_SELFTEST_ENABLE
  Serial.println("Tu kiem tra motor: quay NGUOC -> quay XUOI...");
  long l, r, revL, revR, fwdL, fwdR;
  resetEncoders();
  motors(-C.cfg.motLSign * MOTOR_SELFTEST_CMD, -C.cfg.motRSign * MOTOR_SELFTEST_CMD);
  delay(MOTOR_SELFTEST_MS); motorsOff(); delay(150);
  readEncoders(l, r); revL = C.cfg.encLSign * l; revR = C.cfg.encRSign * r;
  delay(150);
  resetEncoders();
  motors(C.cfg.motLSign * MOTOR_SELFTEST_CMD, C.cfg.motRSign * MOTOR_SELFTEST_CMD);
  delay(MOTOR_SELFTEST_MS); motorsOff(); delay(250);
  readEncoders(l, r); fwdL = C.cfg.encLSign * l; fwdR = C.cfg.encRSign * r;
  Serial.printf("  [NGUOC] encL=%ld encR=%ld | [XUOI] encL=%ld encR=%ld (dung: NGUOC am, XUOI duong)\n", revL, revR, fwdL, fwdR);
  if (labs(revL) <= 2 && labs(fwdL) <= 2)
    Serial.println("  [CANH BAO] Khong thay xung encoder TRAI (3.3V, chan 16/17, day motor trai) - hoac banh dang cham dat.");
  else if (fwdL < 0 && revL > 0)
    Serial.println("  [GOI Y] Encoder trai nguoc chieu -> dao dau encoder trai tren web.");
  if (labs(revR) <= 2 && labs(fwdR) <= 2)
    Serial.printf("  [CANH BAO] Khong thay xung encoder PHAI. Mirror wR=wL dang %s.\n", C.cfg.mirrorR ? "BAT" : "TAT -> nen bat tren web");
  else if (fwdR < 0 && revR > 0)
    Serial.println("  [GOI Y] Encoder phai nguoc chieu -> dao dau encoder phai tren web.");
  resetEncoders();
#endif
}

// ============================================================================
// 9. LENH SERIAL
// ============================================================================
static bool serialLog = false;
#if ENABLE_MOVE_COMMANDS
static float moveSpeed = 0.24f;   // m/s
static float turnRate  = 1.20f;   // rad/s
#endif

static void rebootNow() {
  motorsOff();
  Serial.println("Khoi dong lai ESP32...");
  Serial.flush();
  delay(300);                     // cho task web (core 0) gui xong phan hoi HTTP
  ESP.restart();
}

static bool processSerial() {
  while (Serial.available() > 0) {
    char c = (char)toupper(Serial.read());
    if (c == 'T') { serialLog = !serialLog; Serial.println(serialLog ? "-> BAT log (5 Hz)" : "-> TAT log"); }
    else if (c == 'P') { C.setPaused(!C.paused); webSetManual(0, 0); Serial.println(C.paused ? "-> TAM DUNG" : "-> TIEP TUC (cho dung thang)"); }
    else if (c == 'Z') { C.zeroHere(); saveZero(); Serial.printf("-> Zero tai cho: diem 0 = %.2f deg (da luu)\n", C.zero * R2D); }
    else if (c == 'C') { calibrate(); return true; }
    else if (c == 'R') { rebootNow(); }
#if ENABLE_MOVE_COMMANDS
    else if (c == 'W') { C.targetV = moveSpeed;  C.targetYaw = 0; Serial.println("-> TIEN"); }
    else if (c == 'S') { C.targetV = -moveSpeed; C.targetYaw = 0; Serial.println("-> LUI"); }
    else if (c == 'A' || c == 'D') {
      if (C.cfg.mirrorR) Serial.println("-> Bo qua lenh re: mirror wR=wL dang bat (re se nga - da thu tren mo phong). Sua encoder phai roi tat mirror.");
      else { C.targetYaw = (c == 'A' ? 1.0f : -1.0f) * turnRate; Serial.println(c == 'A' ? "-> RE TRAI" : "-> RE PHAI"); }
    }
    else if (c == 'X' || c == ' ') { C.targetV = 0; C.targetYaw = 0; Serial.println("-> DUNG"); }
    else if (c == '+' || c == '-') {
      moveSpeed = clampf(moveSpeed + (c == '+' ? 0.04f : -0.04f), 0.08f, ctl::MAX_CMD_V);
      if (C.targetV > 0.01f) C.targetV = moveSpeed; else if (C.targetV < -0.01f) C.targetV = -moveSpeed;
      Serial.printf("Toc do: %.2f m/s\n", moveSpeed);
    }
#endif
  }
  return false;
}

// ============================================================================
// 10. LENH TU WEB (giu nguyen ten lenh /cmd cua trang web)
// ============================================================================
static void afterCfgChange() {
  motorsOff();
  long l, r; readEncoders(l, r);
  C.rebase(l, r);
  if (!C.paused) C.trip(ctl::WHY_WAIT);
}

static bool processWeb() {
  WebCmd w;
  while (webPopCommand(w)) {
    switch (w.type) {
      case WCMD_TRIM:
        C.setTrim(clampf(C.trim + w.val * D2R, -10.0f * D2R, 10.0f * D2R)); saveTrim();
        Serial.printf("[WEB] Trim = %.2f deg\n", C.trim * R2D); break;
      case WCMD_AUTO_TRIM:
        if (!C.paused) { Serial.println("[WEB] Auto-trim: can tam dung truoc"); break; }
        C.setTrim(clampf(-Controller::policyZeroPitch(), -10.0f * D2R, 10.0f * D2R)); saveTrim();
        Serial.printf("[WEB] Auto-trim -> trim = %.2f deg\n", C.trim * R2D); break;
      case WCMD_RESET_TRIM:
        C.setTrim(0.0f); saveTrim(); Serial.println("[WEB] Trim = 0"); break;
      case WCMD_SET_OFFSET:
        C.setZero(w.val * D2R); saveZero(); Serial.printf("[WEB] Diem 0 = %.2f deg (da luu)\n", C.zero * R2D); break;
      case WCMD_ZERO:
        C.zeroHere(); saveZero(); Serial.printf("[WEB] Zero tai cho: diem 0 = %.2f deg (da luu)\n", C.zero * R2D); break;
      case WCMD_CALIB:
        Serial.println("[WEB] Calib lai"); calibrate(); return true;
      case WCMD_PAUSE:
        C.setPaused(true); motorsOff(); Serial.println("[WEB] TAM DUNG"); break;
      case WCMD_RESUME: {
        long l, r; readEncoders(l, r); C.rebase(l, r);
        C.setPaused(false); webSetManual(0, 0); Serial.println("[WEB] TIEP TUC (cho dung thang)"); break;
      }
      case WCMD_CFG_AXIS:
        C.cfg.pitchIsX = !C.cfg.pitchIsX; C.zero = 0.0f; saveConfig(); saveZero();
        Serial.printf("[WEB] Truc pitch = %s -> calib lai\n", C.cfg.pitchIsX ? "X (gx, ay)" : "Y (gy, ax)");
        calibrate(); return true;
      case WCMD_CFG_FLIP: {
        int k = (int)w.val;
        if      (k == 0) { C.cfg.pitchSign = -C.cfg.pitchSign; C.zero = -C.zero; }
        else if (k == 1) C.cfg.yawSign  = -C.cfg.yawSign;
        else if (k == 2) C.cfg.encLSign = -C.cfg.encLSign;
        else if (k == 3) C.cfg.encRSign = -C.cfg.encRSign;
        else if (k == 4) C.cfg.motLSign = -C.cfg.motLSign;
        else if (k == 5) C.cfg.motRSign = -C.cfg.motRSign;
        else if (k == 6) { C.cfg.mirrorR = !C.cfg.mirrorR; if (C.cfg.mirrorR) C.targetYaw = 0; }
        saveConfig();
        Serial.printf("[WEB] Dao cau hinh #%d\n", k);
        if (k == 0) { saveZero(); calibrate(); return true; }
        afterCfgChange();
        return true;
      }
      case WCMD_ENC_RESET:
        resetEncoders(); C.rebase(0, 0); Serial.println("[WEB] Encoder = 0"); break;
      case WCMD_REBOOT:
        rebootNow(); break;
      default: break;
    }
  }
  return false;
}

// ============================================================================
// 11. TELEMETRY
// ============================================================================
static void publish(long encL, long encR, float dt) {
  WebTelemetry t;
  t.pitch = C.pitch * R2D; t.pitchAcc = C.pitchAcc * R2D;
  t.pitchRate = C.pitchRate * R2D; t.yawRate = C.yawRate * R2D;
  t.ax = imuRaw[0] / ctl::ACC_LSB; t.ay = imuRaw[1] / ctl::ACC_LSB; t.az = imuRaw[2] / ctl::ACC_LSB;
  t.gx = imuRaw[3] / ctl::GYRO_LSB; t.gy = imuRaw[4] / ctl::GYRO_LSB; t.gz = imuRaw[5] / ctl::GYRO_LSB;
  t.wL = C.wL; t.wR = C.wR; t.vFwd = C.vFwd;
  t.actL = C.out[0]; t.actR = C.out[1];
  t.encL = C.cfg.encLSign * encL; t.encR = C.cfg.encRSign * encR;
  t.pitchZero = C.zero * R2D; t.trim = C.trim * R2D;
  t.dtMs = dt * 1000.0f;
  t.state = C.state; t.why = C.why;
  t.i2cFail = (uint8_t)(i2cFailTotal > 255 ? 255 : i2cFailTotal);
  t.cfg[0] = C.cfg.pitchIsX ? 1 : 0;
  t.cfg[1] = C.cfg.pitchSign; t.cfg[2] = C.cfg.yawSign;
  t.cfg[3] = C.cfg.encLSign;  t.cfg[4] = C.cfg.encRSign;
  t.cfg[5] = C.cfg.motLSign;  t.cfg[6] = C.cfg.motRSign;
  t.cfg[7] = C.cfg.mirrorR ? 1 : 0;
  t.wLs = C.simW[0]; t.wRs = C.simW[1]; t.wRr = C.wRraw;
  webPublish(t);
}

static const char *whyText(uint8_t why) {
  switch (why) {
    case ctl::WHY_WAIT:    return "cho dung thang (< 8 deg, giu yen 0.3 s)";
    case ctl::WHY_TILT:    return "NGA > 45 deg -> cat motor";
    case ctl::WHY_IMU:     return "LOI MPU6050 -> cat motor";
    case ctl::WHY_RUNAWAY: return "BANH QUAY TU DO / bao hoa 0.8 s -> cat motor (xe bi nhac len?)";
    default:               return "";
  }
}

static void logLine(float dt) {
  static uint32_t last = 0;
  if (!serialLog || millis() - last < 200) return;
  last = millis();
  static const char *ST[] = {"BOOT", "CALIB", "RUN", "CAT", "PAUSE"};
  Serial.printf("[%s] pitch=%.2f rate=%.1f | wL=%.1f wR=%.1f (sim %.1f/%.1f) v=%.2f | out=%.2f,%.2f | pos=%.2f yaw=%.2f dt=%.1fms\n",
                ST[C.state < 5 ? C.state : 0], C.pitch * R2D, C.pitchRate * R2D, C.wL, C.wRraw, C.simW[0], C.simW[1],
                C.vFwd, C.out[0], C.out[1], C.posErr, C.yawErr, dt * 1000.0f);
}

// ============================================================================
// 12. SETUP / LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  motorsOff();

  loadAll();
  Serial.println();
  Serial.println("=== XE CAN BANG RL (GA25-370 130 RPM) ===");
  Serial.printf("Cau hinh: truc %s | dau pitch %+d yaw %+d encL %+d encR %+d motL %+d motR %+d | mirror wR=wL %s\n",
                C.cfg.pitchIsX ? "X" : "Y", C.cfg.pitchSign, C.cfg.yawSign, C.cfg.encLSign, C.cfg.encRSign,
                C.cfg.motLSign, C.cfg.motRSign, C.cfg.mirrorR ? "BAT" : "tat");
  Serial.printf("Diem 0 %s = %.2f deg | trim = %.2f deg\n", zeroSaved ? "da luu" : "mac dinh", C.zero * R2D, C.trim * R2D);

  setupEncoders();
  webBegin();
  webSetState(ctl::ST_BOOT);
  motorSelfTest();
  initMpu();
  calibrate();

  Serial.println("--- SAN SANG: dua xe ve gan thang (< 8 deg) va giu yen de bat RL ---");
#if ENABLE_MOVE_COMMANDS
  Serial.println("Lenh: T log | P tam dung | Z zero | C calib | R reset | W/S/A/D/X/+/- di chuyen");
#else
  Serial.println("Lenh: T log | P tam dung | Z zero | C calib | R reset");
#endif
}

void loop() {
  if (processSerial()) return;
  if (processWeb()) return;

  uint32_t now = micros();
  if ((int32_t)(now - nextImuUs) < 0) return;
  nextImuUs += IMU_PERIOD_US;
  if ((int32_t)(now - nextImuUs) > (int32_t)(4 * IMU_PERIOD_US)) nextImuUs = now + IMU_PERIOD_US;  // bi tre lau -> dong bo lai

  float dtImu = (float)(now - lastImuUs) * 1e-6f;
  lastImuUs = now;
  if (!(dtImu > 0.0f) || dtImu > 0.05f) dtImu = IMU_PERIOD_US * 1e-6f;
  int16_t v[6];
  if (mpuReadRaw(v)) {
    memcpy(imuRaw, v, sizeof(v));
    C.imuSample(v[0], v[1], v[2], v[3], v[4], v[5], dtImu);
  } else {
    i2cFailTotal++;
  }

  if (++imuTick < CTRL_DIV) return;
  imuTick = 0;

  // ---- Chu ky dieu khien 50 Hz ----
  float dt = (float)(now - lastCtrlUs) * 1e-6f;
  lastCtrlUs = now;
  long encL, encR;
  readEncoders(encL, encR);
  float manL = 0.0f, manR = 0.0f;
  if (C.paused) webGetManual(manL, manR);

  uint8_t st0 = C.state, why0 = C.why;
  C.step(encL, encR, dt, manL, manR);
  motors(C.out[0], C.out[1]);

  if (C.state != st0 || C.why != why0) {
    if (C.state == ctl::ST_RUN) Serial.println("-> Xe da thang: BAT RL");
    else if (C.state == ctl::ST_TRIP) Serial.printf("-> %s\n", whyText(C.why));
  }

  // LED: sang = RL dang chay / tam dung, nhay cham = cho dung thang, tat = cat motor do loi/nga
  bool led = (C.state == ctl::ST_RUN || C.state == ctl::ST_PAUSE) ||
             (C.why == ctl::WHY_WAIT && (millis() / 250) % 2 == 0);
  digitalWrite(LED_PIN, led ? HIGH : LOW);

  publish(encL, encR, dt);
  logLine(dt);
}
