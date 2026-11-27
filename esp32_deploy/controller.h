#pragma once
// ============================================================================
// controller.h - LOI DIEU KHIEN XE CAN BANG (khong phu thuoc Arduino)
//   - Bo loc bu goc nghieng tu so lieu tho MPU6050
//   - Toc do banh tu encoder, van toc tien, posErr/yawErr, doc lenh di chuyen
//   - Chay mang RL (rl_policy.h) va may trang thai an toan
// File nay bien dich duoc ca tren PC (g++) de chay vong kin voi MuJoCo truoc khi nap.
//
// QUAN TRONG: vector quan sat 12 phan tu, cac hang so va thu tu cap nhat ben duoi
// phai khop 100% voi envs/balance_car_env.py. Khong doi neu chua train lai mang.
// ============================================================================
#include <math.h>
#include <stdint.h>
#include "rl_policy.h"

namespace ctl {

// ---- Khop moi truong huan luyen ----
constexpr float DT            = 0.02f;                    // 50 Hz
constexpr float WHEEL_R       = 0.0325f;                  // banh 65 mm
constexpr float TICKS_PER_REV = 11.0f * 4.0f * 46.8f;     // GA25-370 130 RPM, doc 4x = 2059.2
constexpr float MAX_CMD_V     = 0.28f;                    // m/s
constexpr float MAX_CMD_YAW   = 1.40f;                    // rad/s
constexpr float MAX_ACC_V     = 0.45f;                    // m/s^2
constexpr float MAX_ACC_YAW   = 3.00f;                    // rad/s^2
constexpr float POS_GATE_V    = 0.15f;                    // posErr chi tich phan khi |v| < 0.15
constexpr float YAW_GATE      = 0.30f;                    // yawErr chi tich phan khi |yawRate| < 0.3
constexpr float ERR_DECAY     = 0.85f;
constexpr float POS_ERR_MAX   = 0.5f;
constexpr float YAW_ERR_MAX   = 1.0f;

// ---- MPU6050 (gyro +-500 deg/s, accel +-2 g) ----
constexpr float GYRO_LSB   = 65.5f;
constexpr float ACC_LSB    = 16384.0f;
constexpr float D2R        = 0.017453292f;
constexpr float R2D        = 57.29578f;
constexpr float FUSION_TAU = 1.0f;     // s, hang thoi gian bo loc bu (~ he so 0.98 o 50 Hz)
constexpr float ACC_TRUST  = 0.25f;    // bo qua accel khi |a| lech > 25% so voi 1 g (va cham, rung)

// ---- Mo hinh motor cua mo phong (envs/balance_car_env.py, _apply_motor_model) ----
// Do bang MuJoCo khi banh quay tu do (than xe giu tren khong):
//   w_xac_lap = 12.727 * eff,  eff = (|a| - 0.02) / 0.98 (vung chet 2%),  hang so thoi gian 46 ms
// Dung de ve "mo phong vs thuc te" tren web: cung lenh PWM, banh mo phong quay bao nhieu.
constexpr float SIM_W_PER_EFF = 12.727f;
constexpr float SIM_DEADZONE  = 0.02f;
constexpr float SIM_TAU       = 0.046f;

// ---- An toan (doi qua bien de ban PC co the quet nguong) ----
struct Limits {
  float tiltTrip   = 45.0f * D2R;  // cat motor khi |pitch| vuot (moi truong huan luyen cung dung 45 deg)
  float armPitch   = 8.0f * D2R;   // chi bat RL khi xe gan thang (vung khoi tao luc train: +-9 deg)
  float armRate    = 0.35f;        // rad/s (vung khoi tao luc train: +-0.35 rad/s)
  float armWheel   = 2.0f;         // rad/s, banh phai gan dung yen
  int   armSteps   = 15;           // giu dieu kien tren 0.3 s
  // Chong ket: lenh trung binh 2 banh > 0.8 cung chieu banh dang quay. Trong mo phong (dung yen,
  // day 3.2 N, di chuyen 0.28 m/s, lech zero 4 deg) chuoi nay dai nhat chi 60 ms; khi xe bi nhac
  // len (banh quay tu do) mang ket o ~0.9..1.0 vinh vien -> 0.8 s lien tuc la bat thuong.
  // Nguong toc do de thap (3 rad/s) vi motor that qua L298N co the khong dat 10 rad/s.
  float runSat     = 0.80f;        // |lenh trung binh 2 banh|
  float runWheel   = 3.0f;         // rad/s, banh dang quay theo chieu do
  int   runSteps   = 40;           // lien tuc 0.8 s -> banh quay tu do / mat kiem soat
  int   imuMissMax = 5;            // so chu ky 50 Hz lien tiep khong doc duoc MPU
};

struct Config {
  bool   pitchIsX  = true;   // true: pitch quanh truc X cua MPU (gx, ay); false: quanh Y (gy, ax)
  int8_t pitchSign = 1;
  int8_t yawSign   = 1;
  int8_t encLSign  = 1;
  int8_t encRSign  = 1;
  int8_t motLSign  = 1;
  int8_t motRSign  = 1;
  bool   mirrorR   = true;   // wR = wL (encoder phai hong pha B)
};

// state (khop web): 0 khoi dong, 1 calib, 2 dang can bang, 3 cat motor, 4 tam dung
enum State : uint8_t { ST_BOOT = 0, ST_CALIB = 1, ST_RUN = 2, ST_TRIP = 3, ST_PAUSE = 4 };
// ly do dang cat motor (state 3)
enum Why : uint8_t { WHY_NONE = 0, WHY_WAIT = 1, WHY_TILT = 2, WHY_IMU = 3, WHY_RUNAWAY = 4 };

static inline float clampf(float v, float lo, float hi) { return v > hi ? hi : (v < lo ? lo : v); }
static inline float slewToward(float cur, float target, float maxDelta) {
  float d = target - cur;
  if (d > maxDelta) return cur + maxDelta;
  if (d < -maxDelta) return cur - maxDelta;
  return target;
}

struct Controller {
  Config cfg;
  Limits lim;

  // Hieu chuan
  float gyroBiasP = 0.0f, gyroBiasZ = 0.0f;  // LSB tho
  float zero = 0.0f;                         // rad, goc accel khi xe dung o diem can bang
  float trim = 0.0f;                         // rad, chinh tay (luu NVS)

  // Uoc luong goc
  float pitch = 0.0f, pitchAcc = 0.0f;       // rad (da tru zero + trim)
  float pitchRate = 0.0f, yawRate = 0.0f;    // rad/s
  bool  imuInit = false;
  int   imuMiss = 0;                         // so chu ky 50 Hz khong co mau MPU moi
  bool  imuFresh = false;

  // Dieu khien
  float targetV = 0.0f, targetYaw = 0.0f;    // lenh nguoi dung
  float cmdV = 0.0f, cmdYaw = 0.0f;          // lenh da qua doc (slew)
  float posErr = 0.0f, yawErr = 0.0f;
  float prevA[2] = {0.0f, 0.0f};
  long  prevEncL = 0, prevEncR = 0;
  bool  encInit = false;
  float wL = 0.0f, wR = 0.0f, vFwd = 0.0f;
  float wRraw = 0.0f;                        // toc do banh phai tu encoder phai (khong mirror)
  float simW[2] = {0.0f, 0.0f};              // toc do banh theo mo hinh motor cua mo phong
  float lastFwd[2] = {0.0f, 0.0f};           // lenh da gui chu ky truoc, quy ve chieu tien cua xe
  float act[2] = {0.0f, 0.0f};               // dau ra mang RL (chieu "tien" cua xe)
  float out[2] = {0.0f, 0.0f};               // lenh motor da nhan dau motor, -1..1
  uint8_t state = ST_TRIP, why = WHY_WAIT;
  bool  paused = false;
  int   armCount = 0, runawayCount = 0;

  // ---------------- IMU ----------------
  float accelPitchRaw(int16_t ax, int16_t ay, int16_t az) const {
    // Quy tac ban tay phai MPU6050: +gx khop atan2(+ay, az), +gy khop atan2(-ax, az)
    float p = cfg.pitchIsX ? atan2f((float)ay, (float)az) : atan2f(-(float)ax, (float)az);
    return (float)cfg.pitchSign * p;
  }
  int16_t pitchGyroRaw(int16_t gx, int16_t gy) const { return cfg.pitchIsX ? gx : gy; }

  // Goi moi lan doc duoc MPU (vd 250 Hz). dt = thoi gian tu mau truoc (s).
  void imuSample(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, float dt) {
    pitchRate = (float)cfg.pitchSign * (((float)pitchGyroRaw(gx, gy) - gyroBiasP) / GYRO_LSB) * D2R;
    yawRate   = (float)cfg.yawSign   * (((float)gz - gyroBiasZ) / GYRO_LSB) * D2R;
    float an = sqrtf((float)ax * ax + (float)ay * ay + (float)az * az) / ACC_LSB;
    bool accOk = (an > 1.0f - ACC_TRUST) && (an < 1.0f + ACC_TRUST);
    if (an > 0.05f) pitchAcc = accelPitchRaw(ax, ay, az) - zero - trim;
    if (!imuInit) {
      if (an > 0.05f) { pitch = pitchAcc; imuInit = true; }
    } else {
      pitch += pitchRate * dt;
      if (accOk) {
        float k = dt / (FUSION_TAU + dt);
        pitch += k * (pitchAcc - pitch);
      }
    }
    imuFresh = true;
  }

  // Doi diem 0 them `delta` rad: goc hien thi dich ngay (khong cho bo loc troi)
  void shiftZero(float delta) { pitch -= delta; pitchAcc -= delta; }
  void setZero(float z)  { shiftZero(z - zero); zero = z; }
  void setTrim(float t)  { shiftZero(t - trim); trim = t; }
  void zeroHere()        { setZero(zero + pitch); }  // coi goc hien tai la 0
  void restartFusion()   { imuInit = false; imuMiss = 0; }

  // ---------------- Dieu khien ----------------
  void rebase(long encL, long encR) { prevEncL = encL; prevEncR = encR; encInit = true; }

  void resetCtrl() {
    prevA[0] = prevA[1] = 0.0f;
    act[0] = act[1] = 0.0f;
    posErr = yawErr = 0.0f;
    cmdV = cmdYaw = 0.0f;
    runawayCount = 0;
  }

  void trip(uint8_t reason) {
    if (state == ST_RUN || why != reason) armCount = 0;
    state = ST_TRIP;
    why = reason;
    out[0] = out[1] = 0.0f;
    resetCtrl();
  }

  void setPaused(bool p) {
    paused = p;
    out[0] = out[1] = 0.0f;
    resetCtrl();
    if (!p) trip(WHY_WAIT);   // tiep tuc: cho xe dung thang roi moi bat RL
  }

  static float simWss(float a) {
    float m = fabsf(a);
    if (m < SIM_DEADZONE) return 0.0f;
    float eff = (fminf(m, 1.0f) - SIM_DEADZONE) / (1.0f - SIM_DEADZONE);
    return (a > 0.0f ? 1.0f : -1.0f) * eff * SIM_W_PER_EFF;
  }

  // Goi moi 20 ms. encL/encR = so dem tho (chua nhan dau). dt = thoi gian thuc tu lan truoc (s).
  // manL/manR: lenh chay tay khi tam dung (-1..1, theo chieu tien cua xe).
  void step(long encL, long encR, float dt, float manL = 0.0f, float manR = 0.0f) {
    if (!(dt > 0.0f) || dt > 0.1f) dt = DT;
    if (!encInit) rebase(encL, encR);
    stepCore(encL, encR, dt, manL, manR);
    // Lenh vua quyet dinh, quy ve chieu tien cua xe -> dau vao mo hinh motor chu ky sau
    lastFwd[0] = out[0] * (float)cfg.motLSign;
    lastFwd[1] = out[1] * (float)cfg.motRSign;
  }

 private:
  void stepCore(long encL, long encR, float dt, float manL, float manR) {
    // 1. Toc do banh (rad/s, duong = lan tien) va van toc tien mat dat
    const float k = (2.0f * 3.14159265f / TICKS_PER_REV) / dt;
    wL = (float)cfg.encLSign * (float)(encL - prevEncL) * k;
    wR = (float)cfg.encRSign * (float)(encR - prevEncR) * k;
    wRraw = wR;
    if (cfg.mirrorR) wR = wL;
    // Mo hinh motor mo phong voi lenh da ap trong 20 ms vua qua (banh quay tu do)
    const float a = 1.0f - expf(-dt / SIM_TAU);
    for (int i = 0; i < 2; i++) simW[i] += a * (simWss(lastFwd[i]) - simW[i]);
    prevEncL = encL; prevEncR = encR;
    vFwd = (0.5f * (wL + wR) + pitchRate) * WHEEL_R;

    // 2. MPU con song?
    if (imuFresh) imuMiss = 0; else imuMiss++;
    imuFresh = false;

    // 3. Tam dung: chay tay (deadman do lop web lo)
    if (paused) {
      state = ST_PAUSE; why = WHY_NONE;
      resetCtrl();
      out[0] = clampf((float)cfg.motLSign * manL, -1.0f, 1.0f);
      out[1] = clampf((float)cfg.motRSign * manR, -1.0f, 1.0f);
      return;
    }

    // 4. An toan cung: mat IMU / nga
    if (imuMiss > lim.imuMissMax || !imuInit) { trip(WHY_IMU); return; }
    if (fabsf(pitch) > lim.tiltTrip)          { trip(WHY_TILT); return; }

    // 5. Dang cat motor: cho xe dung thang, dung yen moi bat RL (vung khoi tao luc train)
    if (state != ST_RUN) {
      bool ok = fabsf(pitch) < lim.armPitch && fabsf(pitchRate) < lim.armRate &&
                fabsf(wL) < lim.armWheel && fabsf(wR) < lim.armWheel;
      armCount = ok ? armCount + 1 : 0;
      out[0] = out[1] = 0.0f;
      if (armCount < lim.armSteps) { state = ST_TRIP; return; }
      state = ST_RUN; why = WHY_NONE; armCount = 0;
      resetCtrl();
    }

    // 6. posErr / yawErr: dung thu tu nhu env.step() (gate theo lenh cu, roi moi doc lenh)
    if (fabsf(cmdV) < 1e-3f && fabsf(vFwd) < POS_GATE_V) posErr = clampf(posErr + vFwd * DT, -POS_ERR_MAX, POS_ERR_MAX);
    else posErr *= ERR_DECAY;
    if (fabsf(cmdYaw) < 1e-3f && fabsf(yawRate) < YAW_GATE) yawErr = clampf(yawErr + yawRate * DT, -YAW_ERR_MAX, YAW_ERR_MAX);
    else yawErr *= ERR_DECAY;
    cmdV   = slewToward(cmdV,   clampf(targetV,   -MAX_CMD_V,   MAX_CMD_V),   MAX_ACC_V * DT);
    // Mirror wR = wL thi mang khong thay chenh lech banh khi re -> mo phong: quay tai cho chi con
    // 38% song sot, tien/lui van 100%. Nen khi mirror bat thi bo lenh re.
    float yawTarget = cfg.mirrorR ? 0.0f : targetYaw;
    cmdYaw = slewToward(cmdYaw, clampf(yawTarget, -MAX_CMD_YAW, MAX_CMD_YAW), MAX_ACC_YAW * DT);

    // 7. Mang RL 12 -> 128 -> 128 -> 2
    float obs[RL_OBS_DIM] = { posErr, vFwd, pitch, pitchRate, yawErr, yawRate, wL, wR,
                              cmdV, cmdYaw, prevA[0], prevA[1] };
    rl_policy_predict(obs, act);   // da kep -1..1

    // 8. Chong "ket" khi banh quay tu do (xe bi nhac len / tren tay): mang chua tung thay
    //    trang thai nay luc train -> day mai mot chieu. Lenh TB lon + banh quay cung chieu
    //    lien tuc runSteps chu ky -> cat motor.
    float wAvg = 0.5f * (wL + wR);
    float aAvg = 0.5f * (act[0] + act[1]);
    bool sat = fabsf(aAvg) > lim.runSat && fabsf(wAvg) > lim.runWheel && aAvg * wAvg > 0.0f;
    runawayCount = sat ? runawayCount + 1 : 0;
    if (runawayCount >= lim.runSteps) { trip(WHY_RUNAWAY); return; }

    prevA[0] = act[0]; prevA[1] = act[1];
    out[0] = clampf((float)cfg.motLSign * act[0], -1.0f, 1.0f);
    out[1] = clampf((float)cfg.motRSign * act[1], -1.0f, 1.0f);
  }

 public:
  // Goc pitch (rad) tai do mang cho luc trung binh = 0 khi xe dung yen (dung cho Auto-trim)
  static float policyZeroPitch() {
    float lo = -10.0f * D2R, hi = 10.0f * D2R;
    for (int it = 0; it < 24; it++) {
      float mid = 0.5f * (lo + hi);
      float a[RL_ACT_DIM] = {0.0f, 0.0f};
      for (int k = 0; k < 20; k++) {        // prevAct la 2 dau vao cuoi -> lap den diem co dinh
        float o[RL_OBS_DIM] = {0, 0, mid, 0, 0, 0, 0, 0, 0, 0, a[0], a[1]};
        rl_policy_predict(o, a);
      }
      if (0.5f * (a[0] + a[1]) < 0.0f) lo = mid; else hi = mid;
    }
    return 0.5f * (lo + hi);
  }
};

}  // namespace ctl
