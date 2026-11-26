from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import torch
from stable_baselines3 import PPO

from envs.balance_car_env import SelfBalancingCarEnv


def format_c_array(name: str, arr: np.ndarray) -> str:
    flat = arr.flatten()
    vals = ", ".join(f"{v:.8f}f" for v in flat)
    return f"static const float {name}[{len(flat)}] = {{ {vals} }};"


def build_policy_c_code(model: PPO) -> tuple[str, float]:
    obs_dim = int(len(SelfBalancingCarEnv.OBS_SCALES))
    act_dim = 2

    policy_net = model.policy.mlp_extractor.policy_net
    action_net = model.policy.action_net

    layers: list[tuple[np.ndarray, np.ndarray]] = []
    for mod in policy_net:
        if isinstance(mod, torch.nn.Linear):
            w = mod.weight.detach().cpu().numpy()
            b = mod.bias.detach().cpu().numpy()
            layers.append((w, b))
    w_out = action_net.weight.detach().cpu().numpy()
    b_out = action_net.bias.detach().cpu().numpy()
    layers.append((w_out, b_out))

    if layers[0][0].shape[1] != obs_dim:
        raise RuntimeError(
            f"Model input dim ({layers[0][0].shape[1]}) != OBS_SCALES len ({obs_dim}). "
            "Retrain the model to match the current environment observation space."
        )

    # Sanity check forward pass equivalence
    test_obs = np.linspace(-0.5, 0.5, obs_dim, dtype=np.float32)
    sb3_act, _ = model.predict(test_obs, deterministic=True)

    x = test_obs.astype(np.float32)
    for idx, (w, b) in enumerate(layers):
        x = w @ x + b
        if idx < len(layers) - 1:
            x = np.tanh(x)
    x = np.clip(x, -1.0, 1.0)
    max_diff = float(np.max(np.abs(sb3_act - x)))
    if max_diff > 1e-5:
        raise RuntimeError(f"Exported weights mismatch SB3 policy (diff={max_diff})")

    lines = [
        "// Auto-generated from trained Stable-Baselines3 PPO policy",
        f"// Verified Python forward-pass max error: {max_diff:.2e}",
        "",
        f"#define RL_OBS_DIM {obs_dim}",
        f"#define RL_ACT_DIM {act_dim}",
        format_c_array("RL_OBS_SCALES", SelfBalancingCarEnv.OBS_SCALES),
        "",
    ]

    for idx, (w, b) in enumerate(layers):
        out_dim, in_dim = w.shape
        lines.append(f"static const int LAYER{idx}_IN = {in_dim};")
        lines.append(f"static const int LAYER{idx}_OUT = {out_dim};")
        lines.append(format_c_array(f"LAYER{idx}_W", w))
        lines.append(format_c_array(f"LAYER{idx}_B", b))
        lines.append("")

    # Inference helper
    h0_out = layers[0][0].shape[0]
    h1_out = layers[1][0].shape[0] if len(layers) > 2 else h0_out
    lines.extend(
        [
            "static inline void rl_linear_tanh(const float* in, int in_dim, float* out, int out_dim, const float* w, const float* b, bool use_tanh) {",
            "  for (int i = 0; i < out_dim; ++i) {",
            "    float sum = b[i];",
            "    const float* row = w + i * in_dim;",
            "    for (int j = 0; j < in_dim; ++j) {",
            "      sum += row[j] * in[j];",
            "    }",
            "    out[i] = use_tanh ? tanhf(sum) : sum;",
            "  }",
            "}",
            "",
            "// raw_obs = {pos_err, v_fwd, pitch_rad, theta_dot, yaw_err, yaw_rate, wL, wR, cmd_v, cmd_yaw_rate, prev_aL, prev_aR}",
            "// out_action = {left_norm [-1..1], right_norm [-1..1]}",
            "static inline void rl_policy_predict(const float raw_obs[RL_OBS_DIM], float out_action[RL_ACT_DIM]) {",
            "  float obs[RL_OBS_DIM];",
            "  for (int i = 0; i < RL_OBS_DIM; ++i) {",
            "    float v = raw_obs[i] / RL_OBS_SCALES[i];",
            "    if (v > 10.0f) v = 10.0f;",
            "    if (v < -10.0f) v = -10.0f;",
            "    obs[i] = v;",
            "  }",
            f"  float h0[{h0_out}];",
            f"  float h1[{h1_out}];",
        ]
    )

    if len(layers) == 3:
        lines.extend(
            [
                "  rl_linear_tanh(obs, LAYER0_IN, h0, LAYER0_OUT, LAYER0_W, LAYER0_B, true);",
                "  rl_linear_tanh(h0, LAYER1_IN, h1, LAYER1_OUT, LAYER1_W, LAYER1_B, true);",
                "  rl_linear_tanh(h1, LAYER2_IN, out_action, LAYER2_OUT, LAYER2_W, LAYER2_B, false);",
            ]
        )
    else:
        raise NotImplementedError(f"Expected 3 linear layers (2 hidden + 1 output), got {len(layers)}")

    lines.extend(
        [
            "  for (int i = 0; i < RL_ACT_DIM; ++i) {",
            "    if (out_action[i] > 1.0f) out_action[i] = 1.0f;",
            "    if (out_action[i] < -1.0f) out_action[i] = -1.0f;",
            "  }",
            "}",
            "",
        ]
    )
    return "\n".join(lines), max_diff


def export_policy_to_header(model: PPO, out_header: Path) -> str:
    policy_c, max_diff = build_policy_c_code(model)
    header = "#pragma once\n#include <math.h>\n#include <stdbool.h>\n\n" + policy_c
    out_header.parent.mkdir(parents=True, exist_ok=True)
    out_header.write_text(header, encoding="utf-8")
    print(f"Exported ESP32 C++ header to: {out_header} (max diff={max_diff:.2e})")
    return policy_c


def build_esp32_body() -> str:
    body = """// ============================================================================
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
#define IN3     33
#define IN4     32
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
//   - IMU_PITCH_IS_X_AXIS = 0: truc quay cua xe trung truc Y tren board MPU6050 (gy, ax)
//   - IMU_PITCH_IS_X_AXIS = 1: neu ban gan board MPU6050 xoay 90 do (gx, ay)
//   - +pitch    = xe nghieng ve phia truoc
//   - +motor    = banh quay theo chieu tien
//   - +encoder  = enc tang khi banh lan tien
#define IMU_PITCH_IS_X_AXIS 0
#define IMU_PITCH_SIGN      1.0f
#define IMU_YAW_SIGN        1.0f
#define ENC_L_SIGN          1.0f
#define ENC_R_SIGN          1.0f
#define MOTOR_L_SIGN        1.0f
#define MOTOR_R_SIGN        1.0f
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
static const float WHEEL_RADIUS  = __WHEEL_RADIUS__f; // Banh 65mm -> R = 0.0325 m
static const float CONTROL_DT    = 0.02f;   // 50 Hz (20 ms)
#if ENABLE_MOVE_COMMANDS
static const float MAX_CMD_V     = __MAX_CMD_V__f;     // m/s
static const float MAX_CMD_YAW   = __MAX_CMD_YAW__f;   // rad/s
static const float MAX_ACCEL_V   = __MAX_ACCEL_V__f;   // m/s^2
static const float MAX_ACCEL_YAW = __MAX_ACCEL_YAW__f; // rad/s^2
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
  driveMotor(ENA, IN1, IN2, -MOTOR_L_SIGN * MOTOR_SELFTEST_CMD);
  driveMotor(ENB, IN3, IN4, -MOTOR_R_SIGN * MOTOR_SELFTEST_CMD);
  delay(MOTOR_SELFTEST_MS);
  driveMotor(ENA, IN1, IN2, 0.0f);
  driveMotor(ENB, IN3, IN4, 0.0f);
  delay(150);
  readEncoders(a0, b0);
  long revL = (long)(ENC_L_SIGN * (float)a0);
  long revR = (long)(ENC_R_SIGN * (float)b0);
  Serial.print("  [TEST NGUOC] dEncL="); Serial.print(revL);
  Serial.print(" | dEncR="); Serial.println(revR);

  // Nghi ngan bao ve cau H L298N truoc khi dao chieu
  delay(150);

  // 2. Quay XUOI
  resetEncoders();
  driveMotor(ENA, IN1, IN2, MOTOR_L_SIGN * MOTOR_SELFTEST_CMD);
  driveMotor(ENB, IN3, IN4, MOTOR_R_SIGN * MOTOR_SELFTEST_CMD);
  delay(MOTOR_SELFTEST_MS);
  driveMotor(ENA, IN1, IN2, 0.0f);
  driveMotor(ENB, IN3, IN4, 0.0f);
  delay(250); // Cho xe & banh dung han truoc khi calib MPU6050 5s
  readEncoders(a1, b1);
  long fwdL = (long)(ENC_L_SIGN * (float)a1);
  long fwdR = (long)(ENC_R_SIGN * (float)b1);
  Serial.print("  [TEST XUOI]  dEncL="); Serial.print(fwdL);
  Serial.print(" | dEncR="); Serial.println(fwdR);

  if (revL == 0 && fwdL == 0) {
    Serial.println("  [CANH BAO] Khong thay xung encoder Trai (Kiem tra nguon 3.3V/chan 16-17 hoac day motor L)!");
  } else if (fwdL < 0 && revL > 0) {
    Serial.println("  [GOI Y] Encoder Trai nguoc chieu quay -> doi ENC_L_SIGN thanh -1.0f (hoac dao day motor A).");
  }
  if (revR == 0 && fwdR == 0) {
    Serial.println("  [CANH BAO] Khong thay xung encoder Phai (Kiem tra nguon 3.3V/chan 18-19 hoac day motor R)!");
  } else if (fwdR < 0 && revR > 0) {
    Serial.println("  [GOI Y] Encoder Phai nguoc chieu quay -> doi ENC_R_SIGN thanh -1.0f (hoac dao day motor B).");
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
#if IMU_PITCH_IS_X_AXIS
  return IMU_PITCH_SIGN * atan2f((float)ay, (float)az);
#else
  return IMU_PITCH_SIGN * atan2f(-(float)ax, (float)az);
#endif
}

static int16_t selectPitchGyroRaw(int16_t gx, int16_t gy) {
#if IMU_PITCH_IS_X_AXIS
  return gx;
#else
  return gy;
#endif
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

  // Hieu chuan MPU6050 trong 5 giay dau tien (tat motor, giu xe dung yen tai diem can bang)
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
    lastPitchAccel = computeRawAccelPitch(ax, ay, az) - pitchZeroOffset;
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

void setup() {
  Serial.begin(115200);

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

  // 3. Test dong co quay nguoc & quay xuoi truoc, sau do tat dong co de calib 5s
  runStartupMotorSelfTest();

  // 4. Khoi tao & calib MPU6050 trong 5 giay (luc nay dong co da tat hoan toan)
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

  unsigned long nowUs = micros();
  if (nowUs - lastStepUs < (unsigned long)(CONTROL_DT * 1e6f)) return;
  float dt = (nowUs - lastStepUs) * 1e-6f;
  if (dt > 0.05f || dt <= 0.0f) dt = CONTROL_DT;
  lastStepUs = nowUs;

  // Doc MPU6050 & cap nhat bo loc bu (Complementary Filter)
  int16_t ax, ay, az, gx, gy, gz;
  if (readMpuRaw(ax, ay, az, gx, gy, gz)) {
    i2cFailCount = 0;
    const float degToRad = 3.14159265f / 180.0f;
    float gpRaw = (float)selectPitchGyroRaw(gx, gy);
    lastPitchRate = IMU_PITCH_SIGN * ((gpRaw - gyroPitchOffset) / 65.5f) * degToRad;
    lastYawRate   = IMU_YAW_SIGN   * (((float)gz - gyroZOffset) / 65.5f) * degToRad;
    if (ax != 0 || ay != 0 || az != 0) {
      lastPitchAccel = computeRawAccelPitch(ax, ay, az) - pitchZeroOffset;
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
  float wL = ENC_L_SIGN * ((float)(encA - prevEncA) * radPerTick) / dt;
  float wR = ENC_R_SIGN * ((float)(encB - prevEncB) * radPerTick) / dt;
  prevEncA = encA;
  prevEncB = encB;

  // Bu van toc nghieng than xe de lay dung van toc tinh tien mat dat
  float vFwd = (0.5f * (wL + wR) + pitchRate) * WHEEL_RADIUS;

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

  if (safetyTripped) {
    driveMotor(ENA, IN1, IN2, 0.0f);
    driveMotor(ENB, IN3, IN4, 0.0f);
    resetControllerState(encA, encB);
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

  driveMotor(ENA, IN1, IN2, MOTOR_L_SIGN * action[0]);
  driveMotor(ENB, IN3, IN4, MOTOR_R_SIGN * action[1]);
  prevActL = action[0];
  prevActR = action[1];

  maybeLogTelemetry(encA, encB, wL, wR, vFwd, action[0], action[1]);
}
"""
    replacements = {
        "__WHEEL_RADIUS__": f"{SelfBalancingCarEnv.WHEEL_RADIUS:.4f}",
        "__MAX_CMD_V__": f"{SelfBalancingCarEnv.MAX_CMD_V:.2f}",
        "__MAX_CMD_YAW__": f"{SelfBalancingCarEnv.MAX_CMD_YAW_RATE:.2f}",
        "__MAX_ACCEL_V__": f"{SelfBalancingCarEnv.MAX_ACCEL_V:.2f}",
        "__MAX_ACCEL_YAW__": f"{SelfBalancingCarEnv.MAX_ACCEL_YAW:.2f}",
    }
    for token, val in replacements.items():
        body = body.replace(token, val)
    return body


def write_esp32_sketch(out_cpp: Path) -> None:
    preamble = "\n".join(
        [
            "#include <Arduino.h>",
            "#include <Wire.h>",
            "#include <ctype.h>",
            "#include <math.h>",
            '#include "rl_policy.h"',
            "",
        ]
    )
    out_cpp.parent.mkdir(parents=True, exist_ok=True)
    out_cpp.write_text(preamble + build_esp32_body(), encoding="utf-8")
    print(f"Wrote ESP32 RL firmware to: {out_cpp}")


def write_single_ino(out_ino: Path, policy_c_code: str) -> None:
    preamble = "\n".join(
        [
            "#include <Arduino.h>",
            "#include <Wire.h>",
            "#include <ctype.h>",
            "#include <math.h>",
            "#include <stdbool.h>",
            "",
        ]
    )
    out_ino.parent.mkdir(parents=True, exist_ok=True)
    out_ino.write_text(
        preamble + policy_c_code + "\n" + build_esp32_body(), encoding="utf-8"
    )
    print(f"Wrote single-file Arduino IDE sketch to: {out_ino}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Export trained PPO policy to ESP32 C++ / Arduino IDE.")
    parser.add_argument(
        "--model",
        type=Path,
        default=Path("runs/ppo_xe_can_bang/best_model/best_model.zip"),
        help="Path to trained PPO .zip model.",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("esp32_deploy"),
        help="Output directory for rl_policy.h, main_rl.cpp, and Arduino .ino sketch.",
    )
    args = parser.parse_args()

    model = PPO.load(str(args.model), device="cpu")
    policy_c = export_policy_to_header(model, args.out_dir / "rl_policy.h")
    write_esp32_sketch(args.out_dir / "main_rl.cpp")
    write_single_ino(
        args.out_dir / "xe_can_bang_rl_esp32" / "xe_can_bang_rl_esp32.ino", policy_c
    )


if __name__ == "__main__":
    main()
