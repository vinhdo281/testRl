// ESP32 Processor-in-the-Loop cho xe can bang (MuJoCo <-> ESP32).
// KHONG doc cam bien that, KHONG khoi tao I2C/encoder, dong co luon tat.
//
// Hop dong du lieu (little-endian):
//   PC -> ESP : A5 5A | seq u8 | 12 x float32 RAW obs (gia tri vat ly, CHUA chia scale) | crc16 u16
//   ESP -> PC : 5A A5 | seq u8 | 2 x float32 action [-1..1] | infer_us u32 | crc16 u16
//   crc16 = CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) tinh tren cac byte sau 2 byte header.
// ESP goi nguyen rl_policy_predict() (tu chia OBS_SCALES ben trong) nen PC phai gui obs * OBS_SCALES.
#include <Arduino.h>
#include <string.h>
#include "rl_policy.h"

static const uint32_t BAUD = 921600;
static const int REQ_PAYLOAD = 1 + 12 * 4;      // seq + obs
static const int REP_PAYLOAD = 1 + 2 * 4 + 4;   // seq + action + infer_us

static uint16_t crc16(const uint8_t *d, size_t n) {
  uint16_t c = 0xFFFF;
  while (n--) {
    c ^= (uint16_t)(*d++) << 8;
    for (int i = 0; i < 8; ++i) c = (c & 0x8000) ? (c << 1) ^ 0x1021 : (c << 1);
  }
  return c;
}

void setup() {
  // Dong co tat truoc het (ENA,IN1,IN2,ENB,IN3,IN4)
  const int pins[] = {14, 27, 26, 25, 33, 32};
  for (int p : pins) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }

  Serial.setRxBufferSize(1024);
  Serial.begin(BAUD);
  delay(300);
  Serial.println();
  Serial.println("READY HIL v1 obs=12 act=2");
}

void loop() {
  static uint8_t buf[2 + REQ_PAYLOAD + 2];
  static int n = 0;

  while (Serial.available()) {
    uint8_t b = (uint8_t)Serial.read();
    if (n == 0) { if (b == 0xA5) buf[n++] = b; continue; }
    if (n == 1) { if (b == 0x5A) buf[n++] = b; else n = (b == 0xA5) ? 1 : 0; continue; }
    buf[n++] = b;
    if (n < (int)sizeof(buf)) continue;
    n = 0;

    uint16_t rx = buf[2 + REQ_PAYLOAD] | ((uint16_t)buf[2 + REQ_PAYLOAD + 1] << 8);
    if (crc16(buf + 2, REQ_PAYLOAD) != rx) continue;   // CRC sai: bo, PC se timeout & gui lai

    uint8_t seq = buf[2];
    float obs[RL_OBS_DIM], act[RL_ACT_DIM];
    memcpy(obs, buf + 3, sizeof(obs));

    uint32_t t0 = micros();
    rl_policy_predict(obs, act);
    uint32_t dtUs = micros() - t0;

    uint8_t out[2 + REP_PAYLOAD + 2];
    out[0] = 0x5A; out[1] = 0xA5; out[2] = seq;
    memcpy(out + 3, act, 8);
    memcpy(out + 11, &dtUs, 4);
    uint16_t c = crc16(out + 2, REP_PAYLOAD);
    out[2 + REP_PAYLOAD] = c & 0xFF;
    out[2 + REP_PAYLOAD + 1] = c >> 8;
    Serial.write(out, sizeof(out));
  }
}
