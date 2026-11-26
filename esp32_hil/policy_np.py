"""Doc rl_policy.h (trong esp32_deploy) va chay lai mang MLP bang numpy.
Dung de doi chieu voi ket qua ESP32 (HIL) va de kiem tra policy tren PC."""
import re
from pathlib import Path
import numpy as np

HEADER = Path(__file__).resolve().parent.parent / "esp32_deploy" / "rl_policy.h"


def _arr(text, name):
    m = re.search(rf"{name}\[\d+\]\s*=\s*\{{(.*?)\}};", text, re.S)
    return np.array([float(x.strip().rstrip("f")) for x in m.group(1).replace("\n", " ").split(",") if x.strip()],
                    dtype=np.float32)


class NpPolicy:
    def __init__(self, header=HEADER):
        t = Path(header).read_text()
        self.scales = _arr(t, "RL_OBS_SCALES")
        self.w0 = _arr(t, "LAYER0_W").reshape(128, 12)
        self.b0 = _arr(t, "LAYER0_B")
        self.w1 = _arr(t, "LAYER1_W").reshape(128, 128)
        self.b1 = _arr(t, "LAYER1_B")
        self.w2 = _arr(t, "LAYER2_W").reshape(2, 128)
        self.b2 = _arr(t, "LAYER2_B")

    def predict_raw(self, raw_obs):
        """raw_obs = gia tri vat ly (chua chia scale), giong rl_policy_predict() tren ESP."""
        o = np.clip(np.asarray(raw_obs, np.float32) / self.scales, -10, 10)
        h = np.tanh(self.w0 @ o + self.b0)
        h = np.tanh(self.w1 @ h + self.b1)
        return np.clip(self.w2 @ h + self.b2, -1, 1).astype(np.float32)
