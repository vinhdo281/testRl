"""Kiem tra policy trong header tren MuJoCo (khong can ESP): co can bang? co bam lenh?"""
import sys, math
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import numpy as np
from envs.balance_car_env import SelfBalancingCarEnv
from policy_np import NpPolicy

pol = NpPolicy()
S = SelfBalancingCarEnv.OBS_SCALES


def run(cmd_v, cmd_w, seconds=20, push=False, seed=1):
    env = SelfBalancingCarEnv(domain_rand=False, command_mode="manual", max_episode_steps=10**6)
    obs, _ = env.reset(seed=seed)
    env.set_command(cmd_v, cmd_w)
    n = int(seconds / env.dt)
    vs, falls, maxp = [], 0, 0
    for i in range(n):
        if push and i % 200 == 100:
            env.apply_external_push(2.0 * (1 if (i // 200) % 2 == 0 else -1), 0)
        a = pol.predict_raw(obs * S)
        obs, _, term, trunc, info = env.step(a)
        vs.append((info["v_fwd"], info["yaw_rate"]))
        maxp = max(maxp, abs(info["pitch_deg"]))
        if term:
            falls += 1
            break
    vs = np.array(vs[len(vs) // 2:])
    return dict(cmd=(cmd_v, cmd_w), fell=bool(falls), steps=i + 1, max_pitch=round(maxp, 1),
                v_mean=round(float(vs[:, 0].mean()), 3), yaw_mean=round(float(vs[:, 1].mean()), 3),
                x=round(info["x_pos"], 2), y=round(info["y_pos"], 2))


for c in [(0, 0), (0.4, 0), (-0.4, 0), (0, 1.5), (0.32, 1.2)]:
    print(run(*c, seconds=10))
print("push:", run(0, 0, seconds=20, push=True))
