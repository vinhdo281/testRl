from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

import matplotlib.pyplot as plt
import mujoco
import mujoco.viewer
import numpy as np
from stable_baselines3 import PPO

from envs import SelfBalancingCarEnv

if sys.platform == "win32":
    import msvcrt
else:
    msvcrt = None  # type: ignore[assignment]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Evaluate trained PPO balancing + locomotion policy in MuJoCo."
    )
    parser.add_argument(
        "--model",
        type=Path,
        default=Path("runs/ppo_xe_can_bang/best_model/best_model.zip"),
        help="Path to trained PPO .zip model.",
    )
    parser.add_argument(
        "--episodes",
        type=int,
        default=5,
        help="Number of evaluation episodes in headless mode.",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run headless benchmark and generate eval_plot.png instead of opening 3D viewer.",
    )
    parser.add_argument(
        "--push-test",
        action="store_true",
        help="Apply periodic external push disturbances to test recovery.",
    )
    parser.add_argument(
        "--demo-move",
        action="store_true",
        help="In 3D viewer, run automatic movement choreography (forward/back/turn/stop).",
    )
    parser.add_argument(
        "--stationary",
        action="store_true",
        help="In headless mode, test stationary balancing (0, 0) instead of demo commands.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=123,
        help="Random seed for evaluation.",
    )
    parser.add_argument(
        "--plot-path",
        type=Path,
        default=Path("runs/ppo_xe_can_bang/eval_plot.png"),
        help="Where to save diagnostic plots in headless mode.",
    )
    return parser.parse_args()


def run_headless(
    model: PPO,
    episodes: int,
    seed: int,
    push_test: bool,
    stationary: bool,
    plot_path: Path,
) -> None:
    cmd_mode = "manual" if stationary else "demo"
    env = SelfBalancingCarEnv(
        domain_rand=False, command_mode=cmd_mode, max_episode_steps=1000
    )
    if stationary:
        env.set_command(0.0, 0.0)

    ep_rewards = []
    ep_lengths = []
    rms_pitches = []
    max_pitches = []
    vel_rmses = []
    yaw_rmses = []
    traj = None

    for ep in range(episodes):
        obs, _ = env.reset(seed=seed + ep)
        if stationary:
            env.set_command(0.0, 0.0)
        done = False
        total_r = 0.0
        steps = 0
        pitches, vs, cmd_vs, yaws_r, cmd_yaws_r, xs, ys, actions = (
            [],
            [],
            [],
            [],
            [],
            [],
            [],
            [],
        )

        while not done:
            if push_test and steps in (100, 380, 680, 920):
                fx = 2.0 if (steps // 200) % 2 == 0 else -2.0
                env.apply_external_push(fx=fx, fy=0.2)

            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            done = terminated or truncated
            total_r += reward
            steps += 1
            pitches.append(info["pitch_deg"])
            vs.append(info["v_fwd"])
            cmd_vs.append(info["cmd_v"])
            yaws_r.append(info["yaw_rate"])
            cmd_yaws_r.append(info["cmd_yaw_rate"])
            xs.append(info["x_pos"])
            ys.append(info["y_pos"])
            actions.append(action.copy())

        ep_rewards.append(total_r)
        ep_lengths.append(steps)
        p_arr = np.array(pitches)
        v_arr = np.array(vs)
        cv_arr = np.array(cmd_vs)
        yr_arr = np.array(yaws_r)
        cyr_arr = np.array(cmd_yaws_r)
        rms_pitches.append(float(np.sqrt(np.mean(p_arr ** 2))))
        max_pitches.append(float(np.max(np.abs(p_arr))))
        vel_rmses.append(float(np.sqrt(np.mean((v_arr - cv_arr) ** 2))))
        yaw_rmses.append(float(np.sqrt(np.mean((yr_arr - cyr_arr) ** 2))))

        if ep == 0:
            traj = (
                np.arange(steps) * env.dt,
                p_arr,
                v_arr,
                cv_arr,
                yr_arr,
                cyr_arr,
                np.array(actions),
            )

    env.close()

    print(
        f"\n=== Evaluation Summary ({episodes} episodes, mode={cmd_mode}, push_test={push_test}) ==="
    )
    print(f"  Mean Reward   : {np.mean(ep_rewards):.2f} +/- {np.std(ep_rewards):.2f}")
    print(
        f"  Mean Length   : {np.mean(ep_lengths):.1f} / 1000 steps ({np.mean(ep_lengths)*env.dt:.1f} s)"
    )
    print(f"  RMS Pitch     : {np.mean(rms_pitches):.2f} deg")
    print(f"  Max |Pitch|   : {np.mean(max_pitches):.2f} deg")
    print(f"  Vel Track RMSE: {np.mean(vel_rmses):.3f} m/s")
    print(f"  Yaw Track RMSE: {np.mean(yaw_rmses):.3f} rad/s")

    if traj is not None:
        t, p_arr, v_arr, cv_arr, yr_arr, cyr_arr, act_arr = traj
        plot_path.parent.mkdir(parents=True, exist_ok=True)
        fig, axes = plt.subplots(4, 1, figsize=(10, 9), sharex=True)

        axes[0].plot(t, p_arr, label="Pitch (deg)", color="tab:blue")
        axes[0].axhline(0.0, color="gray", linestyle="--", linewidth=0.8)
        axes[0].set_ylabel("Pitch (deg)")
        axes[0].set_title(
            f"Self-Balancing Robot RL Evaluation ({cmd_mode}, push={push_test})"
        )
        axes[0].grid(True, alpha=0.3)

        axes[1].plot(t, cv_arr, label="Target v_cmd (m/s)", color="black", linestyle="--")
        axes[1].plot(t, v_arr, label="Actual v_fwd (m/s)", color="tab:green")
        axes[1].set_ylabel("Vel (m/s)")
        axes[1].legend(loc="upper right")
        axes[1].grid(True, alpha=0.3)

        axes[2].plot(t, cyr_arr, label="Target yaw_cmd (rad/s)", color="black", linestyle="--")
        axes[2].plot(t, yr_arr, label="Actual yaw_rate (rad/s)", color="tab:orange")
        axes[2].set_ylabel("Yaw Rate (rad/s)")
        axes[2].legend(loc="upper right")
        axes[2].grid(True, alpha=0.3)

        axes[3].plot(t, act_arr[:, 0], label="Left Motor PWM", alpha=0.85)
        axes[3].plot(t, act_arr[:, 1], label="Right Motor PWM", alpha=0.85, linestyle="--")
        axes[3].set_ylabel("Norm. Action")
        axes[3].set_xlabel("Time (s)")
        axes[3].legend(loc="upper right")
        axes[3].grid(True, alpha=0.3)

        fig.tight_layout()
        fig.savefig(str(plot_path), dpi=150)
        plt.close(fig)
        print(f"  Saved plot    : {plot_path}")


def run_viewer(model: PPO, seed: int, push_test: bool, demo_move: bool) -> None:
    cmd_mode = "demo" if demo_move else "manual"
    env = SelfBalancingCarEnv(
        domain_rand=False, command_mode=cmd_mode, max_episode_steps=100_000
    )
    obs, _ = env.reset(seed=seed)

    lock = threading.Lock()
    state = {"cmd_v": 0.0, "cmd_yaw": 0.0, "push": 0.0, "push_sign": 1.0}

    def apply_key_action(ch: str) -> None:
        ch = ch.upper()
        with lock:
            if ch in ("W", "I", "UP"):
                state["cmd_v"] = 0.24
                state["cmd_yaw"] = 0.0
            elif ch in ("S", "K", "DOWN"):
                state["cmd_v"] = -0.24
                state["cmd_yaw"] = 0.0
            elif ch in ("A", "J", "LEFT"):
                state["cmd_yaw"] = 1.20
            elif ch in ("D", "L", "RIGHT"):
                state["cmd_yaw"] = -1.20
            elif ch in ("X", "0"):
                state["cmd_v"] = 0.0
                state["cmd_yaw"] = 0.0
            elif ch == "P":
                state["push"] = 2.0 * state["push_sign"]
                state["push_sign"] *= -1.0
                print(f"[USER] External push fx={state['push']:+.1f} N")
                return
            else:
                return
            print(
                f"[CMD] v_cmd={state['cmd_v']:+.2f} m/s | yaw_cmd={state['cmd_yaw']:+.2f} rad/s"
            )

    def glfw_key_callback(keycode: int) -> None:
        mapping = {
            265: "UP",
            264: "DOWN",
            263: "LEFT",
            262: "RIGHT",
            ord("I"): "I",
            ord("K"): "K",
            ord("J"): "J",
            ord("L"): "L",
            ord("X"): "X",
            ord("0"): "0",
            ord("P"): "P",
        }
        if keycode in mapping:
            apply_key_action(mapping[keycode])

    print("Launching MuJoCo 3D Viewer... Close window to exit.")
    if demo_move:
        print("Demo choreography enabled: Stop -> Forward -> Stop -> Turn -> Backward -> Spin -> Stop.")
    else:
        print("Dieu khien trong cua so 3D Viewer : Mui ten (Up/Down/Left/Right) hoac I/K/J/L, X=Dung, P=Day thu")
        print("Dieu khien trong cua so Terminal  : W (Tien), S (Lui), A (Trai), D (Phai), X (Dung), P (Day thu)")
    if push_test:
        print("Push test enabled: applying periodic nudges every 4 seconds.")

    prev_x = float(env.data.qpos[0])
    prev_y = float(env.data.qpos[1])

    with mujoco.viewer.launch_passive(
        env.model, env.data, key_callback=glfw_key_callback
    ) as viewer:
        step_idx = 0
        while viewer.is_running():
            t_start = time.perf_counter()

            # Non-blocking terminal keys on Windows (W/S/A/D/X/P)
            if not demo_move and msvcrt is not None:
                while msvcrt.kbhit():
                    raw = msvcrt.getch()
                    if raw in (b"\x00", b"\xe0"):
                        arrow = msvcrt.getch()
                        arrow_map = {b"H": "UP", b"P": "DOWN", b"K": "LEFT", b"M": "RIGHT"}
                        if arrow in arrow_map:
                            apply_key_action(arrow_map[arrow])
                    else:
                        try:
                            apply_key_action(raw.decode("ascii", errors="ignore"))
                        except Exception:
                            pass

            if not demo_move:
                with lock:
                    env.set_command(state["cmd_v"], state["cmd_yaw"])
                    if state["push"] != 0.0:
                        env.apply_external_push(fx=state["push"], fy=0.0)
                        state["push"] = 0.0

            if push_test and step_idx > 0 and step_idx % 200 == 0:
                fx = 2.0 if (step_idx // 200) % 2 == 0 else -2.0
                print(f"[t={step_idx * env.dt:.1f}s] Applying external push fx={fx:+.1f} N")
                env.apply_external_push(fx=fx, fy=0.0)

            action, _ = model.predict(obs, deterministic=True)
            obs, _, terminated, truncated, info = env.step(action)
            step_idx += 1

            # Follow car gently without locking user orbit/zoom
            cur_x = float(env.data.qpos[0])
            cur_y = float(env.data.qpos[1])
            viewer.cam.lookat[0] += cur_x - prev_x
            viewer.cam.lookat[1] += cur_y - prev_y
            prev_x, prev_y = cur_x, cur_y

            viewer.sync()

            if terminated or truncated:
                print(
                    f"Episode ended at step {step_idx} "
                    f"(pitch={info['pitch_deg']:.1f} deg) -> Resetting."
                )
                obs, _ = env.reset(seed=seed + step_idx)
                prev_x = float(env.data.qpos[0])
                prev_y = float(env.data.qpos[1])
                step_idx = 0

            elapsed = time.perf_counter() - t_start
            if elapsed < env.dt:
                time.sleep(env.dt - elapsed)

    env.close()


def main() -> None:
    args = parse_args()
    if not args.model.exists():
        raise FileNotFoundError(
            f"Model not found at {args.model}. Run train.py first!"
        )
    model = PPO.load(str(args.model), device="cpu")
    if args.headless:
        run_headless(
            model,
            args.episodes,
            args.seed,
            args.push_test,
            args.stationary,
            args.plot_path,
        )
    else:
        run_viewer(model, args.seed, args.push_test, args.demo_move)


if __name__ == "__main__":
    main()
