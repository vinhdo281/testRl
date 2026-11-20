from __future__ import annotations

import argparse
from pathlib import Path

import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv

from envs import SelfBalancingCarEnv


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Train PPO RL agent to balance the 2-wheeled robot in MuJoCo."
    )
    parser.add_argument(
        "--total-timesteps",
        type=int,
        default=400_000,
        help="Total environment timesteps to train (default: 400000).",
    )
    parser.add_argument(
        "--n-envs",
        type=int,
        default=8,
        help="Number of parallel environments (default: 8).",
    )
    parser.add_argument(
        "--use-subproc",
        action="store_true",
        help="Use SubprocVecEnv instead of DummyVecEnv.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Random seed (default: 42).",
    )
    parser.add_argument(
        "--save-dir",
        type=Path,
        default=Path("runs/ppo_xe_can_bang"),
        help="Directory to save models and TensorBoard logs.",
    )
    parser.add_argument(
        "--resume",
        type=Path,
        default=None,
        help="Optional path to an existing .zip model to resume training from.",
    )
    parser.add_argument(
        "--no-domain-rand",
        action="store_true",
        help="Disable domain randomization during training.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    torch.set_num_threads(2)
    save_dir: Path = args.save_dir
    best_dir = save_dir / "best_model"
    ckpt_dir = save_dir / "checkpoints"
    tb_dir = save_dir / "tb_logs"
    best_dir.mkdir(parents=True, exist_ok=True)
    ckpt_dir.mkdir(parents=True, exist_ok=True)
    tb_dir.mkdir(parents=True, exist_ok=True)

    vec_cls = SubprocVecEnv if args.use_subproc else DummyVecEnv
    domain_rand = not args.no_domain_rand

    env = make_vec_env(
        SelfBalancingCarEnv,
        n_envs=args.n_envs,
        seed=args.seed,
        vec_env_cls=vec_cls,
        env_kwargs={
            "domain_rand": domain_rand,
            "command_mode": "random",
            "max_episode_steps": 1000,
        },
    )

    eval_env = make_vec_env(
        SelfBalancingCarEnv,
        n_envs=2,
        seed=args.seed + 1000,
        vec_env_cls=DummyVecEnv,
        env_kwargs={
            "domain_rand": False,
            "command_mode": "demo",
            "max_episode_steps": 1000,
        },
    )

    eval_freq = max(15_000 // args.n_envs, 500)
    save_freq = max(50_000 // args.n_envs, 2000)

    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=str(best_dir),
        log_path=str(save_dir / "eval_logs"),
        eval_freq=eval_freq,
        n_eval_episodes=5,
        deterministic=True,
        render=False,
    )
    ckpt_callback = CheckpointCallback(
        save_freq=save_freq,
        save_path=str(ckpt_dir),
        name_prefix="ppo_car",
    )

    if args.resume is not None:
        print(f"Resuming training from: {args.resume}")
        model = PPO.load(str(args.resume), env=env, tensorboard_log=str(tb_dir))
    else:
        policy_kwargs = dict(
            activation_fn=torch.nn.Tanh,
            net_arch=dict(pi=[128, 128], vf=[128, 128]),
            log_std_init=-0.5,
        )
        model = PPO(
            policy="MlpPolicy",
            env=env,
            learning_rate=3e-4,
            n_steps=1024,
            batch_size=256,
            n_epochs=10,
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
            ent_coef=0.0005,
            vf_coef=0.5,
            max_grad_norm=0.5,
            policy_kwargs=policy_kwargs,
            tensorboard_log=str(tb_dir),
            seed=args.seed,
            verbose=1,
            device="cpu",
        )

    print(
        f"Training PPO for {args.total_timesteps:,} timesteps "
        f"({args.n_envs} envs, domain_rand={domain_rand})..."
    )
    model.learn(
        total_timesteps=args.total_timesteps,
        callback=[eval_callback, ckpt_callback],
        progress_bar=False,
    )

    final_path = save_dir / "final_model.zip"
    model.save(str(final_path))
    print(f"Training complete! Final model saved to: {final_path}")
    print(f"Best model saved to: {best_dir / 'best_model.zip'}")

    env.close()
    eval_env.close()


if __name__ == "__main__":
    main()
