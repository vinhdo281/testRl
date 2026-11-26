from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import gymnasium as gym
import mujoco
import numpy as np
from gymnasium import spaces


DEFAULT_XML_PATH = Path(__file__).resolve().parent.parent / "assets" / "xe_can_bang.xml"


class SelfBalancingCarEnv(gym.Env):
    """Gymnasium environment for a 2-wheeled self-balancing robot in MuJoCo.

    Supports simultaneous upright balancing and slew-rate-limited velocity/steering
    commands (cmd_v, cmd_yaw_rate) for forward, backward, turning, and stopping.
    Hardware & geometry match the SolidWorks assembly in `xe can băng` and `main.cpp`:
    - 65mm diameter wheels (radius = 0.0325 m), track width = 0.156 m
    - Dual DC geared motors with encoders + L298N driver
    - ESP32 + MPU6050 IMU
    """

    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 50}

    WHEEL_RADIUS = 0.0325
    TRACK_WIDTH = 0.156
    # GA25-370 130 RPM (~13.61 rad/s -> ~0.442 m/s no-load): keep cruising headroom for pitch recovery
    MAX_CMD_V = 0.28         # m/s
    MAX_CMD_YAW_RATE = 1.40  # rad/s
    MAX_ACCEL_V = 0.45       # m/s^2 (0.009 m/s per 20ms step)
    MAX_ACCEL_YAW = 3.00     # rad/s^2 (0.060 rad/s per 20ms step)

    # Fixed normalization scales for 12D observation (O(1), ESP32-deployable)
    OBS_SCALES = np.array(
        [
            0.6,    # pos_err: local position tracking/hold error (m)
            1.5,    # v_fwd: forward linear velocity (m/s)
            0.8,    # pitch: body tilt angle (rad)
            10.0,   # theta_dot: body pitch angular velocity (rad/s)
            1.5,    # yaw_err: local heading tracking/hold error (rad)
            5.0,    # yaw_rate: body yaw angular velocity (rad/s)
            16.0,   # w_l: left wheel angular velocity (rad/s, 130 RPM ~= 13.6 rad/s)
            16.0,   # w_r: right wheel angular velocity (rad/s, 130 RPM ~= 13.6 rad/s)
            1.0,    # cmd_v: active smoothed forward velocity command (m/s)
            2.5,    # cmd_yaw_rate: active smoothed yaw angular rate command (rad/s)
            1.0,    # prev_action L
            1.0,    # prev_action R
        ],
        dtype=np.float32,
    )

    def __init__(
        self,
        xml_path: str | Path = DEFAULT_XML_PATH,
        render_mode: str | None = None,
        max_episode_steps: int = 1000,
        frame_skip: int = 10,
        domain_rand: bool = True,
        command_mode: str = "random",
        max_tilt_deg: float = 45.0,
        max_pos_m: float | None = None,
    ) -> None:
        super().__init__()
        if command_mode not in ("random", "demo", "manual"):
            raise ValueError(f"Unsupported command_mode={command_mode!r}")

        self.xml_path = str(xml_path)
        self.render_mode = render_mode
        self.max_episode_steps = max_episode_steps
        self.frame_skip = frame_skip
        self.domain_rand = domain_rand
        self.command_mode = command_mode
        self.max_tilt_rad = math.radians(max_tilt_deg)
        self.max_pos_m = max_pos_m

        self.model = mujoco.MjModel.from_xml_path(self.xml_path)
        self.data = mujoco.MjData(self.model)

        self.dt = float(self.model.opt.timestep) * self.frame_skip

        # Nominal physical parameters (perturbed per episode if domain_rand=True)
        self._base_chassis_mass = float(self.model.body_mass[1])
        self._base_chassis_inertia = self.model.body_inertia[1].copy()
        self._base_chassis_ipos = self.model.body_ipos[1].copy()
        self._base_floor_friction = self.model.geom_friction[0].copy()

        self.motor_torque_max = 0.42  # N*m per motor at 100% PWM (GA25-370 130 RPM + L298N)
        self.motor_back_emf = 0.031   # N*m / (rad/s) (~13.55 rad/s ~= 129.4 RPM no-load)
        self.motor_deadzone = 0.02    # small L298N deadzone

        # Action: normalized PWM [-1, 1] for [left_motor, right_motor]
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(2,), dtype=np.float32
        )

        # Observation: 12D normalized state + commands
        self.observation_space = spaces.Box(
            low=-10.0, high=10.0, shape=(12,), dtype=np.float32
        )

        self._step_count = 0
        self._prev_action = np.zeros(2, dtype=np.float32)
        self._cur_motor_gain = np.array([1.0, 1.0], dtype=np.float64)
        self._cur_pitch_bias = 0.0
        self._pending_push = np.zeros(2, dtype=np.float64)
        self._target_v = 0.0
        self._target_yaw_rate = 0.0
        self._cmd_v = 0.0
        self._cmd_yaw_rate = 0.0
        self._next_cmd_step = 0
        self._pos_err = 0.0
        self._yaw_err = 0.0
        self._renderer: mujoco.Renderer | None = None

        # Pre-allocate rotation matrix buffer
        self._rot_mat = np.zeros(9, dtype=np.float64)

    def set_command(self, v_fwd: float = 0.0, yaw_rate: float = 0.0) -> None:
        """Set desired target linear velocity (m/s) and yaw angular rate (rad/s)."""
        self._target_v = float(np.clip(v_fwd, -self.MAX_CMD_V, self.MAX_CMD_V))
        self._target_yaw_rate = float(
            np.clip(yaw_rate, -self.MAX_CMD_YAW_RATE, self.MAX_CMD_YAW_RATE)
        )

    def _sample_random_command(self) -> tuple[float, float]:
        p = float(self.np_random.random())
        if p < 0.22:
            return 0.0, 0.0
        if p < 0.50:
            return float(self.np_random.uniform(-self.MAX_CMD_V, self.MAX_CMD_V)), 0.0
        if p < 0.72:
            return 0.0, float(
                self.np_random.uniform(-self.MAX_CMD_YAW_RATE, self.MAX_CMD_YAW_RATE)
            )
        return (
            float(self.np_random.uniform(-0.9 * self.MAX_CMD_V, 0.9 * self.MAX_CMD_V)),
            float(
                self.np_random.uniform(
                    -0.85 * self.MAX_CMD_YAW_RATE, 0.85 * self.MAX_CMD_YAW_RATE
                )
            ),
        )

    @staticmethod
    def _demo_command_at_step(step_idx: int) -> tuple[float, float]:
        s = step_idx % 1000
        if s < 120:
            return 0.0, 0.0
        if s < 320:
            return 0.24, 0.0
        if s < 430:
            return 0.0, 0.0
        if s < 610:
            return 0.20, 1.00
        if s < 780:
            return -0.22, 0.0
        if s < 910:
            return 0.0, -1.20
        return 0.0, 0.0

    @staticmethod
    def _slew_toward(cur: float, target: float, max_delta: float) -> float:
        diff = target - cur
        if diff > max_delta:
            return cur + max_delta
        if diff < -max_delta:
            return cur - max_delta
        return target

    def _update_command_schedule(self) -> None:
        if self.command_mode == "demo":
            self._target_v, self._target_yaw_rate = self._demo_command_at_step(
                self._step_count
            )
        elif self.command_mode == "random" and self._step_count >= self._next_cmd_step:
            self._target_v, self._target_yaw_rate = self._sample_random_command()
            self._next_cmd_step = self._step_count + int(
                self.np_random.integers(150, 260)
            )

        self._cmd_v = self._slew_toward(
            self._cmd_v, self._target_v, self.MAX_ACCEL_V * self.dt
        )
        self._cmd_yaw_rate = self._slew_toward(
            self._cmd_yaw_rate, self._target_yaw_rate, self.MAX_ACCEL_YAW * self.dt
        )

    def _get_pitch_roll_yaw(self) -> tuple[float, float, float]:
        """Compute body-frame (pitch, roll) and world yaw from chassis quaternion."""
        mujoco.mju_quat2Mat(self._rot_mat, self.data.qpos[3:7])
        R = self._rot_mat.reshape(3, 3)
        fwd = R[:, 0]
        left = R[:, 1]
        up = R[:, 2]

        # World +Z expressed in body frame is [fwd[2], left[2], up[2]], independent of yaw.
        # Positive pitch = leaning forward along body +fwd, matching +Y wheel torque direction.
        pitch = math.atan2(-fwd[2], up[2])
        roll = math.atan2(left[2], up[2])
        yaw = math.atan2(fwd[1], fwd[0])
        return pitch, roll, yaw

    def _get_obs(self, add_noise: bool = True) -> np.ndarray:
        pitch, _, yaw = self._get_pitch_roll_yaw()

        vx_world = float(self.data.qvel[0])
        vy_world = float(self.data.qvel[1])
        v_fwd = vx_world * math.cos(yaw) + vy_world * math.sin(yaw)

        theta_dot = float(self.data.qvel[4])
        yaw_rate = float(self.data.qvel[5])

        w_l = float(self.data.qvel[6])
        w_r = float(self.data.qvel[7])

        if add_noise and self.domain_rand:
            v_fwd += float(self.np_random.normal(0.0, 0.015))
            pitch += self._cur_pitch_bias + float(self.np_random.normal(0.0, 0.004))
            theta_dot += float(self.np_random.normal(0.0, 0.03))
            yaw_rate += float(self.np_random.normal(0.0, 0.03))
            w_l += float(self.np_random.normal(0.0, 0.15))
            w_r += float(self.np_random.normal(0.0, 0.15))

        raw_obs = np.array(
            [
                self._pos_err,
                v_fwd,
                pitch,
                theta_dot,
                self._yaw_err,
                yaw_rate,
                w_l,
                w_r,
                self._cmd_v,
                self._cmd_yaw_rate,
                float(self._prev_action[0]),
                float(self._prev_action[1]),
            ],
            dtype=np.float32,
        )
        return np.clip(raw_obs / self.OBS_SCALES, -10.0, 10.0)

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[np.ndarray, dict[str, Any]]:
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)

        if self.domain_rand:
            mass_scale = float(self.np_random.uniform(0.85, 1.15))
            self.model.body_mass[1] = self._base_chassis_mass * mass_scale
            self.model.body_inertia[1] = self._base_chassis_inertia * mass_scale
            self.model.body_ipos[1] = self._base_chassis_ipos + self.np_random.uniform(
                [-0.005, -0.003, -0.005], [0.005, 0.003, 0.008]
            )
            self.model.geom_friction[0, 0] = self._base_floor_friction[0] * float(
                self.np_random.uniform(0.7, 1.3)
            )
            self._cur_motor_gain = self.np_random.uniform(0.85, 1.15, size=2)
            self._cur_pitch_bias = float(self.np_random.uniform(-0.015, 0.015))
        else:
            self.model.body_mass[1] = self._base_chassis_mass
            self.model.body_inertia[1] = self._base_chassis_inertia.copy()
            self.model.body_ipos[1] = self._base_chassis_ipos.copy()
            self.model.geom_friction[0] = self._base_floor_friction.copy()
            self._cur_motor_gain = np.array([1.0, 1.0], dtype=np.float64)
            self._cur_pitch_bias = 0.0

        mujoco.mj_setConst(self.model, self.data)

        init_pitch = float(self.np_random.uniform(-0.16, 0.16))
        init_yaw = float(self.np_random.uniform(-0.15, 0.15))
        init_pitch_vel = float(self.np_random.uniform(-0.35, 0.35))

        cy, sy = math.cos(init_yaw * 0.5), math.sin(init_yaw * 0.5)
        cp, sp = math.cos(init_pitch * 0.5), math.sin(init_pitch * 0.5)
        self.data.qpos[3] = cy * cp
        self.data.qpos[4] = -sy * sp
        self.data.qpos[5] = cy * sp
        self.data.qpos[6] = sy * cp
        self.data.qvel[4] = init_pitch_vel

        mujoco.mj_forward(self.model, self.data)

        self._step_count = 0
        self._prev_action = np.zeros(2, dtype=np.float32)
        self._pending_push[:] = 0.0
        self._pos_err = 0.0
        self._yaw_err = 0.0
        self._cmd_v = 0.0
        self._cmd_yaw_rate = 0.0

        if self.command_mode == "random":
            if self.np_random.random() < 0.4:
                self._target_v, self._target_yaw_rate = 0.0, 0.0
            else:
                self._target_v, self._target_yaw_rate = self._sample_random_command()
            self._next_cmd_step = int(self.np_random.integers(50, 140))
        elif self.command_mode == "demo":
            self._target_v, self._target_yaw_rate = self._demo_command_at_step(0)
            self._next_cmd_step = 0
        else:
            self._next_cmd_step = 0

        obs = self._get_obs()
        info = {
            "pitch_rad": init_pitch,
            "x_pos": 0.0,
            "y_pos": 0.0,
            "cmd_v": self._cmd_v,
            "cmd_yaw_rate": self._cmd_yaw_rate,
        }
        return obs, info

    def _apply_motor_model(self, action: np.ndarray) -> None:
        """Map normalized PWM [-1, 1] to motor torque with deadzone & back-EMF."""
        a = np.clip(action, -1.0, 1.0)
        eff = np.where(
            np.abs(a) < self.motor_deadzone,
            0.0,
            np.sign(a) * (np.abs(a) - self.motor_deadzone) / (1.0 - self.motor_deadzone),
        )
        w = np.array([self.data.qvel[6], self.data.qvel[7]], dtype=np.float64)
        tau = eff * self.motor_torque_max * self._cur_motor_gain - self.motor_back_emf * w
        self.data.ctrl[0] = float(np.clip(tau[0], -0.55, 0.55))
        self.data.ctrl[1] = float(np.clip(tau[1], -0.55, 0.55))

    def step(
        self, action: np.ndarray
    ) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        action = np.asarray(action, dtype=np.float32)
        action_clipped = np.clip(action, -1.0, 1.0)

        # Apply external push (either queued by evaluate/user or random training push)
        if np.any(self._pending_push != 0.0):
            self.data.xfrc_applied[1, 0] = float(self._pending_push[0])
            self.data.xfrc_applied[1, 1] = float(self._pending_push[1])
            self._pending_push[:] = 0.0
        elif self.domain_rand and self._step_count > 25 and self.np_random.random() < 0.015:
            push_fx = float(self.np_random.uniform(-1.6, 1.6))
            push_fy = float(self.np_random.uniform(-0.35, 0.35))
            self.data.xfrc_applied[1, 0] = push_fx
            self.data.xfrc_applied[1, 1] = push_fy
        else:
            self.data.xfrc_applied[1, :] = 0.0

        for _ in range(self.frame_skip):
            self._apply_motor_model(action_clipped)
            mujoco.mj_step(self.model, self.data)

        self._step_count += 1

        pitch, roll, yaw = self._get_pitch_roll_yaw()
        x_pos = float(self.data.qpos[0])
        y_pos = float(self.data.qpos[1])
        dist_origin = math.hypot(x_pos, y_pos)
        v_fwd = float(self.data.qvel[0] * math.cos(yaw) + self.data.qvel[1] * math.sin(yaw))
        theta_dot = float(self.data.qvel[4])
        yaw_rate = float(self.data.qvel[5])

        # Hold-position/heading integrators active only when stopped; bleed smoothly while moving
        if abs(self._cmd_v) < 1e-3 and abs(v_fwd) < 0.15:
            self._pos_err = float(np.clip(self._pos_err + v_fwd * self.dt, -0.5, 0.5))
        else:
            self._pos_err *= 0.85

        if abs(self._cmd_yaw_rate) < 1e-3 and abs(yaw_rate) < 0.30:
            self._yaw_err = float(np.clip(self._yaw_err + yaw_rate * self.dt, -1.0, 1.0))
        else:
            self._yaw_err *= 0.85

        # Reward shaping: upright balance + steep velocity/yaw tracking + smooth control
        v_err = abs(v_fwd - self._cmd_v)
        yaw_rate_err = abs(yaw_rate - self._cmd_yaw_rate)

        upright_reward = 1.2 * math.exp(-12.0 * (pitch ** 2))
        vel_track_reward = 2.0 * math.exp(-8.0 * (v_err ** 2)) - 2.0 * v_err
        yaw_track_reward = 1.2 * math.exp(-2.0 * (yaw_rate_err ** 2)) - 0.6 * yaw_rate_err
        pos_err_reward = 0.5 * math.exp(-8.0 * (self._pos_err ** 2))
        yaw_err_reward = 0.3 * math.exp(-4.0 * (self._yaw_err ** 2))

        rate_penalty = -0.02 * (theta_dot ** 2)
        ctrl_penalty = -0.01 * float(np.sum(action_clipped ** 2))
        smooth_penalty = -0.04 * float(np.sum((action_clipped - self._prev_action) ** 2))

        reward = (
            upright_reward
            + vel_track_reward
            + yaw_track_reward
            + pos_err_reward
            + yaw_err_reward
            + rate_penalty
            + ctrl_penalty
            + smooth_penalty
        )

        # Termination conditions
        fallen = (
            abs(pitch) > self.max_tilt_rad
            or abs(roll) > math.radians(35.0)
            or float(self.data.qpos[2]) < 0.015
            or (self.max_pos_m is not None and dist_origin > self.max_pos_m)
        )
        if fallen:
            reward -= 15.0

        terminated = bool(fallen)
        truncated = bool(self._step_count >= self.max_episode_steps)

        # Advance smoothed command toward current target for the next observation
        self._update_command_schedule()

        self._prev_action = action_clipped.copy()
        obs = self._get_obs()

        info = {
            "pitch_rad": pitch,
            "pitch_deg": math.degrees(pitch),
            "roll_deg": math.degrees(roll),
            "yaw_deg": math.degrees(yaw),
            "x_pos": x_pos,
            "y_pos": y_pos,
            "v_fwd": v_fwd,
            "yaw_rate": yaw_rate,
            "cmd_v": self._cmd_v,
            "cmd_yaw_rate": self._cmd_yaw_rate,
            "pos_err": self._pos_err,
            "yaw_err": self._yaw_err,
            "dist_origin": dist_origin,
        }
        return obs, float(reward), terminated, truncated, info

    def apply_external_push(self, fx: float = 0.0, fy: float = 0.0) -> None:
        """Queue an external push force (N) on the chassis for the next step."""
        self._pending_push[0] = fx
        self._pending_push[1] = fy

    def render(self) -> np.ndarray | None:
        if self.render_mode != "rgb_array":
            return None
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, height=480, width=640)
        self._renderer.update_scene(self.data)
        return self._renderer.render()

    def close(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
