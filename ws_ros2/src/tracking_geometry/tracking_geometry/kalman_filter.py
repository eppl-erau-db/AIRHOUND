#!/usr/bin/env python3
"""
3D Kalman Filter.

A 6-state constant-velocity Kalman filter for 3D target position estimation.

State vector: [x, y, z, vx, vy, vz]
Measurement: [x, y, z] from depth camera
"""

import numpy as np
from typing import Tuple
from dataclasses import dataclass


@dataclass
class KalmanState:
    x: np.ndarray
    P: np.ndarray
    timestamp: float = 0.0
    initialized: bool = False


class KalmanFilter3D:
    """
    Constant-velocity Kalman filter for 3D target tracking.

    State vector: [x, y, z, vx, vy, vz]
    Measurement: [x, y, z]
    """

    def __init__(
        self,
        process_noise_pos: float = 0.1,
        process_noise_vel: float = 1.0,
        measurement_noise: float = 0.05,
    ):
        self.process_noise_pos = float(process_noise_pos)
        self.process_noise_vel = float(process_noise_vel)
        self.measurement_noise = float(measurement_noise)

        self.state = KalmanState(
            x=np.zeros(6, dtype=np.float64),
            P=np.eye(6, dtype=np.float64) * 1.0,
            initialized=False,
        )

        self.H = np.array(
            [
                [1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0],
            ],
            dtype=np.float64,
        )

        self.R = np.eye(3, dtype=np.float64) * self.measurement_noise

    @property
    def position(self) -> np.ndarray:
        return self.state.x[:3].copy()

    @property
    def velocity(self) -> np.ndarray:
        return self.state.x[3:6].copy()

    @property
    def is_initialized(self) -> bool:
        return bool(self.state.initialized)

    def reset(self) -> None:
        self.state = KalmanState(
            x=np.zeros(6, dtype=np.float64),
            P=np.eye(6, dtype=np.float64) * 1.0,
            initialized=False,
        )

    def initialize(self, measurement: np.ndarray, timestamp: float = 0.0) -> None:
        measurement = np.asarray(measurement, dtype=np.float64).reshape(3)
        self.state.x[:3] = measurement
        self.state.x[3:6] = 0.0
        self.state.P = np.diag([0.1, 0.1, 0.1, 1.0, 1.0, 1.0]).astype(np.float64)
        self.state.timestamp = float(timestamp)
        self.state.initialized = True

    def _get_F(self, dt: float) -> np.ndarray:
        F = np.eye(6, dtype=np.float64)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt
        return F

    def _get_Q(self, dt: float) -> np.ndarray:
        q_pos = self.process_noise_pos
        q_vel = self.process_noise_vel

        Q = np.zeros((6, 6), dtype=np.float64)
        Q[0, 0] = q_pos * dt**2
        Q[1, 1] = q_pos * dt**2
        Q[2, 2] = q_pos * dt**2
        Q[3, 3] = q_vel * dt
        Q[4, 4] = q_vel * dt
        Q[5, 5] = q_vel * dt
        return Q

    def predict(self, dt: float) -> np.ndarray:
        if not self.state.initialized:
            return np.zeros(3, dtype=np.float64)

        dt = float(dt)
        if dt <= 0.0:
            return self.position

        F = self._get_F(dt)
        Q = self._get_Q(dt)

        self.state.x = F @ self.state.x
        self.state.P = F @ self.state.P @ F.T + Q
        self.state.timestamp += dt

        return self.position

    def update(self, measurement: np.ndarray) -> np.ndarray:
        measurement = np.asarray(measurement, dtype=np.float64).reshape(3)

        if not self.state.initialized:
            self.initialize(measurement)
            return self.position

        z = measurement
        y = z - self.H @ self.state.x
        S = self.H @ self.state.P @ self.H.T + self.R
        K = self.state.P @ self.H.T @ np.linalg.inv(S)

        self.state.x = self.state.x + K @ y

        # Joseph form
        I = np.eye(6, dtype=np.float64)
        I_KH = I - K @ self.H
        self.state.P = I_KH @ self.state.P @ I_KH.T + K @ self.R @ K.T

        return self.position

    def predict_position(self, dt_future: float) -> np.ndarray:
        if not self.state.initialized:
            return np.zeros(3, dtype=np.float64)
        dt_future = float(dt_future)
        return self.position + self.velocity * dt_future

    def mahalanobis_distance(self, measurement: np.ndarray) -> float:
        if not self.state.initialized:
            return float("inf")

        z = np.asarray(measurement, dtype=np.float64).reshape(3)
        y = z - self.H @ self.state.x
        S = self.H @ self.state.P @ self.H.T + self.R

        d2 = y.T @ np.linalg.inv(S) @ y
        return float(np.sqrt(d2))


class KalmanFilter3DWithGating(KalmanFilter3D):
    def __init__(self, gate_threshold: float = 3.0, max_missed: int = 30, **kwargs):
        super().__init__(**kwargs)
        self.gate_threshold = float(gate_threshold)
        self.max_missed = int(max_missed)
        self.missed_count = 0

    def update(self, measurement: np.ndarray) -> Tuple[np.ndarray, bool]:
        if not self.state.initialized:
            self.initialize(measurement)
            self.missed_count = 0
            return self.position, True

        dist = self.mahalanobis_distance(measurement)

        if dist < self.gate_threshold:
            pos = super().update(measurement)
            self.missed_count = 0
            return pos, True

        # reject
        self.missed_count += 1
        if self.missed_count > self.max_missed:
            self.reset()
        return self.position, False

    def mark_missed(self) -> None:
        self.missed_count += 1
        if self.missed_count > self.max_missed:
            self.reset()


def create_filter_from_params(
    process_noise_pos: float = 0.1,
    process_noise_vel: float = 1.0,
    measurement_noise: float = 0.05,
    use_gating: bool = True,
    gate_threshold: float = 3.0,
) -> KalmanFilter3D:
    if use_gating:
        return KalmanFilter3DWithGating(
            process_noise_pos=process_noise_pos,
            process_noise_vel=process_noise_vel,
            measurement_noise=measurement_noise,
            gate_threshold=gate_threshold,
        )
    return KalmanFilter3D(
        process_noise_pos=process_noise_pos,
        process_noise_vel=process_noise_vel,
        measurement_noise=measurement_noise,
    )
