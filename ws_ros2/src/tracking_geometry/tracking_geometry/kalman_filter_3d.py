"""
3D Kalman Filter for AIRHOUND Target Tracking.

A 6-state constant-velocity Kalman filter for 3D target position estimation.
This is a template/scaffold for the tracking team to integrate with their system.

State vector: [x, y, z, vx, vy, vz]
Measurement: [x, y, z] from depth camera

Usage:
    from tracking_geometry.kalman_filter_3d import KalmanFilter3D
    
    kf = KalmanFilter3D()
    
    # In your tracking loop:
    kf.predict(dt=0.033)  # 30 Hz
    kf.update(measurement=np.array([x, y, z]))
    
    # Get current estimate:
    position = kf.position  # [x, y, z]
    velocity = kf.velocity  # [vx, vy, vz]

Author: Perception Lead (Template)
Date: January 2026
Note: Tracking team to tune Q/R matrices for flight performance
"""

import numpy as np
from typing import Optional, Tuple
from dataclasses import dataclass


@dataclass
class KalmanState:
    """Kalman filter state container."""
    x: np.ndarray          # State vector [6]
    P: np.ndarray          # Covariance matrix [6x6]
    timestamp: float = 0.0  # Last update time (seconds)
    initialized: bool = False


class KalmanFilter3D:
    """
    Constant-velocity Kalman filter for 3D target tracking.
    
    State vector: [x, y, z, vx, vy, vz]
    - x, y, z: Position in camera optical frame (meters)
    - vx, vy, vz: Velocity in camera optical frame (m/s)
    
    Parameters
    ----------
    process_noise_pos : float
        Process noise for position states (m^2). Default: 0.1
    process_noise_vel : float
        Process noise for velocity states ((m/s)^2). Default: 1.0
    measurement_noise : float
        Measurement noise (m^2). Default: 0.05
        
    Note for Tracking Team:
    -----------------------
    These default values are conservative. You should tune them based on:
    - process_noise_pos: Higher if target is maneuvering unpredictably
    - process_noise_vel: Higher if target accelerates frequently
    - measurement_noise: Based on D455 depth accuracy (~1-2% of distance)
    
    For D455 at 3m range:
    - Depth accuracy: ~3cm → measurement_noise ~= 0.001 (m^2)
    - Adjust based on observed noise in /perception/target_3d
    """
    
    def __init__(
        self,
        process_noise_pos: float = 0.1,
        process_noise_vel: float = 1.0,
        measurement_noise: float = 0.05,
    ):
        # Store noise parameters
        self.process_noise_pos = process_noise_pos
        self.process_noise_vel = process_noise_vel
        self.measurement_noise = measurement_noise
        
        # Initialize state
        self.state = KalmanState(
            x=np.zeros(6),
            P=np.eye(6) * 1.0,  # Initial uncertainty
            initialized=False
        )
        
        # Measurement matrix (we observe position only)
        # H maps state [x,y,z,vx,vy,vz] to measurement [x,y,z]
        self.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
        ], dtype=np.float64)
        
        # Measurement noise covariance
        self.R = np.eye(3) * measurement_noise
    
    @property
    def position(self) -> np.ndarray:
        """Current position estimate [x, y, z] in meters."""
        return self.state.x[:3].copy()
    
    @property
    def velocity(self) -> np.ndarray:
        """Current velocity estimate [vx, vy, vz] in m/s."""
        return self.state.x[3:6].copy()
    
    @property
    def position_covariance(self) -> np.ndarray:
        """Position uncertainty (3x3 covariance matrix)."""
        return self.state.P[:3, :3].copy()
    
    @property
    def velocity_covariance(self) -> np.ndarray:
        """Velocity uncertainty (3x3 covariance matrix)."""
        return self.state.P[3:6, 3:6].copy()
    
    @property
    def is_initialized(self) -> bool:
        """Check if filter has been initialized with a measurement."""
        return self.state.initialized
    
    def reset(self):
        """Reset filter to uninitialized state."""
        self.state = KalmanState(
            x=np.zeros(6),
            P=np.eye(6) * 1.0,
            initialized=False
        )
    
    def initialize(self, measurement: np.ndarray, timestamp: float = 0.0):
        """
        Initialize filter with first measurement.
        
        Parameters
        ----------
        measurement : np.ndarray
            First position measurement [x, y, z]
        timestamp : float
            Measurement timestamp (seconds)
        """
        self.state.x[:3] = measurement[:3]
        self.state.x[3:6] = 0.0  # Assume zero initial velocity
        
        # Higher initial uncertainty for velocity
        self.state.P = np.diag([0.1, 0.1, 0.1, 1.0, 1.0, 1.0])
        
        self.state.timestamp = timestamp
        self.state.initialized = True
    
    def _get_F(self, dt: float) -> np.ndarray:
        """
        Get state transition matrix for time step dt.
        
        Constant velocity model:
        x(t+dt) = x(t) + vx(t) * dt
        vx(t+dt) = vx(t)
        """
        F = np.eye(6)
        F[0, 3] = dt  # x += vx * dt
        F[1, 4] = dt  # y += vy * dt
        F[2, 5] = dt  # z += vz * dt
        return F
    
    def _get_Q(self, dt: float) -> np.ndarray:
        """
        Get process noise covariance for time step dt.
        
        Uses discrete white noise acceleration model.
        """
        # Standard discrete white noise acceleration model
        # Assumes constant velocity with random acceleration perturbations
        q_pos = self.process_noise_pos
        q_vel = self.process_noise_vel
        
        # Simplified diagonal Q (can be replaced with full kinematic model)
        Q = np.zeros((6, 6))
        Q[0, 0] = q_pos * dt**2
        Q[1, 1] = q_pos * dt**2
        Q[2, 2] = q_pos * dt**2
        Q[3, 3] = q_vel * dt
        Q[4, 4] = q_vel * dt
        Q[5, 5] = q_vel * dt
        
        return Q
    
    def predict(self, dt: float) -> np.ndarray:
        """
        Predict state forward by dt seconds.
        
        Parameters
        ----------
        dt : float
            Time step in seconds
            
        Returns
        -------
        np.ndarray
            Predicted position [x, y, z]
        """
        if not self.state.initialized:
            return np.zeros(3)
        
        F = self._get_F(dt)
        Q = self._get_Q(dt)
        
        # State prediction: x = F @ x
        self.state.x = F @ self.state.x
        
        # Covariance prediction: P = F @ P @ F.T + Q
        self.state.P = F @ self.state.P @ F.T + Q
        
        self.state.timestamp += dt
        
        return self.position
    
    def update(self, measurement: np.ndarray) -> np.ndarray:
        """
        Update state with new measurement.
        
        Parameters
        ----------
        measurement : np.ndarray
            Position measurement [x, y, z] in meters
            
        Returns
        -------
        np.ndarray
            Updated position estimate [x, y, z]
        """
        if not self.state.initialized:
            self.initialize(measurement)
            return self.position
        
        # Innovation (measurement residual)
        z = measurement[:3]
        y = z - self.H @ self.state.x
        
        # Innovation covariance
        S = self.H @ self.state.P @ self.H.T + self.R
        
        # Kalman gain
        K = self.state.P @ self.H.T @ np.linalg.inv(S)
        
        # State update
        self.state.x = self.state.x + K @ y
        
        # Covariance update (Joseph form for numerical stability)
        I_KH = np.eye(6) - K @ self.H
        self.state.P = I_KH @ self.state.P @ I_KH.T + K @ self.R @ K.T
        
        return self.position
    
    def predict_position(self, dt_future: float) -> np.ndarray:
        """
        Predict where target will be dt_future seconds from now.
        
        Does NOT modify filter state (read-only prediction).
        
        Parameters
        ----------
        dt_future : float
            Time ahead to predict (seconds)
            
        Returns
        -------
        np.ndarray
            Predicted position [x, y, z]
        """
        if not self.state.initialized:
            return np.zeros(3)
        
        # Simple constant-velocity extrapolation
        future_pos = self.position + self.velocity * dt_future
        return future_pos
    
    def get_state_vector(self) -> np.ndarray:
        """Get full state vector [x, y, z, vx, vy, vz]."""
        return self.state.x.copy()
    
    def get_covariance(self) -> np.ndarray:
        """Get full covariance matrix (6x6)."""
        return self.state.P.copy()
    
    def mahalanobis_distance(self, measurement: np.ndarray) -> float:
        """
        Compute Mahalanobis distance of measurement from predicted state.
        
        Useful for gating (rejecting outlier measurements).
        
        Parameters
        ----------
        measurement : np.ndarray
            Position measurement [x, y, z]
            
        Returns
        -------
        float
            Mahalanobis distance (chi-squared with 3 DOF)
            Values > 7.81 are outliers at 95% confidence
        """
        if not self.state.initialized:
            return float('inf')
        
        z = measurement[:3]
        y = z - self.H @ self.state.x  # Innovation
        S = self.H @ self.state.P @ self.H.T + self.R  # Innovation covariance
        
        d2 = y.T @ np.linalg.inv(S) @ y
        return float(np.sqrt(d2))


class KalmanFilter3DWithGating(KalmanFilter3D):
    """
    Kalman filter with measurement gating for outlier rejection.
    
    Extends KalmanFilter3D with automatic rejection of measurements
    that are statistically unlikely given current state estimate.
    
    Parameters
    ----------
    gate_threshold : float
        Mahalanobis distance threshold. Default: 3.0 (99.7% confidence)
    max_missed : int
        Maximum missed updates before reset. Default: 30 (1 second at 30Hz)
    """
    
    def __init__(
        self,
        gate_threshold: float = 3.0,
        max_missed: int = 30,
        **kwargs
    ):
        super().__init__(**kwargs)
        self.gate_threshold = gate_threshold
        self.max_missed = max_missed
        self.missed_count = 0
    
    def update(self, measurement: np.ndarray) -> Tuple[np.ndarray, bool]:
        """
        Update with measurement, rejecting outliers.
        
        Parameters
        ----------
        measurement : np.ndarray
            Position measurement [x, y, z]
            
        Returns
        -------
        Tuple[np.ndarray, bool]
            (updated_position, measurement_accepted)
        """
        if not self.state.initialized:
            self.initialize(measurement)
            self.missed_count = 0
            return self.position, True
        
        # Check gating
        distance = self.mahalanobis_distance(measurement)
        
        if distance < self.gate_threshold:
            # Accept measurement
            result = super().update(measurement)
            self.missed_count = 0
            return result, True
        else:
            # Reject measurement (outlier)
            self.missed_count += 1
            
            # Reset if too many missed
            if self.missed_count > self.max_missed:
                self.reset()
            
            return self.position, False
    
    def mark_missed(self):
        """Mark a missed detection (no measurement this frame)."""
        self.missed_count += 1
        if self.missed_count > self.max_missed:
            self.reset()


# Convenience function for ROS2 integration
def create_filter_from_params(
    process_noise_pos: float = 0.1,
    process_noise_vel: float = 1.0,
    measurement_noise: float = 0.05,
    use_gating: bool = True,
    gate_threshold: float = 3.0,
) -> KalmanFilter3D:
    """
    Factory function to create Kalman filter from ROS2 parameters.
    
    Example ROS2 integration:
    
        # In your node __init__:
        self.declare_parameter('kf_process_noise_pos', 0.1)
        self.declare_parameter('kf_process_noise_vel', 1.0)
        self.declare_parameter('kf_measurement_noise', 0.05)
        self.declare_parameter('kf_use_gating', True)
        self.declare_parameter('kf_gate_threshold', 3.0)
        
        self.kf = create_filter_from_params(
            process_noise_pos=self.get_parameter('kf_process_noise_pos').value,
            process_noise_vel=self.get_parameter('kf_process_noise_vel').value,
            measurement_noise=self.get_parameter('kf_measurement_noise').value,
            use_gating=self.get_parameter('kf_use_gating').value,
            gate_threshold=self.get_parameter('kf_gate_threshold').value,
        )
    """
    if use_gating:
        return KalmanFilter3DWithGating(
            process_noise_pos=process_noise_pos,
            process_noise_vel=process_noise_vel,
            measurement_noise=measurement_noise,
            gate_threshold=gate_threshold,
        )
    else:
        return KalmanFilter3D(
            process_noise_pos=process_noise_pos,
            process_noise_vel=process_noise_vel,
            measurement_noise=measurement_noise,
        )
