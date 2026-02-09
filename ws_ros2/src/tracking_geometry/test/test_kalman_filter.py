"""
Tests for tracking_geometry.kalman_filter module.

Validates the 3D constant-velocity Kalman filter implementation
including gating, reset behavior, and numerical stability.
"""

import numpy as np
import pytest
import sys
import os

# Add package to path for standalone testing (outside colcon)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from tracking_geometry.kalman_filter import (
    KalmanFilter3D,
    KalmanFilter3DWithGating,
    KalmanState,
    create_filter_from_params,
)


# --- KalmanFilter3D ---


class TestKalmanFilter3DInit:
    def test_construction_defaults(self):
        kf = KalmanFilter3D()
        assert not kf.is_initialized
        assert np.allclose(kf.position, [0, 0, 0])
        assert np.allclose(kf.velocity, [0, 0, 0])

    def test_construction_custom_params(self):
        kf = KalmanFilter3D(
            process_noise_pos=0.5, process_noise_vel=2.0, measurement_noise=0.1
        )
        assert kf.process_noise_pos == 0.5
        assert kf.process_noise_vel == 2.0
        assert kf.measurement_noise == 0.1

    def test_initialize(self):
        kf = KalmanFilter3D()
        kf.initialize(np.array([1.0, 2.0, 3.0]), timestamp=1.5)
        assert kf.is_initialized
        assert np.allclose(kf.position, [1, 2, 3])
        assert np.allclose(kf.velocity, [0, 0, 0])
        assert kf.state.timestamp == 1.5

    def test_auto_initialize_on_update(self):
        kf = KalmanFilter3D()
        pos = kf.update(np.array([5.0, 6.0, 7.0]))
        assert kf.is_initialized
        assert np.allclose(pos, [5, 6, 7])


class TestKalmanFilter3DPredict:
    def test_predict_before_init_returns_zeros(self):
        kf = KalmanFilter3D()
        pos = kf.predict(0.1)
        assert np.allclose(pos, [0, 0, 0])

    def test_predict_zero_velocity(self):
        kf = KalmanFilter3D()
        kf.initialize(np.array([1.0, 2.0, 3.0]))
        pos = kf.predict(1.0)
        assert np.allclose(pos, [1, 2, 3])

    def test_predict_negative_dt(self):
        kf = KalmanFilter3D()
        kf.initialize(np.array([1.0, 2.0, 3.0]))
        pos = kf.predict(-0.5)
        assert np.allclose(pos, [1, 2, 3])

    def test_predict_advances_timestamp(self):
        kf = KalmanFilter3D()
        kf.initialize(np.array([0, 0, 0]), timestamp=10.0)
        kf.predict(0.5)
        assert abs(kf.state.timestamp - 10.5) < 1e-10

    def test_predict_grows_covariance(self):
        kf = KalmanFilter3D()
        kf.initialize(np.array([0, 0, 0]))
        P_before = kf.state.P.copy()
        kf.predict(1.0)
        P_after = kf.state.P
        # Process noise should increase uncertainty
        assert np.trace(P_after) > np.trace(P_before)


class TestKalmanFilter3DUpdate:
    def test_update_pulls_toward_measurement(self):
        kf = KalmanFilter3D()
        kf.initialize(np.array([0, 0, 0]))
        kf.predict(0.033)
        pos = kf.update(np.array([1.0, 0, 0]))
        assert pos[0] > 0, "Should pull toward measurement"

    def test_update_reduces_covariance(self):
        kf = KalmanFilter3D()
        kf.initialize(np.array([0, 0, 0]))
        kf.predict(1.0)
        P_before = kf.state.P.copy()
        kf.update(np.array([0, 0, 0]))
        P_after = kf.state.P
        assert np.trace(P_after) < np.trace(P_before)

    def test_update_builds_velocity(self):
        kf = KalmanFilter3D()
        kf.initialize(np.array([0, 0, 0]))
        # Feed consistent motion in +x
        for i in range(20):
            kf.predict(0.033)
            kf.update(np.array([i * 0.1, 0, 0]))
        vel = kf.velocity
        assert vel[0] > 0, "Should estimate positive x velocity"


class TestKalmanFilter3DPredictPosition:
    def test_read_only(self):
        kf = KalmanFilter3D()
        kf.initialize(np.array([0, 0, 0]))
        kf.predict(0.1)
        kf.update(np.array([1, 0, 0]))
        pos_before = kf.position.copy()
        _ = kf.predict_position(5.0)
        assert np.allclose(kf.position, pos_before)

    def test_uninitialized_returns_zeros(self):
        kf = KalmanFilter3D()
        assert np.allclose(kf.predict_position(1.0), [0, 0, 0])


class TestKalmanFilter3DMahalanobis:
    def test_uninitialized_returns_inf(self):
        kf = KalmanFilter3D()
        assert kf.mahalanobis_distance(np.array([0, 0, 0])) == float("inf")

    def test_close_measurement_small_distance(self):
        kf = KalmanFilter3D()
        kf.initialize(np.array([0, 0, 0]))
        d = kf.mahalanobis_distance(np.array([0.01, 0.01, 0.01]))
        assert d < 1.0

    def test_far_measurement_large_distance(self):
        kf = KalmanFilter3D()
        kf.initialize(np.array([0, 0, 0]))
        d_near = kf.mahalanobis_distance(np.array([0.01, 0, 0]))
        d_far = kf.mahalanobis_distance(np.array([100, 0, 0]))
        assert d_far > d_near


class TestKalmanFilter3DReset:
    def test_reset(self):
        kf = KalmanFilter3D()
        kf.initialize(np.array([5, 5, 5]))
        kf.reset()
        assert not kf.is_initialized
        assert np.allclose(kf.position, [0, 0, 0])


# --- KalmanFilter3DWithGating ---


class TestKalmanFilter3DWithGating:
    def test_first_measurement_always_accepted(self):
        kf = KalmanFilter3DWithGating(gate_threshold=3.0, max_missed=5)
        pos, accepted = kf.update(np.array([1, 2, 3]))
        assert accepted
        assert kf.is_initialized

    def test_close_measurement_accepted(self):
        kf = KalmanFilter3DWithGating(gate_threshold=3.0)
        kf.update(np.array([0, 0, 0]))
        kf.predict(0.033)
        pos, accepted = kf.update(np.array([0.001, 0.001, 0.001]))
        assert accepted

    def test_outlier_rejected(self):
        kf = KalmanFilter3DWithGating(gate_threshold=3.0)
        kf.initialize(np.array([0, 0, 0]))
        # Tighten covariance with many updates
        for _ in range(20):
            kf.predict(0.033)
            kf.update(np.array([0.001, 0.001, 0.001]))
        pos, accepted = kf.update(np.array([999, 999, 999]))
        assert not accepted

    def test_max_missed_triggers_reset(self):
        kf = KalmanFilter3DWithGating(gate_threshold=3.0, max_missed=3)
        kf.initialize(np.array([0, 0, 0]))
        for _ in range(20):
            kf.predict(0.033)
            kf.update(np.array([0.001, 0.001, 0.001]))
        # Feed outliers
        for _ in range(4):
            kf.update(np.array([999, 999, 999]))
        assert not kf.is_initialized

    def test_mark_missed(self):
        kf = KalmanFilter3DWithGating(gate_threshold=3.0, max_missed=2)
        kf.initialize(np.array([0, 0, 0]))
        kf.mark_missed()
        kf.mark_missed()
        kf.mark_missed()
        assert not kf.is_initialized


# --- Factory ---


class TestFactory:
    def test_create_with_gating(self):
        f = create_filter_from_params(use_gating=True)
        assert isinstance(f, KalmanFilter3DWithGating)

    def test_create_without_gating(self):
        f = create_filter_from_params(use_gating=False)
        assert isinstance(f, KalmanFilter3D)
        assert not isinstance(f, KalmanFilter3DWithGating)

    def test_params_forwarded(self):
        f = create_filter_from_params(
            process_noise_pos=0.5,
            process_noise_vel=2.0,
            measurement_noise=0.1,
            use_gating=False,
        )
        assert f.process_noise_pos == 0.5
        assert f.process_noise_vel == 2.0
        assert f.measurement_noise == 0.1


# --- Numerical stability ---


class TestNumericalStability:
    def test_covariance_stays_symmetric_psd(self):
        """P must stay symmetric and positive semi-definite through many cycles."""
        kf = KalmanFilter3D()
        kf.initialize(np.array([1.0, 2.0, 3.0]))
        rng = np.random.default_rng(42)
        for i in range(100):
            kf.predict(0.033)
            meas = np.array([1.0, 2.0, 3.0]) + rng.standard_normal(3) * 0.1
            kf.update(meas)
            P = kf.state.P
            assert np.allclose(P, P.T, atol=1e-10), f"P not symmetric at step {i}"
            eigvals = np.linalg.eigvalsh(P)
            assert np.all(eigvals >= -1e-10), (
                f"P not PSD at step {i}, min eigval={eigvals.min()}"
            )

    def test_filter_converges_on_constant_target(self):
        """Filter should converge to true position with low uncertainty."""
        kf = KalmanFilter3D(measurement_noise=0.01)
        true_pos = np.array([5.0, 10.0, 15.0])
        kf.initialize(np.array([0, 0, 0]))
        rng = np.random.default_rng(123)
        for _ in range(200):
            kf.predict(0.033)
            meas = true_pos + rng.standard_normal(3) * 0.1
            kf.update(meas)
        assert np.linalg.norm(kf.position - true_pos) < 0.5
        assert np.linalg.norm(kf.velocity) < 0.5  # Should be near zero

    def test_filter_tracks_constant_velocity(self):
        """Filter should track a linearly moving target."""
        kf = KalmanFilter3D(measurement_noise=0.01)
        kf.initialize(np.array([0, 0, 0]))
        rng = np.random.default_rng(456)
        velocity = np.array([1.0, 0.5, -0.3])
        dt = 0.033
        for i in range(300):
            kf.predict(dt)
            true_pos = velocity * (i + 1) * dt
            meas = true_pos + rng.standard_normal(3) * 0.05
            kf.update(meas)
        # Velocity estimate should be in the right ballpark.
        # Default process noise is conservative so convergence is gradual.
        assert np.linalg.norm(kf.velocity - velocity) < 0.5
