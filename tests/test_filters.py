import numpy as np

from control.filters import KalmanFilter1D


def test_kalman_filter_converges_toward_measurement():
    kf = KalmanFilter1D(dt=0.01)
    estimates = []
    for _ in range(20):
        kf.predict()
        state = kf.update(0.02)
        estimates.append(float(state[0, 0]))
    assert np.isclose(estimates[-1], 0.02, atol=1e-3)
