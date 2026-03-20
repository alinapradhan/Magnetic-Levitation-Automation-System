from control.autotune import estimate_operating_point, heuristic_pid_tune


def test_autotune_returns_positive_gains():
    op = estimate_operating_point(0.05, 9.81, 2.2e-5, 0.015, 0.002)
    config = heuristic_pid_tune(op)
    assert config.kp > 0
    assert config.ki > 0
    assert config.kd > 0
