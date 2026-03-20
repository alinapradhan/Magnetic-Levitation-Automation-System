import pytest

from control.pid import PIDConfig, PIDController


def test_pid_output_respects_limits():
    controller = PIDController(PIDConfig(kp=100.0, ki=0.0, kd=0.0, output_min=0.0, output_max=1.0))
    output = controller.update(1.0, 0.0, 0.01)
    assert output == 1.0


def test_pid_rejects_nonpositive_dt():
    controller = PIDController(PIDConfig(kp=1.0, ki=1.0, kd=1.0))
    with pytest.raises(ValueError):
        controller.update(0.0, 0.0, 0.0)
