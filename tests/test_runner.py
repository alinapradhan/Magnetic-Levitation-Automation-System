from simulations.runner import run_simulation


def test_run_simulation_shapes():
    result = run_simulation(duration=0.1, dt=0.01)
    assert len(result.time) == len(result.position) == len(result.current)
    assert result.position.min() >= 0.0
