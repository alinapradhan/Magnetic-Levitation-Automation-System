from simulations.model import MaglevSystem, MaglevSystemConfig


def test_magnetic_force_increases_with_current():
    system = MaglevSystem(MaglevSystemConfig())
    low = system.magnetic_force(0.5, 0.02)
    high = system.magnetic_force(1.0, 0.02)
    assert high > low


def test_position_is_bounded():
    config = MaglevSystemConfig()
    system = MaglevSystem(config)
    for _ in range(1000):
        system.step(config.current_max, 0.001)
    assert config.min_position <= system.state.position <= config.max_position
