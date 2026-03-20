"""Basic tabular Q-learning controller for a discretized maglev simulation."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from simulations.model import MaglevSystem, MaglevSystemConfig


@dataclass
class QLearningConfig:
    n_position_bins: int = 15
    n_velocity_bins: int = 15
    actions: tuple[float, ...] = (0.0, 0.5, 1.0, 1.5, 2.0)
    alpha: float = 0.1
    gamma: float = 0.95
    epsilon: float = 0.2


class TabularQController:
    def __init__(self, config: QLearningConfig, system_config: MaglevSystemConfig) -> None:
        self.config = config
        self.system_config = system_config
        self.q_table = np.zeros((config.n_position_bins, config.n_velocity_bins, len(config.actions)))
        self.position_bins = np.linspace(system_config.min_position, system_config.max_position, config.n_position_bins)
        self.velocity_bins = np.linspace(-1.0, 1.0, config.n_velocity_bins)

    def discretize(self, position: float, velocity: float) -> tuple[int, int]:
        p = int(np.clip(np.digitize(position, self.position_bins) - 1, 0, self.config.n_position_bins - 1))
        v = int(np.clip(np.digitize(velocity, self.velocity_bins) - 1, 0, self.config.n_velocity_bins - 1))
        return p, v

    def choose_action(self, state: tuple[int, int], explore: bool = True) -> int:
        if explore and np.random.rand() < self.config.epsilon:
            return int(np.random.randint(len(self.config.actions)))
        return int(np.argmax(self.q_table[state[0], state[1], :]))

    def reward(self, position: float, reference: float, velocity: float) -> float:
        return -abs(reference - position) - 0.05 * abs(velocity)


def run_q_learning_demo(episodes: int = 50, steps_per_episode: int = 200, dt: float = 0.005) -> TabularQController:
    system_config = MaglevSystemConfig()
    q_config = QLearningConfig()
    agent = TabularQController(q_config, system_config)

    for _ in range(episodes):
        plant = MaglevSystem(system_config)
        for step in range(steps_per_episode):
            ref = system_config.reference_position(step * dt)
            state = agent.discretize(plant.state.position, plant.state.velocity)
            action_idx = agent.choose_action(state, explore=True)
            current = q_config.actions[action_idx]
            plant.step(current, dt)
            next_state = agent.discretize(plant.state.position, plant.state.velocity)
            reward = agent.reward(plant.state.position, ref, plant.state.velocity)
            best_next = np.max(agent.q_table[next_state[0], next_state[1], :])
            old_value = agent.q_table[state[0], state[1], action_idx]
            agent.q_table[state[0], state[1], action_idx] = old_value + q_config.alpha * (
                reward + q_config.gamma * best_next - old_value
            )

    return agent
