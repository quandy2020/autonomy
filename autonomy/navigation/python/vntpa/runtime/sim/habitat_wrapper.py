"""Habitat-Sim import boundary."""

from __future__ import annotations


def make_simulator(config_path: str):
    """Create a Habitat simulator; keeps habitat_sim optional."""
    import habitat_sim

    return habitat_sim.Simulator(config_path)
