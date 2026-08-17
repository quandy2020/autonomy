"""Module entry: ``python -m autosim``."""

from autosim.runner import Runner


def run() -> None:
    """Start the sensor–actuator bridge with the default config."""
    Runner.main()


if __name__ == "__main__":
    run()
