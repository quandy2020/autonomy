import habitat_sim

print("version", habitat_sim.__version__)
print("SimulatorConfiguration", [a for a in dir(habitat_sim.SimulatorConfiguration) if not a.startswith("_")])
print("CameraSensorSpec", [a for a in dir(habitat_sim.CameraSensorSpec) if not a.startswith("_")])
print("Simulator", [a for a in dir(habitat_sim.simulator.Simulator) if "ray" in a.lower() or "light" in a.lower()])
print("sensors", [a for a in dir(habitat_sim.sensors) if "Sensor" in a])
for name in ("override_scene_light_defaults", "scene_light_setup"):
    attr = getattr(habitat_sim.SimulatorConfiguration, name, None)
    print(name, type(attr), getattr(attr, "__doc__", None))
