"""The following imports are necessary for updating the registry"""
try:
    import thirdparty.frontier_exploration.base_explorer
    import thirdparty.frontier_exploration.frontier_detection
    import thirdparty.frontier_exploration.frontier_sensor
    import thirdparty.frontier_exploration.measurements
    import thirdparty.frontier_exploration.objnav_explorer
    import thirdparty.frontier_exploration.policy
    import thirdparty.frontier_exploration.trainer
    import thirdparty.frontier_exploration.utils.inflection_sensor
    import thirdparty.frontier_exploration.utils.multistory_episode_finder
except ModuleNotFoundError as e:
    # If the error was due to the habitat package not being installed, then pass, but
    # print a warning. Do not pass if it was due to another package being missing.
    if "habitat" not in e.name:
        raise e
    else:
        print(
            "Warning: importing habitat failed. Cannot register habitat_baselines "
            "components."
        )
