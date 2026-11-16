# Perception behaviors

Behavior components consume timestamped observations over AutoLink and
produce task-level outputs. Follow uses tracked people, registered depth,
camera calibration and odometry to publish a target, path and local grid.
Run the perception producer before starting `behaviors.launch`.

Charging and exploration currently have separate entry points in the
repository. Their migration requires preserving the existing command
authority and configuration contracts.
