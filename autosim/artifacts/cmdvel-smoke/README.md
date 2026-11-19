# cmd_vel verification

Google Robot, G1 and Go2 received real Autolink TwistStamped commands on isolated verification channels. Forward, reverse, in-place turn and arc motion were checked against Habitat AO translation, odometry and TF. All stopped after command timeout. JSON files record the trajectories. Viewer HTTP motion controls also passed for all three models.
