# Tianracer Simulation Referee

ROS 1 Noetic runtime package for judging Tianracer races in Gazebo. It observes
`/clock` and `/gazebo/model_states`; it does not start Gazebo, publish navigation
goals, or control the vehicle.

Start Gazebo first, then run:

```bash
roslaunch tianracer_sim_referee referee.launch
```

The public `referee_node.py` loader starts the private `_referee_core.so` business
core. Official gate geometry and its revision are compiled into that core. An
unknown or incompatible world is rejected instead of loading customer-provided
checkpoint data.

The core automatically identifies the actual course and vehicle through Gazebo
model properties and model states. Shell world/robot variables and legacy launch
hints do not select the judged course. The GUI refreshes both names automatically
and provides a Detect Environment button. Only verified `tianracer_racetrack` runs
are eligible for submission; other official courses remain local practice.
Missing or ambiguous models prevent arming, and course changes invalidate an active run.
This is a client gate; independent server track authorization is not implemented.

ROS 1 Noetic is end-of-life; this package maintains the existing simulation stack.
