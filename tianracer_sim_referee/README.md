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

## Event binding and ROS2GO authorization

Update the competition platform before installing this core. The binding dialog
loads eligible events from `https://race.tianbot.com`, selects a sole event
automatically, and uses discovered events directly. Choose Manual input in the
dropdown to enter another event slug or URL; manual events require verification.
The browser asks for the event's six-character license code and confirms the team.
The client reads the local ROS2GO SN automatically and displays the team name.

Authorization is checked after binding, approximately every 30 seconds, and before
arming. Network errors retain credentials and show an unconfirmed status; an
explicit server 401 invalidates authorization. Administrators can revoke a device
and separately allow rebinding; allowing rebinding does not restore the old token.
Participants must complete binding again. Results started without a valid binding
remain local practice and cannot be made submittable by binding afterward.

Score requests contain only `completedLaps` and `totalTimeUs`, with a device Bearer
token and an idempotency key. The platform determines event, team and device
ownership and checks authorization on every submission. Its race configuration
API is separate from this core's compiled track gate; this release does not claim
server-side proof of the simulated track or hardware authentication.

The bundled core is an x86-64 Linux binary. Source, protocol documentation and
build instructions are maintained in the private `tianbot/judge_system_dev` repo.

ROS 1 Noetic is end-of-life; this package maintains the existing simulation stack.
