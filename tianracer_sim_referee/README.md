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
The platform also checks the published event track before issuing a round permit.

## Event binding and ROS2GO authorization

The referee core and ROS runtime package version are `1.8.0`; the GUI title and
startup log show the core version. Score requests carry the compiled-in header
`X-Tianbot-Referee-Version: 1.8.0`. The matching platform accepts only explicitly
approved versions (currently `1.8.0`), rejecting missing, old or unknown versions
with `426 REFEREE_UPDATE_REQUIRED`. A version declaration is compatibility
metadata, not proof of an authentic referee or score.

After this version has been published to your checkout's upstream branch, run
`git pull` in the `tianracer` repository and restart the referee. No OTA is used.
Restarting discards live upload eligibility; previous rounds cannot be recovered
from history files and must be run again. Keep local changes intact when updating;
do not reset your checkout to force an update.

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

Clicking Prepare requests a server-issued round permit, valid for ten minutes of
real time including waiting, driving and submission. Only a valid five-lap finish
is automatically submitted immediately; unfinished runs remain local. Expired
rounds must start again. The live challenge and upload eligibility stay in memory
and are never restored from history files.

Requests are authenticated using the compiled release signature as well as device
authorization. The server binds each round to its event, stage, team, device,
track and token, checks the deadline, and accepts it only once. Altered or unsigned
results are rejected. Transport uses HTTPS on the official platform. This does not
claim hardware-backed attestation of a participant-controlled computer.

Completed results include a small signed process summary: per-lap times, sampled
distance, time-weighted average speed, sampled maximum speed and sample count.
It uses existing Gazebo data without new subscriptions or trajectory recording.
The summary stays in client memory, is excluded from local history, and is stored
by the platform for administrator review only. It adds no automatic plausibility
penalties and is not proof of simulation authenticity.

The bundled core is an x86-64 Linux binary. Source, protocol documentation and
build instructions are maintained in the private `tianbot/judge_system_dev` repo.

ROS 1 Noetic is end-of-life; this package maintains the existing simulation stack.
