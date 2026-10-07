# Tianracer Simulation Referee

ROS 1 Noetic runtime package for Tianracer races in Gazebo. The referee observes
the simulation; it does not start Gazebo or control the vehicle.

## Requirements and startup

Use the supported ROS2GO environment with Ubuntu 20.04, ROS 1 Noetic and Python
3.8 on x86-64 Linux. Install the dependencies declared in `package.xml` and build
the ROS workspace before starting the referee.

Start the official Gazebo simulation, then run:

```bash
source /opt/ros/noetic/setup.bash
source /path/to/catkin_ws/devel/setup.bash
roslaunch tianracer_sim_referee referee.launch
```

Use the course specified by the event organizer and check the course and vehicle
shown in the GUI. Unsupported environments can be used only for local practice.

## Event binding and racing

1. Click Bind Event, select your event, then open the browser binding page.
   If needed, choose Manual input and verify the event slug or URL.
2. Enter the license code provided by the organizer and confirm the team.
   Binding applies to the ROS2GO computer running the simulation.
3. Confirm that the GUI shows the correct team and an eligible environment.
   A round started without valid binding remains local practice.

Click Prepare when the vehicle is stationary near the start/finish line.
Timing begins when the vehicle starts moving. Complete the required laps and
follow the referee's on-screen instructions to submit an eligible result.

Gazebo pauses do not add to the measured driving time. Follow the official course
in order; a missed checkpoint or reverse crossing does not advance the lap count.
Local practice results cannot be converted into competition results afterward.

## Updates and troubleshooting

After the organizer publishes an update to your checkout's upstream branch, run
`git pull` in the `tianracer` repository and restart the referee. Preserve local
changes when updating; do not reset the checkout to force an update.

- If an update is required, install the published release before starting again.
- If the course or vehicle is unavailable or ambiguous, correct the Gazebo
  environment and click Detect Environment.
- If binding cannot be confirmed, check the network and the event selection.
  A revoked authorization requires the organizer to allow rebinding first.
- Keep the referee open while retrying a failed submission. If the GUI says the
  round is invalid or expired, prepare a new round and run again.

The official competition platform uses HTTPS. Client checks do not provide a
complete guarantee of score authenticity; disputed results require organizer
review under the competition rules.

ROS 1 Noetic is end-of-life; this package maintains the existing simulation stack.
