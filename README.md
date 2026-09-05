# Tread Pathing

A path planner and follower for **FIRST Tech Challenge tank drives**, written to run inside
**OnBotJava**. What Pedro Pathing does for mecanum, for differential drive, with no Android
Studio and no Gradle.

Pure Java 7 source you paste into the OnBotJava editor. No jars to upload, no dependencies to
resolve.

---

Three ideas shape everything here:

- **A path is not enough.** The follower needs a time-parameterised trajectory, because
  correction authority is proportional to reference velocity.
- **Heading is not a free parameter.** On a tank robot, heading *is* the direction of travel
  (or its opposite). There is no heading interpolator, because there is nothing to interpolate.
- **No trajectory follower converges at its own endpoint.** Reference velocity is zero there,
  and lateral authority goes with it. Terminal accuracy comes from a separate pose-hold
  primitive, not from the follower.

---

## Coordinate frame

Origin at a field corner, x and y both running 0 to 144 inches, heading 0 pointing along +x,
counter-clockwise positive — the same convention as Pedro Pathing, so field drawings and
mental models carry straight over.

Everything is **inches, radians and seconds**. Every pose you write is where the robot's
**nose** points, which is what you see standing at the field. Reversed segments are handled
for you.

---

## Installing it in OnBotJava

1. In the OnBotJava editor, create the folder path `org/treadpathing/geometry` (type the path
   with slashes in the new-file dialog; folders are created implicitly).
2. Paste in each file from `src/org/treadpathing/...`, keeping the package structure. The tree
   is a **sibling** of `org/firstinspires/ftc/teamcode`, not nested inside it.
3. Copy `quickstart/org/firstinspires/ftc/teamcode/tread/` in the same way.
4. **Build Everything.**
5. If you are using a Pinpoint, restart the robot afterwards so the configuration UI rescans
   for device types, then add it under I2C Devices as *Tread Pinpoint Odometry Computer*.

In Android Studio, drop `src/org/treadpathing` into `TeamCode/src/main/java/` and it works
unchanged.

### Pasting 51 files is miserable, so push them over adb instead

There is no zip import. OnBotJava's upload button takes `.java` files (into whichever folder
you are uploading to, without reconstructing a package tree) and `.jar`/`.aar` archives, which
it extracts into `ExternalLibraries` as compiled libraries rather than editable source. Zip
appears only in the other direction: every Build Everything snapshots the source to
`FIRST/java/srcBackups/`, keeping the last 30, and there is no path back in through the UI.

The OnBotJava source root is `/sdcard/FIRST/java/src/` — the same tree `Datalogger` writes
into — so adb puts the whole library there in one shot, package structure intact:

```
adb connect 192.168.43.1:5555          # or plain USB, then skip this line
adb push src/org/treadpathing \
         /sdcard/FIRST/java/src/org/
adb push quickstart/org/firstinspires/ftc/teamcode/tread \
         /sdcard/FIRST/java/src/org/firstinspires/ftc/teamcode/
```

Note the destinations: the **parent** directory, not the directory being pushed. `adb push`
copies a directory *into* an existing destination, so naming the target itself works on the
first install, when it does not exist yet, and then silently nests a second copy underneath it
on every push after that -- `org/treadpathing/treadpathing/...`. The build fails with dozens of
`duplicate class` errors that say nothing about the actual cause. Pushing the parent is right
both times.

Reload `192.168.43.1:8080` so the editor rescans, then **Build Everything**.

Over USB you can skip the robot's network entirely and drive OnBotJava from a browser on the
laptop, which wants **two** forwards rather than one:

```
adb forward tcp:8080 tcp:8080   # console and editor
adb forward tcp:8081 tcp:8081   # websocket
```

Without 8081 the console loads, the editor does not: it hangs on *Loading OnBotJava* and
throws `Cannot read properties of undefined (reading 'isWebSocketConnected')`, which reads
like a broken install and is not one. Then open `http://localhost:8080` and use the
**OnBotJava** link in the nav — going straight to `/java/editor.html` bounces you back to the
connection page.

### Why not just upload a jar

`build.sh` writes one to `build/jar/treadpathing.jar` and OnBotJava will happily take it.

That jar is compiled against `stubs/`, which are not the SDK. Every signature in there was
transcribed by hand from the FTC SDK sources and javadoc.

### The file-count problem, stated honestly

The library is **51 source files**. OnBotJava compiles them all, in-process, on a Control Hub
with 1 GB of RAM shared with the whole Android system. On a **Control Hub v1.0 running SDK 11.1**,
the library plus the quickstart built in **23 s** from cold and **14 s** for an incremental
rebuild after editing one file. That is one hub with one set of team code also present.

If it is too slow, delete what you do not use. The localizers are independent: keeping only
the one your robot has removes 3 or 4 files, and `LtvUnicycleController` plus `LtvGainTable`
come out cleanly if you stay on Ramsete.

### Java 7, on purpose

OnBotJava's Java 8 support is an **opt-in checkbox** in the editor settings, off by default,
and it only works when the Robot Controller runs Android 7.0 or later. So nothing here uses
lambdas, method references, streams, `java.util.function`, or `java.time` (which is API 26 and
simply absent on the Control Hub's Android 7.1). Callbacks are anonymous inner classes.

`tools/lint_java7.py` enforces this. One stray lambda is a compile error a rookie cannot
diagnose, on a robot, at a competition.

### There is no live dashboard

FTC Dashboard serves its web UI out of Android assets, and OnBotJava explicitly cannot upload
an AAR containing assets. Its `@Config` scan also reads only the APK's own dex, which
OnBotJava code is not in. Panels has the same problem.

So tuning goes through gamepad plus telemetry, and through CSV logs. `Datalogger` writes to
`/sdcard/FIRST/java/src/Datalogs`, which is inside the OnBotJava source tree, so a log appears
in the editor's file browser with a one-click download. Drop it on `visualizer.html` and you
get the plots — offline instead of live, but with a record you can keep between matches.

**Every run writes its own file.** The first square test is `tread_square.txt`, the next is
`tread_square_2.txt`, and so on: a tuning run you cannot repeat is worth more than a tidy
directory. The OpMode's telemetry names the file it opened, which is the one to download.

That does mean the directory only grows, so each tuning OpMode has a `LOGGING` constant at the
top — set it false and the run writes nothing. In your own OpModes, `Datalogger.disabled()`
returns a logger that swallows every write, so the logging can go on or off without the code
around it changing shape.

---

## Writing an auto

```java
Follower follower = Constants.buildFollower(hardwareMap);
follower.setPose(new Pose(9.0, 60.0, 0.0));

Route route = follower.route()
        .splineTo(new Pose(34.0, 60.0, 0.0))
        .splineTo(new Pose(52.0, 84.0, Math.toRadians(70.0)), 1.25, 20.0)  // slow approach
        .marker(0.60, raiseArm)          // fires 60% along, without pausing the drive
        .stopAndHold(1.5)                // this is what makes it accurate
        .action(scoreSample)             // blocks the route, holds position while it runs
        .turnToDegrees(20.0)             // profiled turn in place
        .reversed()                      // cusp: the robot stops and changes direction
        .splineTo(new Pose(20.0, 72.0, Math.toRadians(20.0)))
        .stopAndHold(2.0)
        .build();

follower.follow(route);
while (opModeIsActive() && follower.isBusy()) {
    follower.update();
    telemetry.addLine(follower.telemetry());
    telemetry.update();
}
```

Actions are anonymous classes, not lambdas:

```java
Action raiseArm = new InstantAction() {
    @Override public void execute() { arm.setTargetPosition(HIGH); }
};

Action scoreSample = new Action() {
    @Override public void start() { claw.setPosition(OPEN); }
    @Override public boolean run() { return !arm.isBusy(); }
};
```

### Slowing down for one leg

The third argument to `splineTo` is a speed ceiling in inches per second for the leg **into**
that waypoint, and nothing after it — one careful approach onto a scoring position does not
cost you the rest of the auto. The second argument is the tangent scale, which you have to
spell out to reach the third; `1.25` is the default.

It is a ceiling, not a target: a corner or the wheel-speed limit can still hold the robot
below it. And it is not `constraints()`, which closes the path and therefore stops the robot
where it takes effect. The cap goes into the same forward/backward sweep as every other limit,
so the profile eases down into the slow leg and back up out of it, with no seam.

The visualizer's **max speed** field on a waypoint is the same number, and a capped step shows
its cap in the route list.

### The three things people get wrong

**`stopAndHold` is what makes an auto accurate.** Drive segments get the robot roughly where
it is going, fast. The pose hold is what gets it exactly there. Put one wherever position
matters — before scoring, at the end of the route — and spend your time budget there.

**`reversed()` inserts a cusp, and that is not a bug.** The path being accumulated is closed
off, the profile decelerates to zero, and a new one starts the other way. Every trajectory
begins and ends at rest, so the seam is dynamically feasible. It is the only honest way for a
differential drive to change direction.

**`lineTo` may insert a turn.** A tank robot cannot slide sideways onto a line, so if it is not
already pointing along the line, a turn goes in first. Check the segment count in
`route.summary()` if you are surprised by how long an auto takes.

---

## Building autos in Blocks

`TreadBlocks.java` exposes the route builder to the Blocks editor, so a team that has not
started writing Java can still plan and drive an auto:

```
start route at 9, 60 facing 0
spline to 34, 60 facing 0
spline to slowly 52, 84 facing 70 at 20
stop and hold 1.5
run route
```

Blocks cannot hold a builder and chain onto it, so the fluent API is flattened into statements
against one route the class keeps for you. Inches and degrees throughout, because that is what
the field is marked in and what the planner shows.

`PinpointDriver` exports its pose, velocity and encoder getters the same way, so a Blocks team
with a Pinpoint gets those blocks under **Additional Hardware** without doing anything.

**Markers and actions are deliberately missing.** They take a callback and Blocks has no way to
hand one across. If something has to happen part-way along a route, split it: run the first
half, do the thing, run the second. Clumsier than a marker, and honest about what is possible.

One trap worth knowing if you extend this. The Blocks entry points are `static`, because a
`BlocksOpModeCompanion` gives you nowhere else to put state, and the Robot Controller keeps the
class loaded between runs. Anything you keep there outlives the OpMode, so `start route`
rebuilds the follower from scratch every time rather than reusing one that would still be
holding the last run's pose.

---

## The tuning ladder

Work down it in order — each rung genuinely depends on the ones below. Every OpMode prints the
exact line to paste into `Constants.java`.

| # | Measure | OpMode | Method |
|---|---|---|---|
| 0 | Motor and encoder directions | LocalizationTest | Forward raises x, left raises y, CCW raises heading |
| 1 | `ticksPerInch` | PushTest | Push 96 in along a wall |
| 2 | **Effective** `trackWidth` | SpinTest | 5+ rotations; the answer is 2-4 in wider than the tape measure |
| 3 | `kS`, `kV` | RampTest | Voltage ramp, then fit the log in the visualizer |
| 4 | `kA` | StraightTest | From the time constant of the rise. Start at 0.002 if impatient |
| 5 | `maxVelocity`, `maxAcceleration` | StraightTest | Take 85% of the peak, so the controller has headroom |
| 6 | `maxCentripetalAcceleration` | CircleTest | Raise speed on a fixed radius until the wheels slip; take 80% |
| 7 | Ramsete `b`, `zeta` | SquareTest | Read cross-track error out of the log |
| 8 | Turn gains | TurnTest | 90 and 180, both directions |
| 9 | Pose-hold gains | PoseTest | Push the robot off and watch it come back |

Rungs 0-2 are geometry, 3-6 are physics, 7-9 are gains. A team that stops after rung 6 already
has a working feedforward-only auto.

### A note on rung 2

The effective track width is **wider than the tape measure**, typically by 2 to 4 inches. Skid
steer scrubs: the wheels slide sideways through every turn, so the robot rotates less than pure
rolling predicts. This is a property of the drivetrain, not an error, and pretending otherwise
makes every curve in every auto slightly wrong.

---

## Localization

Three options behind one interface, plus one optional extra. Switch with one line in
`Constants.ODOMETRY`.

| | Needs | Honest assessment |
|---|---|---|
| `DriveEncoderLocalizer` | nothing | Fine on straights; every turn scrubs. Use it to bring the library up, not to win |
| `TwoWheelLocalizer` | 2 pods + IMU | The real target |
| `PinpointLocalizer` | goBILDA Pinpoint | Same accuracy, one I2C read, least code to get wrong |
| `BbrExpanderLocalizer` | BBR Digital Expander | In `optional/`, not the default drop &mdash; see below |

### The optional one

`optional/org/treadpathing/localization/BbrExpanderLocalizer.java` drives the
[BotBuilders Digital Expander](https://expander.buildingblockrobotics.com), which fuses its own
gyro with two dead wheels and returns a field pose in millimetres. It is an adapter, not a
driver: the board's own `BBRDigitalExpander.java` and `BBRRegMap.java` do the talking.

That dependency is why it is not in `src/`. Everything in `src/` compiles against the FTC SDK
and nothing else, and a file there that imports a third-party driver would mean every team
pasting a driver for a board most of them do not own, or watching Build Everything fail on a
library they just installed. Copy it only if you have the board; `build.sh` still compiles it
against a stub.

The pods, their ticks per millimetre and the tracking-point offsets live in the board's own
flash rather than in `Constants` &mdash; configure them once with BBR's tooling. The
adapter waits for the board to report its localizer ready before trusting a reading, for the
same reason `HeadingFuser` waits for the IMU.

**Heading always comes from the IMU.** Deriving it from `(vRight - vLeft) / trackWidth` on a
skid-steer chassis accumulates error on every turn — ten degrees over a thirty-second auto is
routine. Wheels give translation; the IMU gives theta.

The IMU costs about 7 ms because I2C is excluded from bulk reads, so `HeadingFuser` reads it
every Nth loop and integrates wheel-derived rotation in between. Drive encoders can supply that
estimate, so they use a decimation of 4; two pods cannot, so they use 1.

### The loop budget

Every hardware call is a command serialised to the hub's coprocessor: roughly 2 ms over the
Control Hub's internal bus, about 7 ms for anything on I2C. Bulk caching collapses all encoder
reads into one command; `BulkReader` puts every hub in `MANUAL` mode and clears once per loop,
which also guarantees a coherent snapshot — every encoder sampled at the same instant, which
matters when the robot is turning.

| Configuration | Per loop | Rate |
|---|---|---|
| Drive encoders + IMU every loop | ~17 ms | 59 Hz |
| Drive encoders + IMU at 1/4 rate | ~12 ms | 83 Hz |
| Pinpoint | ~15 ms | 67 Hz |

Motor write caching claws back another 2-6 ms: a write is skipped when the command has barely
moved, so most loops on a straight write nothing at all. Design floor is 50 Hz.

Never move hardware calls to a worker thread. The SDK holds a lock per USB device, so extra
threads only interleave and get slower.

---

## Controllers

**Ramsete is the default.** Its gain is a closed form costing one square root per loop:

```
k     = 2 * zeta * sqrt(w_ref^2 + b * v_ref^2)
v     = v_ref * cos(e_theta) + k * e_x
omega = w_ref + k * e_theta + b * v_ref * sinc(e_theta) * e_y
```

That last term is the whole story: cross-track error enters the **angular** command, scaled by
reference speed.

> **Units.** `b` is in rad²/in². WPILib's well-tested 2.0 assumes metres, which is 0.00129 in
> inches — getting this wrong by a factor of 1550 is the most common Ramsete bug in FTC. The
> default here is **0.0025**, deliberately higher than the direct conversion: cross-track
> authority is `b * v`, and an FTC robot running at a third of FRC speeds needs roughly three
> times the `b` for the same authority. `RamseteController.fromMetricB()` exists if you are
> working from the papers.

**LTV is the upgrade.** LQR-optimal gains, tuned by stating error tolerances rather than
picking a number in rad²/in². Its Riccati solutions cannot be computed on a Control Hub inside
an OpMode's init, so they are generated offline by `tools/gen_ltv_table.py` and pasted into
`LtvGainTable`. Switch with `constants.controller(Controller.LTV)`.

Regenerate the table if your loop period is far from 50 Hz, or to change how hard it pushes.
Halving a tolerance roughly doubles the corresponding gain.

**Inner loop.** Per side, `V = kS*ramp(v) + kV*v + kA*a`, scaled by measured battery voltage
each loop. The `ramp()` matters: the textbook `kS*signum(v)` flips discontinuously through zero
and makes the drivetrain buzz at every stop in a route.

Setting `VelocityMode.HUB_PIDF` delegates velocity closure to the hub's onboard PIDF instead,
which runs far faster than the OpMode and sees better encoder data. Never run a Java-side
velocity D term — it differentiates an already-noisy signal.

---

## The visualizer

Open `visualizer.html`. One self-contained file, no install, works offline. On a desktop-sized
window it does not scroll: the field is sized to the viewport and the side panel scrolls inside
itself.

**As a path editor:** click the field to add waypoints, drag to move, drag the nose to aim,
drag the robot itself to set the start. Every route row carries move up, move down and delete.
Live curvature and velocity profile underneath, so you can see *why* a pretty curve crawls — at
0.1 /in curvature with a 14 in track, the wheel-speed limit alone cuts commanded speed by 41%.
Exports a paste-ready `RouteBuilder` chain.

A selected waypoint has a **tangent scale** and a **max speed** — the cap on the leg into it,
described above. The rest lives in the tabs:

| tab | what is in it |
|---|---|
| start | the starting pose, if you would rather type it than drag it |
| robot | length, width, track width |
| limits | velocity, acceleration, deceleration, centripetal, wheel speed |
| java | the export |
| field | the field image, its opacity and rotation, and the grid |
| log | where you drop a datalog |

**Robot length and width are drawing only.** They set the footprint you see and the area you
can grab to drag the robot, and nothing else. **Track width is real maths:** it is what turns
path curvature into left and right wheel speeds, so it moves the wheel-speed limit, the peak
wheel speed readout, and how much a corner costs you. Put in the number your robot actually
runs — the tuned effective track width from rung 2, not the tape measure, or the tool will
promise you corners the robot cannot hold. The wheels are drawn straddling it, so a track width
you typed wrong looks wrong.

**As a log replay:** drop a datalog from OnBotJava on it. A square-test log overlays measured
pose on planned and reports worst and RMS cross-track error; a ramp log fits kS and kV with an
r² so you know whether to trust it.

### The duplication, and what keeps it honest

The spline and profile maths exists twice: Java on the robot, JavaScript in the tool. That was
a deliberate call — the alternative is shipping a toolchain, which defeats the point of an
OnBotJava library. But two implementations drift, and a visualizer that draws a path the robot
will not drive is worse than no visualizer.

So `tools/gen_golden.js` samples the JavaScript into `tools/golden.csv`, and the Java test
suite asserts agreement to 1e-6 on every run. They currently agree to 5e-10. Change either
side without the other and the tests fail.

---

## Building and testing

```
./build.sh
```

Generates the SDK stubs, compiles the library and quickstart, runs the Java 7 lint, regenerates
the golden file, runs the test suite, and rebuilds the visualizer.

The test suite runs entirely on a laptop with no robot and no FTC SDK: `Sim` is a first-order
differential drive with wheel-speed limits, actuator lag and optional odometry noise, and it
implements the same `SegmentHost` interface the real follower does. Every segment type and both
controllers are exercised against it, including a full route end to end.

`stubs/` contains minimal fakes of the FTC SDK classes the hardware layer touches, so `javac`
can type-check it off-robot. They are not the SDK and they never ship.

---

## Known limitations

- **Curvature is zero at every waypoint.** Segments are built with zero second derivatives at
  their ends, which is C² but flattens the path slightly as it passes through a waypoint. It
  costs a little path efficiency and nothing in feasibility.
- **A pure lateral offset converges to about an inch, not to the dot.** The one direction a
  tank robot cannot move is exactly the direction it needs to go; it gets there by arcing out
  and back. Chasing the last inch by raising gains buys nothing and costs the heading.
- **Trajectories are planned, not re-planned.** If the robot enters a segment off its planned
  start, the trajectory is not shifted — the controller pulls the robot onto the planned path
  instead. That keeps the driven path identical to the one in the visualizer.

---

## Layout

```
src/org/treadpathing/
  geometry/      Pose, Vector2, Twist2, MathUtil
  spline/        quintic Hermite splines, arc-length tables
  trajectory/    two-pass parameterizer, constraints, motion profiles
  control/       Ramsete, LTV, feedforward, PID, pose hold
  route/         segments, RouteBuilder, actions, markers
  localization/  three localizers, Pinpoint driver, IMU fusion
  hardware/      bulk reads, tank drivetrain, encoders, constants
  follower/      Follower and FollowerConstants
  util/          clock, datalogger

quickstart/      Constants.java, ten tuning OpModes, an example auto
test/            desktop test suite and the differential-drive simulator
tools/           stub generator, lint, LTV table generator, shared JS maths
stubs/           generated FTC SDK fakes for off-robot compilation
visualizer.html  the path editor and log viewer
```

BSD-3-Clause, same as Pedro Pathing.
