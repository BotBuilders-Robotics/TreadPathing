# A holonomic follower, as an experiment

Branch `mecanum-experiment`. Nothing here ships: `tools/publish.sh` copies a fixed list of
directories and this is not on it.

The question was whether Tread Pathing could grow a mecanum follower later. The way to answer
it was to build one and see what broke. It works — 20 checks, a simulated robot following
strafes and curves with the nose turning independently — and the interesting part is which
half of the library came along unchanged and which half could not.

```
./build.sh          # compiles it and runs both suites
```

## What it does

```java
HolonomicRoute route = HolonomicRoute.builder(start, limits, kinematics)
        .faceAngle(0.0)                  // nose fixed while the robot strafes
        .to(20.0, 80.0)
        .withPlan(HeadingPlans.interpolate(0.0, Math.toRadians(135.0)))
        .to(60.0, 40.0, Math.toRadians(20.0))
        .to(90.0, 96.0, Math.toRadians(80.0))
        .holdFor(0.8)
        .build();
```

From the simulator, on the same 60 inches of travel:

| plan | duration |
|---|---|
| nose along the path | 2.26 s |
| strafing sideways the whole way | 2.41 s |
| driving forward while turning the nose 180° | 3.18 s |

Worst cross-track error 0.9 in, arriving 0.02 in from the planned end, with the peak wheel
speed pinned exactly at the 60 in/s limit and never over it. With 8% of every strafe lost to
roller slip — a systematic error the feedforward cannot know about — the proportional term
absorbs it and the numbers barely move.

## What came along unchanged

The geometry, which was never the differential-drive part:

- `SplinePath`, `SplineSegment`, `QuinticHermite`, `ArcLengthTable` — a path is a path.
- `Pose`, `Vector2`, `MathUtil`.
- `Feedforward`, and most of `DriveConstants` (kS, kV, kA, ticks per inch, voltage).
- Two of the three localizers. The `Localizer` interface already reports lateral velocity, so
  `TwoWheelLocalizer` and `PinpointLocalizer` need no changes at all.

The two-pass velocity sweep also survives as a *shape*: accelerate forward, decelerate
backward, take the lower. Only the ceiling at each station changes.

## What could not come along

**`ChassisSpeeds` is two numbers.** Forward and turn. A mecanum command is three, and a third
number is not an extra field — it is a different type, and `TrajectoryController`,
`PoseHoldController`, `SegmentHost`, `Segment`, `DriveSegment`, `TurnSegment`, `HoldSegment`,
`TankDrive` and `Follower` are all typed to the two-number one. That is the real cost, and it
is structural rather than mathematical.

**Heading becomes a free parameter.** The README's second principle — "there is no heading
interpolator, because there is nothing to interpolate" — stops being true, and something has
to say what the nose does. `HeadingPlan` is that something, with three implementations that
cover what teams actually ask for.

**The speed envelope changes shape.** On a tank drive the wheel-speed limit falls out of
curvature alone. On a mecanum three demands share one budget: forward travel at 1× wheel
speed, sideways travel at `lateralMultiplier` × (the rollers scrub, so 1.0 is always
optimistic), and rotation at `omega × turnRadius` whether or not the robot is moving. So the
fastest legal speed depends on the angle between where the robot is going and where it is
pointing — a quantity a differential drive does not have, because for it that angle is always
zero. Expressing the heading plan as radians per *inch* rather than per second keeps this a
closed-form solve instead of a fixed-point iteration.

**The controller gets simpler, not harder.** Ramsete and LTV exist because a differential
drive cannot correct a sideways error directly — it must steer, and the gain that does that
safely needs a Lyapunov argument or an LQR solution. A mecanum robot corrects a cross-track
error by driving along it. Feedforward plus a proportional term in the field frame is enough,
and 124 lines of `RamseteController` plus 129 of `LtvGainTable` have no counterpart here.

**Cusps stop existing.** `reversed()` and the stop it forces are the tank library's most
distinctive primitive, and a holonomic route has no use for either.

## What it would cost to ship

1314 lines against the library's 5613, and that is the optimistic half — it has no route
markers, no actions, no `TurnSegment`, no tuning OpModes, no Blocks surface, and no visualizer
support. Three routes exist from here:

1. **Parallel stacks.** What this branch is: a second control path sharing the geometry.
   Cheapest to write, and it doubles the number of things that can drift apart. The golden
   file already exists because two implementations of the same maths drift; this would add a
   third and a fourth.
2. **Generalise the command type.** An interface over chassis commands, with the segment
   machinery parameterised on it. Cleanest in principle, and it touches nearly every file in
   `src/`, in Java 7, with no lambdas and no generic sugar worth having — on a codebase whose
   headline constraint is that a team pastes it file by file into a browser editor.
3. **A separate library that depends on the geometry package.** Honest about the split, and it
   halves the paste burden for any given team, since nobody owns both drivetrains.

The file-count problem is the one that decides it. The README already opens with an apology
for 51 files and a section on pushing them over adb because pasting them is miserable. Adding
a drivetrain nobody's robot has to everybody's paste list makes the library's worst property
worse.

## The planner speaks both

`visualizer.html` has a **tank / mecanum** selector in its robot tab, and the choice reaches
further than a drawing:

- The maths switches to the holonomic generator, mirrored in `tools/tread_math.js` the same
  way the tank maths is. Four more golden cases keep the two honest — Java and JavaScript agree
  to 5e-10 on strafes, per-waypoint noses, tangent plans and interpolated turns.
- Each waypoint grows a **nose** control: *follow path*, or *face* an angle. The arrow you drag
  on the field still shapes the path; a dashed second arrow shows where the robot looks, drawn
  only where the two differ.
- The robot is drawn with rollers, straddling the turn radius rather than the track width.
- The readout gains turn rate and the angle between travel and the nose, which is zero on a
  tank drive by definition.
- The **+ reverse/forward** button disappears, because there is no cusp to insert. A tank route
  loaded in mecanum mode keeps its reverse steps, inert, rather than refusing to open.
- The Java export emits the holonomic builder, with the branch named in a comment, because
  code that will not compile against a release is worse than no code.
- Saved routes record the drivetrain and the per-waypoint noses, so loading one switches the
  page to match.

Building the export is what found the last real gap: a route needs a turn on the spot, and no
heading plan can express it — a plan says what the nose does *while travelling*. So the
experiment grew `turnTo`, profiled, and the first version of it ended 21 degrees short because
the leg finished when the *reference* did. `TurnSegment` settles on tolerance for exactly that
reason; so does this now.

## The constants a team would actually write

`MecanumDriveConstants` is the mecanum answer to `DriveConstants`, and it is a separate class
for a reason that shows up in the first line of either file: **a differential drive is two
sides, a mecanum is four corners.** Grouping the corners loses the distinction the drivetrain
is built on, since front-left and back-left turn opposite ways to strafe.

```java
MecanumDriveConstants drive = new MecanumDriveConstants()
        .motors("leftFront", "rightFront", "leftBack", "rightBack")
        .reversed(true, false, true, false)
        .trackWidth(14.0)          // tape measure, left wheel centre to right
        .wheelBase(12.0)           // tape measure, front wheel centre to back
        .lateralMultiplier(1.15)   // from a strafe test, never 1.0 in practice
        .feedforward(0.08, 0.0207, 0.0024)
        .maxWheelVelocity(36.0);
```

The geometry differs twice over. A tank drive is described by one distance and it is not even
a real one -- the effective track width from SpinTest runs two to four inches wider than the
wheels physically are, because skid steer scrubs in the turn. A mecanum takes two lengths a
tape can measure and derives the turn radius from them, then accounts for the scrubbing it
*does* do -- sideways, through the rollers -- in the lateral multiplier.

Everything electrical is identical: kS, kV, kA, the deadband, voltage compensation, the write
cache, and both velocity modes. Those are properties of motors and batteries rather than of
wheels. They are duplicated here rather than shared only because this is an experiment beside
the library instead of inside it, and the size of that duplicated part is itself one of the
measurements this experiment exists to take: roughly half the class.

One failure mode the constants cannot catch is wrong motor *order*. A mecanum with two corners
swapped drives straight perfectly well and strafes the wrong way, which no amount of validation
sees. `validate()` catches the rest in init -- an empty name, a lateral multiplier below 1.0,
a zero length, `HUB_PIDF` without ticks per inch -- and the setter's documentation says to
drive each wheel on its own before trusting the file.

## What a team would write

Two files, beside the tank quickstart they mirror:

- `org/firstinspires/ftc/teamcode/tread/mecanum/MecanumConstants.java` — the only file a team
  edits. Read it next to `quickstart/.../Constants.java` and the experiment is one screen:
  four corners instead of two sides, two tape-measure lengths instead of one effective track
  width, translation and heading gains instead of Ramsete, a turn-rate limit that has no tank
  counterpart, and no `buildFollower`.
- `.../mecanum/ExampleMecanumAuto.java` — a complete autonomous. The route reads the way the
  tank one does and says things a tank route cannot: strafe to the scoring position with the
  nose fixed on the goal, then curve away while the nose comes round.

**The loop underneath does not read that way, and that is the argument.** On the tank side the
whole of `runOpMode` is `follower.update()` inside `while (follower.isBusy())`, because
`Follower` owns the localizer, the clock, the segment cursor and the drivetrain. With no
holonomic follower, the example writes all of it out by hand — reading the localizer, timing
the loop, walking the legs, sampling the trajectory, deciding when a leg is done. It is left
inline rather than tidied into a helper, because that helper *is* the missing follower.

The example's route is not just compiled, it is flown: the suite rebuilds it from the example's
own constants and runs it in the simulator, arriving 0.0 in from the planned end, 1.4° off
heading, never more than 0.7 in off the path, and never asking that robot's 36 in/s wheels for
more than 36 in/s. An example that compiles is not an example that drives.

## The tuning ladder

None of the tank tuning OpModes run on a mecanum: every one starts with
`Constants.buildFollower`, which builds a `TankDrive` out of two named sides. Below that, the
rungs divide three ways.

| rung | on a mecanum |
|---|---|
| 0 LocalizationTest | transfers — it is about the localizer, which does not care about wheels |
| 1 PushTest, ticks per inch | transfers, averaging four wheels rather than two sides |
| **2 SpinTest, track width** | **does not exist.** There is no effective track width to find: you measure track and wheelbase with a tape. The unknown it is replaced by is the lateral multiplier |
| 3 RampTest, kS and kV | **ported**, as `MecanumRampTest` |
| 4–5 StraightTest, kA and limits | **ported**, as `MecanumStraightTest` |
| 6 CircleTest, centripetal | transfers — drive it nose-tangent and it is the same test |
| 7 SquareTest, follower gains | the shape and the datalog transfer, but it tunes Ramsete gains that do not exist here; it would tune translation and heading gains |
| 8 TurnTest | transfers to `turnTo` |
| 9 PoseTest, hold gains | transfers to `HolonomicPoseHold` |

`StrafeTest` is the rung with no tank counterpart. Drive forward for a fixed time at a fixed
power and measure how far the robot went; strafe for the same time at the same power and
measure that. The same wheel speed buys less travel sideways, and the ratio is the multiplier.
Both distances come from the **localizer**, never the drive encoders: strafing turns the wheels
in opposing pairs, so their mean is near zero however far the robot has moved — the encoders
cannot see the thing being measured.

Making any of this possible needed a tuning surface on `MecanumDrive`, which had none. The low
rungs do not use the follower at all: they drive the motors raw and read them back. It now has
per-wheel positions and velocities, raw powers, a named `setStrafePowers` (four signs at a call
site is how you get a strafe test that quietly measures a diagonal), battery voltage, and a
mean forward reading that is documented as meaningless during a strafe for the reason above.

`MecanumRampTest` and `MecanumStraightTest` write the same log columns as their tank
counterparts, so the visualizer fits kS, kV and kA from them with no changes at all.

The straight test carries a caveat the tank one cannot have: its numbers describe the robot
going **forwards**, which is the cheapest direction a mecanum has. Strafing costs the lateral
multiplier times as much wheel speed for the same travel, and turning costs more again, so a
route that strafes will never reach the peak it measures. That is not a hole in the
measurement — `HolonomicConstraints` works in wheel speed and derates translation by the angle
between travel and the nose, so measuring forwards and planning in wheel terms agree. It does
mean the run has to be nose-first: a full-power strafe measures the rollers, not the drivetrain.

That leaves rungs 0, 1, 6, 7, 8 and 9 unported. Each transfers in substance and needs the same
mechanical rework — a mecanum harness in place of `buildFollower` — except SquareTest and
PoseTest, which also tune different gains because there is no Ramsete here.

## What is missing before this is a real answer

- **Odometry.** `DriveEncoderLocalizer` is differential by construction, and mecanum wheel
  odometry is bad enough that no serious library relies on it. A mecanum user needs dead
  wheels or a Pinpoint, which narrows the audience.
- **Real hardware.** Everything above is a simulator. `lateralMultiplier` in particular is a
  number you can only get from a real strafe test on a real floor.
- **A holonomic follower object.** There is no `Follower` here: the sim and the example
  OpMode drive the legs directly. Writing one means either a second follower or a generalised one, which is the
  `ChassisSpeeds` problem again wearing a different hat.
- **Pedro Pathing already exists**, is good at mecanum, and the README recommends it by name.
  The reason this library exists is that nobody served tank drives in OnBotJava. Nobody is
  underserved on the mecanum side.

## Verdict

Possible, and less work than it looks on the maths — the hard parts of a follower (the
geometry, the profile sweep, the feedforward, the localizers) are drivetrain-agnostic and
already written. The obstacle is the command type threaded through every signature, and the
cost is paid in files a team has to paste and numbers a team has to tune, not in algorithms.

If it happens, route 3 — a separate library over a shared geometry package — is the one that
does not make the existing library worse.
