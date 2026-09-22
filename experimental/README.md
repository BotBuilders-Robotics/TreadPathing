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

## What is missing before this is a real answer

- **Odometry.** `DriveEncoderLocalizer` is differential by construction, and mecanum wheel
  odometry is bad enough that no serious library relies on it. A mecanum user needs dead
  wheels or a Pinpoint, which narrows the audience.
- **Real hardware.** Everything above is a simulator. `lateralMultiplier` in particular is a
  number you can only get from a real strafe test on a real floor.
- **A holonomic follower object.** There is no `Follower` here: the sim drives the legs
  directly. Writing one means either a second follower or a generalised one, which is the
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
