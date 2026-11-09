This is the repository for team 16965 for the 2026-2027 FTC season BIOBUZZ. All of the team's code is in /TeamCode, split by category in subfolders.

The game manual is `compmanual.pdf` in the repo root. Read it for the game rules, arena element dimensions and locations, and any numbers the code has to match.

## Build

```
export JAVA_HOME="/Applications/Android Studio.app/Contents/jbr/Contents/Home"
./gradlew :TeamCode:compileDebugJavaWithJavac -q
```

## TeamCode layout

All Java is under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`. Saved Path Planner routes (JSON) live in `TeamCode/src/main/assets/pathJsons/`: this season's in `biobuzz/`, last season's in `decode/`. Red routes are the blue ones rotated 180° (`forAlliance`).

### `code/`: current robot code (this season)
- `RobotHardware`: the ONLY place that touches `hardwareMap`. Builds the two shooters (`pollen` = flywheelP/hoodP/blockerP, `nectar` = flywheelN/hoodN/blockerN), the `intake` (intake + transfer motors), the `flower` (servos `flowerL` and `flowerR`, R reversed), and (via `createDrivetrain()`) the TeleOp odometry drivetrain with the Pinpoint. Flywheel PIDF (`pollenPIDF`, `nectarPIDF`) is tunable in FTC Dashboard.
- `FieldMap`: field and robot dimensions, the obstacles (two HIVE base bars, four FLOWERS, the walls), the CELL aim point (`cellAimX/Y`, dashboard-tunable), `TargetSide` LEFT/RIGHT, `forAlliance()` (blue → red, 180° rotation), and `routeTo()` (Dijkstra route around the obstacles for the guide button).
- `ShotTable`: settings tuned at the spots in `POINTS` (the robot heading that scored, each shooter's velocity and hood angle in degrees; `UNTUNED` where not tuned yet). `at()` interpolates each setting by position (inverse-distance-squared over the spots where it's tuned; exact on a spot), so the shooters and aim heading are set for any position. Distance to the CELL alone wasn't enough: spots the same distance away needed different settings. Copy rule from the ShotTuner screen: x = −X, y = −Y, heading = heading + 180. Written for ONE quadrant (blue, LEFT, +X +Y) and mirrored to the other three by `toTableQuadrant`. Places in the drivers' shooting region still to tune are listed in a TODO above `POINTS`.
- `parts/`: mechanisms. `Shooter` (one flywheel + hood + blocker, velocity PIDF, at-speed / below-speed checks, hood angle ↔ servo conversion), `Intake` (intake + transfer, always together), `Flower` (two servos moving together, open/closed).
- `helpers/`: shared logic. `AutoAimer` (active CELL, aim heading = straight at the CELL + the ShotTable heading offset (mirrored on the RIGHT side), aimed pose at a position, ShotTable settings for the robot's position, heading check), `BaseTeleOp` (all competition TeleOp logic; subclasses only pick alliance and field/robot-centric), `BaseAuto` (follows a Path Planner route with Pedro Pathing and runs route tags; stores `lastPose` for TeleOp).
- `opmodes/`: `ShotTuner` (red; drives exactly like the old `ManualTestTeleOp` it replaced: plain robot-centric velocity drive, NOT a `BaseTeleOp` subclass, so no `positionP` and no Y/guide. D-pad sets velocity/hood by hand, A switches pollen/nectar, B toggles intake, hold X reverses it, blockers always open; telemetry is position/heading, D-pad mode, one line per shooter, controls), `MotorTester`, `PathPlanner` (runs whatever route is uploaded live through `PathServer`).
- `opmodes/TeleOps/`: `Red`/`Blue` × `Field`/`Robot`-centric TeleOps, each a tiny subclass of `BaseTeleOp`.
- `opmodes/Autos/`: `Red`/`Blue` × `BothCells` (shoot RIGHT, GARDEN pickup, shoot LEFT, park) / `OneCell` (shoot RIGHT twice, park) / `DelayedShot` (start on the +X wall at (63, 40), clear of the FLOWER, wait 17 s, shoot LEFT, park) Autos, each a tiny subclass of `BaseAuto` pointing at its `pathJsons/biobuzz/` route.

### `system/`: reusable drivetrain, odometry and path-following library (from earlier seasons)
- `BasicHolonomicDrivetrain`: mecanum motor control (velocity/position/stop), counts-per-inch constants, `setPositionP`.
- `OdometryHolonomicDrivetrain`: adds odometry: heading correction, field-centric driving, go-to-position and waypoint path driving (`setPositionDrive(Pose2D[])`, straight lines between waypoints, re-aimed every loop from the Pinpoint).
- `OdometryModule` and friends, `FollowerOdometryModule` (Pedro follower pose as an `OdometryModule`).
- `PathServer`, `PathRoute`, `PedroPathBuilder`, `PathPlanRunner`: the Overlake Path Planner web app → Pedro paths + Ivy commands + route tags.

### `components/`: `OdometryModule` implementations
- `GoBildaPinpointOdometry` (uses the pod offsets from `pedro/Constants`), `AprilTagOdometry`, `IMUOdometry`.

### `pedro/`: Pedro Pathing configuration and tuning
- `Constants`: tuned Pedro config, including `X_POD_OFFSET_INCHES` / `Y_POD_OFFSET_INCHES`, which define the robot center (= turning center) for all odometry.
- `Tuning` and `procedures/`: AutoTune entry points and tuner procedures.

### `examples/`: basic drive examples (`TeleOpRobotCentric`, `TeleOpFieldCentric`), `PathPlanExample`, `PathPlanLegacy`, and `tuners/`. Not competition code.

### `zOldCode/`: archived code
`decode/` is last season's (DECODE) code: turret shooter, old helpers, autos, TeleOps. Everything in `zOldCode` is `@Disabled`. Reference only; don't build on it.

## Field conventions

- Inches everywhere, origin at the field center. Write converted inch values directly (e.g. `15.039`), never `/ 2.54` or centimeters in code or comments.
- +Y is the BLUE HIVE half, +X is 90° clockwise from +Y looking down. Heading 0 = +X, counter-clockwise positive. Checked on the robot: facing red, forward is −Y and right is −X.
- Everything is written once, for BLUE. Red = `FieldMap.forAlliance()` (x, y, h) → (−x, −y, h + 180).

### Orientation: where the drivers are and which way is which

Top-down view, drawn with blue on the left (same as the team's field map):

```
                               +X
                                ▲
              ┌──────────────────────────────────┐
              │ blue LEFT shot     red RIGHT shot  │
              │   (50, 13)            (50, -13)    │
   BLUE       │         ┃ ●          ● ┃          │        RED
   driver ──► │         ┃              ┃          │ ◄── driver
   faces −Y   │    bar  ┃      ·       ┃  bar     │    faces +Y
              │         ┃   (0, 0)     ┃          │
              │         ┃ ●          ● ┃          │
              │ blue RIGHT shot    red LEFT shot   │
              │   (-50, 13)           (-50, -13)   │
              └──────────────────────────────────┘
      +Y ◄──  (blue HIVE half)      (red HIVE half)  ──► −Y
                                ▼
                               −X
   ┃ = HIVE base bar (y = ±24.73, x from −19.5 to 19.5)    ● = CELL aim points (±13.5, ±13)
```

- The BLUE driver stands on the blue (+Y) side looking toward −Y. Their left is +X, their right is −X.
- The RED driver stands on the red (−Y) side looking toward +Y. Their left is −X, their right is +X.
- Field-centric "stick up" = away from the driver: blue → −Y, red → +Y.
- Robot heading: 90° faces +Y (toward blue), −90° faces −Y (toward red), 0° faces +X, 180° faces −X.
- The shooters fire out the robot's LEFT side, so to shoot the robot faces 90° away from the CELL. At the blue LEFT Auto shot spot (50.1, 13.2) it faces 91.3° (+Y, tuned); its left side points at the −X direction, at the CELL (13.5, 13). The aim heading is the ShotTable's tuned heading, interpolated: up to 10° clockwise of straight at the CELL on the LEFT side (counter-clockwise on the mirrored RIGHT side).
- Pinpoint check: start the robot at (0, 0) facing red (−90°, the Shot Tuner start). Driving forward should make Y go down, strafing right should make X go down, turning left (CCW) should make H go up.
- LEFT / RIGHT CELL are as the driver sees them, standing on their own alliance's side looking at the HIVE. Blue LEFT is the +X CELL; red LEFT is the −X CELL.
- The active CELL is the one on the robot's half of the field (`AutoAimer.getActiveSide()`); you can't score into the other one from there.
- Shot points and the CELL aim point are written for one quadrant (blue, LEFT: +X, +Y) and mirrored/rotated to the others. Never duplicate them per quadrant.
- Robot center = turning center, defined by the Pedro-tuned Pinpoint pod offsets. Shooter offsets (`AutoAimer.nectarForwardOffset = 3.732`, `pollenForwardOffset = -2.828`) are measured from it.
- Routing avoids the two HIVE base bars, the four FLOWERS (on the walls, 24 in from each wall's center: (72, 24), (−72, −24), (24, −72), (−24, 72), about 7 in wide × 5 in out) and the walls. The HIVES hang 30.6 in up, so the robot drives under them. Keep-out zone = obstacle grown by robot radius (half diagonal) + `OBSTACLE_MARGIN` (3 in), so the robot center staying outside means nothing can hit at any heading. Detour waypoints sit `WAYPOINT_CLEARANCE` (4 in, ≥ `routeTolerance`) outside the zones because the drivetrain cuts corners by up to `routeTolerance`. A target inside a keep-out zone isn't driven to.

## Robot behavior (competition TeleOp, gamepad 1)

| Input | Action |
|---|---|
| Sticks | Drive (field- or robot-centric by OpMode). Field-centric forward: blue → −Y (right → −X), red → +Y (`blueFieldCentricOffset = -90`, red + 180). |
| Y | Turn in place to the aim heading for where the robot is (the ShotTable's tuned heading, interpolated), then HOLD that spot and heading until any stick moves. Doesn't route, so it also works next to a wall or the HIVE. |
| Guide (center "mode"/logo button) | Drive back to the TeleOp default start pose ((0, 0), blue 90°, red −90°) around the HIVE bars, then HOLD it until any stick moves. For stopping and re-initing from the same spot during testing. |
| D-pad up / down | Pollen target speed +/- `velocityStep` (50 ticks/s). Hold LT (> 0.3): nectar instead. Offsets add to the ShotTable speeds at every point and last for the TeleOp run. |
| Hold RT (> 0.3) | Shoot. Blockers open while heading is within `headingTolerance` (3°) of the aim heading (the ShotTable heading for where the robot is). Intake feeds while aimed, pauses whenever either flywheel is BELOW speed (over speed is fine). A/B ignored while shooting. |
| B | Toggle intake + transfer |
| Hold A | Reverse intake + transfer |
| X | Toggle flower open (0.47) / closed (0). Starts open at init. |

- Flywheels ALWAYS spin, at the ShotTable velocities interpolated for the robot's position plus the D-pad offsets; hoods follow the ShotTable too (recomputed every loop). No off switch.
- No presets. Y turns to the aim heading once and holds; otherwise turning is manual.
- TeleOp default start (no Auto first): (0, 0), blue facing +Y (90°), red facing −Y (−90°). After Auto, TeleOp starts from `BaseAuto.lastPose` (static, lost on app restart/deploy).
- TeleOp position driving (guide button, Y align) uses the team's `OdometryHolonomicDrivetrain`, NOT Pedro (tuned by `routeVelocity`, `routeTolerance`, `positionP`). Pedro is only used in Auto.

## How we write code here

- Don't add unnecessary code. No speculative features, no "just in case" branches, no extra abstractions. Remove things that become unused.
- No duplicated code, and it should be quick to find where something lives: hardware in `RobotHardware`, mechanisms in `parts/`, field data in `FieldMap`, shot data in `ShotTable`, logic in `helpers/`, OpModes are thin wrappers.
- Match the surrounding code: comment density, naming, idioms. Comments explain why, in plain words.
- Dashboard-tunable values: use plain `public static` primitive fields in an `@Config` class (last season's pattern). Read them every loop so edits apply live. Avoid nested config objects (see known issues).
- Hood values in shot data are ANGLES in degrees; `Shooter.setHoodAngle` converts to servo position. `ShotTuner` telemetry shows the angle to copy.
- Both shooters always fire together into the same CELL; they have separate velocities and hoods.
- Ask questions until the requirement is clear before writing a lot of code; give the plan first when asked for one.
- Don't commit or push unless asked.

## Lessons learned (principles)

- Prove telemetry is live before deciding a change "didn't work". A stopped OpMode leaves the last telemetry on screen; keep a counter like `Runtime (s)` visible.
- Prefer plain `public static` primitive fields for dashboard tuning. Nested config objects have been unreliable for us; don't rely on them for values you need to tune live.
- FTC Dashboard lists `@Config` classes by simple class name, so two classes with the same name (e.g. `code/parts/Shooter` and an old `Shooter`) collide and only one shows up. Edits then go to the wrong class. Keep `@Config` class names unique, or mark old ones `@Disabled` (the dashboard skips those).
- Know which values are live and which are only sent at init (motor PIDF/position P, start poses). Either re-send on change or say in a comment that a restart is needed.
- Telemetry should show the same condition the code acts on. A "ready" readout that uses a different rule than the actual shooting logic is misleading.
- Check the basics before tuning on top of them: confirm the flywheels actually reach their target speed before tuning shot values.
- Read each piece of hardware once per loop and reuse the value (or use bulk caching). Repeated reads cost loop time and can disagree within one loop.
- Every name `RobotHardware` asks for must exist in the robot config. Adding a device affects every OpMode, not just the one that uses it.
- Placeholders are marked `TODO` and treated as guesses until measured on the real field or robot.
- Before tuning a value, work out how sensitive the result is to it, and spend effort on the values that matter.
- Comments must match the values next to them. When a value changes, update its comment (e.g. where a number came from).
- Remove code that becomes unused instead of leaving it around.
- Settle conventions (axes, headings, LEFT/RIGHT) by testing on the robot, then write them down here.
- When behavior from the older `system/` library looks wrong, check its TODOs first.
- Big control or behavior changes on a branch will conflict with master; agree on them with the team before building.
