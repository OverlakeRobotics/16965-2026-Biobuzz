This is the repository for team 16965 for the 2026-2027 FTC season BIOBUZZ. All of the team's code is in /TeamCode, split by category in subfolders.

The game manual is `compmanual.pdf` in the repo root. Read it for the game rules, arena element dimensions and locations, and any numbers the code has to match.

## Build

```
export JAVA_HOME="/Applications/Android Studio.app/Contents/jbr/Contents/Home"
./gradlew :TeamCode:compileDebugJavaWithJavac -q
```

## TeamCode layout

All Java is under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`. Saved Path Planner routes (JSON) live in `TeamCode/src/main/assets/pathJsons/` (last season's are in `decode/`).

### `code/`: current robot code (this season)
- `RobotHardware`: the ONLY place that touches `hardwareMap`. Builds the two shooters (`pollen` = flywheelP/hoodP/blockerP, `nectar` = flywheelN/hoodN/blockerN), the `intake` (intake + transfer motors), the `flower` servo, and (via `createDrivetrain()`) the TeleOp odometry drivetrain with the Pinpoint. Flywheel PIDF (`pollenPIDF`, `nectarPIDF`) is tunable in FTC Dashboard.
- `FieldMap`: field and robot dimensions, the two HIVE base bars (the only obstacles), the CELL aim point (`cellAimX/Y`, dashboard-tunable), `TargetSide` LEFT/RIGHT, `forAlliance()` (blue → red, 180° rotation), and `routeTo()` (Dijkstra route around the bars for the TeleOp presets).
- `ShotTable`: known-good shooter settings (position, both flywheel velocities, both hood angles in degrees) for spots on the field. The preset is the plain dashboard fields `presetX/Y`, `presetPollenVelocity`, etc.; more fixed spots go in `OTHER_POINTS`. Written for ONE quadrant (blue, LEFT, +X +Y) and mirrored to the other three by `toTableQuadrant` / `fromTableQuadrant`.
- `parts/`: mechanisms. `Shooter` (one flywheel + hood + blocker, velocity PIDF, at-speed / below-speed checks, hood angle ↔ servo conversion), `Intake` (intake + transfer, always together), `Flower` (one servo, open/closed).
- `helpers/`: shared logic. `AutoAimer` (active CELL, aim heading, preset pose, closest shot point, position/heading checks), `BaseTeleOp` (all competition TeleOp logic; subclasses only pick alliance and field/robot-centric), `BaseAuto` (follows a Path Planner route with Pedro Pathing and runs route tags; stores `lastPose` for TeleOp).
- `opmodes/`: `ManualTestTeleOp` (temporary; for finding shot-table values by hand), `MotorTester`, `PathPlanner` (runs whatever route is uploaded live through `PathServer`).
- `opmodes/TeleOps/`: `Red`/`Blue` × `Field`/`Robot`-centric TeleOps, each a tiny subclass of `BaseTeleOp`.

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
              │ blue LEFT preset   red RIGHT preset│
              │   (48, 13)            (48, -13)    │
   BLUE       │         ┃ ●          ● ┃          │        RED
   driver ──► │         ┃              ┃          │ ◄── driver
   faces −Y   │    bar  ┃      ·       ┃  bar     │    faces +Y
              │         ┃   (0, 0)     ┃          │
              │         ┃ ●          ● ┃          │
              │ blue RIGHT preset  red LEFT preset │
              │   (-48, 13)           (-48, -13)   │
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
- The shooters fire out the robot's LEFT side, so to shoot the robot faces 90° away from the CELL. At the blue LEFT preset (48, 13) it faces about 90.75° (+Y); its left side points at the −X direction, at the CELL (13.5, 13).
- Pinpoint check: start the robot at (0, 0) facing red (−90°, the Manual Test start). Driving forward should make Y go down, strafing right should make X go down, turning left (CCW) should make H go up.
- LEFT / RIGHT CELL are as the driver sees them, standing on their own alliance's side looking at the HIVE. Blue LEFT is the +X CELL; red LEFT is the −X CELL.
- The active CELL is the one on the robot's half of the field (`AutoAimer.getActiveSide()`); you can't score into the other one from there.
- Shot points and the CELL aim point are written for one quadrant (blue, LEFT: +X, +Y) and mirrored/rotated to the others. Never duplicate them per quadrant.
- Robot center = turning center, defined by the Pedro-tuned Pinpoint pod offsets. Shooter offsets (`AutoAimer.nectarForwardOffset = 3.732`, `pollenForwardOffset = -2.828`) are measured from it.
- Routing avoids ONLY the two HIVE base bars. Keep-out zone = bar grown by robot radius (half diagonal) + 2 in margin, so the robot center staying outside means nothing can hit at any heading.

## Robot behavior (competition TeleOp, gamepad 1)

| Input | Action |
|---|---|
| Sticks | Drive (field- or robot-centric by OpMode). Field-centric forward: blue → −Y (right → −X), red → +Y (`blueFieldCentricOffset = -90`, red + 180). |
| LB / RB | Drive to the LEFT / RIGHT preset around the HIVE bars, arrive facing the CELL, then HOLD that pose (pushes back if bumped) until any stick moves. |
| Hold RT (> 0.3) | Shoot. Blockers open while heading is within `headingTolerance` (3°) AND within `positionTolerance` (6 in) of a shot point. Intake feeds while in position, pauses whenever either flywheel is BELOW speed (over speed is fine). A/B ignored while shooting. |
| B | Toggle intake + transfer |
| Hold A | Reverse intake + transfer |
| X | Toggle flower open (0.47) / closed (0). Starts open at init. |

- Flywheels ALWAYS spin, at the closest shot point's velocities (recomputed every loop). No off switch.
- No aim lock: after a preset the hold keeps the heading; once you drive, turning is manual.
- TeleOp default start (no Auto first): (0, 0), blue facing +Y (90°), red facing −Y (−90°). After Auto, TeleOp starts from `BaseAuto.lastPose` (static, lost on app restart/deploy).
- TeleOp preset driving uses the team's `OdometryHolonomicDrivetrain`, NOT Pedro (tuned by `routeVelocity`, `routeTolerance`, `positionP`). Pedro is only used in Auto.

## How we write code here

- Don't add unnecessary code. No speculative features, no "just in case" branches, no extra abstractions. Remove things that become unused.
- No duplicated code, and it should be quick to find where something lives: hardware in `RobotHardware`, mechanisms in `parts/`, field data in `FieldMap`, shot data in `ShotTable`, logic in `helpers/`, OpModes are thin wrappers.
- Match the surrounding code: comment density, naming, idioms. Comments explain why, in plain words.
- Dashboard-tunable values: use plain `public static` primitive fields in an `@Config` class (last season's pattern). Read them every loop so edits apply live. Avoid nested config objects (see known issues).
- Hood values in shot data are ANGLES in degrees; `Shooter.setHoodAngle` converts to servo position. `ManualTestTeleOp` telemetry shows the angle to copy.
- Both shooters always fire together into the same CELL; they have separate velocities and hoods.
- Ask questions until the requirement is clear before writing a lot of code; give the plan first when asked for one.
- Don't commit or push unless asked.

## Known issues and open items

- Dashboard edits to the old nested `ShotTable.PRESET` object did not change the flywheel targets on the robot; root cause never found. It was switched to flat `preset…` fields (not yet confirmed on the robot). `RobotHardware.pollenPIDF` / `nectarPIDF` are still nested `PIDFCoefficients` objects. If their edits don't apply either, switch them to flat `pollenP/I/D/F` fields. `BaseTeleOp` shows `Runtime (s)` so you can tell whether telemetry is live.
- Flywheels were measured at ~1400/1440 against a 1500 target, so both read FLYWHEEL LOW and RT never feeds. Raise F (then P) in the flywheel PIDF.
- Telemetry "Ready to shoot" uses `AutoAimer.readyToShoot()` (both flywheels within tolerance either way + heading, no position check). That's not the same rule RT uses. Auto also uses `readyToShoot()` and has no position check.
- `positionP` is only sent to the motors in `init()`; dashboard changes need an OpMode restart.
- No bulk caching: each flywheel velocity is read up to 4 times per loop, each a separate hub call. Fix if loop time matters: `LynxModule` MANUAL bulk caching + `clearBulkCache()` at the start of each loop.
- The robot config must have a servo named `"flower"` (and motors `intake` and `transfer`); a missing name makes every OpMode fail at init.
- Placeholders still to measure/tune: preset spot (48, 13), all shot-table velocities and hood angles, `Shooter.HOOD_SERVO_RANGE_DEGREES = 300` (servo model unconfirmed), `Flower` positions.
- CELL aim point (13.5, 13) was estimated from a top-down field image (±1 in). From the presets, `cellAimY` matters a lot (1 in ≈ 1.7°), `cellAimX` barely matters. The two shooters land about ±3.3 in either side of the aim point.
- `FieldMap.ROBOT_LENGTH/WIDTH` are 18 × 18 but the comment says "CAD" (CAD was 15.118 × 16.378). Confirm which is intended; it changes the keep-out zones.
- Unused right now: `FieldMap.Rect.intersects`, `FieldMap.HIVE_PIVOT_HEIGHT`, `AutoAimer.LAUNCH_HEIGHT` (kept for a future physics shot calculation).
- `OdometryHolonomicDrivetrain` has old TODOs (untested straight-line angle correction, line ~118). Check there first if preset routes curve or wobble.
- No Auto routines exist for this robot yet (only `BaseAuto` and `PathPlanner`).
- The `hive-tip-detection` branch replaces LB/RB with a single Y button driven by `HiveTipDetector`, and adds a flower servo to `ManualTestTeleOp` on RB. Merging it will conflict with `BaseTeleOp` and `ManualTestTeleOp` on master.
