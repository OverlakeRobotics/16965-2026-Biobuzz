This is the repository for team 16965 for the 2026-2027 FTC season Biobuzz. The repo includes all the team's code  in the folder /TeamCode, split by category in subfolders. 

## TeamCode layout

All Java is under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`. Other resources: `TeamCode/src/main/assets/pathJsons/decode/` holds saved Path Planner routes (JSON) used by autos. Field coordinates are inches with the origin at field center; see `code/FieldMap.java` for axes.

### `code/` — current robot code (this season)
- `RobotHardware` — builds and owns all mechanisms from the hardwareMap: two shooters (`pollen` = flywheelP/hoodP/blockerP, `nectar` = flywheelN/hoodN/blockerN), the `intake`, and (via `createDrivetrain()`) the TeleOp odometry drivetrain with the Pinpoint. Also has helpers to set both shooters/blockers at once. Flywheel PIDF is tunable in FTC Dashboard.
- `FieldMap` — field/robot dimensions, HIVE obstacle rectangles, the LEFT/RIGHT CELL aim points (blue side), and `forAlliance()` to convert blue values to red (180° rotation).
- `ShotTable` — known-good shooter settings (position, flywheel velocities, hood angles) for spots on the field, written for one quadrant and mirrored to the others. Includes the `PRESET` spot the TeleOp drives to.
- `parts/` — mechanisms: `Shooter` (one flywheel + hood + blocker, velocity PIDF, "at speed" check) and `Intake` (intake + transfer motors that always run together).
- `helpers/` — shared logic: `AutoAimer` (which CELL to target, aim heading, flywheel speed and hood from `ShotTable`), `BaseTeleOp` (all competition TeleOp logic; subclasses only pick alliance and field/robot-centric), and `BaseAuto` (follows a Path Planner route with Pedro Pathing and runs route tags such as intake/autoAim/launch; also stores `lastPose` for TeleOp to start from).
- `opmodes/` — runnable OpModes: `ManualTestTeleOp` (temporary manual shooter/intake tuning for finding shot-table values), `MotorTester` (spin one named motor from Dashboard), `PathPlanner` (autonomous that runs whatever route is uploaded live through `PathServer`).
- `opmodes/TeleOps/` — `Red`/`Blue` × `Field`/`Robot`-centric TeleOps; each is a tiny subclass of `BaseTeleOp`.

### `system/` — reusable drivetrain, odometry and path-following library
- `BasicHolonomicDrivetrain` — mecanum motor control (velocity/position/stop) and counts-per-inch constants.
- `OdometryHolonomicDrivetrain` — extends the above with odometry: heading correction, turn-to-angle, field-centric driving, go-to-position and path driving.
- `OdometryModule` — interface for anything that reports robot pose; `OdometryCollection` fuses several modules by priority; `OdometryModuleLocalizer` adapts a module to a Pedro `Localizer`; `FollowerOdometryModule` does the reverse (exposes a Pedro `Follower`'s pose as an `OdometryModule`).
- `PathServer` — HTTP server on the robot that the Overlake Robotics Path Planner web app uploads routes to (and reads robot pose/config from). `PathRoute` parses the planner's JSON (versions 1 and 2). `PedroPathBuilder` converts a route into Pedro paths split into chunks at stops. `PathPlanRunner` turns those chunks into one Ivy command and dispatches tags (built-in `velocity` and `pause`; the rest go to the OpMode's `TagHandler`).

### `components/` — `OdometryModule` implementations
- `GoBildaPinpointOdometry` (Pinpoint, using the pod offsets from `pedro/Constants`), `AprilTagOdometry` (AprilTag-based pose), `IMUOdometry` (heading only).

### `pedro/` — Pedro Pathing configuration and tuning
- `Constants` — tuned Pedro config (mecanum drive, Pinpoint localizer, Foresight follower, pod offsets) and `create(hardwareMap)` for the `Follower`.
- `Tuning` — AutoTune entry points (run in the order Mecanum, Pinpoint, Foresight at http://192.168.43.1:10158 and paste results into `Constants`). `procedures/` holds the tuner/test procedures they use (Mecanum, Pinpoint, Foresight, OTOS, OctoQuad, two/three-wheel, `Tests`).

### `examples/` — example and experimental OpModes (not the competition code)
- `TeleOpRobotCentric`, `TeleOpFieldCentric` — basic drive examples.
- `PathPlanExample` (Pedro follower + Path Planner) and `PathPlanLegacy` (original point-chasing follower, kept to compare against Pedro).
- `tuners/` — `Forward`/`Strafe`/`TurnCountsTuner` autos for measuring drivetrain counts per inch/degree.
- Shooter TeleOp variants: `ShooterTeleOpBase` (shared goal selection, shot math, aiming, controls), `SingleShooterTeleOp`, `DualShooterTeleOp`, `DualTurretTeleOp`, their `Red`/`Blue` OpMode subclasses, and `GoalConfig` (goal locations, placeholders). NOTE: these still import `code.helpers.AutoAligner` and `code.parts.Turret`, which no longer exist in `code/` (the old versions are in `zOldCode/decode`), so they are stale and need updating before they will compile.

### `zOldCode/` — archived code from the previous (DECODE) season and old experiments
Kept for reference only; don't build new features on it. `decode/` has the old parts (`Shooter`, `Turret`, `Intake`, `Potentiometer`, `LEDIndicator`), helpers (`AutoAligner`, `BaseAuto`, `BaseTeleOp`, `HoodZero`, `BlockerZero`, GoBilda `Prism` LED driver) and OpModes (`Autos/`, `TeleOps/`, `PathPlanner`). The top level has old standalone test/example OpModes (sensor, LED, OTOS, color sorting, AprilTag follow, etc.).

The game manual is located at compmanual.pdf in the main directory. Read that for instructions about the game, dimensions/measurements/movement of arena elements and the field, locations of objects, and all logistics and numbers to write code that matches.