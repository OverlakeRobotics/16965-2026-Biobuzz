package org.firstinspires.ftc.teamcode.examples;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.code.helpers.AutoAligner;
import org.firstinspires.ftc.teamcode.code.parts.Shooter;
import org.firstinspires.ftc.teamcode.code.parts.Turret;
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointOdometry;
import org.firstinspires.ftc.teamcode.system.BasicHolonomicDrivetrain;
import org.firstinspires.ftc.teamcode.system.OdometryHolonomicDrivetrain;


// Shared code for the shooter TeleOps in this folder (SingleShooterTeleOp, DualShooterTeleOp and
// DualTurretTeleOp). It is TeleOpRobotCentric (same robot-centric drive and intake) plus what all
// the shooter versions need: picking the nearest of the alliance's two goals, running AutoAligner's
// shot math for a shooter mounted anywhere on the robot, and turning the whole robot to aim.
//
// Gamepad 1 controls:
//   Left stick: Drive/strafe (robot centric)      Right stick X: Turn
//   B: Toggle intake on/off                       A (hold): Reverse intake
//   Left trigger (hold): Auto aim at the nearest goal
//   Right trigger (hold): Shoot. Also auto aims. Artifacts are only fed once every shooter is up
//                         to speed and aimed.
//   X: Toggle flywheels on/off (on by default)
//   D-Pad Down: Reset odometry to the alliance start pose (put the robot on its start spot first)
//
// Field coordinates are the Pinpoint's: inches, +x forward, +y left, heading in degrees with CCW
// positive. Robot mount offsets are measured from the point the Pinpoint tracks (the same point
// xOffset/yOffset are measured from), +forward toward the robot's front and +left toward its left.
@Config
public abstract class ShooterTeleOpBase extends OpMode {
    public static double velocity = 2800;

    public static final double yOffset = -156.0; // -168.0 // mm
    public static final double xOffset = 72.0; // -84.0 // mm

    // The goal locations are in GoalConfig.
    // The other goal has to be this much closer (inches) before the robot switches to it, so it
    // doesn't flip back and forth when both are about the same distance away. The goal never
    // switches while shooting.
    public static double goalSwitchHysteresis = 6;

    // If true, the pose is set to the start pose below when START is pressed, like
    // TeleOpRobotCentric. Set it to false when running TeleOp after an auto so the pose is kept.
    public static boolean resetPoseOnStart = true;
    public static double redStartX = -64;
    public static double redStartY = -33;
    public static double redStartHeading = -90;
    public static double blueStartX = -64;
    public static double blueStartY = 33;
    public static double blueStartHeading = 90;

    // Shooting is allowed once the flywheel speed and the aim are within these tolerances. Shots
    // closer than closeDistance (inches) use the close tolerances.
    public static double closeDistance = 110;
    public static double closeShooterTolerance = 150; // ticks/s
    public static double farShooterTolerance = 60; // ticks/s
    public static double closeAimTolerance = 6; // degrees
    public static double farAimTolerance = 2; // degrees

    // Turning the robot to aim. turn (ticks/s) = headingP * error (deg)
    //     + headingD * (target rate - heading rate) (deg/s)
    //     + headingFeedforward * turnTicksPerDegree * target rate (deg/s)
    // headingP matches OdometryHolonomicDrivetrain's heading correction. The feedforward keeps the
    // aim on the goal while driving past it; turnTicksPerDegree matches the drivetrain's
    // COUNTS_PER_DEGREE (see TurnCountsTuner).
    public static double headingP = 50;
    public static double headingD = 0;
    public static double headingFeedforward = 1;
    public static double turnTicksPerDegree = 7.39179;
    public static double maxAimTurnVelocity = 2000;

    // Hood angle is lowered by this many degrees per in/s of velocity toward the goal, same as
    // BaseTeleOp.hoodAngleVelScale.
    public static double hoodAngleVelScale = 0.5;

    public static double intakePower = -1;
    public static double reverseIntakePower = 1;
    public static double feedPower = -1;

    private static final double TRIGGER_THRESHOLD = 0.5;

    // Shot settings by distance from the shooter to the goal. Linear interpolation between rows,
    // clamped at the ends. Rows must be sorted by distance.
    // {distance (in), hood angle (deg, 0 = straight up, 90 = flat), kSlip}
    // TODO: Tune for this robot. The defaults are AutoAligner's tuned points from the old robot,
    //  converted to distance from its goal.
    private static final double[][] SHOT_TABLE = {
            {35, 23, 0.495},
            {52, 35, 0.51},
            {63, 40, 0.5},
            {70, 50, 0.52},
            {82, 50, 0.51},
            {99, 50, 0.5},
            {135, 50, 0.475},
            {145, 50, 0.471},
            {152, 50, 0.468},
            {165, 50, 0.458},
    };

    protected static final class ShotState {
        public double distance; // inches from the shooter to the goal
        public double hoodAngle; // degrees
        public double velocity; // ticks/s; the last good value is kept while there is no solution
        public double leadAngle; // degrees to rotate the aim by when shooting on the move
        public boolean valid; // false if the goal can't be reached from here
    }

    protected OdometryHolonomicDrivetrain driveTrain;
    protected GoBildaPinpointDriver pinpointDriver;

    private DcMotorEx intake;
    private boolean intakeOn = false;
    protected boolean flywheelsOn = true;

    private int targetGoal = -1;

    private double savedGoalX;
    private double savedGoalY;
    private double savedGoalDZ;
    private double savedKSlip;
    private double savedCloseDist;

    protected abstract boolean isRedAlliance();

    // Behavior: Gets the shooter hardware and creates the AutoAligners.
    protected abstract void initShooters();

    // Behavior: Aims, drives and runs the shooters for this loop. Must call driveRobot once.
    // Parameters:
    //      - double forward, strafe, turn: The driver's robot-centric velocities.
    //      - boolean aiming: Whether the driver wants the robot to aim.
    //      - boolean firing: Whether the driver wants to shoot.
    // Returns: Whether every shooter is ready, so the intake can feed artifacts.
    protected abstract boolean updateShooters(double forward, double strafe, double turn, boolean aiming, boolean firing);

    @Override
    public void init() {
        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpointDriver.setOffsets(xOffset, yOffset, DistanceUnit.MM);
        pinpointDriver.recalibrateIMU();
        driveTrain = new OdometryHolonomicDrivetrain(
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight"),
                new GoBildaPinpointOdometry(pinpointDriver)
        );
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        initShooters();
    }

    @Override
    public void start() {
        if (resetPoseOnStart) {
            driveTrain.setPosition(getStartPose());
        }
    }

    @Override
    public void loop() {
        driveTrain.updatePosition();

        if (gamepad1.dpadDownWasPressed()) {
            driveTrain.setPosition(getStartPose());
        }
        if (gamepad1.xWasPressed()) {
            flywheelsOn = !flywheelsOn;
        }

        boolean firing = gamepad1.right_trigger > TRIGGER_THRESHOLD;
        boolean aiming = firing || gamepad1.left_trigger > TRIGGER_THRESHOLD;

        updateTargetGoal(firing);

        Pose2D pos = driveTrain.getPosition();
        telemetry.addData("Position", "X: %.2f, Y: %.2f, H: %.2f",
                pos.getX(DistanceUnit.INCH),
                pos.getY(DistanceUnit.INCH),
                pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Goal", "%s %d (%.1f, %.1f)", isRedAlliance() ? "Red" : "Blue",
                targetGoal + 1, getGoalX(targetGoal), getGoalY(targetGoal));
        telemetry.addData("Flywheels", flywheelsOn ? "ON" : "OFF");
        telemetry.addData("Aiming", aiming);

        // Robot-centric drive, same as TeleOpRobotCentric. The shooter version replaces the turn
        // with its own while aiming.
        boolean readyToFeed = updateShooters(
                -gamepad1.left_stick_y * velocity,
                -gamepad1.left_stick_x * velocity,
                -gamepad1.right_stick_x * velocity,
                aiming,
                firing
        );

        //intake
        // B toggles the intake on/off (-1). Holding A reverses it (1) until released.
        // Holding the right trigger feeds artifacts into the shooters only while they are ready.
        if (gamepad1.bWasPressed()) {
            intakeOn = !intakeOn;
        }
        if (gamepad1.a) {
            intake.setPower(reverseIntakePower);
        } else if (firing) {
            intake.setPower(readyToFeed ? feedPower : 0);
        } else if (intakeOn) {
            intake.setPower(intakePower);
        } else {
            intake.setPower(0);
        }

        telemetry.addData("Ready To Shoot", readyToFeed);

        // Power the motors
        driveTrain.drive();
        telemetry.update();
    }

    // Behavior: Creates an AutoAligner for these TeleOps. The limelight is not used because
    //           AutoAligner only knows one AprilTag per alliance, so aiming uses odometry.
    // Parameters:
    //      - Turret turret: The turret this AutoAligner aims, or null if there is no turret.
    protected AutoAligner createAutoAligner(Turret turret) {
        AutoAligner autoAligner = new AutoAligner(driveTrain, turret, null, isRedAlliance());
        autoAligner.limelightEnabled = false;
        return autoAligner;
    }

    // Behavior: Calculates the hood angle, flywheel velocity and lead angle for one shooter.
    // Parameters:
    //      - AutoAligner autoAligner: This shooter's AutoAligner. Each shooter needs its own because
    //                                 AutoAligner remembers the last velocity it saw.
    //      - double forward, left: Where the shooter (or turret pivot) is on the robot, in inches.
    //      - boolean hasTurret: Whether autoAligner was created with a turret.
    //      - ShotState shot: Where the results are stored.
    protected void solveShot(AutoAligner autoAligner, double forward, double left, boolean hasTurret, ShotState shot) {
        pointAutoAlignerAt(forward, left, hasTurret);
        try {
            shot.distance = autoAligner.getDistanceToGoal();
            AutoAligner.kSlip = interpolateShotTable(shot.distance, 2);

            double hoodAngle = interpolateShotTable(shot.distance, 1);
            if (autoAligner.useShootMove) {
                hoodAngle -= autoAligner.lastVelPar * hoodAngleVelScale;
            }
            shot.hoodAngle = Range.clip(hoodAngle, Shooter.minHoodAngle, Shooter.maxHoodAngle);

            // Returns -1 if there is no solution, and is clipped to maxShooterVelocity if the shot
            // needs more than that.
            double shooterVelocity = autoAligner.getShooterVelocityFromAngle(shot.hoodAngle);
            if (shooterVelocity > 0) {
                shot.velocity = shooterVelocity;
                shot.leadAngle = autoAligner.useShootMove ? autoAligner.angleOffset : 0;
            }
            shot.valid = shooterVelocity > 0 && shooterVelocity < AutoAligner.maxShooterVelocity;
        } finally {
            restoreAutoAligner();
        }
    }

    // Behavior: Gets the robot heading that aims a fixed (turretless) shooter at the goal.
    // Parameters:
    //      - AutoAligner autoAligner: An AutoAligner created without a turret.
    //      - double forward, left: Where the shooter is on the robot, in inches.
    //      - double fireDirection: The direction the shooter fires relative to the robot's front,
    //                              in degrees (CCW positive, 180 = out the back).
    // Returns: The wanted heading in degrees, not including the shot's lead angle.
    protected double getAimHeading(AutoAligner autoAligner, double forward, double left, double fireDirection) {
        pointAutoAlignerAt(forward, left, false);
        try {
            // AutoAligner returns the heading that points the robot's back at the goal.
            return normalize(autoAligner.getDrivetrainAutoAlignAngleNoTurret() + 180 - fireDirection);
        } finally {
            restoreAutoAligner();
        }
    }

    // Behavior: Gets the turret angle that aims a turret at the goal, ignoring the turret's limits.
    // Parameters:
    //      - AutoAligner autoAligner: The turret's AutoAligner.
    //      - double forward, left: Where the turret pivot is on the robot, in inches.
    //      - double zeroDirection: The direction the shooter fires at turret angle 0, relative to the
    //                              robot's front, in degrees (CCW positive, 180 = out the back).
    // Returns: The wanted turret angle in degrees, CCW positive, not including the lead angle.
    protected double getTurretAimAngle(AutoAligner autoAligner, double forward, double left, double zeroDirection) {
        pointAutoAlignerAt(forward, left, true);
        try {
            // AutoAligner's turret angle assumes angle 0 fires out the robot's back.
            return normalize(autoAligner.getRawTurretAutoAlignAngle() - 180 - zeroDirection);
        } finally {
            restoreAutoAligner();
        }
    }

    // Behavior: Gets how fast the direction from a point on the robot to the goal is changing
    //           because the robot is driving.
    // Parameters:
    //      - double forward, left: The point on the robot, in inches.
    // Returns: The rate in degrees per second, CCW positive.
    protected double getGoalBearingRate(double forward, double left) {
        double dx = getGoalX(targetGoal) - getMountX(forward, left);
        double dy = getGoalY(targetGoal) - getMountY(forward, left);
        double distSq = dx * dx + dy * dy;
        if (distSq < 1) {
            return 0;
        }
        return Math.toDegrees((dy * driveTrain.getXVelocity() - dx * driveTrain.getYVelocity()) / distSq);
    }

    // Behavior: Gets the turn velocity that turns the robot to a heading and keeps it there.
    // Parameters:
    //      - double targetHeading: The wanted heading in degrees.
    //      - double targetRate: How fast the wanted heading is changing in degrees per second.
    // Returns: The turn velocity in ticks/s.
    protected double getAimTurnVelocity(double targetHeading, double targetRate) {
        double error = normalize(targetHeading - driveTrain.getPosition().getHeading(AngleUnit.DEGREES));
        double headingRate = pinpointDriver.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
        double turn = headingP * error
                + headingD * (targetRate - headingRate)
                + headingFeedforward * turnTicksPerDegree * targetRate;
        return Range.clip(turn, -maxAimTurnVelocity, maxAimTurnVelocity);
    }

    // Behavior: Sets the robot-centric drive velocity.
    // Parameters:
    //      - double forward, strafe, turn: The velocities in ticks/s.
    //      - boolean prioritizeTurn: If true and the motors would saturate, the forward/strafe
    //                                velocity is scaled down instead of everything, so aiming
    //                                keeps its full turn speed.
    protected void driveRobot(double forward, double strafe, double turn, boolean prioritizeTurn) {
        if (prioritizeTurn) {
            double translationLimit = Math.max(0, BasicHolonomicDrivetrain.MAX_VELOCITY - Math.abs(turn));
            double translation = Math.abs(forward) + Math.abs(strafe);
            if (translation > translationLimit) {
                forward *= translationLimit / translation;
                strafe *= translationLimit / translation;
            }
        }
        driveTrain.setVelocityDrive(forward, strafe, turn);
    }

    // Behavior: Sends a shot to a shooter and opens its blocker while shooting.
    protected void applyShot(Shooter shooter, ShotState shot, boolean firing) {
        shooter.setHoodAngle(shot.hoodAngle);
        shooter.setShooterVelocity(flywheelsOn ? shot.velocity : 0);
        if (firing) {
            shooter.open();
        } else {
            shooter.close();
        }
    }

    // Behavior: Gets whether a shooter is up to speed and aimed. Call after applyShot.
    // Parameters:
    //      - double aimError: How far off the shooter's aim is, in degrees.
    protected boolean isShotReady(Shooter shooter, ShotState shot, double aimError) {
        boolean close = shot.distance < closeDistance;
        double shooterTolerance = close ? closeShooterTolerance : farShooterTolerance;
        double aimTolerance = close ? closeAimTolerance : farAimTolerance;
        double shooterError = Math.abs(shooter.getShooter1Velocity() - shooter.getShooterTargetVelocity());
        return flywheelsOn && shot.valid && shooterError <= shooterTolerance && Math.abs(aimError) <= aimTolerance;
    }

    protected void addShotTelemetry(String name, Shooter shooter, ShotState shot, double aimError, boolean ready) {
        telemetry.addData(name, "%s dist %.1f in, hood %.1f, vel %.0f/%.0f, aim err %.1f, %s",
                shot.valid ? "OK" : "NO SHOT",
                shot.distance,
                shot.hoodAngle,
                shooter.getShooter1Velocity(),
                shooter.getShooterTargetVelocity(),
                aimError,
                ready ? "READY" : "WAIT");
    }

    // Behavior: Points AutoAligner at the target goal as seen from a point on the robot. AutoAligner
    //           keeps its goal in static fields and measures from the robot's center, so the goal is
    //           shifted by the point's offset: (shifted goal - robot center) equals (goal - point).
    //           Always followed by restoreAutoAligner() so other OpModes never see these values.
    // Parameters:
    //      - double forward, left: The point on the robot, in inches.
    //      - boolean hasTurret: If false, forces AutoAligner's close-shot branch so it never reads
    //                           the (missing) turret. That branch only skips a turret rotation
    //                           correction, which is zero without a turret.
    private void pointAutoAlignerAt(double forward, double left, boolean hasTurret) {
        savedGoalX = AutoAligner.goalX;
        savedGoalY = AutoAligner.goalY;
        savedGoalDZ = AutoAligner.goalDZ;
        savedKSlip = AutoAligner.kSlip;
        savedCloseDist = AutoAligner.closeDist;

        Pose2D pos = driveTrain.getPosition();
        AutoAligner.goalX = getGoalX(targetGoal) - (getMountX(forward, left) - pos.getX(DistanceUnit.INCH));
        AutoAligner.goalY = getGoalY(targetGoal) - (getMountY(forward, left) - pos.getY(DistanceUnit.INCH));
        AutoAligner.goalDZ = GoalConfig.goalHeight;
        if (!hasTurret) {
            AutoAligner.closeDist = Double.POSITIVE_INFINITY;
        }
    }

    private void restoreAutoAligner() {
        AutoAligner.goalX = savedGoalX;
        AutoAligner.goalY = savedGoalY;
        AutoAligner.goalDZ = savedGoalDZ;
        AutoAligner.kSlip = savedKSlip;
        AutoAligner.closeDist = savedCloseDist;
    }

    // Behavior: Picks the closer of the alliance's two goals, switching only if the other goal is
    //           goalSwitchHysteresis inches closer.
    // Parameters:
    //      - boolean holdCurrentGoal: If true, keeps the current goal (used while shooting).
    private void updateTargetGoal(boolean holdCurrentGoal) {
        double dist1 = getDistanceToGoal(0);
        double dist2 = getDistanceToGoal(1);
        if (targetGoal < 0) {
            targetGoal = dist1 <= dist2 ? 0 : 1;
        } else if (!holdCurrentGoal) {
            double currentDist = targetGoal == 0 ? dist1 : dist2;
            double otherDist = targetGoal == 0 ? dist2 : dist1;
            if (otherDist < currentDist - goalSwitchHysteresis) {
                targetGoal = 1 - targetGoal;
            }
        }
    }

    private double getDistanceToGoal(int goal) {
        Pose2D pos = driveTrain.getPosition();
        return Math.hypot(getGoalX(goal) - pos.getX(DistanceUnit.INCH), getGoalY(goal) - pos.getY(DistanceUnit.INCH));
    }

    private double getGoalX(int goal) {
        if (isRedAlliance()) {
            return goal == 0 ? GoalConfig.redGoal1X : GoalConfig.redGoal2X;
        }
        return goal == 0 ? GoalConfig.blueGoal1X : GoalConfig.blueGoal2X;
    }

    private double getGoalY(int goal) {
        if (isRedAlliance()) {
            return goal == 0 ? GoalConfig.redGoal1Y : GoalConfig.redGoal2Y;
        }
        return goal == 0 ? GoalConfig.blueGoal1Y : GoalConfig.blueGoal2Y;
    }

    // Behavior: Gets the field x of a point on the robot.
    private double getMountX(double forward, double left) {
        Pose2D pos = driveTrain.getPosition();
        double heading = pos.getHeading(AngleUnit.RADIANS);
        return pos.getX(DistanceUnit.INCH) + forward * Math.cos(heading) - left * Math.sin(heading);
    }

    // Behavior: Gets the field y of a point on the robot.
    private double getMountY(double forward, double left) {
        Pose2D pos = driveTrain.getPosition();
        double heading = pos.getHeading(AngleUnit.RADIANS);
        return pos.getY(DistanceUnit.INCH) + forward * Math.sin(heading) + left * Math.cos(heading);
    }

    private Pose2D getStartPose() {
        if (isRedAlliance()) {
            return new Pose2D(DistanceUnit.INCH, redStartX, redStartY, AngleUnit.DEGREES, redStartHeading);
        }
        return new Pose2D(DistanceUnit.INCH, blueStartX, blueStartY, AngleUnit.DEGREES, blueStartHeading);
    }

    private static double interpolateShotTable(double distance, int column) {
        if (distance <= SHOT_TABLE[0][0]) {
            return SHOT_TABLE[0][column];
        }
        for (int i = 1; i < SHOT_TABLE.length; i++) {
            if (distance <= SHOT_TABLE[i][0]) {
                double t = (distance - SHOT_TABLE[i - 1][0]) / (SHOT_TABLE[i][0] - SHOT_TABLE[i - 1][0]);
                return SHOT_TABLE[i - 1][column] + t * (SHOT_TABLE[i][column] - SHOT_TABLE[i - 1][column]);
            }
        }
        return SHOT_TABLE[SHOT_TABLE.length - 1][column];
    }

    // Behavior: Normalizes an angle to (-180, 180]
    protected static double normalize(double degrees) {
        double angle = degrees % 360;
        if (angle > 180) {
            angle -= 360;
        } else if (angle <= -180) {
            angle += 360;
        }
        return angle;
    }
}
