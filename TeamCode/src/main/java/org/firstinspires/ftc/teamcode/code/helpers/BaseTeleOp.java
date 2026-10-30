package org.firstinspires.ftc.teamcode.code.helpers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.code.FieldMap;
import org.firstinspires.ftc.teamcode.code.RobotHardware;
import org.firstinspires.ftc.teamcode.system.OdometryHolonomicDrivetrain;

// All competition TeleOp logic. The OpModes in opmodes/TeleOps only pick the alliance and whether
// driving is field-centric or robot-centric.
//
// Controls (gamepad 1):
//   Sticks         drive
//   Y              drive to the preset of the CELL that's facing up (from HiveTipDetector, around the
//                  HIVE legs) and face that CELL; any stick cancels. Afterwards the heading stays locked
//                  on the CELL until you turn.
//   Hold RT        shoot: blockers open while aimed at the CELL; the intake feeds except while a flywheel is low
//   B              toggle intake + transfer
//   Hold A         reverse intake + transfer
// The flywheels always spin at the closest ShotTable point's speeds.
@Config
public abstract class BaseTeleOp extends OpMode {
    public static double driveVelocity = 2800;
    public static double routeVelocity = 2800;
    public static double routeTolerance = 4;     // inches; how close to a waypoint before moving to the next
    public static double positionP = 3;
    public static double stickDeadband = 0.05;
    // Field-centric "forward" for BLUE (degrees); red is this + 180. TODO: check on the field.
    public static double blueFieldCentricOffset = 90;
    // Used when TeleOp is started without running Auto first: field center, facing your own HIVE half
    // (blue faces blue; red is rotated, so it faces red)
    public static double defaultStartX = 0;
    public static double defaultStartY = 0;
    public static double defaultStartHeading = 90;

    protected RobotHardware robot;
    protected OdometryHolonomicDrivetrain driveTrain;
    protected AutoAimer autoAimer;
    protected HiveTipDetector hiveTip;

    protected boolean routing = false;       // following a preset route
    protected boolean aimLock = false;       // holding the heading on the CELL
    protected boolean intakeOn = false;
    protected boolean shooting = false;

    protected abstract boolean isRedAlliance();

    protected abstract boolean isFieldCentric();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new RobotHardware(hardwareMap);
        driveTrain = robot.createDrivetrain();
        driveTrain.setPositionP(positionP);
        autoAimer = new AutoAimer(robot, driveTrain::getPosition, isRedAlliance());
        hiveTip = new HiveTipDetector(hardwareMap, driveTrain::getPosition);
    }

    @Override
    public void start() {
        Pose2D startPose = BaseAuto.lastPose != null
                ? BaseAuto.lastPose
                : FieldMap.forAlliance(new Pose2D(DistanceUnit.INCH, defaultStartX, defaultStartY,
                        AngleUnit.DEGREES, defaultStartHeading), isRedAlliance());
        driveTrain.setPosition(startPose);
        // Use the Auto pose once; a second TeleOp run without Auto should fall back to the default
        BaseAuto.lastPose = null;
    }

    @Override
    public void loop() {
        driveTrain.updatePosition();
        hiveTip.update();
        handleAiming();
        handleDrive();
        handleIntake();
        handleShooting();
        driveTrain.drive();
        addTelemetry();
    }

    // Y starts a route to the preset of the CELL that's up (nothing if the tip isn't known); sticks cancel
    // it. When the route finishes, lock the heading on the CELL.
    protected void handleAiming() {
        if (gamepad1.yWasPressed()) {
            FieldMap.TargetSide upSide = hiveTip.getTip(isRedAlliance()).toTargetSide();
            if (upSide != null) {
                routeToPreset(upSide);
            }
        }

        if (routing && (sticksMoved() || !driveTrain.isDriving())) {
            routing = false;
            aimLock = true;
        }

        if (aimLock && Math.abs(gamepad1.right_stick_x) > stickDeadband) {
            aimLock = false;
        }
    }

    private void routeToPreset(FieldMap.TargetSide side) {
        driveTrain.setVelocity((int) routeVelocity);
        driveTrain.setTolerance(routeTolerance);
        driveTrain.setPositionDrive(FieldMap.routeTo(driveTrain.getPosition(), autoAimer.getPresetPose(side)));
        routing = true;
        aimLock = false;
    }

    protected void handleDrive() {
        if (routing) return;   // the drivetrain is following the route

        double forward = -gamepad1.left_stick_y * driveVelocity;
        double strafe = -gamepad1.left_stick_x * driveVelocity;
        double turn = -gamepad1.right_stick_x * driveVelocity;

        if (aimLock) {
            driveTrain.setWantedHeading(autoAimer.getAimHeading());
            turn = driveTrain.getHeadingCorrectionVelocity();
        }

        if (isFieldCentric()) {
            double offset = blueFieldCentricOffset + (isRedAlliance() ? 180 : 0);
            driveTrain.setVelocityDriveFieldCentric(forward, strafe, turn, offset);
        } else {
            driveTrain.setVelocityDrive(forward, strafe, turn);
        }
    }

    protected void handleIntake() {
        if (gamepad1.bWasPressed()) {
            intakeOn = !intakeOn;
        }
    }

    // Flywheels and hoods are always set to the closest ShotTable point. Holding RT shoots whenever ready;
    // otherwise the intake does what B/A set.
    protected void handleShooting() {
        autoAimer.aimShooters();

        shooting = gamepad1.right_trigger > 0.5;
        if (shooting) {
            // Blockers stay open while aimed; the intake pauses whenever a flywheel is low
            if (autoAimer.isHeadingOnTarget()) {
                robot.openBlockers();
            } else {
                robot.closeBlockers();
            }
            if (autoAimer.isHeadingOnTarget() && !robot.pollen.isBelowSpeed() && !robot.nectar.isBelowSpeed()) {
                robot.intake.in();
            } else {
                robot.intake.stop();
            }
        } else {
            robot.closeBlockers();
            if (gamepad1.a) {
                robot.intake.reverse();
            } else if (intakeOn) {
                robot.intake.in();
            } else {
                robot.intake.stop();
            }
        }
    }

    protected void addTelemetry() {
        Pose2D pose = driveTrain.getPosition();
        telemetry.addData("Position", "X: %.1f, Y: %.1f, H: %.1f",
                pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Drive", isFieldCentric() ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
        telemetry.addData("Aim", routing ? "ROUTING" : aimLock ? "LOCKED" : "MANUAL");
        telemetry.addData("Target side", autoAimer.getActiveSide().name);
        hiveTip.addTelemetry(telemetry, isRedAlliance());
        telemetry.addData("Heading error", autoAimer.getHeadingError());
        telemetry.addData("Shot point", autoAimer.getShot().name);
        telemetry.addData("Ready to shoot", autoAimer.readyToShoot());
        robot.pollen.addTelemetry(telemetry, "Pollen");
        robot.nectar.addTelemetry(telemetry, "Nectar");
        telemetry.addData("Intake", gamepad1.a ? "REVERSE" : intakeOn ? "IN" : "OFF");
        telemetry.update();
    }

    @Override
    public void stop() {
        hiveTip.close();
    }

    private boolean sticksMoved() {
        return Math.abs(gamepad1.left_stick_x) > stickDeadband
                || Math.abs(gamepad1.left_stick_y) > stickDeadband
                || Math.abs(gamepad1.right_stick_x) > stickDeadband;
    }
}
