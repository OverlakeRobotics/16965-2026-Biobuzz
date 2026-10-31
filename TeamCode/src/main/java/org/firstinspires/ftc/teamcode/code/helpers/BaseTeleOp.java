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
//   LB / RB        drive to the LEFT / RIGHT preset (as the driver sees it, around the HIVE legs) and
//                  face that CELL, then hold there; any stick cancels
//   Hold RT        shoot: blockers open while aimed at the CELL and within AutoAimer.positionTolerance of a
//                  ShotTable spot; the intake feeds except while a flywheel is low
//   B              toggle intake + transfer
//   Hold A         reverse intake + transfer
//   X              toggle the flower mechanism open/closed (starts open)
// The flywheels always spin at the closest ShotTable point's speeds.
@Config
public abstract class BaseTeleOp extends OpMode {
    public static double driveVelocity = 2800;
    public static double routeVelocity = 2800;
    public static double routeTolerance = 4;     // inches; how close to a waypoint before moving to the next
    public static double positionP = 3;
    public static double stickDeadband = 0.05;
    // Field-centric "forward" for BLUE (degrees): stick up drives toward -Y and stick right toward -X.
    // Red is this + 180 (stick up toward +Y, stick right toward +X).
    public static double blueFieldCentricOffset = -90;
    // Used when TeleOp is started without running Auto first: field center, facing your own HIVE half
    // (blue faces blue; red is rotated, so it faces red)
    public static double defaultStartX = 0;
    public static double defaultStartY = 0;
    public static double defaultStartHeading = 90;

    protected RobotHardware robot;
    protected OdometryHolonomicDrivetrain driveTrain;
    protected AutoAimer autoAimer;

    protected boolean routing = false;       // driving to, or holding at, a preset
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
        handleAiming();
        handleDrive();
        handleIntake();
        handleShooting();
        driveTrain.drive();
        addTelemetry();
    }

    // LB/RB start a route to that side's preset. After arriving the robot keeps holding the preset spot
    // and heading (pushing back if bumped) until any stick moves.
    protected void handleAiming() {
        if (gamepad1.leftBumperWasPressed()) {
            routeToPreset(FieldMap.LEFT);
        } else if (gamepad1.rightBumperWasPressed()) {
            routeToPreset(FieldMap.RIGHT);
        }

        if (routing && sticksMoved()) {
            routing = false;
        }
    }

    private void routeToPreset(FieldMap.TargetSide side) {
        driveTrain.setVelocity((int) routeVelocity);
        driveTrain.setTolerance(routeTolerance);
        driveTrain.setPositionDrive(FieldMap.routeTo(driveTrain.getPosition(), autoAimer.getPresetPose(side)));
        routing = true;
    }

    protected void handleDrive() {
        if (routing) return;   // the drivetrain is following the route

        double forward = -gamepad1.left_stick_y * driveVelocity;
        double strafe = -gamepad1.left_stick_x * driveVelocity;
        double turn = -gamepad1.right_stick_x * driveVelocity;

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
        if (gamepad1.xWasPressed()) {
            robot.flower.toggle();
        }
    }

    // Flywheels and hoods are always set to the closest ShotTable point. Holding RT shoots whenever ready;
    // otherwise the intake does what B/A set.
    protected void handleShooting() {
        autoAimer.aimShooters();

        shooting = gamepad1.right_trigger > 0.3;
        if (shooting) {
            // Blockers stay open while aimed and near a shot spot; the intake pauses whenever a flywheel is low
            boolean inPosition = autoAimer.isHeadingOnTarget() && autoAimer.isAtShotPoint();
            if (inPosition) {
                robot.openBlockers();
            } else {
                robot.closeBlockers();
            }
            if (inPosition && !robot.pollen.isBelowSpeed() && !robot.nectar.isBelowSpeed()) {
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
        telemetry.addData("Runtime (s)", "%.1f", getRuntime());   // keeps counting while telemetry is live
        telemetry.addData("Position", "X: %.1f, Y: %.1f, H: %.1f",
                pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Drive", isFieldCentric() ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
        telemetry.addData("Aim", !routing ? "MANUAL" : driveTrain.isDriving() ? "ROUTING" : "HOLDING");
        telemetry.addData("Target side", autoAimer.getActiveSide().name);
        telemetry.addData("Heading error", autoAimer.getHeadingError());
        telemetry.addData("Shot point", "%s, %.1f in away", autoAimer.getShot().name, autoAimer.getShotDistance());
        telemetry.addData("Ready to shoot", autoAimer.readyToShoot());
        robot.pollen.addTelemetry(telemetry, "Pollen");
        robot.nectar.addTelemetry(telemetry, "Nectar");
        telemetry.addData("Intake", gamepad1.a ? "REVERSE" : intakeOn ? "IN" : "OFF");
        telemetry.addData("Flower", robot.flower.isOpen() ? "OPEN" : "CLOSED");
        telemetry.update();
    }

    private boolean sticksMoved() {
        return Math.abs(gamepad1.left_stick_x) > stickDeadband
                || Math.abs(gamepad1.left_stick_y) > stickDeadband
                || Math.abs(gamepad1.right_stick_x) > stickDeadband;
    }
}
