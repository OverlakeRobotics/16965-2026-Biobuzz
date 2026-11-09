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
//   Y              turn in place to the aim heading for where the robot is (the ShotTable's tuned heading,
//                  interpolated), then hold that spot and heading; any stick cancels
//   Guide (center) drive back to the TeleOp start pose around the field elements, then hold there, so the
//                  OpMode can be stopped and re-inited from the same spot; any stick cancels
//   D-pad up/down  pollen target speed +/- velocityStep (hold LT: nectar instead), on top of the ShotTable
//   Hold RT        shoot: blockers open while aimed at the CELL; the intake feeds except while a flywheel
//                  is low
//   B              toggle intake + transfer
//   Hold A         reverse intake + transfer
//   X              toggle the flower mechanism open/closed (starts open)
// The flywheels and hoods are always set by the ShotTable for where the robot is, plus the D-pad offsets.
@Config
public abstract class BaseTeleOp extends OpMode {
    public static double driveVelocity = 2800;
    public static double routeVelocity = 2800;
    public static double routeTolerance = 4;     // inches; how close to a waypoint before moving to the next
    public static double positionP = 3;
    public static double stickDeadband = 0.05;
    public static double velocityStep = 50;      // ticks/s per D-pad press
    // Field-centric "forward" for BLUE (degrees): stick up drives toward -Y and stick right toward -X.
    // Red is this + 180 (stick up toward +Y, stick right toward +X).
    public static double blueFieldCentricOffset = -90;
    // Used when TeleOp is started without running Auto first, and where the guide button drives back to:
    // field center, facing your own HIVE half (blue faces blue; red is rotated, so it faces red)
    public static double defaultStartX = 0;
    public static double defaultStartY = 0;
    public static double defaultStartHeading = 90;

    protected RobotHardware robot;
    protected OdometryHolonomicDrivetrain driveTrain;
    protected AutoAimer autoAimer;

    protected boolean routing = false;       // driving to / holding the start pose, or turning to / holding the aim heading
    protected boolean intakeOn = false;
    protected boolean shooting = false;
    // D-pad changes to the ShotTable velocities (ticks/s), kept for the rest of this TeleOp run
    protected double pollenOffset = 0;
    protected double nectarOffset = 0;

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
        driveTrain.setPosition(BaseAuto.lastPose != null ? BaseAuto.lastPose : getDefaultStartPose());
        // Use the Auto pose once; a second TeleOp run without Auto should fall back to the default
        BaseAuto.lastPose = null;
    }

    @Override
    public void loop() {
        driveTrain.updatePosition();
        handleAiming();
        handleDrive();
        handleIntake();
        handleSpeedAdjust();
        handleShooting();
        driveTrain.drive();
        addTelemetry();
    }

    // Guide starts a route back to the start pose; Y turns in place to the aim heading. Either way the
    // robot then keeps holding that spot and heading (pushing back if bumped) until any stick moves.
    protected void handleAiming() {
        if (gamepad1.guideWasPressed()) {
            routeTo(getDefaultStartPose());
        } else if (gamepad1.yWasPressed()) {
            alignToShot();
        }

        if (routing && sticksMoved()) {
            routing = false;
        }
    }

    private void routeTo(Pose2D target) {
        driveTrain.setVelocity((int) routeVelocity);
        driveTrain.setTolerance(routeTolerance);
        driveTrain.setPositionDrive(FieldMap.routeTo(driveTrain.getPosition(), target));
        routing = true;
    }

    // Only turns: the robot stays where the driver put it, even next to a wall or the HIVE, so this
    // doesn't route (routing would first move it away from anything close)
    private void alignToShot() {
        Pose2D current = driveTrain.getPosition();
        driveTrain.setVelocity((int) routeVelocity);
        driveTrain.setPositionDrive(new Pose2D(DistanceUnit.INCH, current.getX(DistanceUnit.INCH),
                current.getY(DistanceUnit.INCH), AngleUnit.DEGREES, autoAimer.getAimHeading()));
        routing = true;
    }

    private Pose2D getDefaultStartPose() {
        return FieldMap.forAlliance(new Pose2D(DistanceUnit.INCH, defaultStartX, defaultStartY,
                AngleUnit.DEGREES, defaultStartHeading), isRedAlliance());
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

    // D-pad up/down changes pollen's target speed; with LT held it changes nectar's instead
    protected void handleSpeedAdjust() {
        double change = 0;
        if (gamepad1.dpadUpWasPressed()) change += velocityStep;
        if (gamepad1.dpadDownWasPressed()) change -= velocityStep;
        if (change == 0) return;
        if (gamepad1.left_trigger > 0.3) {
            nectarOffset += change;
        } else {
            pollenOffset += change;
        }
    }

    // Flywheels and hoods are always set by the ShotTable for where the robot is (plus the D-pad offsets).
    // Holding RT shoots whenever ready; otherwise the intake does what B/A set.
    protected void handleShooting() {
        autoAimer.aimShooters(pollenOffset, nectarOffset);

        shooting = gamepad1.right_trigger > 0.3;
        if (shooting) {
            // Blockers stay open while aimed; the intake pauses whenever a flywheel is low
            boolean aimed = autoAimer.isHeadingOnTarget();
            if (aimed) {
                robot.openBlockers();
            } else {
                robot.closeBlockers();
            }
            if (aimed && !robot.pollen.isBelowSpeed() && !robot.nectar.isBelowSpeed()) {
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
        telemetry.addData("Ready to shoot", autoAimer.readyToShoot());
        robot.pollen.addTelemetry(telemetry, "Pollen");
        robot.nectar.addTelemetry(telemetry, "Nectar");
        telemetry.addData("Speed offset", "Pollen %+.0f, Nectar %+.0f", pollenOffset, nectarOffset);
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
