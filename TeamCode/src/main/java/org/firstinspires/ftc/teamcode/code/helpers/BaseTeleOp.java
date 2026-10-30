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
//   Y              drive to the active side's preset (around the HIVE legs) and face the CELL;
//                  any stick cancels. Afterwards the heading stays locked on the CELL until you turn.
//   Hold A         shoot: open both blockers, feed only when both flywheels are at speed and aimed
//   X              toggle intake + transfer
//   B              reverse intake + transfer (X to stop)
//   D-pad right    flywheels on/off (on by default, always at the AutoAimer speed)
@Config
public abstract class BaseTeleOp extends OpMode {
    public static double driveVelocity = 2800;
    public static double routeVelocity = 2800;
    public static double routeTolerance = 4;     // inches; how close to a waypoint before moving to the next
    public static double positionP = 3;
    public static double stickDeadband = 0.05;
    // Field-centric "forward" for BLUE (degrees); red is this + 180. TODO: check on the field.
    public static double blueFieldCentricOffset = 90;
    // Used when TeleOp is started without running Auto first (blue side; red is rotated). TODO: real start spot.
    public static double defaultStartX = -64;
    public static double defaultStartY = -33;
    public static double defaultStartHeading = -90;

    protected RobotHardware robot;
    protected OdometryHolonomicDrivetrain driveTrain;
    protected AutoAimer autoAimer;

    protected boolean routing = false;       // following the Y route
    protected boolean aimLock = false;       // holding the heading on the CELL
    protected boolean flywheelsOn = true;
    protected boolean intakeOn = false;
    protected boolean intakeReversed = false;
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

    // Y starts a route to the preset; sticks cancel it. When the route finishes, lock the heading on the CELL.
    protected void handleAiming() {
        if (gamepad1.yWasPressed()) {
            driveTrain.setVelocity((int) routeVelocity);
            driveTrain.setTolerance(routeTolerance);
            driveTrain.setPositionDrive(FieldMap.routeTo(driveTrain.getPosition(), autoAimer.getPresetPose()));
            routing = true;
            aimLock = false;
        }

        if (routing && (sticksMoved() || !driveTrain.isDriving())) {
            routing = false;
            aimLock = true;
        }

        if (aimLock && Math.abs(gamepad1.right_stick_x) > stickDeadband) {
            aimLock = false;
        }
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
        if (gamepad1.xWasPressed()) {
            // From reverse, X stops; otherwise it toggles intaking
            intakeOn = !intakeOn && !intakeReversed;
            intakeReversed = false;
        }
        if (gamepad1.bWasPressed()) {
            intakeOn = true;
            intakeReversed = true;
        }
    }

    // Flywheels always track the AutoAimer speed. Holding A opens the blockers and feeds only when ready;
    // otherwise the intake does what X/B set.
    protected void handleShooting() {
        if (gamepad1.dpadRightWasPressed()) {
            flywheelsOn = !flywheelsOn;
        }
        if (flywheelsOn) {
            autoAimer.aimShooters();
        } else {
            robot.setShooterVelocities(0, 0);
        }

        shooting = gamepad1.a;
        if (shooting) {
            robot.openBlockers();
            if (autoAimer.readyToShoot()) {
                robot.intake.in();
            } else {
                robot.intake.stop();
            }
        } else {
            robot.closeBlockers();
            if (!intakeOn) {
                robot.intake.stop();
            } else if (intakeReversed) {
                robot.intake.reverse();
            } else {
                robot.intake.in();
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
        telemetry.addData("Heading error", autoAimer.getHeadingError());
        telemetry.addData("Shot point", autoAimer.getShot().name);
        telemetry.addData("Ready to shoot", autoAimer.readyToShoot());
        robot.pollen.addTelemetry(telemetry, "Pollen");
        robot.nectar.addTelemetry(telemetry, "Nectar");
        telemetry.addData("Intake", !intakeOn ? "OFF" : intakeReversed ? "REVERSE" : "IN");
        telemetry.update();
    }

    private boolean sticksMoved() {
        return Math.abs(gamepad1.left_stick_x) > stickDeadband
                || Math.abs(gamepad1.left_stick_y) > stickDeadband
                || Math.abs(gamepad1.right_stick_x) > stickDeadband;
    }
}
