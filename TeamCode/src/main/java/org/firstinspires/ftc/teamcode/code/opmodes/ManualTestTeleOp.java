package org.firstinspires.ftc.teamcode.code.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.code.RobotHardware;
import org.firstinspires.ftc.teamcode.code.parts.Shooter;
import org.firstinspires.ftc.teamcode.system.OdometryHolonomicDrivetrain;

// Temporary robot-centric TeleOp for testing the shooters/intake by hand. Delete once auto aim works.
//
// Controls (gamepad 1):
//   Sticks             drive (robot-centric)
//   B                  toggle intake + transfer
//   Hold A             reverse intake + transfer
//   Hold LB            d-pad adjusts NECTAR (otherwise POLLEN)
//   D-pad up/down      flywheel target velocity +/- flywheelStep
//   D-pad right/left   hood position +/- hoodStep (hold)
//   Y / X              open / close both blockers
//   RB                 toggle flower servo (picks up from the flower) between flowerDown and flowerUp
//
// FTC Dashboard (http://192.168.43.1:8080/dash): graph "Pollen velocity" vs "Pollen target" (same for
// Nectar) and tune RobotHardware.pollenPIDF / nectarPIDF live.
@Config
@TeleOp(name = "Manual Test TeleOp", group = "TeleOp")
public class ManualTestTeleOp extends OpMode {
    public static double driveVelocity = 2800;
    public static double flywheelStep = 50;
    public static double hoodStep = 0.01;
    public static double flowerDown = 0;
    public static double flowerUp = 0.4;

    private RobotHardware robot;
    private OdometryHolonomicDrivetrain driveTrain;
    private boolean intakeOn = false;
    private boolean blockersOpen = false;
    private Servo flower;
    private boolean flowerRaised = false;

    @Override
    public void init() {
        // Sends telemetry to the Driver Station and the dashboard (where numeric values can be graphed)
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new RobotHardware(hardwareMap);
        driveTrain = robot.createDrivetrain();
        flower = hardwareMap.get(Servo.class, "flower");
    }

    @Override
    public void start() {
        // Start on the field center, facing the red side of the HIVE (-Y in FieldMap)
        driveTrain.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, -90));
    }

    @Override
    public void loop() {
        driveTrain.updatePosition();
        driveTrain.setVelocityDrive(
                -gamepad1.left_stick_y * driveVelocity,
                -gamepad1.left_stick_x * driveVelocity,
                -gamepad1.right_stick_x * driveVelocity
        );

        if (gamepad1.bWasPressed()) {
            intakeOn = !intakeOn;
        }
        if (gamepad1.a) {
            robot.intake.reverse();
        } else if (intakeOn) {
            robot.intake.in();
        } else {
            robot.intake.stop();
        }

        // WasPressed() is only true once per press, so read it once and pass it to the selected shooter
        boolean nectarSelected = gamepad1.left_bumper;
        adjustShooter(nectarSelected ? robot.nectar : robot.pollen,
                gamepad1.dpadUpWasPressed(), gamepad1.dpadDownWasPressed());
        // Re-send every loop so PIDF edits from the dashboard take effect immediately
        robot.setShooterVelocities(robot.pollen.getTargetVelocity(), robot.nectar.getTargetVelocity());

        if (gamepad1.y) {
            blockersOpen = true;
        } else if (gamepad1.x) {
            blockersOpen = false;
        }
        if (blockersOpen) {
            robot.openBlockers();
        } else {
            robot.closeBlockers();
        }

        if (gamepad1.rightBumperWasPressed()) {
            flowerRaised = !flowerRaised;
        }
        flower.setPosition(flowerRaised ? flowerUp : flowerDown);

        driveTrain.drive();

        Pose2D pose = driveTrain.getPosition();
        telemetry.addData("Position", "X: %.1f, Y: %.1f, H: %.1f",
                pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("D-pad controls", nectarSelected ? "NECTAR" : "POLLEN");
        robot.pollen.addTelemetry(telemetry, "Pollen");
        robot.nectar.addTelemetry(telemetry, "Nectar");
        telemetry.addData("Blockers", blockersOpen ? "OPEN" : "CLOSED");
        telemetry.addData("Flower", flowerRaised ? "UP" : "DOWN");
        telemetry.addData("Intake on", intakeOn);
        telemetry.addData("Intake velocity (ticks/s)", "%.0f", robot.intake.getIntakeVelocity());
        telemetry.addData("Transfer velocity (ticks/s)", "%.0f", robot.intake.getTransferVelocity());
        telemetry.update();
    }

    private void adjustShooter(Shooter shooter, boolean velocityUp, boolean velocityDown) {
        if (velocityUp) {
            shooter.setVelocity(shooter.getTargetVelocity() + flywheelStep);
        } else if (velocityDown) {
            shooter.setVelocity(shooter.getTargetVelocity() - flywheelStep);
        }

        if (gamepad1.dpad_right) {
            shooter.setHoodPosition(shooter.getHoodPosition() + hoodStep);
        } else if (gamepad1.dpad_left) {
            shooter.setHoodPosition(shooter.getHoodPosition() - hoodStep);
        }
    }
}
