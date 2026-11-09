package org.firstinspires.ftc.teamcode.code.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.code.FieldMap;
import org.firstinspires.ftc.teamcode.code.RobotHardware;
import org.firstinspires.ftc.teamcode.code.ShotTable;
import org.firstinspires.ftc.teamcode.code.parts.Shooter;
import org.firstinspires.ftc.teamcode.system.OdometryHolonomicDrivetrain;

// Drive anywhere and set the shooters by hand, to find ShotTable values and to test the shooters and
// intake. Drives exactly like the old Manual Test TeleOp: plain robot-centric velocity drive, nothing from
// BaseTeleOp (which also changes how the robot brakes). Red alliance, so the screen matches the ShotTable
// copy rule. Starts on the field center, facing red (-90 degrees).
//
// Controls (gamepad 1):
//   Sticks             drive (robot-centric)
//   A                  switch the D-pad between POLLEN and NECTAR
//   D-pad up/down      flywheel velocity +/- tuneVelocityStep ticks/s, 0 to Shooter.MAX_VELOCITY (hold to repeat)
//   D-pad right/left   hood servo position +/- hoodPositionStep (hold to repeat)
//   B                  toggle intake + transfer (starts off)
//   Hold X             reverse intake + transfer
// The flywheels and hoods stay at the values set here wherever the robot drives; they start at the
// ShotTable values for the start pose. The blockers are always open, so the intake is what starts and
// stops shooting. ShotTable.POINTS says how to copy the telemetry into it.
@Config
@TeleOp(name = "Shot Tuner", group = "TeleOp")
public class ShotTuner extends OpMode {
    public static double driveVelocity = 2800;
    public static double tuneVelocityStep = 28;   // ticks/s per step, 1% of Shooter.MAX_VELOCITY
    public static double hoodPositionStep = 0.01;
    public static double repeatDelay = 0.3;       // seconds a D-pad button is held before it starts repeating
    public static double repeatInterval = 0.08;   // seconds between repeated steps

    private RobotHardware robot;
    private OdometryHolonomicDrivetrain driveTrain;
    private boolean intakeOn = false;
    private boolean nectarSelected = false;

    // The values being tuned: velocities in ticks/s, hoods as servo positions
    private double pollenVelocity, nectarVelocity;
    private double pollenHood, nectarHood;
    private final Stepper velocityStepper = new Stepper();
    private final Stepper hoodStepper = new Stepper();

    @Override
    public void init() {
        // Sends telemetry to the Driver Station and the dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new RobotHardware(hardwareMap);
        driveTrain = robot.createDrivetrain();
    }

    @Override
    public void start() {
        // Start on the field center, facing the red side of the HIVE (-Y in FieldMap)
        driveTrain.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, -90));
        ShotTable.ShotPoint shot = ShotTable.at(new FieldMap.Point(0, 0), true);
        pollenVelocity = Range.clip(shot.pollenVelocity, 0, Shooter.MAX_VELOCITY);
        nectarVelocity = Range.clip(shot.nectarVelocity, 0, Shooter.MAX_VELOCITY);
        pollenHood = Range.clip(robot.pollen.hoodPositionFor(shot.pollenHoodAngle), 0, 1);
        nectarHood = Range.clip(robot.nectar.hoodPositionFor(shot.nectarHoodAngle), 0, 1);
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
        if (gamepad1.x) {
            robot.intake.reverse();
        } else if (intakeOn) {
            robot.intake.in();
        } else {
            robot.intake.stop();
        }

        adjustShooters();
        // Sent every loop, so PIDF edits from the dashboard take effect immediately
        robot.setShooterVelocities(pollenVelocity, nectarVelocity);
        robot.pollen.setHoodPosition(pollenHood);
        robot.nectar.setHoodPosition(nectarHood);
        robot.openBlockers();

        driveTrain.drive();
        addTelemetry();
    }

    private void adjustShooters() {
        if (gamepad1.aWasPressed()) {
            nectarSelected = !nectarSelected;
        }
        int velocityChange = velocityStepper.step(gamepad1.dpad_up, gamepad1.dpad_down);
        int hoodChange = hoodStepper.step(gamepad1.dpad_right, gamepad1.dpad_left);
        if (nectarSelected) {
            nectarVelocity = Range.clip(nectarVelocity + velocityChange * tuneVelocityStep, 0, Shooter.MAX_VELOCITY);
            nectarHood = Range.clip(nectarHood + hoodChange * hoodPositionStep, 0, 1);
        } else {
            pollenVelocity = Range.clip(pollenVelocity + velocityChange * tuneVelocityStep, 0, Shooter.MAX_VELOCITY);
            pollenHood = Range.clip(pollenHood + hoodChange * hoodPositionStep, 0, 1);
        }
    }

    // Only what's needed while tuning: where the robot is, which shooter the D-pad changes, and each
    // shooter's target / measured velocity and hood
    private void addTelemetry() {
        Pose2D pose = driveTrain.getPosition();
        telemetry.addData("Position (Pinpoint)", "X %.1f in, Y %.1f in, heading %.1f deg",
                pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("D-pad controls", nectarSelected ? "NECTAR" : "POLLEN");
        addShooterLine("Pollen", robot.pollen);
        addShooterLine("Nectar", robot.nectar);
        telemetry.addLine("Sticks drive | A pollen/nectar | D-pad up/down velocity, right/left hood | B intake, hold X reverse");
        telemetry.update();
    }

    private void addShooterLine(String name, Shooter shooter) {
        telemetry.addData(name, "target %.0f, current %.0f ticks/s, hood %.2f (%.2f deg)",
                shooter.getTargetVelocity(), shooter.getVelocity(), shooter.getHoodPosition(), shooter.getHoodAngle());
    }

    // Two buttons that step a value up/down: one step when pressed, then repeating while held
    private static class Stepper {
        private final ElapsedTime heldTimer = new ElapsedTime();
        private int direction = 0;
        private double nextStepTime = 0;

        // +1 / -1 on the loops that should take a step, else 0
        int step(boolean plus, boolean minus) {
            int newDirection = plus == minus ? 0 : plus ? 1 : -1;
            if (newDirection != direction) {
                direction = newDirection;
                heldTimer.reset();
                nextStepTime = 0;
            }
            if (direction == 0 || heldTimer.seconds() < nextStepTime) return 0;
            nextStepTime = nextStepTime == 0 ? repeatDelay : nextStepTime + repeatInterval;
            return direction;
        }
    }
}
