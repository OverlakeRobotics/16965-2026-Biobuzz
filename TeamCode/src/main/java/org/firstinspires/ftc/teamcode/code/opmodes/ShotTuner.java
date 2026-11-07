package org.firstinspires.ftc.teamcode.code.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.code.FieldMap;
import org.firstinspires.ftc.teamcode.code.ShotTable;
import org.firstinspires.ftc.teamcode.code.helpers.BaseTeleOp;
import org.firstinspires.ftc.teamcode.code.parts.Shooter;

import java.util.Arrays;

// Tunes the ShotTable spots, on the red alliance's LEFT side (-X). Starts like the red TeleOp: field
// center, facing red (-90 degrees). The robot only drives itself; by hand it can only turn.
//
// Controls (gamepad 1):
//   X / Y              drive to the next / previous ShotTable spot (around the HIVE legs, FLOWERS and
//                      walls), at the ShotTable heading for that spot, then hold there
//   Guide (center)     drive back to the start pose, to stop and re-init from the same spot
//   Right stick        turn in place (still holding the spot), to try shots at other headings
//   A                  switch the D-pad between POLLEN and NECTAR
//   D-pad up/down      flywheel velocity +/- tuneVelocityStep ticks/s, 0 to Shooter.MAX_VELOCITY (hold to repeat)
//   D-pad right/left   hood servo position +/- hoodPositionStep (hold to repeat)
//   B                  toggle intake + transfer (starts off)
// The blockers stay open whenever the flywheels are running at the tuning values (once a spot is picked),
// whatever their measured speed, so the intake toggle is what starts and stops shooting. Each spot starts from
// the ShotTable (interpolated if it isn't tuned yet) and keeps its changes for the rest of the run, so
// going back to a spot picks up where you left off. Telemetry shows the position and heading, and each
// shooter's target velocity, measured velocity and hood: ShotTable.POINTS says how to copy them in.
@Config
@TeleOp(name = "Shot Tuner (Red LEFT)", group = "TeleOp")
public class ShotTuner extends BaseTeleOp {
    public static double tuneVelocityStep = 28;   // ticks/s per step, 1% of Shooter.MAX_VELOCITY
    public static double hoodPositionStep = 0.01;
    public static double turnSpeed = 30;          // degrees/s at full right stick
    public static double repeatDelay = 0.3;       // seconds a D-pad button is held before it starts repeating
    public static double repeatInterval = 0.08;   // seconds between repeated steps

    private static final ShotTable.ShotPoint[] SPOTS = ShotTable.POINTS;

    // This run's settings for each spot, NaN until visited. Velocity in ticks/s, hood as the servo position.
    private final double[] pollenVelocity = nan(SPOTS.length);
    private final double[] nectarVelocity = nan(SPOTS.length);
    private final double[] pollenHood = nan(SPOTS.length);
    private final double[] nectarHood = nan(SPOTS.length);

    private int spot = -1;                 // index into SPOTS, -1 before the first X
    private boolean nectarSelected = false;
    private Pose2D target;                 // where the robot is driving to / holding
    private double heldHeading;
    private boolean arrived = false;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final Stepper velocityStepper = new Stepper();
    private final Stepper hoodStepper = new Stepper();

    @Override
    protected boolean isRedAlliance() {
        return true;
    }

    // Not used: there's no stick driving here
    @Override
    protected boolean isFieldCentric() {
        return false;
    }

    // Always the default start (field center), even after an Auto
    @Override
    public void start() {
        driveTrain.setPosition(getDefaultStartPose());
    }

    @Override
    protected void handleAiming() {
        if (gamepad1.xWasPressed() && spot < SPOTS.length - 1) {
            goToSpot(spot + 1);
        } else if (gamepad1.yWasPressed() && spot > 0) {
            goToSpot(spot - 1);
        } else if (gamepad1.guideWasPressed()) {
            driveTo(getDefaultStartPose());
        }
    }

    private void goToSpot(int index) {
        spot = index;
        FieldMap.Point field = spotOnField(index);
        if (Double.isNaN(pollenVelocity[index])) {
            // First visit: start from what the ShotTable would use here
            ShotTable.ShotPoint shot = ShotTable.at(field, true);
            pollenVelocity[index] = Range.clip(shot.pollenVelocity, 0, Shooter.MAX_VELOCITY);
            nectarVelocity[index] = Range.clip(shot.nectarVelocity, 0, Shooter.MAX_VELOCITY);
            pollenHood[index] = Range.clip(robot.pollen.hoodPositionFor(shot.pollenHoodAngle), 0, 1);
            nectarHood[index] = Range.clip(robot.nectar.hoodPositionFor(shot.nectarHoodAngle), 0, 1);
        }
        driveTo(autoAimer.getAimedPose(field));
    }

    private static FieldMap.Point spotOnField(int index) {
        return ShotTable.fromTableQuadrant(SPOTS[index].position(), FieldMap.LEFT, true);
    }

    private void driveTo(Pose2D pose) {
        routeTo(pose);
        target = pose;
        heldHeading = pose.getHeading(AngleUnit.DEGREES);
        arrived = false;
    }

    // Only turning by hand, and only once the robot has arrived (turning mid-route would change the
    // route). Turning re-sends the hold at the same spot with the new heading.
    @Override
    protected void handleDrive() {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (target == null) return;
        if (!arrived) {
            arrived = !driveTrain.isDriving();
            return;
        }
        double turn = -gamepad1.right_stick_x;
        if (Math.abs(turn) <= stickDeadband) return;
        heldHeading += turn * turnSpeed * dt;
        driveTrain.setPositionDrive(new Pose2D(DistanceUnit.INCH, target.getX(DistanceUnit.INCH),
                target.getY(DistanceUnit.INCH), AngleUnit.DEGREES, heldHeading));
    }

    @Override
    protected void handleIntake() {
        if (gamepad1.bWasPressed()) {
            intakeOn = !intakeOn;
        }
        if (intakeOn) {
            robot.intake.in();
        } else {
            robot.intake.stop();
        }
    }

    @Override
    protected void handleSpeedAdjust() {
        if (gamepad1.aWasPressed()) {
            nectarSelected = !nectarSelected;
        }
        int velocityChange = velocityStepper.step(gamepad1.dpad_up, gamepad1.dpad_down);
        int hoodChange = hoodStepper.step(gamepad1.dpad_right, gamepad1.dpad_left);
        if (spot < 0) return;

        double[] velocity = nectarSelected ? nectarVelocity : pollenVelocity;
        double[] hood = nectarSelected ? nectarHood : pollenHood;
        velocity[spot] = Range.clip(velocity[spot] + velocityChange * tuneVelocityStep, 0, Shooter.MAX_VELOCITY);
        hood[spot] = Range.clip(hood[spot] + hoodChange * hoodPositionStep, 0, 1);
    }

    // Before the first spot the flywheels are off and the blockers stay closed (as built in RobotHardware)
    @Override
    protected void handleShooting() {
        if (spot < 0) return;
        robot.setShooterVelocities(pollenVelocity[spot], nectarVelocity[spot]);
        robot.pollen.setHoodPosition(pollenHood[spot]);
        robot.nectar.setHoodPosition(nectarHood[spot]);

        robot.openBlockers();
    }

    // Only what's needed while tuning: where the robot is, which shooter the D-pad changes, and each
    // shooter's target / measured velocity and hood
    @Override
    protected void addTelemetry() {
        Pose2D pose = driveTrain.getPosition();
        telemetry.addData("Position (Pinpoint)", "X %.1f in, Y %.1f in, heading %.1f deg",
                pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("D-pad controls", nectarSelected ? "NECTAR" : "POLLEN");
        addShooterLine("Pollen", robot.pollen);
        addShooterLine("Nectar", robot.nectar);
        telemetry.addLine("X/Y next/prev spot | A pollen/nectar | D-pad up/down velocity, right/left hood | B intake | R stick turn | Guide start");
        telemetry.update();
    }

    private void addShooterLine(String name, Shooter shooter) {
        telemetry.addData(name, "target %.0f, current %.0f ticks/s, hood %.2f (%.2f deg)",
                shooter.getTargetVelocity(), shooter.getVelocity(), shooter.getHoodPosition(), shooter.getHoodAngle());
    }

    private static double[] nan(int length) {
        double[] values = new double[length];
        Arrays.fill(values, Double.NaN);
        return values;
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
