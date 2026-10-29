package org.firstinspires.ftc.teamcode.code.helpers;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.code.FieldMap;
import org.firstinspires.ftc.teamcode.code.RobotHardware;

import java.util.function.Supplier;

// Aiming math, shared by TeleOp and Auto: which CELL to shoot at, which way to face, how fast to spin
// each flywheel and where to put each hood. Field positions come from FieldMap.
//
// Both shooters face out the robot's LEFT side and fire into the same CELL at the same time. They sit
// at different points along the robot, so each gets its own distance (and so its own speed and hood).
@Config
public class AutoAimer {
    // Where each shooter's exit sits on the robot, inches from the robot center.
    // Forward = toward the robot's front, left = toward the side the shooters face.
    // Nectar is in front of center and pollen behind it (CAD).
    public static double nectarForwardOffset = 3.732;
    public static double pollenForwardOffset = -2.828;
    public static double shooterLeftOffset = 7;   // TODO: measure
    // Height of the ball's exit above the tiles with the hood at 0 degrees (CAD), for the shot calculation
    public static final double LAUNCH_HEIGHT_AT_ZERO_ANGLE = 15.039;

    // How close the heading has to be to the aim heading to shoot (degrees)
    public static double headingTolerance = 3;

    // PLACEHOLDER shot settings, used until the real calculation exists (see the get*Velocity / get*Hood methods)
    public static double defaultPollenVelocity = 1500;
    public static double defaultNectarVelocity = 1500;
    public static double defaultPollenHood = 0.0;
    public static double defaultNectarHood = 0.0;

    private final RobotHardware robot;
    private final Supplier<Pose2D> pose;
    private boolean isRed;

    // pose: where the robot is right now (driveTrain::getPosition in TeleOp, the follower in Auto)
    public AutoAimer(RobotHardware robot, Supplier<Pose2D> pose, boolean isRed) {
        this.robot = robot;
        this.pose = pose;
        this.isRed = isRed;
    }

    public void setRed(boolean isRed) {
        this.isRed = isRed;
    }

    // ---- target ------------------------------------------------------------------------------

    // The CELL to shoot at. For now: whichever side of the field the robot is on.
    // TODO: when the HIVE tips the scoring CELL changes; replace this with a driver button or detection.
    public FieldMap.TargetSide getActiveSide() {
        Pose2D bluePose = FieldMap.forAlliance(pose.get(), isRed);
        return bluePose.getX(DistanceUnit.INCH) < 0 ? FieldMap.LEFT : FieldMap.RIGHT;
    }

    public FieldMap.Point getAimPoint() {
        return FieldMap.forAlliance(getActiveSide().aimPoint, isRed);
    }

    // The preset spot for the active side, already facing the CELL
    public Pose2D getPresetPose() {
        FieldMap.Point preset = FieldMap.forAlliance(getActiveSide().presetPoint, isRed);
        return new Pose2D(DistanceUnit.INCH, preset.x, preset.y, AngleUnit.DEGREES,
                getAimHeadingFrom(preset.x, preset.y));
    }

    // ---- heading -----------------------------------------------------------------------------

    // Robot heading (degrees) that points the shooters (left side) at the CELL from where the robot is now
    public double getAimHeading() {
        Pose2D current = pose.get();
        return getAimHeadingFrom(current.getX(DistanceUnit.INCH), current.getY(DistanceUnit.INCH));
    }

    // With the shooters at the robot center this is just bearing - 90. Since they sit forward/back of
    // center, the robot turns a bit more or less so the line out of the shooters' average position
    // passes through the CELL.
    private double getAimHeadingFrom(double x, double y) {
        FieldMap.Point aim = getAimPoint();
        double dx = aim.x - x;
        double dy = aim.y - y;
        double distance = Math.max(Math.hypot(dx, dy), 1e-6);
        double bearing = Math.toDegrees(Math.atan2(dy, dx));
        double forwardOffset = (pollenForwardOffset + nectarForwardOffset) / 2;
        double ratio = Math.max(-1, Math.min(1, forwardOffset / distance));
        return normalize(bearing - Math.toDegrees(Math.acos(ratio)));
    }

    public double getHeadingError() {
        return normalize(getAimHeading() - pose.get().getHeading(AngleUnit.DEGREES));
    }

    public boolean isHeadingOnTarget() {
        return Math.abs(getHeadingError()) <= headingTolerance;
    }

    // ---- distance ----------------------------------------------------------------------------

    public double getPollenDistance() {
        return getShooterDistance(pollenForwardOffset);
    }

    public double getNectarDistance() {
        return getShooterDistance(nectarForwardOffset);
    }

    // Distance (inches, along the floor) from a shooter's exit to the CELL
    private double getShooterDistance(double forwardOffset) {
        Pose2D current = pose.get();
        double h = current.getHeading(AngleUnit.RADIANS);
        double shooterX = current.getX(DistanceUnit.INCH) + forwardOffset * Math.cos(h) - shooterLeftOffset * Math.sin(h);
        double shooterY = current.getY(DistanceUnit.INCH) + forwardOffset * Math.sin(h) + shooterLeftOffset * Math.cos(h);
        FieldMap.Point aim = getAimPoint();
        return Math.hypot(aim.x - shooterX, aim.y - shooterY);
    }

    // ---- shot settings (PLACEHOLDERS) ---------------------------------------------------------
    // TODO: replace the insides of these four with the real calculation (lookup table from
    //   ManualTestTeleOp data, or physics). Callers already pass the distance, so nothing else changes.

    // Flywheel velocity (ticks/s) for the pollen shooter at this distance (inches)
    public double getPollenVelocity(double distance) {
        return defaultPollenVelocity;
    }

    public double getNectarVelocity(double distance) {
        return defaultNectarVelocity;
    }

    // Hood servo position (0 to 1) for the pollen shooter at this distance (inches)
    public double getPollenHood(double distance) {
        return defaultPollenHood;
    }

    public double getNectarHood(double distance) {
        return defaultNectarHood;
    }

    // Sets both flywheels and hoods for the current position
    public void aimShooters() {
        double pollenDistance = getPollenDistance();
        double nectarDistance = getNectarDistance();
        robot.setShooterVelocities(getPollenVelocity(pollenDistance), getNectarVelocity(nectarDistance));
        robot.pollen.setHoodPosition(getPollenHood(pollenDistance));
        robot.nectar.setHoodPosition(getNectarHood(nectarDistance));
    }

    // ---- ready -------------------------------------------------------------------------------

    public boolean readyToShoot() {
        return robot.shootersAtSpeed() && isHeadingOnTarget();
    }

    private static double normalize(double degrees) {
        double a = degrees;
        while (a > 180) a -= 360;
        while (a <= -180) a += 360;
        return a;
    }
}
