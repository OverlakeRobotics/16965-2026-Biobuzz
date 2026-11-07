package org.firstinspires.ftc.teamcode.code.helpers;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.code.FieldMap;
import org.firstinspires.ftc.teamcode.code.RobotHardware;
import org.firstinspires.ftc.teamcode.code.ShotTable;

import java.util.function.Supplier;

// Aiming math, shared by TeleOp and Auto: which CELL to shoot at, which way to face, how fast to spin
// each flywheel and where to put each hood. Field positions come from FieldMap, shooter settings from
// ShotTable.
//
// Both shooters face out the robot's LEFT side and fire into the same CELL at the same time.
@Config
public class AutoAimer {
    // Where each shooter's exit sits on the robot, inches in front of the robot center (CAD).
    // Nectar is in front of center and pollen behind it.
    public static double nectarForwardOffset = 3.732;
    public static double pollenForwardOffset = -2.828;
    // Height of the ball's exit above the tiles (CAD), for the shot calculation. Treated as the same for
    // both shooters at every hood angle.
    public static final double LAUNCH_HEIGHT = 15.039;

    // How close the heading has to be to the aim heading to shoot (degrees)
    public static double headingTolerance = 3;

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

    // The CELL to shoot at: the one on the robot's half of the field (the other one can't be scored from here)
    public FieldMap.TargetSide getActiveSide() {
        return sideAt(robotPoint());
    }

    private FieldMap.TargetSide sideAt(FieldMap.Point position) {
        return FieldMap.forAlliance(position, isRed).x > 0 ? FieldMap.LEFT : FieldMap.RIGHT;
    }

    private FieldMap.Point getAimPoint(FieldMap.TargetSide side) {
        return FieldMap.forAlliance(side.aimPoint(), isRed);
    }

    private FieldMap.Point robotPoint() {
        Pose2D current = pose.get();
        return new FieldMap.Point(current.getX(DistanceUnit.INCH), current.getY(DistanceUnit.INCH));
    }

    // A pose at a field position (e.g. a ShotTable spot), turned to shoot from there
    public Pose2D getAimedPose(FieldMap.Point position) {
        return new Pose2D(DistanceUnit.INCH, position.x, position.y, AngleUnit.DEGREES, getAimHeadingAt(position));
    }

    // ---- heading -----------------------------------------------------------------------------

    // Robot heading (degrees) to shoot from where the robot is now
    public double getAimHeading() {
        return getAimHeadingAt(robotPoint());
    }

    // Points the shooters at that half's CELL, then turns by the ShotTable heading offset (how far the
    // tuned headings were turned from that). The offsets are tuned for the LEFT CELL, so on the mirrored
    // RIGHT side the robot turns the other way. Red is rotated, not mirrored, so it turns the same way.
    private double getAimHeadingAt(FieldMap.Point position) {
        FieldMap.TargetSide side = sideAt(position);
        double offset = ShotTable.at(position, isRed).headingOffset();
        return normalize(aimHeadingFrom(position.x, position.y, getAimPoint(side)) + (side == FieldMap.LEFT ? offset : -offset));
    }

    // Heading (degrees) that points the shooters (left side) straight at 'aim' from (x, y). With the
    // shooters at the robot center this is just bearing - 90. Since they sit forward/back of center, the
    // robot turns a bit more or less so the line out of the shooters' average position passes through 'aim'.
    public static double aimHeadingFrom(double x, double y, FieldMap.Point aim) {
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

    // ---- shot settings -----------------------------------------------------------------------

    // ShotTable settings for where the robot is now
    public ShotTable.ShotPoint getShot() {
        return ShotTable.at(robotPoint(), isRed);
    }

    // Sets both flywheels and hoods to the ShotTable settings for where the robot is
    public void aimShooters() {
        aimShooters(0, 0);
    }

    // Same, with each flywheel's velocity raised (or lowered) by an offset in ticks/s
    public void aimShooters(double pollenOffset, double nectarOffset) {
        ShotTable.ShotPoint shot = getShot();
        robot.setShooterVelocities(shot.pollenVelocity + pollenOffset, shot.nectarVelocity + nectarOffset);
        robot.pollen.setHoodAngle(shot.pollenHoodAngle);
        robot.nectar.setHoodAngle(shot.nectarHoodAngle);
    }

    // ---- ready -------------------------------------------------------------------------------

    public boolean readyToShoot() {
        return robot.shootersAtSpeed() && isHeadingOnTarget();
    }

    public static double normalize(double degrees) {
        double a = degrees;
        while (a > 180) a -= 360;
        while (a <= -180) a += 360;
        return a;
    }
}
