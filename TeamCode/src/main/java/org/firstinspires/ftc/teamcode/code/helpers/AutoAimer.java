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
        Pose2D bluePose = FieldMap.forAlliance(pose.get(), isRed);
        return bluePose.getX(DistanceUnit.INCH) > 0 ? FieldMap.LEFT : FieldMap.RIGHT;
    }

    public FieldMap.Point getAimPoint() {
        return getAimPoint(getActiveSide());
    }

    private FieldMap.Point getAimPoint(FieldMap.TargetSide side) {
        return FieldMap.forAlliance(side.aimPoint, isRed);
    }

    // The preset spot for a side (FieldMap.LEFT or RIGHT), already facing that side's CELL
    public Pose2D getPresetPose(FieldMap.TargetSide side) {
        FieldMap.Point preset = ShotTable.fromTableQuadrant(ShotTable.PRESET.position, side, isRed);
        return new Pose2D(DistanceUnit.INCH, preset.x, preset.y, AngleUnit.DEGREES,
                getAimHeadingFrom(preset.x, preset.y, getAimPoint(side)));
    }

    // ---- heading -----------------------------------------------------------------------------

    // Robot heading (degrees) that points the shooters (left side) at the CELL from where the robot is now
    public double getAimHeading() {
        Pose2D current = pose.get();
        return getAimHeadingFrom(current.getX(DistanceUnit.INCH), current.getY(DistanceUnit.INCH), getAimPoint());
    }

    // With the shooters at the robot center this is just bearing - 90. Since they sit forward/back of
    // center, the robot turns a bit more or less so the line out of the shooters' average position
    // passes through the CELL.
    private double getAimHeadingFrom(double x, double y, FieldMap.Point aim) {
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

    // The ShotTable spot closest to the robot
    public ShotTable.ShotPoint getShot() {
        Pose2D current = pose.get();
        return ShotTable.closest(new FieldMap.Point(current.getX(DistanceUnit.INCH), current.getY(DistanceUnit.INCH)), isRed);
    }

    // Sets both flywheels and hoods to the closest shot point's settings
    public void aimShooters() {
        ShotTable.ShotPoint shot = getShot();
        robot.setShooterVelocities(shot.pollenVelocity, shot.nectarVelocity);
        robot.pollen.setHoodAngle(shot.pollenHoodAngle);
        robot.nectar.setHoodAngle(shot.nectarHoodAngle);
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
