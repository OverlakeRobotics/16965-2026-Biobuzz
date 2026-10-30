package org.firstinspires.ftc.teamcode.code;

// Spots on the field where we know good shooter settings (found with ManualTestTeleOp). AutoAimer uses
// the settings of whichever spot is closest to the robot.
//
// Only written for ONE quadrant: the blue half, LEFT CELL side (x < 0). The field is symmetric, so the
// same spots work in the other three quadrants: RIGHT side mirrors x, red rotates 180 degrees.
// toTableQuadrant / fromTableQuadrant do the conversion, so nothing here needs duplicating.
public class ShotTable {
    // Where "go to range" (TeleOp Y) drives. Also a shot point. TODO: measure and tune.
    public static final ShotPoint PRESET = new ShotPoint("Preset", -48, 13, 1500, 1500, 16.2, 17.04);

    // To add a spot: drive there in ManualTestTeleOp, tune until both shooters score, then copy the
    // Position X/Y, both target velocities and both hood angles from telemetry into a new line.
    public static final ShotPoint[] POINTS = {
            PRESET,
    };

    public static final class ShotPoint {
        public final String name;
        public final FieldMap.Point position;   // blue, LEFT side quadrant (inches)
        public final double pollenVelocity, nectarVelocity;   // ticks/s
        public final double pollenHoodAngle, nectarHoodAngle; // degrees

        public ShotPoint(String name, double x, double y, double pollenVelocity, double nectarVelocity,
                         double pollenHoodAngle, double nectarHoodAngle) {
            this.name = name;
            this.position = new FieldMap.Point(x, y);
            this.pollenVelocity = pollenVelocity;
            this.nectarVelocity = nectarVelocity;
            this.pollenHoodAngle = pollenHoodAngle;
            this.nectarHoodAngle = nectarHoodAngle;
        }
    }

    // The shot point closest to where the robot is
    public static ShotPoint closest(FieldMap.Point robotPosition, boolean isRed) {
        FieldMap.Point p = toTableQuadrant(robotPosition, isRed);
        ShotPoint best = POINTS[0];
        for (ShotPoint shot : POINTS) {
            if (shot.position.distanceTo(p) < best.position.distanceTo(p)) best = shot;
        }
        return best;
    }

    // Field position -> the matching spot in the table's quadrant
    public static FieldMap.Point toTableQuadrant(FieldMap.Point fieldPoint, boolean isRed) {
        FieldMap.Point blue = FieldMap.forAlliance(fieldPoint, isRed);
        return new FieldMap.Point(-Math.abs(blue.x), blue.y);
    }

    // Table quadrant position -> the matching field position on the given side and alliance
    public static FieldMap.Point fromTableQuadrant(FieldMap.Point tablePoint, FieldMap.TargetSide side, boolean isRed) {
        FieldMap.Point blue = side == FieldMap.RIGHT ? new FieldMap.Point(-tablePoint.x, tablePoint.y) : tablePoint;
        return FieldMap.forAlliance(blue, isRed);
    }
}
