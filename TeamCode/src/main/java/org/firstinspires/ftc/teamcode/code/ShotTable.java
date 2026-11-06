package org.firstinspires.ftc.teamcode.code;

// Shooter settings tuned at spots on the field (with ShotTuner). Between spots the settings are
// interpolated by distance to the CELL, so the shooters are set for wherever the robot is.
//
// Only written for ONE quadrant: the blue half, LEFT CELL side (+X, +Y). The field is symmetric, so the
// same spots work in the other three quadrants: RIGHT side mirrors x, red rotates 180 degrees.
// toTableQuadrant / fromTableQuadrant do the conversion, so nothing here needs duplicating.
public class ShotTable {
    // The spots ShotTuner steps through, closest to the CELL first. A spot with only x, y isn't tuned
    // yet and is left out of the interpolation. To tune one: drive there in ShotTuner, adjust until both
    // shooters score, then copy that spot's "Copy" line from telemetry over its line here.
    // Every spot has to be FieldMap.isClear (the robot can turn all the way around there without hitting
    // the HIVE, a FLOWER or a wall); ShotTuner won't drive to one that isn't.
    // Distances are to the CELL aim point at (13.5, 13). The two 56.3 in spots are the same distance from
    // either side of the CELL: if they need different settings, distance alone isn't enough.
    // TODO: tune all but the Auto spot.
    public static final ShotPoint[] POINTS = {
            new ShotPoint(40, 13),                           // 26.5 in from the CELL; 22.5 in was too close to shoot
            new ShotPoint(43, 13),                           // 29.5
            new ShotPoint(50, 13, 1450, 1900, 16.2, 17.04),  // 36.5, where the Autos shoot
            new ShotPoint(56, 0),                            // 44.4
            new ShotPoint(56, -13),                          // 49.8
            new ShotPoint(56, 50),                           // 56.3, from the +Y side
            new ShotPoint(56, -24),                          // 56.3, from the -Y side
            new ShotPoint(56, -45),                          // 71.9
            new ShotPoint(55, -55),                          // 79.7
    };

    public static final class ShotPoint {
        public final double x, y;                              // blue, LEFT side quadrant (inches)
        public final double pollenVelocity, nectarVelocity;    // ticks/s
        public final double pollenHoodAngle, nectarHoodAngle;  // degrees

        public ShotPoint(double x, double y, double pollenVelocity, double nectarVelocity,
                         double pollenHoodAngle, double nectarHoodAngle) {
            this.x = x;
            this.y = y;
            this.pollenVelocity = pollenVelocity;
            this.nectarVelocity = nectarVelocity;
            this.pollenHoodAngle = pollenHoodAngle;
            this.nectarHoodAngle = nectarHoodAngle;
        }

        // A spot that isn't tuned yet
        public ShotPoint(double x, double y) {
            this(x, y, Double.NaN, Double.NaN, Double.NaN, Double.NaN);
        }

        public boolean isTuned() {
            return !Double.isNaN(pollenVelocity);
        }

        public FieldMap.Point position() {
            return new FieldMap.Point(x, y);
        }

        public double distanceToCell() {
            return position().distanceTo(FieldMap.LEFT.aimPoint());
        }
    }

    // Settings for a robot position, interpolated in a straight line between the tuned spots just closer
    // to and just farther from the CELL. Closer or farther than every tuned spot uses the nearest one.
    public static ShotPoint at(FieldMap.Point robotPosition, boolean isRed) {
        FieldMap.Point p = toTableQuadrant(robotPosition, isRed);
        double distance = p.distanceTo(FieldMap.LEFT.aimPoint());
        ShotPoint closer = null;
        ShotPoint farther = null;
        for (ShotPoint spot : POINTS) {
            if (!spot.isTuned()) continue;
            double d = spot.distanceToCell();
            if (d <= distance && (closer == null || d > closer.distanceToCell())) closer = spot;
            if (d >= distance && (farther == null || d < farther.distanceToCell())) farther = spot;
        }
        if (closer == null) closer = farther;
        if (farther == null) farther = closer;

        double span = farther.distanceToCell() - closer.distanceToCell();
        double t = span > 0 ? (distance - closer.distanceToCell()) / span : 0;
        return new ShotPoint(p.x, p.y,
                lerp(closer.pollenVelocity, farther.pollenVelocity, t),
                lerp(closer.nectarVelocity, farther.nectarVelocity, t),
                lerp(closer.pollenHoodAngle, farther.pollenHoodAngle, t),
                lerp(closer.nectarHoodAngle, farther.nectarHoodAngle, t));
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    // Field position -> the matching spot in the table's quadrant
    public static FieldMap.Point toTableQuadrant(FieldMap.Point fieldPoint, boolean isRed) {
        FieldMap.Point blue = FieldMap.forAlliance(fieldPoint, isRed);
        return new FieldMap.Point(Math.abs(blue.x), blue.y);
    }

    // Table quadrant position -> the matching field position on the given side and alliance
    public static FieldMap.Point fromTableQuadrant(FieldMap.Point tablePoint, FieldMap.TargetSide side, boolean isRed) {
        FieldMap.Point blue = side == FieldMap.RIGHT ? new FieldMap.Point(-tablePoint.x, tablePoint.y) : tablePoint;
        return FieldMap.forAlliance(blue, isRed);
    }
}
