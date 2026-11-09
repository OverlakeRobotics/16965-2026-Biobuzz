package org.firstinspires.ftc.teamcode.code;

import org.firstinspires.ftc.teamcode.code.helpers.AutoAimer;

import java.util.function.ToDoubleFunction;

// Shooter settings tuned at spots on the field (with ShotTuner): the robot heading that scored, and each
// shooter's velocity and hood angle. Between spots every setting is interpolated by position, so the
// shooters are set for wherever the robot is. Position matters, not just distance to the CELL: tuned spots
// the same distance away needed different settings.
//
// Only written for ONE quadrant: the blue half, LEFT CELL side (+X, +Y). The field is symmetric, so the
// same spots work in the other three quadrants: RIGHT side mirrors x, red rotates 180 degrees.
// toTableQuadrant does the conversion, so nothing here needs duplicating.
public class ShotTable {
    private static final double UNTUNED = Double.NaN;

    // Tuned spots, anywhere on the field. A setting that isn't tuned at a spot is UNTUNED and left out of
    // that setting's interpolation (so far only pollen and the heading are tuned, plus nectar at the Auto spot).
    //
    // Copying from the ShotTuner screen (red alliance), Position line X, Y, heading:
    //   x = -X, y = -Y, heading = heading + 180
    //   velocities = the target ticks/s, hood angles = the degrees in brackets
    //
    // TODO: the drivers shoot from red field X -72 to -38, Y -72 to 44.6 (and its mirror images). Places in
    // there still far from a tuned spot, as red field X, Y on the ShotTuner screen; each leaves room for the
    // robot to turn all the way around: (-50, -30), (-40, -30), (-40, -50), the corner (-52, -52),
    // (-52, 40), (-40, 40), (-40, 22), (-40, 5).
    public static final ShotPoint[] POINTS = {
            //            x      y      heading pollen vel/hood   nectar vel/hood
            new ShotPoint(40.4,  12.9,  90.5,   1562, 16.2,       UNTUNED, UNTUNED),
            new ShotPoint(43.3,  13.0,  90.2,   1618, 16.2,       UNTUNED, UNTUNED),
            new ShotPoint(50.1,  13.2,  91.3,   1618, 24.2,       1900, 17.04),   // where the Autos shoot
            new ShotPoint(56.1,  49.9,  127.5,  1646, 28.2,       UNTUNED, UNTUNED),
            new ShotPoint(56.1,  0.0,   63.6,   1590, 20.2,       UNTUNED, UNTUNED),
            new ShotPoint(55.9,  -13.1, 55.5,   1562, 16.2,       UNTUNED, UNTUNED),
            new ShotPoint(55.8,  -24.0, 42.8,   1674, 20.2,       UNTUNED, UNTUNED),
    };

    public static final class ShotPoint {
        public final double x, y;                              // blue, LEFT side quadrant (inches)
        public final double heading;                           // robot heading that scored, same quadrant (degrees)
        public final double pollenVelocity, pollenHoodAngle;   // ticks/s, degrees
        public final double nectarVelocity, nectarHoodAngle;

        public ShotPoint(double x, double y, double heading, double pollenVelocity, double pollenHoodAngle,
                         double nectarVelocity, double nectarHoodAngle) {
            this.x = x;
            this.y = y;
            this.heading = heading;
            this.pollenVelocity = pollenVelocity;
            this.pollenHoodAngle = pollenHoodAngle;
            this.nectarVelocity = nectarVelocity;
            this.nectarHoodAngle = nectarHoodAngle;
        }

        public FieldMap.Point position() {
            return new FieldMap.Point(x, y);
        }

        // How far the heading is turned from pointing the shooters straight at the CELL (degrees,
        // counter-clockwise positive). Kept as a turn, not a heading, so it can be interpolated between spots.
        public double headingOffset() {
            return AutoAimer.normalize(heading - AutoAimer.aimHeadingFrom(x, y, FieldMap.LEFT.aimPoint()));
        }
    }

    // Settings for a robot position. Each one is interpolated on its own over the spots where it's tuned;
    // the heading offset is 0 (straight at the CELL) if no spot has a heading.
    public static ShotPoint at(FieldMap.Point robotPosition, boolean isRed) {
        FieldMap.Point p = toTableQuadrant(robotPosition, isRed);
        double straightAtCell = AutoAimer.aimHeadingFrom(p.x, p.y, FieldMap.LEFT.aimPoint());
        return new ShotPoint(p.x, p.y,
                straightAtCell + interpolate(p, ShotPoint::headingOffset, 0),
                interpolate(p, spot -> spot.pollenVelocity, UNTUNED),
                interpolate(p, spot -> spot.pollenHoodAngle, UNTUNED),
                interpolate(p, spot -> spot.nectarVelocity, UNTUNED),
                interpolate(p, spot -> spot.nectarHoodAngle, UNTUNED));
    }

    // Inverse-distance-squared average of one setting over the spots where it's tuned: standing on a spot
    // gives exactly its value, and in between the closest spots count the most. 'untuned' if no spot has it.
    private static double interpolate(FieldMap.Point p, ToDoubleFunction<ShotPoint> setting, double untuned) {
        double weightedSum = 0;
        double totalWeight = 0;
        for (ShotPoint spot : POINTS) {
            double value = setting.applyAsDouble(spot);
            if (Double.isNaN(value)) continue;
            double distance = spot.position().distanceTo(p);
            if (distance < 0.01) return value;
            double weight = 1 / (distance * distance);
            weightedSum += weight * value;
            totalWeight += weight;
        }
        return totalWeight > 0 ? weightedSum / totalWeight : untuned;
    }

    // Field position -> the matching spot in the table's quadrant
    public static FieldMap.Point toTableQuadrant(FieldMap.Point fieldPoint, boolean isRed) {
        FieldMap.Point blue = FieldMap.forAlliance(fieldPoint, isRed);
        return new FieldMap.Point(Math.abs(blue.x), blue.y);
    }
}
