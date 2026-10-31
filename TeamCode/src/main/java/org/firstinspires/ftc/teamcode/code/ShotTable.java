package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.dashboard.config.Config;

// Spots on the field where we know good shooter settings (found with ManualTestTeleOp). AutoAimer uses
// the settings of whichever spot is closest to the robot.
//
// Only written for ONE quadrant: the blue half, LEFT CELL side (+X, +Y). The field is symmetric, so the
// same spots work in the other three quadrants: RIGHT side mirrors x, red rotates 180 degrees.
// toTableQuadrant / fromTableQuadrant do the conversion, so nothing here needs duplicating.
@Config
public class ShotTable {
    // The preset: where the TeleOp bumpers drive (left bumper = LEFT side, right bumper = RIGHT side).
    // Also a shot point. Tunable live in the dashboard (ShotTable). TODO: measure and tune.
     public static double presetX = 48;
    public static double presetY = 13;
    public static double presetPollenVelocity = 1500;   // ticks/s
    public static double presetNectarVelocity = 1500;
    public static double presetPollenHoodAngle = 16.2;  // degrees
    public static double presetNectarHoodAngle = 17.04;

    // More spots. To add one: drive there in ManualTestTeleOp, tune until both shooters score, then copy
    // the Position X/Y, both target velocities and both hood angles from telemetry into a new line.
    private static final ShotPoint[] OTHER_POINTS = {
    };

    public static final class ShotPoint {
        public final String name;
        public final double x, y;                              // blue, LEFT side quadrant (inches)
        public final double pollenVelocity, nectarVelocity;    // ticks/s
        public final double pollenHoodAngle, nectarHoodAngle;  // degrees

        public ShotPoint(String name, double x, double y, double pollenVelocity, double nectarVelocity,
                         double pollenHoodAngle, double nectarHoodAngle) {
            this.name = name;
            this.x = x;
            this.y = y;
            this.pollenVelocity = pollenVelocity;
            this.nectarVelocity = nectarVelocity;
            this.pollenHoodAngle = pollenHoodAngle;
            this.nectarHoodAngle = nectarHoodAngle;
        }

        public FieldMap.Point position() {
            return new FieldMap.Point(x, y);
        }
    }

    // Built from the dashboard values every call, so edits apply immediately
    public static ShotPoint preset() {
        return new ShotPoint("Preset", presetX, presetY, presetPollenVelocity, presetNectarVelocity,
                presetPollenHoodAngle, presetNectarHoodAngle);
    }

    // The shot point closest to where the robot is
    public static ShotPoint closest(FieldMap.Point robotPosition, boolean isRed) {
        FieldMap.Point p = toTableQuadrant(robotPosition, isRed);
        ShotPoint best = preset();
        for (ShotPoint shot : OTHER_POINTS) {
            if (shot.position().distanceTo(p) < best.position().distanceTo(p)) best = shot;
        }
        return best;
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
