package org.firstinspires.ftc.teamcode.code;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;
import java.util.List;

// Where things are on the BIOBUZZ field, and how to drive around the HIVE legs. Inches, origin at the
// field center (same as Pinpoint and the Path Planner). Numbers come from the Competition Manual
// section 9 (TU06), +/- 1 in.
//
// Axes (TODO: verify on the robot by driving and reading the Pinpoint pose):
//   +Y points toward the BLUE HIVE half of the field (same sign convention as last year's code),
//   +X is 90 degrees clockwise from +Y when looking down at the field.
//
// Targets are written for BLUE only. The field is 180-degree rotationally symmetric, so red values
// come from forAlliance(): (x, y, h) -> (-x, -y, h + 180).
public class FieldMap {
    public static final double FIELD_SIZE = 144;
    public static final double HALF_FIELD = FIELD_SIZE / 2;

    // Robot footprint (CAD)
    public static final double ROBOT_LENGTH = 15.118;
    public static final double ROBOT_WIDTH = 16.378;
    // Distance from robot center to a corner: the robot's center must stay this far from an obstacle
    // so no part of it can hit, whatever its heading
    public static final double ROBOT_RADIUS = Math.hypot(ROBOT_LENGTH, ROBOT_WIDTH) / 2;
    public static final double OBSTACLE_MARGIN = 2;
    // Extra "distance" (inches) charged per detour waypoint when picking a route, so that between two
    // similar-length routes the one with fewer turns wins
    public static final double WAYPOINT_PENALTY = 6;

    // HIVE Structure frame, centered on the field. Two triangular legs whose bases sit on the tiles:
    // the frame is 49.46 in "wide" (distance between the two legs) and 38.95 in "deep" (length of each leg's base).
    public static final double HIVE_FRAME_WIDTH = 49.46;
    public static final double HIVE_FRAME_DEPTH = 38.95;
    public static final double HIVE_PIVOT_HEIGHT = 43.95;
    // Each HIVE has 2 CELLS ~18.8 in apart; the HIVE tips so one CELL faces up at a time
    public static final double HIVE_CELL_SPACING = 18.8;

    // The two leg bases are the only parts of the HIVE the robot can't drive through. Each is a thin
    // bar along X at Y = +/- HIVE_FRAME_WIDTH / 2. They're symmetric, so the same for both alliances.
    public static final Rect[] OBSTACLES = {
            new Rect(-HIVE_FRAME_DEPTH / 2, HIVE_FRAME_WIDTH / 2 - 1, HIVE_FRAME_DEPTH / 2, HIVE_FRAME_WIDTH / 2 + 1),
            new Rect(-HIVE_FRAME_DEPTH / 2, -HIVE_FRAME_WIDTH / 2 - 1, HIVE_FRAME_DEPTH / 2, -HIVE_FRAME_WIDTH / 2 + 1),
    };

    // The two CELLS of the blue HIVE. When the HIVE tips, the scoring CELL switches sides.
    // TODO: measure aim points and preset spots. These are placeholders from the manual drawings.
    public static final TargetSide LEFT = new TargetSide(
            "LEFT",
            new Point(-HIVE_CELL_SPACING / 2, 12),   // aim point: the -X CELL
            new Point(-48, 12)                       // preset: where "go to range" drives
    );
    public static final TargetSide RIGHT = new TargetSide(
            "RIGHT",
            new Point(HIVE_CELL_SPACING / 2, 12),    // aim point: the +X CELL
            new Point(48, 12)
    );

    public static final class Point {
        public final double x, y;

        public Point(double x, double y) {
            this.x = x;
            this.y = y;
        }

        public double distanceTo(Point other) {
            return Math.hypot(other.x - x, other.y - y);
        }
    }

    // One CELL to shoot at, plus the spot the robot drives to before shooting at it (blue side)
    public static final class TargetSide {
        public final String name;
        public final Point aimPoint;
        public final Point presetPoint;

        public TargetSide(String name, Point aimPoint, Point presetPoint) {
            this.name = name;
            this.aimPoint = aimPoint;
            this.presetPoint = presetPoint;
        }
    }

    // Axis-aligned rectangle, inches
    public static final class Rect {
        public final double minX, minY, maxX, maxY;

        public Rect(double minX, double minY, double maxX, double maxY) {
            this.minX = minX;
            this.minY = minY;
            this.maxX = maxX;
            this.maxY = maxY;
        }

        // This rectangle grown on every side, e.g. by ROBOT_RADIUS to get the zone the robot's center must avoid
        public Rect grow(double amount) {
            return new Rect(minX - amount, minY - amount, maxX + amount, maxY + amount);
        }

        public boolean contains(Point p) {
            return p.x > minX && p.x < maxX && p.y > minY && p.y < maxY;
        }

        public Point[] corners() {
            return new Point[]{
                    new Point(minX, minY), new Point(minX, maxY),
                    new Point(maxX, minY), new Point(maxX, maxY)
            };
        }

        // True if the segment a-b passes through the inside of this rectangle
        public boolean intersects(Point a, Point b) {
            return entryFraction(a, b) >= 0;
        }

        // How far along a-b (0 = at a, 1 = at b) the segment enters this rectangle, or -1 if it
        // never passes through the inside (Liang-Barsky clipping)
        public double entryFraction(Point a, Point b) {
            double dx = b.x - a.x;
            double dy = b.y - a.y;
            double[] p = {-dx, dx, -dy, dy};
            double[] q = {a.x - minX, maxX - a.x, a.y - minY, maxY - a.y};
            double tEnter = 0;
            double tExit = 1;
            for (int i = 0; i < 4; i++) {
                if (p[i] == 0) {
                    if (q[i] <= 0) return -1;   // parallel to this edge and outside (or on) it
                } else {
                    double t = q[i] / p[i];
                    if (p[i] < 0) tEnter = Math.max(tEnter, t);
                    else tExit = Math.min(tExit, t);
                }
            }
            return tEnter < tExit ? tEnter : -1;
        }
    }

    // ---- alliance ----------------------------------------------------------------------------

    public static Point forAlliance(Point bluePoint, boolean isRed) {
        return isRed ? new Point(-bluePoint.x, -bluePoint.y) : bluePoint;
    }

    // Also converts a red pose back to blue, since rotating 180 degrees twice is a no-op
    public static Pose2D forAlliance(Pose2D bluePose, boolean isRed) {
        if (!isRed) return bluePose;
        return new Pose2D(DistanceUnit.INCH,
                -bluePose.getX(DistanceUnit.INCH), -bluePose.getY(DistanceUnit.INCH),
                AngleUnit.DEGREES, bluePose.getHeading(AngleUnit.DEGREES) + 180);
    }

    // ---- routing -----------------------------------------------------------------------------

    // Zones the robot's center must stay out of: each obstacle grown by the robot's radius + margin
    private static Rect[] keepOutZones() {
        Rect[] zones = new Rect[OBSTACLES.length];
        for (int i = 0; i < OBSTACLES.length; i++) {
            zones[i] = OBSTACLES[i].grow(ROBOT_RADIUS + OBSTACLE_MARGIN);
        }
        return zones;
    }

    // Waypoints from 'from' to 'to' where no straight leg crosses a HIVE leg, for
    // OdometryHolonomicDrivetrain.setPositionDrive(Pose2D[]), which drives straight between waypoints.
    // Detour waypoints keep the starting heading so the robot doesn't swing around next to the frame;
    // only the last waypoint uses 'to's heading.
    public static Pose2D[] routeTo(Pose2D from, Pose2D to) {
        Rect[] zones = keepOutZones();
        double heading = from.getHeading(AngleUnit.DEGREES);
        Point current = new Point(from.getX(DistanceUnit.INCH), from.getY(DistanceUnit.INCH));
        Point end = new Point(to.getX(DistanceUnit.INCH), to.getY(DistanceUnit.INCH));
        List<Pose2D> path = new ArrayList<>();

        // Starting inside a keep-out zone (e.g. pushed there by another robot): first move straight
        // out along Y, to whichever side is closer
        for (Rect zone : zones) {
            if (zone.contains(current)) {
                double outY = current.y - zone.minY < zone.maxY - current.y ? zone.minY - 1 : zone.maxY + 1;
                current = new Point(current.x, outY);
                path.add(pose(current, heading));
            }
        }

        for (Point corner : shortestDetour(zones, current, end)) {
            path.add(pose(corner, heading));
        }
        path.add(to);
        return path.toArray(new Pose2D[0]);
    }

    // Shortest way from start to end using the keep-out zone corners as possible waypoints (the
    // shortest path around rectangles always bends only at their corners). Returns just the corners
    // to drive through, empty if the straight line is clear.
    //
    // Small graph (start, 8 corners, end), so a plain Dijkstra is plenty fast to run on a button press.
    private static List<Point> shortestDetour(Rect[] zones, Point start, Point end) {
        List<Point> nodes = new ArrayList<>();
        nodes.add(start);
        double limit = HALF_FIELD - ROBOT_LENGTH / 2;   // corners the robot can't reach next to a wall are skipped
        for (Rect zone : zones) {
            for (Point corner : zone.grow(1).corners()) {
                if (Math.abs(corner.x) <= limit && Math.abs(corner.y) <= limit) nodes.add(corner);
            }
        }
        nodes.add(end);
        int endIndex = nodes.size() - 1;

        double[] cost = new double[nodes.size()];
        int[] previous = new int[nodes.size()];
        boolean[] done = new boolean[nodes.size()];
        java.util.Arrays.fill(cost, Double.MAX_VALUE);
        java.util.Arrays.fill(previous, -1);
        cost[0] = 0;

        while (true) {
            int u = -1;
            for (int i = 0; i < nodes.size(); i++) {
                if (!done[i] && cost[i] < Double.MAX_VALUE && (u == -1 || cost[i] < cost[u])) u = i;
            }
            if (u == -1 || u == endIndex) break;
            done[u] = true;
            for (int v = 0; v < nodes.size(); v++) {
                if (done[v] || firstBlockingZone(zones, nodes.get(u), nodes.get(v)) != null) continue;
                // Every corner is a slow-down-and-turn, so a waypoint costs a bit more than its distance
                double legCost = nodes.get(u).distanceTo(nodes.get(v)) + (v == endIndex ? 0 : WAYPOINT_PENALTY);
                if (cost[u] + legCost < cost[v]) {
                    cost[v] = cost[u] + legCost;
                    previous[v] = u;
                }
            }
        }

        List<Point> corners = new ArrayList<>();
        if (previous[endIndex] == -1) return corners;   // no clear way (bad placeholder preset): drive straight
        for (int i = previous[endIndex]; i > 0; i = previous[i]) {
            corners.add(0, nodes.get(i));
        }
        return corners;
    }

    // The zone the segment a-b runs into first (the one to go around next), or null if the way is clear
    private static Rect firstBlockingZone(Rect[] zones, Point a, Point b) {
        Rect first = null;
        double firstEntry = Double.MAX_VALUE;
        for (Rect zone : zones) {
            double entry = zone.entryFraction(a, b);
            if (entry >= 0 && entry < firstEntry) {
                firstEntry = entry;
                first = zone;
            }
        }
        return first;
    }

    private static Pose2D pose(Point p, double headingDegrees) {
        return new Pose2D(DistanceUnit.INCH, p.x, p.y, AngleUnit.DEGREES, headingDegrees);
    }
}
