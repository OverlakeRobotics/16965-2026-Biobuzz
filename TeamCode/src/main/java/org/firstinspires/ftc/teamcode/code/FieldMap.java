package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;
import java.util.List;

// Where things are on the BIOBUZZ field, and how to drive around the HIVE legs. Inches, origin at the
// field center (same as Pinpoint and the Path Planner). Numbers come from the Competition Manual
// section 9 (TU06), +/- 1 in.
//
// Axes (checked on the robot: facing red, forward is -Y and right is -X):
//   +Y points toward the BLUE HIVE half of the field (same sign convention as last year's code),
//   +X is 90 degrees clockwise from +Y when looking down at the field.
//
// Targets are written for BLUE only. The field is 180-degree rotationally symmetric, so red values
// come from forAlliance(): (x, y, h) -> (-x, -y, h + 180).
@Config
public class FieldMap {
    public static final double FIELD_SIZE = 144;
    public static final double HALF_FIELD = FIELD_SIZE / 2;

    // Robot footprint (CAD)
    public static final double ROBOT_LENGTH = 18.0;
    public static final double ROBOT_WIDTH = 18.0;
    // Distance from robot center to a corner: the robot's center must stay this far from an obstacle
    // so no part of it can hit, whatever its heading
    public static final double ROBOT_RADIUS = Math.hypot(ROBOT_LENGTH, ROBOT_WIDTH) / 2;
    public static final double OBSTACLE_MARGIN = 3;
    // Extra "distance" (inches) charged per detour waypoint when picking a route, so that between two
    // similar-length routes the one with fewer turns wins
    public static final double WAYPOINT_PENALTY = 6;
    // How far outside a keep-out zone the route's detour waypoints sit. At least BaseTeleOp.routeTolerance:
    // the drivetrain heads for the next waypoint once it's that close to one, cutting the corner by that much.
    public static final double WAYPOINT_CLEARANCE = 4;

    // HIVE Structure frame, centered on the field. Two triangular legs whose bases sit on the tiles:
    // the frame is 49.46 in "wide" (distance between the two legs) and 38.95 in "deep" (length of each leg's base).
    public static final double HIVE_FRAME_WIDTH = 49.46;
    public static final double HIVE_FRAME_DEPTH = 38.95;
    public static final double HIVE_PIVOT_HEIGHT = 43.95;

    // FLOWERS: one on each perimeter wall, centered FLOWER_OFFSET along it (Figure 9-2: +X wall at y = +24,
    // -X wall at y = -24, +Y wall at x = -24, -Y wall at x = +24). Size measured from Figures 9-2 and 9-12.
    public static final double FLOWER_OFFSET = 24;
    public static final double FLOWER_WIDTH = 7;   // along the wall
    public static final double FLOWER_DEPTH = 5;   // out from the wall

    // Everything the robot can hit. Each obstacle is 180-degree symmetric with another (or itself), so the
    // list is the same for both alliances.
    //   HIVE: the two leg bases, thin bars along X at Y = +/- HIVE_FRAME_WIDTH / 2. The legs rise straight
    //         up from them, and the HIVES hang with their bottom 30.6 in above the tiles (Figure 9-10), so
    //         the robot can drive under the rest.
    //   The four FLOWERS, and the perimeter walls (1 in thick, just outside the field).
    public static final Rect[] OBSTACLES = {
            new Rect(-HIVE_FRAME_DEPTH / 2, HIVE_FRAME_WIDTH / 2 - 1, HIVE_FRAME_DEPTH / 2, HIVE_FRAME_WIDTH / 2 + 1),
            new Rect(-HIVE_FRAME_DEPTH / 2, -HIVE_FRAME_WIDTH / 2 - 1, HIVE_FRAME_DEPTH / 2, -HIVE_FRAME_WIDTH / 2 + 1),

            new Rect(HALF_FIELD - FLOWER_DEPTH, FLOWER_OFFSET - FLOWER_WIDTH / 2, HALF_FIELD, FLOWER_OFFSET + FLOWER_WIDTH / 2),
            new Rect(-HALF_FIELD, -FLOWER_OFFSET - FLOWER_WIDTH / 2, -HALF_FIELD + FLOWER_DEPTH, -FLOWER_OFFSET + FLOWER_WIDTH / 2),
            new Rect(-FLOWER_OFFSET - FLOWER_WIDTH / 2, HALF_FIELD - FLOWER_DEPTH, -FLOWER_OFFSET + FLOWER_WIDTH / 2, HALF_FIELD),
            new Rect(FLOWER_OFFSET - FLOWER_WIDTH / 2, -HALF_FIELD, FLOWER_OFFSET + FLOWER_WIDTH / 2, -HALF_FIELD + FLOWER_DEPTH),

            new Rect(HALF_FIELD, -HALF_FIELD - 1, HALF_FIELD + 1, HALF_FIELD + 1),
            new Rect(-HALF_FIELD - 1, -HALF_FIELD - 1, -HALF_FIELD, HALF_FIELD + 1),
            new Rect(-HALF_FIELD - 1, HALF_FIELD, HALF_FIELD + 1, HALF_FIELD + 1),
            new Rect(-HALF_FIELD - 1, -HALF_FIELD - 1, HALF_FIELD + 1, -HALF_FIELD),
    };

    // The two CELLS of the blue HIVE. When the HIVE tips, the scoring CELL switches sides.
    // LEFT/RIGHT are as the driver sees them, standing on their own alliance's side looking at the HIVE.
    // Aim point of the blue LEFT CELL (+X, +Y), tunable in the dashboard; the RIGHT CELL is the same point
    // mirrored across x = 0. Estimated from the top-down field image (+/- ~1 in). Shot spots are in ShotTable.
    public static double cellAimX = 13.5;
    public static double cellAimY = 13;
    public static final TargetSide LEFT = new TargetSide("LEFT", 1);     // the +X CELL
    public static final TargetSide RIGHT = new TargetSide("RIGHT", -1);  // the -X CELL

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

    // One CELL to shoot at (blue side)
    public static final class TargetSide {
        public final String name;
        private final int xSign;   // +1 for the +X CELL, -1 for the -X CELL

        public TargetSide(String name, int xSign) {
            this.name = name;
            this.xSign = xSign;
        }

        public Point aimPoint() {
            return new Point(xSign * cellAimX, cellAimY);
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

        // The point 'clearance' outside the nearest edge, straight out along x or y from p (inside)
        public Point nearestOutside(Point p, double clearance) {
            double left = p.x - minX;
            double right = maxX - p.x;
            double down = p.y - minY;
            double up = maxY - p.y;
            double nearest = Math.min(Math.min(left, right), Math.min(down, up));
            if (nearest == left) return new Point(minX - clearance, p.y);
            if (nearest == right) return new Point(maxX + clearance, p.y);
            if (nearest == down) return new Point(p.x, minY - clearance);
            return new Point(p.x, maxY + clearance);
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

    // Waypoints from 'from' to 'to' where no straight leg comes near an obstacle, for
    // OdometryHolonomicDrivetrain.setPositionDrive(Pose2D[]), which drives straight between waypoints.
    // Detour waypoints keep the starting heading so the robot doesn't swing around next to an obstacle;
    // only the last waypoint uses 'to's heading. If 'to' is inside a keep-out zone the robot stays put.
    public static Pose2D[] routeTo(Pose2D from, Pose2D to) {
        Rect[] zones = keepOutZones();
        double heading = from.getHeading(AngleUnit.DEGREES);
        Point current = new Point(from.getX(DistanceUnit.INCH), from.getY(DistanceUnit.INCH));
        Point end = new Point(to.getX(DistanceUnit.INCH), to.getY(DistanceUnit.INCH));
        List<Pose2D> path = new ArrayList<>();

        // Starting inside a keep-out zone (e.g. pushed there by another robot): first move straight out
        // past its nearest edge, away from what's inside it. Zones only overlap next to the walls, where
        // the FLOWER is checked before the wall, so a FLOWER exit is still checked against the wall.
        for (Rect zone : zones) {
            if (zone.contains(current)) {
                current = zone.nearestOutside(current, WAYPOINT_CLEARANCE);
                path.add(pose(current, heading));
            }
        }

        List<Point> corners = shortestDetour(zones, current, end);
        if (corners == null) {
            path.add(pose(current, heading));
            return path.toArray(new Pose2D[0]);
        }
        for (Point corner : corners) {
            path.add(pose(corner, heading));
        }
        path.add(to);
        return path.toArray(new Pose2D[0]);
    }

    // Shortest way from start to end using the keep-out zone corners as possible waypoints (the
    // shortest path around rectangles always bends only at their corners). Returns just the corners
    // to drive through, empty if the straight line is clear, or null if there's no clear way (end
    // inside a zone). Corners inside another zone are never connected, so they're never used.
    //
    // Small graph (start, 40 corners, end), so a plain Dijkstra is plenty fast to run on a button press.
    private static List<Point> shortestDetour(Rect[] zones, Point start, Point end) {
        List<Point> nodes = new ArrayList<>();
        nodes.add(start);
        for (Rect zone : zones) {
            for (Point corner : zone.grow(WAYPOINT_CLEARANCE).corners()) {
                nodes.add(corner);
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

        if (previous[endIndex] == -1) return null;
        List<Point> corners = new ArrayList<>();
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
