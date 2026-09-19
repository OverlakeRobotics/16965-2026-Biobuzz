package org.firstinspires.ftc.teamcode.system;


import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;


// Turns a PathRoute from the Overlake Robotics Path Planner into Pedro Pathing paths.
//
// The route is cut into "chunks". A chunk is one compound Pedro Path made of one sub-path per
// planner segment, plus the tags that fire inside it (keyed by the sub-path index the follower
// reports through Follower.pathIndex()). Chunks are only split where the robot has to come to a
// stop, which by default is at every "pause" tag. PathPlanRunner turns the chunks into a single
// Ivy command.
//
// Segment mapping:
//   line   -> Paths.line(a, b)
//   bezier -> Paths.curve(a, control, b)   (quadratic; the control point has no heading)
//   arc    -> Paths.through(a, mid, b)
//   free   -> chain of Paths.line over its samples, decimated to roughly every 3 inches
// Heading mapping:
//   straight -> .linear(startHeading, endHeading) where the start heading is the previous
//               segment's end heading (or the start pose heading for segment 0)
//   tangent  -> .tangent()
// Units: the planner and Pedro share axes and inches; only headings are converted (degrees to
// radians).
public final class PedroPathBuilder {
    public static final String TAG_PAUSE = "pause";

    // Below this length (inches) a line segment is treated as a turn in place: it adds no
    // sub-path, its end heading is simply carried into the next segment / hold pose.
    public static final double MIN_SEGMENT_LENGTH_IN = 0.25;

    // Decides where chunks are split and which segments are driven by user code.
    public interface SegmentPolicy {
        // True if the robot must stop at the START of this segment (i.e. at the end of the
        // previous one). All tags of the segment then fire while the robot is stopped.
        boolean stopsBefore(int segmentIndex, PathRoute.Segment segment, List<PathServer.Tag> tags);

        // True if this segment is not followed by the Pedro follower but by a command that the
        // TagHandler provides (PathPlanRunner.TagHandler.driveSegment). Such a segment becomes its
        // own chunk. Defaults to false.
        default boolean isCustom(int segmentIndex, PathRoute.Segment segment, List<PathServer.Tag> tags) {
            return false;
        }
    }

    // Lets callers replace the heading interpolation of individual segments (e.g. aim at a goal).
    // Return null to keep the default straight/tangent behavior.
    public interface HeadingResolver {
        Interpolator headingFor(int segmentIndex, PathRoute.Segment segment, List<PathServer.Tag> tags,
                                double startHeadingRad, double endHeadingRad);
    }

    // Default policy: split only at "pause" tags.
    public static final SegmentPolicy PAUSE_ONLY = (i, segment, tags) -> hasTag(tags, TAG_PAUSE);

    public static final class Options {
        public SegmentPolicy policy = PAUSE_ONLY;
        public HeadingResolver headings = null;
        public double freeDecimationInches = 3.0;
    }

    // A tag placed inside a chunk. subPathIndex is the index Follower.pathIndex() reports when the
    // tag's segment is being followed (flattened over nested compound paths).
    public static final class PlacedTag {
        public final PathServer.Tag tag;
        public final int segmentIndex;
        public final int subPathIndex;

        PlacedTag(PathServer.Tag tag, int segmentIndex, int subPathIndex) {
            this.tag = tag;
            this.segmentIndex = segmentIndex;
            this.subPathIndex = subPathIndex;
        }
    }

    public static final class Chunk {
        public final int firstSegment;
        public final int lastSegment;
        public final Path path;               // compound path, or null if custom / turn-only
        public final Pose startPose;          // where the chunk begins (radians)
        public final Pose endPose;            // where the chunk ends (radians)
        public final boolean stopBefore;      // the robot is stopped at startPose before following
        public final boolean custom;          // driven by TagHandler.driveSegment instead of path
        public final PathRoute.Segment customSegment;
        public final List<PlacedTag> tags;

        Chunk(int firstSegment, int lastSegment, Path path, Pose startPose, Pose endPose,
              boolean stopBefore, boolean custom, PathRoute.Segment customSegment, List<PlacedTag> tags) {
            this.firstSegment = firstSegment;
            this.lastSegment = lastSegment;
            this.path = path;
            this.startPose = startPose;
            this.endPose = endPose;
            this.stopBefore = stopBefore;
            this.custom = custom;
            this.customSegment = customSegment;
            this.tags = Collections.unmodifiableList(tags);
        }

        // Tags that fire while the robot is stopped at the start of this chunk.
        public List<PlacedTag> stopTags() {
            List<PlacedTag> out = new ArrayList<>();
            if (!stopBefore) return out;
            for (PlacedTag t : tags) if (t.subPathIndex == 0 && t.segmentIndex == firstSegment) out.add(t);
            return out;
        }

        // Tags that fire while the chunk is being followed.
        public List<PlacedTag> followTags() {
            List<PlacedTag> out = new ArrayList<>();
            List<PlacedTag> stop = stopTags();
            for (PlacedTag t : tags) if (!stop.contains(t)) out.add(t);
            return out;
        }

        // Largest "pause" value among the stop tags, in seconds (0 when none).
        public double pauseSeconds() {
            double s = 0;
            for (PlacedTag t : stopTags()) {
                if (TAG_PAUSE.equals(t.tag.name)) s = Math.max(s, t.tag.value);
            }
            return s;
        }
    }

    private PedroPathBuilder() {}

    public static List<Chunk> build(PathRoute route) {
        return build(route, new Options());
    }

    public static List<Chunk> build(PathRoute route, SegmentPolicy policy) {
        Options o = new Options();
        o.policy = policy;
        return build(route, o);
    }

    // Builds the chunks for a route. Chunk 0 starts at the route start; every later chunk starts
    // with the robot stopped (either because of a stop tag or because the previous chunk was a
    // custom segment). Tags placed past the last segment (attached to the end node) become a final
    // stop chunk without a path.
    public static List<Chunk> build(PathRoute route, Options options) {
        List<Chunk> chunks = new ArrayList<>();
        int n = route.segments.size();
        if (n == 0) return chunks;

        SegmentPolicy policy = options.policy != null ? options.policy : PAUSE_ONLY;

        int i = 0;
        while (i < n) {
            PathRoute.Segment first = route.segments.get(i);
            List<PathServer.Tag> firstTags = route.tagsAt(i);
            boolean stopBefore = i > 0 || policy.stopsBefore(0, first, firstTags);
            Pose startPose = toPose(route.endOf(i - 1));

            if (policy.isCustom(i, first, firstTags)) {
                List<PlacedTag> tags = new ArrayList<>();
                for (PathServer.Tag t : firstTags) tags.add(new PlacedTag(t, i, 0));
                chunks.add(new Chunk(i, i, null, startPose, toPose(first.end), stopBefore, true, first, tags));
                i++;
                continue;
            }

            // Grow the chunk until the next stop / custom segment.
            List<Path> subPaths = new ArrayList<>();
            List<PlacedTag> tags = new ArrayList<>();
            int subPathOffset = 0;
            int last = i;
            double[] segStart = route.endOf(i - 1);
            for (int j = i; j < n; j++) {
                PathRoute.Segment seg = route.segments.get(j);
                List<PathServer.Tag> segTags = route.tagsAt(j);
                if (j > i && (policy.stopsBefore(j, seg, segTags) || policy.isCustom(j, seg, segTags))) break;

                for (PathServer.Tag t : segTags) tags.add(new PlacedTag(t, j, subPathOffset));

                Path p = segmentPath(seg, j, segStart, segTags, options);
                if (p != null) {
                    subPaths.add(p);
                    subPathOffset += p.getSegments().size();
                }
                segStart = seg.end;
                last = j;
            }

            Path chunkPath = null;
            if (subPaths.size() == 1) chunkPath = subPaths.get(0);
            else if (subPaths.size() > 1) chunkPath = Paths.path(subPaths.toArray(new Path[0]));

            chunks.add(new Chunk(i, last, chunkPath, startPose, toPose(route.endOf(last)),
                    stopBefore, false, null, tags));
            i = last + 1;
        }

        // Tags attached to the end node fire once the route is complete, with the robot holding
        // the final pose.
        List<PathServer.Tag> endTags = route.tagsAt(n);
        if (!endTags.isEmpty()) {
            List<PlacedTag> tags = new ArrayList<>();
            for (PathServer.Tag t : endTags) tags.add(new PlacedTag(t, n, 0));
            Pose end = toPose(route.endOf(n - 1));
            chunks.add(new Chunk(n, n, null, end, end, true, false, null, tags));
        }
        return chunks;
    }

    // Finds the chunk index and sub-path index a tag fires at, or null if the tag is not placed.
    public static int[] locate(List<Chunk> chunks, PathServer.Tag tag) {
        for (int c = 0; c < chunks.size(); c++) {
            for (PlacedTag p : chunks.get(c).tags) {
                if (p.tag == tag) return new int[]{c, p.subPathIndex};
            }
        }
        return null;
    }

    // Builds the Pedro path for one planner segment starting at startPose ({x, y, hDeg}).
    // Returns null for segments too short to follow (turn in place).
    public static Path segmentPath(PathRoute.Segment seg, int segmentIndex, double[] startPose,
                                   List<PathServer.Tag> tags, Options options) {
        double startH = Math.toRadians(startPose[2]);
        double endH = Math.toRadians(seg.end[2]);
        Pose a = toPose(startPose);
        Pose b = toPose(seg.end);

        Interpolator override = options.headings != null
                ? options.headings.headingFor(segmentIndex, seg, tags, startH, endH) : null;
        boolean tangent = PathRoute.HEADING_TANGENT.equals(seg.headingMode);

        switch (seg.type) {
            case "bezier": {
                if (seg.control == null) return withHeading(lineOrNull(a, b), override, tangent, startH, endH);
                Pose c = new Pose(seg.control[0], seg.control[1], 0);
                if (a.distance(b) < MIN_SEGMENT_LENGTH_IN && a.distance(c) < MIN_SEGMENT_LENGTH_IN) return null;
                return withHeading(Paths.curve(a, c, b), override, tangent, startH, endH);
            }
            case "arc": {
                if (seg.mid == null) return withHeading(lineOrNull(a, b), override, tangent, startH, endH);
                Pose m = new Pose(seg.mid[0], seg.mid[1], 0);
                if (a.distance(b) < MIN_SEGMENT_LENGTH_IN && a.distance(m) < MIN_SEGMENT_LENGTH_IN) return null;
                return withHeading(Paths.through(a, m, b), override, tangent, startH, endH);
            }
            case "free": {
                List<Pose> pts = decimate(a, seg.samples, b, options.freeDecimationInches);
                if (pts.size() < 2) return null;
                if (pts.size() == 2) return withHeading(lineOrNull(pts.get(0), pts.get(1)), override, tangent, startH, endH);

                // Straight heading is interpolated over the whole chain by arc length; tangent
                // and overrides apply per line.
                double total = 0;
                for (int k = 1; k < pts.size(); k++) total += pts.get(k - 1).distance(pts.get(k));
                List<Path> lines = new ArrayList<>();
                double acc = 0;
                for (int k = 1; k < pts.size(); k++) {
                    Pose p0 = pts.get(k - 1), p1 = pts.get(k);
                    double h0 = lerpAngle(startH, endH, total > 0 ? acc / total : 0);
                    acc += p0.distance(p1);
                    double h1 = lerpAngle(startH, endH, total > 0 ? acc / total : 1);
                    lines.add(withHeading(Paths.line(p0, p1), override, tangent, h0, h1));
                }
                return Paths.path(lines.toArray(new Path[0]));
            }
            case "line":
            default:
                return withHeading(lineOrNull(a, b), override, tangent, startH, endH);
        }
    }

    // Converts a planner pose {x, y, hDeg} into a Pedro pose (radians).
    public static Pose toPose(double[] p) {
        return new Pose(p[0], p[1], Math.toRadians(p[2]));
    }

    // Converts a Pedro pose back into planner units {x, y, hDeg}.
    public static double[] fromPose(Pose p) {
        return new double[]{p.x(), p.y(), Math.toDegrees(p.heading())};
    }

    public static boolean hasTag(List<PathServer.Tag> tags, String name) {
        for (PathServer.Tag t : tags) if (name.equals(t.name)) return true;
        return false;
    }

    private static Path lineOrNull(Pose a, Pose b) {
        if (a.distance(b) < MIN_SEGMENT_LENGTH_IN) return null;
        return Paths.line(a, b);
    }

    private static Path withHeading(Path p, Interpolator override, boolean tangent, double startH, double endH) {
        if (p == null) return null;
        if (override != null) return p.heading(override);
        if (tangent) return p.tangent();
        return p.linear(startH, endH);
    }

    // Reduces a free-hand sample list to points roughly spacing inches apart, always keeping the
    // first and last point.
    private static List<Pose> decimate(Pose first, double[][] samples, Pose last, double spacing) {
        List<Pose> out = new ArrayList<>();
        out.add(first);
        Pose prev = first;
        for (double[] s : samples) {
            Pose p = new Pose(s[0], s[1], 0);
            if (prev.distance(p) >= spacing) {
                out.add(p);
                prev = p;
            }
        }
        if (out.get(out.size() - 1).distance(last) < MIN_SEGMENT_LENGTH_IN) {
            out.set(out.size() - 1, last);
        } else {
            out.add(last);
        }
        return out;
    }

    // Interpolates between two angles the short way round.
    private static double lerpAngle(double a, double b, double t) {
        double d = Math.atan2(Math.sin(b - a), Math.cos(b - a));
        return a + d * t;
    }
}
