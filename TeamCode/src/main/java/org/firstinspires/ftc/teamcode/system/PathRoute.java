package org.firstinspires.ftc.teamcode.system;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;


// A route exported by the Overlake Robotics Path Planner, parsed into plain Java objects.
// This is the structure PedroPathBuilder consumes. It is produced either by PathServer (live
// upload from the web app) or by PathRoute.fromJson() when reading a saved asset file.
//
// Coordinate frame: centered field in inches (-72..72), x forward, y left, headings in DEGREES
// counter-clockwise. Pedro Pathing uses the same axes; only the heading unit differs.
//
// Two payload versions are understood:
//   - Version 1 (older web app and all saved asset files): "segments" exist but may lack a
//     per-segment "headingMode" (falls back to the top level "headingMode", default "straight")
//     and tags only carry "index", a POINT index. Point indices are resolved to segment indices
//     using the cumulative "samples" counts of the segments.
//   - Version 2: every segment has its own "headingMode" and every tag also carries "segment".
//     The old "velocity" and "tolerance" keys are gone and are ignored if present.
public final class PathRoute {
    public static final String HEADING_STRAIGHT = "straight";
    public static final String HEADING_TANGENT = "tangent";

    // A single planner segment. The segment starts where the previous one ended (or at the
    // route start pose for segment 0) and ends at end.
    public static final class Segment {
        public final String type;          // "line", "bezier", "arc" or "free"
        public final String headingMode;   // "straight" or "tangent"
        public final double[] end;         // {x, y, hDeg}
        public final double[] control;     // {x, y} for bezier, else null
        public final double[] mid;         // {x, y} for arc, else null
        public final double[][] samples;   // {x, y, hDeg} per sampled point, may be empty

        public Segment(String type, String headingMode, double[] end, double[] control,
                       double[] mid, double[][] samples) {
            this.type = type;
            this.headingMode = headingMode;
            this.end = end;
            this.control = control;
            this.mid = mid;
            this.samples = samples != null ? samples : new double[0][];
        }

        // Number of planner points this segment contributes (at least 1).
        public int pointCount() {
            return Math.max(1, samples.length);
        }
    }

    public final int version;
    public final String alliance;
    public final double[] start;              // {x, y, hDeg}
    public final double[][] points;           // {x, y, hDeg}, excluding start
    public final List<Segment> segments;
    public final List<PathServer.Tag> tags;   // segment index resolved; == segments.size() means "at the end"

    public PathRoute(int version, String alliance, double[] start, double[][] points,
                     List<Segment> segments, List<PathServer.Tag> tags) {
        this.version = version;
        this.alliance = alliance;
        this.start = start;
        this.points = points;
        this.segments = Collections.unmodifiableList(new ArrayList<>(segments));
        this.tags = Collections.unmodifiableList(new ArrayList<>(tags));
    }

    // An empty route that does nothing when followed.
    public static PathRoute empty() {
        return new PathRoute(2, "unknown", new double[]{0, 0, 0}, new double[0][],
                new ArrayList<>(), new ArrayList<>());
    }

    public Pose2D startPose() {
        return new Pose2D(DistanceUnit.INCH, start[0], start[1], AngleUnit.DEGREES, start[2]);
    }

    // Pose at the end of the given segment (degrees). Segment -1 gives the start pose and an
    // index past the last segment gives the final pose.
    public double[] endOf(int segmentIndex) {
        if (segmentIndex < 0 || segments.isEmpty()) return start;
        return segments.get(Math.min(segmentIndex, segments.size() - 1)).end;
    }

    // All tags whose resolved segment index equals segmentIndex, in upload order.
    public List<PathServer.Tag> tagsAt(int segmentIndex) {
        List<PathServer.Tag> out = new ArrayList<>();
        for (PathServer.Tag t : tags) if (t.segment == segmentIndex) out.add(t);
        return out;
    }

    // Parses a planner payload (version 1 or version 2) into a route.
    public static PathRoute fromJson(JSONObject root) throws JSONException {
        int version = root.optInt("version", 1);

        String alliance = root.optString("alliance", "unknown");
        if (alliance == null || alliance.trim().isEmpty()) alliance = "unknown";
        alliance = alliance.trim().toLowerCase();

        double[] start = readPose(root.opt("start"), new double[]{0, 0, 0});

        JSONArray ptsArr = root.optJSONArray("points");
        int n = ptsArr != null ? ptsArr.length() : 0;
        double[][] points = new double[n][];
        for (int i = 0; i < n; i++) points[i] = readPose(ptsArr.opt(i), new double[]{0, 0, 0});

        String defaultHeadingMode = normalizeHeadingMode(root.optString("headingMode", HEADING_STRAIGHT));

        List<Segment> segments = new ArrayList<>();
        JSONArray segArr = root.optJSONArray("segments");
        if (segArr != null && segArr.length() > 0) {
            for (int i = 0; i < segArr.length(); i++) {
                JSONObject s = segArr.getJSONObject(i);
                String type = s.optString("type", "line").trim().toLowerCase();
                String mode = s.has("headingMode")
                        ? normalizeHeadingMode(s.optString("headingMode", defaultHeadingMode))
                        : defaultHeadingMode;
                double[] end = readPose(s.opt("end"), null);
                double[] control = readPoint(s.opt("control"));
                double[] mid = readPoint(s.opt("mid"));

                double[][] samples = new double[0][];
                JSONArray sampArr = s.optJSONArray("samples");
                if (sampArr != null) {
                    samples = new double[sampArr.length()][];
                    for (int k = 0; k < sampArr.length(); k++) {
                        samples[k] = readPose(sampArr.opt(k), new double[]{0, 0, 0});
                    }
                }
                if (end == null) {
                    // Fall back to the last sample, or the last point, so a malformed segment
                    // still ends somewhere sensible.
                    if (samples.length > 0) end = samples[samples.length - 1];
                    else if (n > 0) end = points[n - 1];
                    else end = start;
                }
                segments.add(new Segment(type, mode, end, control, mid, samples));
            }
        } else {
            // No segment information at all: every point is its own straight line segment.
            for (double[] p : points) {
                segments.add(new Segment("line", defaultHeadingMode, p, null, null, new double[][]{p}));
            }
        }

        // Cumulative point counts so version 1 point indices can be mapped to segments.
        int[] cumulative = new int[segments.size()];
        int running = 0;
        for (int i = 0; i < segments.size(); i++) {
            running += segments.get(i).pointCount();
            cumulative[i] = running;
        }

        // Tags. "index" is an index into [start, ...points] (0 = start pose). A tag attached to a
        // node fires when the robot leaves that node, i.e. at the start of the segment that begins
        // there: node k is left by the segment containing points[k]. Version 2 tags also carry
        // "segment": the index of the segment whose END the tag sits at (-1 = the start pose), so
        // the tag fires at the start of segment + 1. That gives 0 for the start pose (fire before
        // segment 0) and segments.size() for the end node (fire once the route is complete).
        int totalPoints = segments.isEmpty() ? 0 : cumulative[cumulative.length - 1];
        List<PathServer.Tag> tags = new ArrayList<>();
        JSONArray tagsArr = root.optJSONArray("tags");
        if (tagsArr != null) {
            for (int i = 0; i < tagsArr.length(); i++) {
                JSONObject t = tagsArr.getJSONObject(i);
                String name = t.optString("name", "");
                double value = t.optDouble("value", 0.0);
                int pointIndex = t.optInt("index", 0);
                int segment;
                if (t.has("segment")) {
                    segment = t.optInt("segment", -1) + 1;
                    if (segment < 0) segment = 0;
                } else if (pointIndex >= totalPoints) {
                    segment = segments.size();
                } else {
                    segment = segmentForPoint(pointIndex, cumulative);
                }
                segment = Math.min(segment, segments.size());
                tags.add(new PathServer.Tag(name, value, pointIndex, segment));
            }
        }

        return new PathRoute(version, alliance, start, points, segments, tags);
    }

    // Maps a 0-based point index (points exclude the start pose) to the segment containing it.
    // Segment i owns the point indices [cumulative[i-1], cumulative[i]). An index past the last
    // point maps to cumulative.length, the "end of route" marker.
    public static int segmentForPoint(int pointIndex, int[] cumulative) {
        if (cumulative.length == 0) return 0;
        if (pointIndex < 0) return 0;
        for (int i = 0; i < cumulative.length; i++) {
            if (pointIndex < cumulative[i]) return i;
        }
        return cumulative.length;
    }

    // Reads {x, y, h} from either an array [x, y, h] or an object {"x", "y", "h"}.
    static double[] readPose(Object o, double[] fallback) {
        if (o instanceof JSONArray) {
            JSONArray a = (JSONArray) o;
            return new double[]{a.optDouble(0, 0.0), a.optDouble(1, 0.0),
                    a.length() >= 3 ? a.optDouble(2, 0.0) : 0.0};
        }
        if (o instanceof JSONObject) {
            JSONObject j = (JSONObject) o;
            return new double[]{j.optDouble("x", 0.0), j.optDouble("y", 0.0), j.optDouble("h", 0.0)};
        }
        return fallback;
    }

    // Reads {x, y} from either an array or an object, or null when absent.
    static double[] readPoint(Object o) {
        if (o instanceof JSONArray) {
            JSONArray a = (JSONArray) o;
            return new double[]{a.optDouble(0, 0.0), a.optDouble(1, 0.0)};
        }
        if (o instanceof JSONObject) {
            JSONObject j = (JSONObject) o;
            return new double[]{j.optDouble("x", 0.0), j.optDouble("y", 0.0)};
        }
        return null;
    }

    // "straight" stays straight; "tangent", "orth-left" and "orth-right" all become tangent.
    static String normalizeHeadingMode(String mode) {
        if (mode == null) return HEADING_STRAIGHT;
        String m = mode.trim().toLowerCase();
        if (m.isEmpty() || m.equals(HEADING_STRAIGHT)) return HEADING_STRAIGHT;
        return HEADING_TANGENT;
    }
}
