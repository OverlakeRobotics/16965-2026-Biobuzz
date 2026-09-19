package org.firstinspires.ftc.teamcode.system;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import fi.iki.elonen.NanoHTTPD;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;


// Server that runs on the robot to connect with the Overlake Robotics Path Planner and allow
// path uploading. Uploads are parsed by PathRoute.fromJson() (both the version 1 and version 2
// payloads are accepted) and the parsed route is available through getRoute() for
// PedroPathBuilder. The older getPath()/getStartPose()/getTags() accessors keep working.
//
// Endpoints (all CORS enabled):
//   POST /points  upload a route            -> {ok, version, points, segments, tags}
//   GET  /pose    last reported robot pose  -> {ok, x, y, h, t}   (inches, degrees)
//   GET  /start   start pose of the route   -> {ok, x, y, h}
//   GET  /config  alliance, tags and limits -> {ok, version, alliance, tags:[{index, segment, name, value}],
//                                               robot:{maxSpeedInS, maxStrafeSpeedInS, maxDecelInS2, maxAccelInS2}}
// The robot limits come from setRobotLimits(), which the OpMode calls once at init with the
// numbers from its Pedro Foresight config (inches and seconds). They let the web app preview
// timing with real values. Missing values are published as null.
@Config
public class PathServer extends NanoHTTPD {
    private static final int PORT = 8099;
    private static final String JSON = "application/json";

    public static volatile double[][] RAW_POINTS = {{0, 0, 0}};
    public static volatile String ALLIANCE = "unknown";

    private static volatile double ROBOT_X_IN = 0.0;
    private static volatile double ROBOT_Y_IN = 0.0;
    private static volatile double ROBOT_H_DEG = 0.0;
    private static volatile long ROBOT_TS_MS = 0L;

    public static final class Tag implements Comparable<Tag> {
        public final int index;      // planner node index into [start, ...points] (0 = start pose)
        public final int segment;    // resolved planner segment index, or -1 if unknown
        public final String name;
        public final double value;

        public Tag(String name, double value, int index) {
            this(name, value, index, -1);
        }

        public Tag(String name, double value, int index, int segment) {
            this.name = name;
            this.value = value;
            this.index = index;
            this.segment = segment;
        }

        @Override public int compareTo(Tag other) {
            if (this.segment != other.segment && this.segment >= 0 && other.segment >= 0) {
                return this.segment - other.segment;
            }
            return this.index - other.index;
        }

        @Override public String toString() {
            return name + "=" + value + "@node" + index + "/seg" + segment;
        }
    }

    private static volatile Double ROBOT_MAX_SPEED = null;
    private static volatile Double ROBOT_MAX_STRAFE_SPEED = null;
    private static volatile Double ROBOT_MAX_DECEL = null;
    private static volatile Double ROBOT_MAX_ACCEL = null;

    private static volatile PathRoute ROUTE = PathRoute.empty();
    private static volatile PathServer instance;

    // Starts the embedded HTTP server.
    public static void startServer() {
        if (instance != null) return;
        try {
            instance = new PathServer();
            System.out.println("PathServer started on port " + PORT);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    // Stops the embedded HTTP server.
    public static void stopServer() {
        PathServer s = instance;
        if (s == null) return;
        s.stop();
        instance = null;
        System.out.println("PathServer stopped.");
    }

    // Returns the most recently uploaded route (segments + tags), or an empty route.
    public static PathRoute getRoute() {
        return ROUTE;
    }

    // Replaces the current route, for example one loaded from an asset file.
    public static void setRoute(PathRoute route) {
        applyRoute(route != null ? route : PathRoute.empty());
    }

    // Returns the uploaded path points as Pose2D waypoints, excluding the start pose.
    public static Pose2D[] getPath() {
        double[][] pts = RAW_POINTS;
        if (pts == null || pts.length <= 1) return new Pose2D[0];

        Pose2D[] poses = new Pose2D[pts.length - 1];
        for (int i = 1; i < pts.length; i++) {
            poses[i - 1] = new Pose2D(
                    DistanceUnit.INCH, pts[i][0], pts[i][1],
                    AngleUnit.DEGREES, pts[i][2]
            );
        }
        return poses;
    }

    // Returns the current start pose (index 0 of RAW_POINTS), or (0,0,0) if missing.
    public static Pose2D getStartPose() {
        double[][] pts = RAW_POINTS;
        if (pts == null || pts.length == 0) {
            return new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        }
        double[] s = pts[0];
        return new Pose2D(DistanceUnit.INCH, s[0], s[1], AngleUnit.DEGREES, s[2]);
    }

    // Returns a snapshot of the current tags (each tag knows its segment index).
    public static Tag[] getTags() {
        List<Tag> src = ROUTE.tags;
        return src.toArray(new Tag[0]);
    }

    // Returns the currently selected alliance string.
    public static String getAlliance() {
        return ALLIANCE;
    }

    // Publishes the robot's speed limits (inches, seconds) in /config and the upload response.
    // maxAccel may be null when the robot has no acceleration constraint.
    public static void setRobotLimits(double maxSpeed, double maxStrafeSpeed, double maxDecel, Double maxAccel) {
        ROBOT_MAX_SPEED = finiteOrNull(maxSpeed);
        ROBOT_MAX_STRAFE_SPEED = finiteOrNull(maxStrafeSpeed);
        ROBOT_MAX_DECEL = finiteOrNull(maxDecel);
        ROBOT_MAX_ACCEL = maxAccel == null ? null : finiteOrNull(maxAccel);
    }

    private static Double finiteOrNull(double v) {
        return Double.isFinite(v) ? v : null;
    }

    // Updates the robot pose reported by the /pose endpoint.
    public static void setRobotPose(Pose2D pose) {
        ROBOT_X_IN  = pose.getX(DistanceUnit.INCH);
        ROBOT_Y_IN  = pose.getY(DistanceUnit.INCH);
        ROBOT_H_DEG = pose.getHeading(AngleUnit.DEGREES);
        ROBOT_TS_MS = System.currentTimeMillis();
    }

    // Handles HTTP requests for points upload and telemetry/config reads.
    @Override
    public Response serve(IHTTPSession session) {
        try {
            if (Method.OPTIONS.equals(session.getMethod())) return preflightResponse();

            String uri = session.getUri();
            Method method = session.getMethod();

            if (Method.POST.equals(method) && "/points".equals(uri)) {
                JSONObject payload = readJsonBody(session);
                PathRoute route = PathRoute.fromJson(payload);
                applyRoute(route);

                try { FtcDashboard.getInstance().updateConfig(); } catch (Throwable ignored) {}

                JSONObject ok = new JSONObject()
                        .put("ok", true)
                        .put("version", route.version)
                        .put("points", RAW_POINTS.length)
                        .put("segments", route.segments.size())
                        .put("tags", route.tags.size())
                        .put("robot", robotToJson());
                return withCors(newFixedLengthResponse(Response.Status.OK, JSON, ok.toString()));
            }

            if (Method.GET.equals(method) && "/pose".equals(uri)) {
                JSONObject out = new JSONObject()
                        .put("ok", true)
                        .put("x", ROBOT_X_IN)
                        .put("y", ROBOT_Y_IN)
                        .put("h", ROBOT_H_DEG)
                        .put("t", ROBOT_TS_MS);
                return withCors(newFixedLengthResponse(Response.Status.OK, JSON, out.toString()));
            }

            if (Method.GET.equals(method) && "/start".equals(uri)) {
                Pose2D start = getStartPose();
                JSONObject out = new JSONObject()
                        .put("ok", true)
                        .put("x", start.getX(DistanceUnit.INCH))
                        .put("y", start.getY(DistanceUnit.INCH))
                        .put("h", start.getHeading(AngleUnit.DEGREES));
                return withCors(newFixedLengthResponse(Response.Status.OK, JSON, out.toString()));
            }

            if (Method.GET.equals(method) && "/config".equals(uri)) {
                PathRoute route = ROUTE;
                JSONObject out = new JSONObject()
                        .put("ok", true)
                        .put("version", route.version)
                        .put("alliance", ALLIANCE)
                        .put("tags", tagsToJson(route.tags))
                        .put("robot", robotToJson());
                return withCors(newFixedLengthResponse(Response.Status.OK, JSON, out.toString()));
            }

            return withCors(newFixedLengthResponse(Response.Status.NOT_FOUND, "text/plain", "not found"));
        } catch (Exception e) {
            e.printStackTrace();
            return withCors(newFixedLengthResponse(
                    Response.Status.INTERNAL_ERROR,
                    "text/plain",
                    "error: " + e.getMessage()
            ));
        }
    }

    // Creates and starts the NanoHTTPD server instance.
    private PathServer() throws IOException {
        super(PORT);
        start(SOCKET_READ_TIMEOUT, false);
    }

    // Parses the request body as JSON and returns the root object.
    private static JSONObject readJsonBody(IHTTPSession session) throws IOException, ResponseException, JSONException {
        Map<String, String> files = new HashMap<>();
        session.parseBody(files);

        String body = files.get("postData");
        if (body == null) body = "";
        body = body.trim();

        if (!body.startsWith("{")) throw new IllegalArgumentException("Expected JSON object body");
        return new JSONObject(body);
    }

    // Publishes a parsed route to the static fields read by the accessors above.
    private static void applyRoute(PathRoute route) {
        double[][] pts = new double[route.points.length + 1][3];
        pts[0] = route.start.clone();
        for (int i = 0; i < route.points.length; i++) pts[i + 1] = route.points[i].clone();

        ROUTE = route;
        RAW_POINTS = pts;
        ALLIANCE = route.alliance;
    }

    // Robot limits for the /config endpoint; null entries become JSON null.
    private static JSONObject robotToJson() throws JSONException {
        JSONObject r = new JSONObject();
        r.put("maxSpeedInS", ROBOT_MAX_SPEED != null ? ROBOT_MAX_SPEED : JSONObject.NULL);
        r.put("maxStrafeSpeedInS", ROBOT_MAX_STRAFE_SPEED != null ? ROBOT_MAX_STRAFE_SPEED : JSONObject.NULL);
        r.put("maxDecelInS2", ROBOT_MAX_DECEL != null ? ROBOT_MAX_DECEL : JSONObject.NULL);
        r.put("maxAccelInS2", ROBOT_MAX_ACCEL != null ? ROBOT_MAX_ACCEL : JSONObject.NULL);
        return r;
    }

    // Converts the tag list into a JSON array for the /config endpoint.
    private static JSONArray tagsToJson(List<Tag> tags) throws JSONException {
        JSONArray arr = new JSONArray();
        for (Tag t : tags) {
            arr.put(new JSONObject()
                    .put("index", t.index)
                    .put("segment", t.segment)
                    .put("name", t.name)
                    .put("value", t.value));
        }
        return arr;
    }

    // Builds the CORS response for browser preflight requests.
    private Response preflightResponse() {
        Response r = newFixedLengthResponse(Response.Status.NO_CONTENT, "text/plain", "");
        r.addHeader("Access-Control-Allow-Origin", "*");
        r.addHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        r.addHeader("Access-Control-Allow-Headers", "Content-Type");
        r.addHeader("Access-Control-Allow-Private-Network", "true");
        r.addHeader("Access-Control-Max-Age", "600");
        return r;
    }

    // Adds CORS headers to a normal response.
    private Response withCors(Response r) {
        r.addHeader("Access-Control-Allow-Origin", "*");
        r.addHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        r.addHeader("Access-Control-Allow-Headers", "Content-Type");
        r.addHeader("Access-Control-Allow-Private-Network", "true");
        return r;
    }
}
