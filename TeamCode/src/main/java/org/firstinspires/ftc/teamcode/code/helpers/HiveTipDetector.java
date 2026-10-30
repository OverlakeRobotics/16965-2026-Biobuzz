package org.firstinspires.ftc.teamcode.code.helpers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.code.FieldMap;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseRaw;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;
import java.util.function.Supplier;

// Which way each HIVE is tipped, from an upward-facing USB webcam that looks at the AprilTag clusters under
// the CELLS. Tracks both HIVES at once; ask for either with getTip(isRed).
//
// A HIVE is a see-saw with two stable states, rotated 30 degrees one way or the other (manual Fig 9-10). The
// raised CELL faces up (that's the one to LAUNCH into); the lowered one points down and has spilled. The tags
// are on the bottom of each CELL, and both CELLS' tags face roughly down in either state, so an upward camera
// can see the lowered CELL's tags too: just seeing a tag doesn't say which way the HIVE is tipped. What does is
// how high the tag is. The raised CELL's tags are ~50 in above the tiles, the lowered CELL's ~36 in.
//
// How it decides, in order:
//   1. SEEN: a tag cluster of the HIVE is in view. Its height above the tiles (the vertical part of the
//      camera-to-tag distance, so it doesn't need odometry) says whether that CELL is raised or lowered.
//   2. INFERRED: nothing of the HIVE is in view, but from where the robot is, one state would put a cluster
//      squarely in the camera's view while the other state couldn't show any. Seeing nothing for a while
//      then means the second state. Needs the robot pose. This rarely decides anything: from below, a CELL's
//      cluster is in nearly the same spot either way (~13.1 vs ~11.4 in from the pivot), only its height
//      changes, so wherever one state's cluster would surely be in view the other's usually could be too.
//      Tightening the "maybe" limits below to what the camera really detects (measure it) lets it fire more.
//   3. Otherwise the last answer is kept. The HIVE is bi-stable and only tips when scored into, so the last
//      state stays right until the camera sees otherwise. Before anything is seen, it starts from the
//      match-start state in the manual (assumeMatchStart), so getTip() only returns UNKNOWN if that's turned off.
//
// Results use FieldMap's LEFT/RIGHT CELL names for the given alliance (as the driver sees them: blue LEFT is
// the +X CELL, red is the 180 degree rotation). Which physical CELL each name means is read from FieldMap's
// aim points, so this follows FieldMap if the names ever change. For your own alliance,
// getTip(isRed).toTargetSide() is the CELL to shoot at.
//
// Field axes and positions are FieldMap's (inches, origin at field center, +Y = blue HIVE half, +X 90 degrees
// clockwise from +Y, heading CCW from +X). The tags are on the CELLS per manual Fig 9-17, where +X is the
// audience side: 38-41 (blue) and 34-37 (red) on the +X CELLS, 42-45 (blue) and 30-33 (red) on the -X CELLS.
// HIVE dimensions are from the Competition Manual section 9 (+/- 1 in).
@Config
public class HiveTipDetector {
    // Name of the webcam in the robot configuration
    public static String webcamName = "Webcam 1";

    // Camera mount, measured on the robot. Offsets are from the robot center (the odometry point).
    public static double cameraForwardOffset = 0;
    public static double cameraLeftOffset = 0;
    public static double cameraHeight = 10;          // lens height above the tiles
    // Which way the TOP edge of the camera image points, degrees CCW from robot forward (0 = front, 90 = left)
    public static double cameraImageTopDirection = 0;
    // How far the lens leans from straight up toward the image's top edge (degrees; 0 = straight up)
    public static double cameraTiltDegrees = 0;

    // Visibility model for INFERRED. "Sure" = a cluster there would definitely be detected, "maybe" = it might
    // be. Keep sure tight and maybe loose: a wrong "sure" gives a wrong answer, a wrong "maybe" just means no
    // inference. Half FOV is the cone around the lens axis (sure: half the narrow side of the image; maybe:
    // half the diagonal). View angle is between the tag's face and the line to the camera (0 = head-on).
    public static double sureHalfFov = 20;
    public static double sureRange = 55;
    public static double sureViewAngle = 40;
    public static double maybeHalfFov = 40;
    public static double maybeRange = 110;
    public static double maybeViewAngle = 75;
    // How long nothing must be seen while the robot is somewhere that rules out one state
    public static double absenceConfirmSeconds = 0.75;

    // Consecutive camera frames that must agree before a seen tip replaces the current answer
    public static int framesToConfirm = 2;
    // Total height evidence (inches past the threshold) needed to call it; less = the HIVE is mid-tip
    public static double undecidedBand = 2;
    // Tag heights further than this outside the possible range are bad pose solves and are ignored
    public static double plausibleError = 10;

    // Start from the manual's match setup (Fig 10-2: the blue HIVE starts with its -X CELL up, red with its +X
    // CELL up, i.e. RIGHT on both). Turn off to get UNKNOWN until the camera sees a HIVE.
    public static boolean assumeMatchStart = true;

    // ---- HIVE geometry -------------------------------------------------------------------------------------

    public static final double TIP_ANGLE = 30;                    // each stable state, Fig 9-10
    // The blue HIVE's centerline: the CELL aim points' Y (13; the manual's 25.5 center to center gives 12.75)
    public static final double HIVE_CENTER_Y = FieldMap.LEFT.aimPoint.y;
    // Pivot to the CELL's front (opening) end along the arm: half the HIVE's 42.91 length (Fig 9-9)
    public static final double CELL_FRONT_FROM_PIVOT = 42.91 / 2;
    // Tag cluster center to the CELL front: 9.938 to the reference holes minus 2.75 back up to the tag
    // centers (Fig 9-15). Same as the 7.1874 offset in the SDK's AprilTagGameDatabase.
    public static final double TAG_FROM_FRONT = 7.1874;
    public static final double TAG_ALONG_ARM = CELL_FRONT_FROM_PIVOT - TAG_FROM_FRONT;
    // How far the bottom of a CELL is below the pivot, with the HIVE level. From the raised CELL's bottom of
    // opening at 53.5 above the tiles (Fig 9-10): 53.5 = pivot + front * sin(30) - this * cos(30)
    public static final double TAG_BELOW_PIVOT = (FieldMap.HIVE_PIVOT_HEIGHT
            + CELL_FRONT_FROM_PIVOT * Math.sin(Math.toRadians(TIP_ANGLE)) - 53.5) / Math.cos(Math.toRadians(TIP_ANGLE));

    // Tag cluster center height above the tiles when its CELL is raised (~49.9) or lowered (~35.6), and the
    // midpoint between them that splits the two
    public static final double RAISED_TAG_HEIGHT = tagHeight(true);
    public static final double LOWERED_TAG_HEIGHT = tagHeight(false);
    public static final double TAG_HEIGHT_THRESHOLD = (RAISED_TAG_HEIGHT + LOWERED_TAG_HEIGHT) / 2;

    // Where the tag cluster's center is in the SDK's cluster frame (AprilTagGameDatabase.getBioBuzzCluster):
    // the cluster origin is the CELL opening, so this is the offset back to the tags themselves
    private static final double CLUSTER_TAG_CENTER_Y = 7.1874;
    private static final double CLUSTER_TAG_CENTER_Z = -5.622;

    // ---- results -------------------------------------------------------------------------------------------

    // Which CELL of the HIVE is raised / facing up, in FieldMap's names for that alliance
    public enum Tip {
        LEFT, RIGHT, UNKNOWN;

        // The CELL to LAUNCH into (blue-side TargetSide, like AutoAimer uses), or null if UNKNOWN
        public FieldMap.TargetSide toTargetSide() {
            return this == LEFT ? FieldMap.LEFT : this == RIGHT ? FieldMap.RIGHT : null;
        }

        public Tip other() {
            return this == LEFT ? RIGHT : this == RIGHT ? LEFT : UNKNOWN;
        }
    }

    // Where the current answer came from
    public enum Source { SEEN, INFERRED, MATCH_START, OVERRIDE, NONE }

    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;   // null if the processor came from somewhere else
    private final Supplier<Pose2D> pose;
    private final ElapsedTime timer = new ElapsedTime();
    private final HiveState blue = new HiveState(false);
    private final HiveState red = new HiveState(true);

    // Opens the webcam (webcamName) with its own AprilTag processor.
    // pose: where the robot is right now (driveTrain::getPosition in TeleOp, the follower in Auto), or null to
    // skip INFERRED
    public HiveTipDetector(HardwareMap hardwareMap, Supplier<Pose2D> pose) {
        this.pose = pose;
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, webcamName))
                .addProcessor(aprilTag)
                .build();
    }

    // Uses an AprilTag processor that's already attached to the upward camera. Nothing else may call
    // getFreshDetections() on it, or this will miss those frames.
    public HiveTipDetector(AprilTagProcessor aprilTag, Supplier<Pose2D> pose) {
        this.pose = pose;
        this.aprilTag = aprilTag;
        this.visionPortal = null;
    }

    // ---- per loop ------------------------------------------------------------------------------------------

    // Call every loop. Does nothing until the camera has a new frame.
    public void update() {
        List<AprilTagDetection> detections = aprilTag.getFreshDetections();
        if (detections == null) return;
        Pose2D robotPose = pose == null ? null : pose.get();
        double now = timer.seconds();
        blue.process(detections, robotPose, now);
        red.process(detections, robotPose, now);
    }

    // Which CELL of that HIVE is up. Pass your own alliance to get the CELL you should be shooting at.
    public Tip getTip(boolean isRed) {
        return hive(isRed).tip;
    }

    public Source getSource(boolean isRed) {
        return hive(isRed).source;
    }

    // How long since the camera last saw or inferred this HIVE's state (infinite if it never has)
    public double getSecondsSinceConfirmed(boolean isRed) {
        double last = hive(isRed).lastConfirmed;
        return Double.isNaN(last) ? Double.POSITIVE_INFINITY : timer.seconds() - last;
    }

    // Set the state by hand (e.g. a driver button, or from Auto). Kept until the camera sees otherwise.
    public void setTip(boolean isRed, Tip tip) {
        HiveState hive = hive(isRed);
        hive.tip = tip;
        hive.source = tip == Tip.UNKNOWN ? Source.NONE : Source.OVERRIDE;
        hive.lastConfirmed = timer.seconds();
        hive.pendingFrames = 0;
        hive.absenceTip = Tip.UNKNOWN;
    }

    public void addTelemetry(Telemetry telemetry, boolean isRed) {
        HiveState hive = hive(isRed);
        String name = isRed ? "Red HIVE" : "Blue HIVE";
        telemetry.addData(name + " up", "%s (%s, %.1f s ago)", hive.tip, hive.source, getSecondsSinceConfirmed(isRed));
        telemetry.addData(name + " tags seen", hive.lastTagsSeen);
        if (hive.lastTagsSeen > 0) {
            telemetry.addData(name + " tag height", "%.1f in (raised %.1f / lowered %.1f)",
                    hive.lastHeight, RAISED_TAG_HEIGHT, LOWERED_TAG_HEIGHT);
        }
    }

    // Stops the camera, if this class opened it
    public void close() {
        if (visionPortal != null) visionPortal.close();
    }

    private HiveState hive(boolean isRed) {
        return isRed ? red : blue;
    }

    // ---- one HIVE ------------------------------------------------------------------------------------------

    private class HiveState {
        final boolean isRed;
        Tip tip;
        Source source;
        double lastConfirmed = Double.NaN;
        // A seen tip that differs from the current one, waiting for framesToConfirm frames in a row
        Tip pendingTip = Tip.UNKNOWN;
        int pendingFrames = 0;
        // What the empty view says, and since when it has said it
        Tip absenceTip = Tip.UNKNOWN;
        double absenceStart = 0;
        // For telemetry
        int lastTagsSeen = 0;
        double lastHeight = Double.NaN;

        HiveState(boolean isRed) {
            this.isRed = isRed;
            // In the blue-side frame the -X CELL starts up on both HIVES
            tip = assumeMatchStart ? sideAtBlueX(-1) : Tip.UNKNOWN;
            source = assumeMatchStart ? Source.MATCH_START : Source.NONE;
        }

        void process(List<AprilTagDetection> detections, Pose2D robotPose, double now) {
            // Each cluster votes by how far its height is past the threshold (+ = LEFT up). Adding them up lets a
            // clear view outvote a poor one if two clusters ever disagree.
            double leftUpEvidence = 0;
            int tagsSeen = 0;
            boolean anyDetected = false;   // includes detections without a usable height
            for (AprilTagDetection detection : detections) {
                Cell cell = Cell.of(detection);
                if (cell == null || cell.isRed != isRed) continue;
                anyDetected = true;
                double height = tagHeightAboveTiles(detection);
                if (Double.isNaN(height)
                        || height < LOWERED_TAG_HEIGHT - plausibleError
                        || height > RAISED_TAG_HEIGHT + plausibleError) continue;
                tagsSeen++;
                lastHeight = height;
                double raisedBy = height - TAG_HEIGHT_THRESHOLD;
                leftUpEvidence += cell.side() == Tip.LEFT ? raisedBy : -raisedBy;
            }
            lastTagsSeen = tagsSeen;

            if (tagsSeen > 0) {
                absenceTip = Tip.UNKNOWN;
                if (Math.abs(leftUpEvidence) < undecidedBand) {
                    pendingFrames = 0;   // in the middle of tipping: wait for it to settle
                    return;
                }
                seen(leftUpEvidence > 0 ? Tip.LEFT : Tip.RIGHT, now);
                return;
            }

            pendingFrames = 0;
            if (anyDetected) {
                // Its tags are in view but couldn't be measured, so the view isn't empty: nothing to infer
                absenceTip = Tip.UNKNOWN;
                return;
            }
            Tip inferred = robotPose == null ? Tip.UNKNOWN : inferFromEmptyView(robotPose);
            if (inferred == Tip.UNKNOWN) {
                absenceTip = Tip.UNKNOWN;
                return;
            }
            if (inferred != absenceTip) {
                absenceTip = inferred;
                absenceStart = now;
            }
            if (now - absenceStart >= absenceConfirmSeconds) {
                if (tip != inferred) {
                    tip = inferred;
                    source = Source.INFERRED;
                } else if (source == Source.MATCH_START || source == Source.NONE) {
                    source = Source.INFERRED;
                }
                lastConfirmed = now;
            }
        }

        private void seen(Tip seenTip, double now) {
            if (seenTip == tip) {
                pendingFrames = 0;
                source = Source.SEEN;
                lastConfirmed = now;
                return;
            }
            if (seenTip == pendingTip) {
                pendingFrames++;
            } else {
                pendingTip = seenTip;
                pendingFrames = 1;
            }
            if (pendingFrames >= framesToConfirm) {
                tip = seenTip;
                source = Source.SEEN;
                lastConfirmed = now;
                pendingFrames = 0;
            }
        }

        // Nothing of this HIVE is in view. If one state would surely show a cluster from here and the other
        // couldn't show any, it's the other one. Otherwise no conclusion.
        private Tip inferFromEmptyView(Pose2D robotPose) {
            // Work on the blue side, where FieldMap's aim points are written
            Pose2D p = FieldMap.forAlliance(robotPose, isRed);
            double x = p.getX(DistanceUnit.INCH);
            double y = p.getY(DistanceUnit.INCH);
            double h = p.getHeading(AngleUnit.RADIANS);
            double[] camera = {
                    x + cameraForwardOffset * Math.cos(h) - cameraLeftOffset * Math.sin(h),
                    y + cameraForwardOffset * Math.sin(h) + cameraLeftOffset * Math.cos(h),
                    cameraHeight
            };
            double top = h + Math.toRadians(cameraImageTopDirection);
            double tilt = Math.toRadians(cameraTiltDegrees);
            double[] axis = {Math.sin(tilt) * Math.cos(top), Math.sin(tilt) * Math.sin(top), Math.cos(tilt)};

            boolean leftUpSure = anyClusterVisible(Tip.LEFT, camera, axis, sureHalfFov, sureRange, sureViewAngle);
            boolean leftUpMaybe = anyClusterVisible(Tip.LEFT, camera, axis, maybeHalfFov, maybeRange, maybeViewAngle);
            boolean rightUpSure = anyClusterVisible(Tip.RIGHT, camera, axis, sureHalfFov, sureRange, sureViewAngle);
            boolean rightUpMaybe = anyClusterVisible(Tip.RIGHT, camera, axis, maybeHalfFov, maybeRange, maybeViewAngle);

            if (leftUpSure && !rightUpMaybe) return Tip.RIGHT;
            if (rightUpSure && !leftUpMaybe) return Tip.LEFT;
            return Tip.UNKNOWN;
        }
    }

    // True if, with the HIVE in state 'up', either CELL's cluster would be detected from this camera spot
    // (blue side coordinates)
    private static boolean anyClusterVisible(Tip up, double[] camera, double[] axis,
                                             double halfFovDeg, double range, double viewAngleDeg) {
        for (Tip side : new Tip[]{Tip.LEFT, Tip.RIGHT}) {
            boolean raised = side == up;
            double out = sideX(side);   // which way is "outward" along X for this CELL
            double phi = Math.toRadians(raised ? TIP_ANGLE : -TIP_ANGLE);
            // Tag center and the direction its face points (the CELL bottom's outward normal, down when level)
            double[] tag = {
                    out * (TAG_ALONG_ARM * Math.cos(phi) + TAG_BELOW_PIVOT * Math.sin(phi)),
                    HIVE_CENTER_Y,
                    FieldMap.HIVE_PIVOT_HEIGHT + TAG_ALONG_ARM * Math.sin(phi) - TAG_BELOW_PIVOT * Math.cos(phi)
            };
            double[] normal = {out * Math.sin(phi), 0, -Math.cos(phi)};

            double[] toTag = {tag[0] - camera[0], tag[1] - camera[1], tag[2] - camera[2]};
            double distance = Math.sqrt(dot(toTag, toTag));
            if (distance < 1e-6 || distance > range) continue;
            double offAxis = Math.toDegrees(Math.acos(clamp(dot(toTag, axis) / distance)));
            // Angle between the tag's face direction and the line from the tag back to the camera
            double viewAngle = Math.toDegrees(Math.acos(clamp(-dot(toTag, normal) / distance)));
            if (offAxis <= halfFovDeg && viewAngle <= viewAngleDeg) return true;
        }
        return false;
    }

    // Which way along X (blue side) a CELL is from the pivot, +1 or -1, from FieldMap's aim points
    // (currently LEFT = +X, RIGHT = -X)
    private static double sideX(Tip side) {
        return Math.signum((side == Tip.LEFT ? FieldMap.LEFT : FieldMap.RIGHT).aimPoint.x);
    }

    // The CELL on the +X (blueX > 0) or -X (blueX < 0) end of the HIVE, on the blue side
    private static Tip sideAtBlueX(double blueX) {
        return Math.signum(blueX) == sideX(Tip.LEFT) ? Tip.LEFT : Tip.RIGHT;
    }

    // ---- measuring a detection -----------------------------------------------------------------------------

    // Height of the detection's tag center above the tiles, NaN if it has no pose
    static double tagHeightAboveTiles(AprilTagDetection detection) {
        AprilTagPoseRaw raw = detection.rawPose;
        if (raw == null) return Double.NaN;
        DistanceUnit unit = detection.distanceUnit;

        // Tag center in the raw camera frame (x right, y down, z out of the lens)
        double x = raw.x, y = raw.y, z = raw.z;
        if (detection instanceof AprilTagClusterDetection) {
            // The cluster pose is at the CELL opening; move back to the tags: p = R * offset + t
            MatrixF r = raw.R;
            double oy = unit.fromUnit(DistanceUnit.INCH, CLUSTER_TAG_CENTER_Y);
            double oz = unit.fromUnit(DistanceUnit.INCH, CLUSTER_TAG_CENTER_Z);
            x += r.get(0, 1) * oy + r.get(0, 2) * oz;
            y += r.get(1, 1) * oy + r.get(1, 2) * oz;
            z += r.get(2, 1) * oy + r.get(2, 2) * oz;
        }

        // Up component. Straight up it's just the distance out of the lens; tilted toward the image top by a,
        // up = out * cos(a) + down-the-image * sin(a).
        double tilt = Math.toRadians(cameraTiltDegrees);
        double up = z * Math.cos(tilt) + y * Math.sin(tilt);
        return cameraHeight + DistanceUnit.INCH.fromUnit(unit, up);
    }

    // Tag cluster center height above the tiles with its CELL raised or lowered
    private static double tagHeight(boolean raised) {
        double phi = Math.toRadians(raised ? TIP_ANGLE : -TIP_ANGLE);
        return FieldMap.HIVE_PIVOT_HEIGHT + TAG_ALONG_ARM * Math.sin(phi) - TAG_BELOW_PIVOT * Math.cos(phi);
    }

    // Which CELL a detection's tags are on (Fig 9-17, +X = audience side): the "AUDIENCE" clusters are on the
    // +X CELLS, the "SCORING" clusters (opposite the audience) on the -X CELLS. With FieldMap's names that's
    // blue LEFT = 38-41, blue RIGHT = 42-45, red LEFT = 30-33, red RIGHT = 34-37.
    private enum Cell {
        RED_SCORING(true, -1, 30, "RED SCORING"),
        RED_AUDIENCE(true, 1, 34, "RED AUDIENCE"),
        BLUE_AUDIENCE(false, 1, 38, "BLUE AUDIENCE"),
        BLUE_SCORING(false, -1, 42, "BLUE SCORING");

        final boolean isRed;
        final double fieldX;   // which end of the field the CELL is on: +1 = +X, -1 = -X
        final int firstId;
        final String clusterName;

        Cell(boolean isRed, double fieldX, int firstId, String clusterName) {
            this.isRed = isRed;
            this.fieldX = fieldX;
            this.firstId = firstId;
            this.clusterName = clusterName;
        }

        // This CELL's name for its alliance. Red is the blue side rotated 180 degrees, so X flips.
        Tip side() {
            return sideAtBlueX(isRed ? -fieldX : fieldX);
        }

        // The SDK library reports these tags as one cluster per CELL; a custom library might report single tags
        static Cell of(AprilTagDetection detection) {
            for (Cell cell : values()) {
                if (detection instanceof AprilTagClusterDetection) {
                    AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                    if (cluster.metadata != null && cell.clusterName.equals(cluster.metadata.name)) return cell;
                } else if (detection instanceof AprilTagSingleDetection) {
                    int id = ((AprilTagSingleDetection) detection).id;
                    if (id >= cell.firstId && id < cell.firstId + 4) return cell;
                }
            }
            return null;
        }
    }

    private static double dot(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    private static double clamp(double v) {
        return Math.max(-1, Math.min(1, v));
    }
}
