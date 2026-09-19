package org.firstinspires.ftc.teamcode.components;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.system.OdometryModule;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;


// Implementation of the OdometryModule interface for April Tags.
public class AprilTagOdometry implements OdometryModule {
    private static final double MAX_ACCURATE_DISTANCE = 40;
    private final AprilTagProcessor aprilTagProcessor;
    private Pose2D position;
    private Pose2D startPosition;
    private int positionPriority;
    private int headingPriority;
    private boolean doPositionReset;
    private boolean doHeadingReset;
    private boolean canSeeAprilTag;

    public AprilTagOdometry(AprilTagProcessor aprilTagProcessor, Pose2D startPosition) {
        this.aprilTagProcessor = aprilTagProcessor;

        positionPriority = 1;
        headingPriority = 2;
        doPositionReset = false;
        doHeadingReset = false;
        this.startPosition = startPosition;
    }

    public void updatePosition() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        canSeeAprilTag = false;
        // Process detections that have a valid robotPose
        for (AprilTagDetection detection : detections) {
            if (detection.robotPose != null) {
                // Get robot's position from robotPose
                double xPos = detection.robotPose.getPosition().x - startPosition.getX(DistanceUnit.INCH);
                double yPos = detection.robotPose.getPosition().y - startPosition.getY(DistanceUnit.INCH);
                double hPos = detection.robotPose.getOrientation().getYaw() - startPosition.getHeading(AngleUnit.DEGREES);
                position = new Pose2D(DistanceUnit.INCH, xPos, yPos, AngleUnit.DEGREES, hPos);
                VectorF tagPosition = getTagFieldPosition(detection);
                if (tagPosition != null) {
                    double distance = Math.hypot(tagPosition.get(0) - detection.robotPose.getPosition().x,
                                                 tagPosition.get(1) - detection.robotPose.getPosition().y);
                    if (distance < MAX_ACCURATE_DISTANCE) {
                        canSeeAprilTag = true;
                    }
                }
                break;
            }
        }
    }

    // SDK 12.0 split AprilTagDetection into single-tag and cluster detections, each with its own
    // metadata type, so look up the tag's field position based on which kind this is.
    private static VectorF getTagFieldPosition(AprilTagDetection detection) {
        if (detection instanceof AprilTagSingleDetection) {
            AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
            return single.metadata == null ? null : single.metadata.fieldPosition;
        } else if (detection instanceof AprilTagClusterDetection) {
            AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
            return cluster.metadata == null ? null : cluster.metadata.fieldPosition;
        }
        return null;
    }

    public Pose2D getPosition() {
        return position;
    }

    public void setPosition(Pose2D position) {
        double xDiff = this.position.getX(DistanceUnit.INCH) - position.getX(DistanceUnit.INCH);
        double yDiff = this.position.getY(DistanceUnit.INCH) - position.getY(DistanceUnit.INCH);
        double angleDiff = this.position.getHeading(AngleUnit.DEGREES) - position.getHeading(AngleUnit.DEGREES);
        this.startPosition = new Pose2D(DistanceUnit.INCH,
                                     this.startPosition.getX(DistanceUnit.INCH) + xDiff,
                                     this.startPosition.getY(DistanceUnit.INCH) + yDiff,
                                        AngleUnit.DEGREES,
                                this.startPosition.getHeading(AngleUnit.DEGREES) + angleDiff);
    }

    public void setPositionPriority(int priority) {
        positionPriority = priority;
    }

    public int getPositionPriority() {
        return positionPriority;
    }

    public void setHeadingPriority(int priority) {
        headingPriority = priority;
    }

    public int getHeadingPriority() {
        return headingPriority;
    }

    public void setDoPositionResetToHigherPriority(boolean doReset) {
        doPositionReset = doReset;
    }

    public boolean doPositionResetToHigherPriority() {
        return doPositionReset;
    }

    public void setDoHeadingResetToHigherPriority(boolean doReset) {
        doHeadingReset = doReset;
    }

    public boolean doHeadingResetToHigherPriority() {
        return doHeadingReset;
    }

    public boolean isPositionAccurate() {
        return canSeeAprilTag;
    }

    public boolean isHeadingAccurate() {
        return canSeeAprilTag;
    }

    public void reset() {
        setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
    }

    public void recalibrate() {}
}
