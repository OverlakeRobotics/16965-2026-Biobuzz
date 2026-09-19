package org.firstinspires.ftc.teamcode.system;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


// Extension of the BasicHolonomicDrivetrain class adding the ability to use odometry. This adds
// features like heading correction, turning to a specific angle, field centric driving, moving to
// specific positions, and driving along a predefined path.
// Coordinate system used by this class: Positive x is forward, positive y is left, and
// positive heading is turning to the left.
@Config
public class OdometryHolonomicDrivetrain extends BasicHolonomicDrivetrain {
    private static final double P_GAIN = 50;
    private static final double MIN_DIST_TO_STOP = 0.5;
    private static final double COUNTS_PER_DEGREE = 7.39179;
    private static final double MIN_ANGLE_DIF_TO_STOP = 1;
    private double pathTolerance = 4;
    private boolean doPositionHeadingCorrection;
    private boolean positionDriveUsingOdometry;
    public final OdometryModule odometry;
    private double lastHeading;
    private Pose2D currentPosition;
    private Pose2D wantedPosition;
    private double positionDriveDirection;
    private Pose2D[] currentPath;
    private int currentPoint = -1;

    public OdometryHolonomicDrivetrain(DcMotorEx backLeft, DcMotorEx backRight, DcMotorEx frontLeft,
                                       DcMotorEx frontRight, OdometryModule odometry) {
        super(backLeft, backRight, frontLeft, frontRight);
        this.odometry = odometry;
        this.odometry.updatePosition();
        this.currentPosition = this.odometry.getPosition();
        this.wantedPosition = currentPosition;

        lastHeading = this.currentPosition.getHeading(AngleUnit.DEGREES);
    }

    // Behavior: Overrides super classes drive method to include odometry and heading correction.
    @Override
    public void drive() {
        switch (currentDriveState) {
            case STOPPED:
            case VELOCITY_DRIVE:
                break;

            case POSITION_DRIVE:
                if (currentPoint >= 0) {
                    boolean atNextPoint = true;
                    while (atNextPoint) {
                        if ((currentPoint != currentPath.length - 1) && getDistanceToDestination() < pathTolerance) {
                            currentPoint++;
                            wantedPosition = currentPath[currentPoint];
                        } else {
                            atNextPoint = false;
                        }
                    }

                    int nextPoint = currentPoint;
                    setPositionDrive(currentPath[nextPoint]);
                    currentPoint = nextPoint;
                } else if (positionDriveUsingOdometry) {
                    setPositionDrive(wantedPosition);
                } else if (doPositionHeadingCorrection) {
                    setPositionDriveCorrection(getPositionDriveDistanceLeft(), positionDriveDirection, wantedPosition.getHeading(AngleUnit.DEGREES));
                } else {
                    // TODO: Check if this is right, because its using positionDriveDirection, but that is only set if doPositionHeadingCorrection is true.
                    super.setPositionDrive(getPositionDriveDistanceLeft(), positionDriveDirection - currentPosition.getHeading(AngleUnit.DEGREES));
                }
                break;
        }

        lastHeading = currentPosition.getHeading(AngleUnit.DEGREES);
        super.drive();
    }

    // Behavior: Sets the velocity of the robot while accounting for a field centric view.
    //           This means when that the forward value causes the robot to move in the 0 degree
    //           direction, no matter which way the robot is facing. Similarly, the strafe value
    //           will cause the robot to move in the 90 degree direction.
    public void setVelocityDriveFieldCentric(double forward, double strafe, double turn, double angleOffset) {
        double heading = currentPosition.getHeading(AngleUnit.RADIANS) - Math.toRadians(angleOffset);
        super.setVelocityDrive(
                forward * Math.cos(heading) + strafe * Math.sin(heading),
                strafe * Math.cos(heading) - forward * Math.sin(heading),
                turn
        );
    }

    // Behavior: Sets the velocity of the robot while accounting for a field centric view.
    //           This means when that the forward value causes the robot to move in the 0 degree
    //           direction, no matter which way the robot is facing. Similarly, the strafe value
    //           will cause the robot to move in the 90 degree direction.
    public void setVelocityDriveFieldCentric(double forward, double strafe, double turn) {
        this.setVelocityDriveFieldCentric(forward, strafe, turn, 0);
    }

    // Behavior: Overrides basic setPositionDrive so whenever a position drive is set, the state
    //           of the current position drive is reset.
    @Override
    public void setPositionDrive(int backLeftTarget, int backRightTarget, int frontLeftTarget, int frontRightTarget) {
        super.setPositionDrive(backLeftTarget, backRightTarget, frontLeftTarget, frontRightTarget);
        doPositionHeadingCorrection = false;
        positionDriveUsingOdometry = false;
        currentPoint = -1;
    }

    // Behavior: Method just like setPosition drive but also includes a wantedH for heading correction.
    public void setPositionDriveCorrection(int distance, double direction, double wantedH) {
        double dh = normalize(currentPosition.getHeading(AngleUnit.DEGREES) - lastHeading);
        double angleOffset = dh / 2;
        // TODO: Test angle offset by driving in a straight line.
        super.setPositionDrive(distance, normalize(direction - currentPosition.getHeading(AngleUnit.DEGREES) - angleOffset),
                COUNTS_PER_DEGREE * normalize(wantedH - currentPosition.getHeading(AngleUnit.DEGREES)));
        positionDriveDirection = direction;
        setWantedHeading(wantedH);
        doPositionHeadingCorrection = true;
    }

    // Behavior: Overloads the setPositionDrive function to go to a specified location using odometry.
    // Parameters:
    //      - Pose2D wantedPosition: The wanted position (x, y, heading) of the robot.
    public void setPositionDrive(Pose2D wantedPosition) {
        this.wantedPosition = wantedPosition;
        double dx = (this.wantedPosition.getX(DistanceUnit.INCH) - this.currentPosition.getX(DistanceUnit.INCH));
        double dy = (this.wantedPosition.getY(DistanceUnit.INCH) - this.currentPosition.getY(DistanceUnit.INCH));
        int counts = (int) Math.round(Math.hypot(dx * FORWARD_COUNTS_PER_INCH, dy * FORWARD_COUNTS_PER_INCH));
        double direction = Math.toDegrees(Math.atan2(dy, dx));
        setPositionDriveCorrection(counts, direction, wantedPosition.getHeading(AngleUnit.DEGREES));
        positionDriveUsingOdometry = true;
    }

    // Behavior: Has the robot drive along a path defined by a list of sequential points. The tolerance
    //           for how close it has to stick to this path is defined by pathTolerance. A higher value
    //           will give less jitter, and a lower value will give more accuracy.
    // Parameters:
    //      - Pose2D[] path: The path for the robot to follow as an array of Pose2Ds.
    public void setPositionDrive(Pose2D[] path, int initialPointIndex) {
        currentPath = path;
        setPositionDrive(currentPath[initialPointIndex]);
        currentPoint = initialPointIndex;
    }

    // Behavior: Overloads setPositionDrive to default as 0 as the initial point.
    public void setPositionDrive(Pose2D[] path) {
        setPositionDrive(path, 0);
    }

    // Behavior: Sets the tolerance parameter for driving along a path.
    // Parameters:
    //      - double tolerance: How far the robot can deviate from the path in inches. A lower tolerance
    //                          will make the robot slower and jittery but more accurate, while a
    //                          tolerance that is higher will be smoother but less accurate.
    public void setTolerance(double tolerance) {
        pathTolerance = tolerance;
    }

    // Behavior: Gets the distance from the current position to the wanted position.
    // Returns: A double, in inches, containing the distance to the destination.
    public double getDistanceToDestination() {
        return Math.hypot(wantedPosition.getX(DistanceUnit.INCH) - currentPosition.getX(DistanceUnit.INCH),
                wantedPosition.getY(DistanceUnit.INCH) - currentPosition.getY(DistanceUnit.INCH));
    }

    @Override
    public boolean isDriving() {
        if (currentPoint >= 0) {
            return !((currentPoint == currentPath.length - 1) && (getDistanceToDestination() < MIN_DIST_TO_STOP && Math.abs(getTurnCountsLeft() / COUNTS_PER_DEGREE) < MIN_ANGLE_DIF_TO_STOP));
        } else if (positionDriveUsingOdometry) {
            return !(getDistanceToDestination() < MIN_DIST_TO_STOP && Math.abs(getTurnCountsLeft() / COUNTS_PER_DEGREE) < MIN_ANGLE_DIF_TO_STOP);
        }
        return super.isDriving();
    }

    // Behavior: Sets the wanted heading of the robot.
    // Parameters:
    //      - double wantedHeading: The wanted heading of the robot in degrees.
    public void setWantedHeading(double wantedHeading) {
        wantedPosition = new Pose2D(DistanceUnit.INCH, wantedPosition.getX(DistanceUnit.INCH),
                wantedPosition.getY(DistanceUnit.INCH), AngleUnit.DEGREES, wantedHeading);
    }

    // Behavior: Gets the turn velocity needed to heading correct the robot.
    // Returns: The turn velocity.
    public double getHeadingCorrectionVelocity() {
        return P_GAIN * normalize(normalize(wantedPosition.getHeading(AngleUnit.DEGREES)) -
                normalize(currentPosition.getHeading(AngleUnit.DEGREES)));
    }

    // TODO: Don't keep this; just a test
    public double getHeadingCorrectionVelocity(double backupHeadingDegrees) {
        return P_GAIN * normalize(normalize(wantedPosition.getHeading(AngleUnit.DEGREES)) -
                normalize(backupHeadingDegrees));
    }

    // Behavior: Updates the position of the robot. This needs to be called in the loop for accurate
    //           positioning.
    public void updatePosition() {
        odometry.updatePosition();
        currentPosition = odometry.getPosition();
    }

    // Behavior: Gets whether the robot is stopped.
    // Returns: Whether or not the current drive state is stopped.
    public boolean isStopped() {
        return currentDriveState == DriveState.STOPPED;
    }

    // Behavior: Gets the current position of the robot.
    // Returns: A Pose2D with the position of the robot, x, y, and heading.
    public Pose2D getPosition() {
        return currentPosition;
    }

    // Behavior: Sets the position of the robot. For example, if you set the position to (10, -10, 45),
    //           the robot will think its current x and y are (10, -10) and its heading is 45 degrees.
    // Parameters:
    //      - Pose2D position: The position to set the robot to.
    public void setPosition(Pose2D position) {
        odometry.setPosition(position);
        this.updatePosition();
    }

    // Behavior: Normalizes an angle to (-180,180]
    // Returns: The normalized angle
    private static double normalize(double degrees) {
        double normalizedAngle = degrees;
        while (normalizedAngle > 180) normalizedAngle -= 360;
        while (normalizedAngle <= -180) normalizedAngle += 360;
        return normalizedAngle;
    }

    // Behavior: Returns the index of the current point if path driving, otherwise -1.
    // Returns: The index of the current point in the path, or -1 if not path driving.
    public int getNextPointIndex() {
        return currentPoint;
    }

    // Behavior: Gets the velocity of the robot.
    // Returns: The velocity of the robot in inches per second.
    public double getVelocity() {
        return Math.hypot(odometry.getXVelocity(), odometry.getYVelocity());
    }

    // Behavior: Gets the velocity of the robot in the x direction.
    // Returns: The signed x velocity of the robot in inches per second.
    public double getXVelocity() {
        return odometry.getXVelocity();
    }

    // Behavior: Gets the velocity of the robot in the y direction.
    // Returns: The signed y velocity of the robot in inches per second.
    public double getYVelocity() {
        return odometry.getYVelocity();
    }

    // Behavior: Gets the angular velocity of the robot.
    // Returns: The angular velocity of the robot in radians per second.
    public double getAngularVelocity() {
        return odometry.getAngularVelocity();
    }
}
