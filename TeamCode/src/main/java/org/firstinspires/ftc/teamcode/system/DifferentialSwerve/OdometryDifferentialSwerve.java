package org.firstinspires.ftc.teamcode.system.DifferentialSwerve;

import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.system.OdometryModule;

public class OdometryDifferentialSwerve extends BasicDifferentialSwerve {
    public static double xVelocityP = 0.0;
    public static double xVelocityI = 0.0;
    public static double xVelocityD = 0.0;
    public static double yVelocityP = 0.0;
    public static double yVelocityI = 0.0;
    public static double yVelocityD = 0.0;
    public static double turnVelocityP = 0.0;
    public static double turnVelocityI = 0.0;
    public static double turnVelocityD = 0.0;
    protected final OdometryModule odometry;
    protected final PIDFController xVelocityPIDF = new PIDFController(xVelocityP, xVelocityI, xVelocityD, 0.0);
    protected final PIDFController yVelocityPIDF = new PIDFController(yVelocityP, yVelocityI, yVelocityD, 0.0);
    protected final PIDFController turnVelocityPIDF = new PIDFController(turnVelocityP, turnVelocityI, turnVelocityD, 0.0);
    private double pathTolerance = 4;
    protected Pose2D currentPosition;
    protected Pose2D targetPosition;
    private Pose2D[] currentPath;
    private int currentPoint = -1;
    public OdometryDifferentialSwerve(DifferentialSwerveModule[] modules, OdometryModule odometry) {
        super(modules);
        this.odometry = odometry;
    }

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
                            targetPosition = currentPath[currentPoint];
                        } else {
                            atNextPoint = false;
                        }
                    }

                    int nextPoint = currentPoint;
                    setPositionDrive(currentPath[nextPoint]);
                    currentPoint = nextPoint;
                }
                double xVelocity = xVelocityPIDF.calculate(currentPosition.getX(DistanceUnit.INCH), targetPosition.getX(DistanceUnit.INCH));
                double yVelocity = yVelocityPIDF.calculate(currentPosition.getY(DistanceUnit.INCH), targetPosition.getY(DistanceUnit.INCH));
                double turnVelocity = turnVelocityPIDF.calculate(currentPosition.getHeading(AngleUnit.DEGREES), targetPosition.getHeading(AngleUnit.DEGREES));
                setVelocityDriveFieldCentric(xVelocity, yVelocity, turnVelocity);
                break;
        }
        super.drive();
    }

    public void setPositionDrive(double targetXIn, double targetYIn, double targetHeadingDeg) {
        targetPosition = new Pose2D(DistanceUnit.INCH, targetXIn, targetYIn, AngleUnit.DEGREES, targetHeadingDeg);
        currentDriveState = DriveState.POSITION_DRIVE;
        currentPoint = -1;
    }

    public void setPositionDrive(Pose2D targetPosition) {
        this.targetPosition = targetPosition;
        currentDriveState = DriveState.POSITION_DRIVE;
        currentPoint = -1;
    }

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
        return Math.hypot(targetPosition.getX(DistanceUnit.INCH) - currentPosition.getX(DistanceUnit.INCH),
                targetPosition.getY(DistanceUnit.INCH) - currentPosition.getY(DistanceUnit.INCH));
    }

    public void setVelocityDriveFieldCentric(double xVelocity, double yVelocity, double turn, double angleOffset) {
        double currentHeading = Math.toRadians(currentPosition.getHeading(AngleUnit.DEGREES) - angleOffset);
        double robotCentricX = xVelocity * Math.cos(currentHeading) + yVelocity * Math.sin(currentHeading);
        double robotCentricY = -xVelocity * Math.sin(currentHeading) + yVelocity * Math.cos(currentHeading);
        super.setVelocityDrive(robotCentricX, robotCentricY, turn);
    }

    public void setVelocityDriveFieldCentric(double xVelocity, double yVelocity, double turn) {
        setVelocityDriveFieldCentric(xVelocity, yVelocity, turn, 0);
    }

    public void updatePosition() {
        odometry.updatePosition();
        currentPosition = odometry.getPosition();
    }

    public void setPosition(Pose2D position) {
        odometry.setPosition(position);
        this.updatePosition();
    }

    @Override
    public void resetPIDF() {
        this.xVelocityPIDF.reset();
        this.yVelocityPIDF.reset();
        this.turnVelocityPIDF.reset();
        super.resetPIDF();
    }

    @Override
    public void updatePIDFValues() {
        this.xVelocityPIDF.setPIDF(xVelocityP, xVelocityI, xVelocityD, 0.0);
        this.yVelocityPIDF.setPIDF(yVelocityP, yVelocityI, yVelocityD, 0.0);
        this.turnVelocityPIDF.setPIDF(turnVelocityP, turnVelocityI, turnVelocityD, 0.0);
        super.updatePIDFValues();
    }
}
