package org.firstinspires.ftc.teamcode.components;


import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.system.OdometryModule;


// Implementation of the OdometryModule interface for SparkFunOTOS sensor.
// TODO: Check if x and y are correct.
public class SparkFunOTOSOdometry implements OdometryModule {
    private final SparkFunOTOS sparkFunSensor;
    private SparkFunOTOS.Pose2D position;
    private int positionPriority;
    private int headingPriority;
    private boolean doPositionReset;
    private boolean doHeadingReset;

    public SparkFunOTOSOdometry(SparkFunOTOS sparkFunSensor) {
        this.sparkFunSensor = sparkFunSensor;

        positionPriority = 5;
        headingPriority = 5;
        doPositionReset = true;
        doHeadingReset = false;
    }

    public void updatePosition() {
        position = sparkFunSensor.getPosition();
    }

    public Pose2D getPosition() {
        return new Pose2D(DistanceUnit.INCH, position.x, position.y, AngleUnit.DEGREES, position.h);
    }

    public void setPosition(Pose2D position) {
        sparkFunSensor.setPosition(new SparkFunOTOS.Pose2D(position.getX(DistanceUnit.INCH),
                position.getY(DistanceUnit.INCH), position.getHeading(AngleUnit.DEGREES)));
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
        return true;
    }

    public boolean isHeadingAccurate() {
        return true;
    }

    public void reset() {
        sparkFunSensor.resetTracking();
    }
}
