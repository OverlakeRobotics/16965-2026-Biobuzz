package org.firstinspires.ftc.teamcode.system;


import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


// The opposite of OdometryModuleLocalizer: exposes a Pedro Follower's localization as one of this
// library's OdometryModules, so code written against OdometryModule (aimers, telemetry, an
// OdometryCollection) can read the pose Pedro is driving with.
//
// updatePosition() is intentionally a no-op: the follower is updated by Follower.update() in the
// OpMode loop, and doing that here as well would drive the motors twice per loop.
public class FollowerOdometryModule implements OdometryModule {
    private final Follower follower;
    private int positionPriority = 0;
    private int headingPriority = 0;
    private boolean doPositionReset = false;
    private boolean doHeadingReset = false;

    public FollowerOdometryModule(Follower follower) {
        this.follower = follower;
    }

    public Follower follower() {
        return follower;
    }

    @Override
    public Pose2D getPosition() {
        Pose p = follower.pose();
        return new Pose2D(DistanceUnit.INCH, p.x(), p.y(), AngleUnit.DEGREES, Math.toDegrees(p.heading()));
    }

    @Override
    public void updatePosition() {
        // Handled by Follower.update() in the OpMode loop.
    }

    @Override
    public void setPosition(Pose2D position) {
        follower.setPose(new Pose(position.getX(DistanceUnit.INCH), position.getY(DistanceUnit.INCH),
                position.getHeading(AngleUnit.RADIANS)));
    }

    @Override
    public void reset() {
        follower.setPose(Pose.zero());
    }

    @Override public double getXVelocity() { return velocity().vx; }
    @Override public double getYVelocity() { return velocity().vy; }
    @Override public double getAngularVelocity() { return velocity().omega; }

    private Velocity velocity() {
        Velocity v = follower.velocity();
        return v != null ? v : Velocity.zero();
    }

    @Override public void setPositionPriority(int priority) { positionPriority = priority; }
    @Override public int getPositionPriority() { return positionPriority; }
    @Override public void setHeadingPriority(int priority) { headingPriority = priority; }
    @Override public int getHeadingPriority() { return headingPriority; }
    @Override public void setDoPositionResetToHigherPriority(boolean doReset) { doPositionReset = doReset; }
    @Override public boolean doPositionResetToHigherPriority() { return doPositionReset; }
    @Override public void setDoHeadingResetToHigherPriority(boolean doReset) { doHeadingReset = doReset; }
    @Override public boolean doHeadingResetToHigherPriority() { return doHeadingReset; }
    @Override public boolean isPositionAccurate() { return true; }
    @Override public boolean isHeadingAccurate() { return true; }
}
