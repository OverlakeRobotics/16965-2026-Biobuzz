package org.firstinspires.ftc.teamcode.system;


import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.HashMap;
import java.util.Map;


// Adapts any OdometryModule from this library (a single sensor, or an OdometryCollection fusing
// several, e.g. Pinpoint + AprilTags) into a Pedro Pathing Localizer, so it can be handed to a
// Pedro Follower:
//     new Follower(new OdometryModuleLocalizer(odometry), new Mecanum(h, cfg), new Foresight(cfg))
//
// The module's velocity getters are used when implemented; otherwise velocity is estimated by
// finite differencing the position between update() calls. Both frames use inches, x forward,
// y left, heading counter-clockwise; Pedro headings are radians.
public class OdometryModuleLocalizer implements Localizer {
    private final OdometryModule module;
    private MotionState state = MotionState.zero();
    private Pose lastPose = null;
    private long lastNanos = 0;
    private boolean useModuleVelocity = true;

    public OdometryModuleLocalizer(OdometryModule module) {
        this.module = module;
        update();
    }

    public OdometryModule module() {
        return module;
    }

    @Override
    public void setPose(Pose pose) {
        module.setPosition(new Pose2D(DistanceUnit.INCH, pose.x(), pose.y(), AngleUnit.RADIANS, pose.heading()));
        state = state.withPose(pose);
        lastPose = pose;
        lastNanos = System.nanoTime();
    }

    @Override
    public MotionState state() {
        return state;
    }

    @Override
    public void update() {
        module.updatePosition();
        Pose2D p = module.getPosition();
        long now = System.nanoTime();
        if (p == null) {
            lastNanos = now;
            return;
        }
        Pose pose = new Pose(p.getX(DistanceUnit.INCH), p.getY(DistanceUnit.INCH), p.getHeading(AngleUnit.RADIANS));

        Velocity velocity = null;
        if (useModuleVelocity) {
            try {
                velocity = new Velocity(module.getXVelocity(), module.getYVelocity(), module.getAngularVelocity());
            } catch (UnsupportedOperationException e) {
                useModuleVelocity = false;
            }
        }
        if (velocity == null) {
            double dt = (now - lastNanos) / 1e9;
            if (lastPose != null && dt > 1e-4) {
                double dh = Math.atan2(Math.sin(pose.heading() - lastPose.heading()), Math.cos(pose.heading() - lastPose.heading()));
                velocity = new Velocity((pose.x() - lastPose.x()) / dt, (pose.y() - lastPose.y()) / dt, dh / dt);
            } else {
                velocity = state.velocity();
            }
        }

        state = MotionState.ofVelocity(pose, velocity);
        lastPose = pose;
        lastNanos = now;
    }

    @Override
    public void reset() {
        module.reset();
        state = MotionState.zero();
        lastPose = null;
        lastNanos = System.nanoTime();
    }

    @Override
    public Map<String, Object> debug() {
        Map<String, Object> out = new HashMap<>();
        out.put("module", module.getClass().getSimpleName());
        out.put("moduleVelocity", useModuleVelocity);
        out.put("pose", state.pose().toString());
        return out;
    }
}
