package org.firstinspires.ftc.teamcode.system;


import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.ivy.pedro.PedroCommands;

import java.util.ArrayList;
import java.util.List;


// Builds ONE Ivy command that drives a whole Path Planner route on a Pedro Follower.
//
// For every chunk from PedroPathBuilder the command is
//     parallel(follow(follower, chunkPath),
//              sequential(waitUntil(pathIndex >= tagSubPath), tagCommand), ...)
// and between chunks (at every stop) it is
//     sequential(hold(follower, poseAtStop), parallel(wait(pauseSeconds), stopTagCommands...))
//
// Tag commands come from a TagHandler the OpMode implements. Two tags are built in:
//   velocity -> instant command setting Foresight's maxPathSpeed to value / 100 (a value <= 1 is
//               taken as a fraction already). maxPathSpeed is a fraction of the robot's max speed.
//   pause    -> handled by splitting the route into chunks and waiting at the split.
// When the last chunk finishes the follower keeps holding the final pose on its own
// (Follower.holdEnd is true by default), so no explicit hold command is needed at the end.
public final class PathPlanRunner {
    public static final String TAG_VELOCITY = "velocity";

    // What a tag command can look at while it runs.
    public static final class TagContext {
        public final Follower follower;
        public final PedroPathBuilder.Chunk chunk;
        public final int chunkIndex;
        public final int subPathIndex;
        public final boolean atStop;   // fired while the robot is stopped at the start of the chunk

        TagContext(Follower follower, PedroPathBuilder.Chunk chunk, int chunkIndex, int subPathIndex, boolean atStop) {
            this.follower = follower;
            this.chunk = chunk;
            this.chunkIndex = chunkIndex;
            this.subPathIndex = subPathIndex;
            this.atStop = atStop;
        }

        // True while the follower is on the tag's own sub-path.
        public boolean segmentActive() {
            return follower.following() && follower.pathIndex() == subPathIndex;
        }

        // True while the follower is still following the tag's chunk.
        public boolean chunkActive() {
            return follower.following();
        }

        // True once the follower has moved past the tag's sub-path (or stopped following).
        public boolean segmentPassed() {
            return !follower.following() || follower.pathIndex() > subPathIndex;
        }
    }

    // Implemented by the OpMode. Return null to ignore a tag.
    public interface TagHandler {
        Command commandFor(String name, double value);

        // Richer variant; defaults to the simple one.
        default Command commandFor(PathServer.Tag tag, TagContext context) {
            return commandFor(tag.name, tag.value);
        }

        // Drive command for a chunk the SegmentPolicy marked custom. Return null to fall back to
        // holding the end pose of the segment.
        default Command driveSegment(PathServer.Tag[] tags, TagContext context) {
            return null;
        }
    }

    private PathPlanRunner() {}

    // Builds the route command.
    public static Command build(Follower follower, List<PedroPathBuilder.Chunk> chunks, TagHandler handler) {
        List<Command> steps = new ArrayList<>();

        // The Foresight config is usually a static in Constants, so a "velocity" tag from a previous
        // run would otherwise still be capping the speed. Start every route at full speed.
        Command resetSpeed = resetSpeedCommand(follower);
        if (resetSpeed != null) steps.add(resetSpeed);

        for (int ci = 0; ci < chunks.size(); ci++) {
            PedroPathBuilder.Chunk chunk = chunks.get(ci);

            if (chunk.stopBefore) {
                List<Command> atStop = new ArrayList<>();
                double pause = chunk.pauseSeconds();
                if (pause > 0) atStop.add(Commands.waitMs(pause * 1000.0));
                for (PedroPathBuilder.PlacedTag t : chunk.stopTags()) {
                    Command c = commandFor(follower, handler, t.tag, new TagContext(follower, chunk, ci, 0, true));
                    if (c != null) atStop.add(c);
                }
                Command hold = PedroCommands.hold(follower, chunk.startPose);
                if (atStop.isEmpty()) steps.add(hold);
                else steps.add(Groups.sequential(hold, Groups.parallel(atStop.toArray(new Command[0]))));
            }

            List<Command> during = new ArrayList<>();
            if (chunk.custom) {
                List<PathServer.Tag> raw = new ArrayList<>();
                for (PedroPathBuilder.PlacedTag t : chunk.tags) raw.add(t.tag);
                Command drive = handler != null
                        ? handler.driveSegment(raw.toArray(new PathServer.Tag[0]), new TagContext(follower, chunk, ci, 0, false))
                        : null;
                during.add(drive != null ? drive : PedroCommands.hold(follower, chunk.endPose));
            } else if (chunk.path != null) {
                during.add(PedroCommands.follow(follower, chunk.path));
                for (PedroPathBuilder.PlacedTag t : chunk.followTags()) {
                    final int k = t.subPathIndex;
                    Command c = commandFor(follower, handler, t.tag, new TagContext(follower, chunk, ci, k, false));
                    if (c == null) continue;
                    // Fire when the sub-path starts, or at the latest when the chunk ends.
                    during.add(Groups.sequential(
                            Commands.waitUntil(() -> !follower.following() || follower.pathIndex() >= k),
                            c));
                }
            } else {
                // Turn-in-place only chunk: nothing to follow, just settle on the new heading.
                during.add(PedroCommands.hold(follower, chunk.endPose));
                for (PedroPathBuilder.PlacedTag t : chunk.followTags()) {
                    Command c = commandFor(follower, handler, t.tag, new TagContext(follower, chunk, ci, 0, true));
                    if (c != null) during.add(c);
                }
            }

            steps.add(during.size() == 1 ? during.get(0) : Groups.parallel(during.toArray(new Command[0])));
        }

        if (steps.isEmpty()) return Command.NOOP;
        if (steps.size() == 1) return steps.get(0);
        return Groups.sequential(steps.toArray(new Command[0]));
    }

    // Built-in tags first, then the handler.
    private static Command commandFor(Follower follower, TagHandler handler, PathServer.Tag tag, TagContext ctx) {
        if (TAG_VELOCITY.equals(tag.name)) {
            Command c = velocityCommand(follower, tag.value);
            if (c != null) return c;
        }
        if (PedroPathBuilder.TAG_PAUSE.equals(tag.name)) return null;   // handled by the split
        return handler != null ? handler.commandFor(tag, ctx) : null;
    }

    // Instant command that caps Foresight's path speed. Returns null when the follower does not
    // run Foresight, so a TagHandler can take over.
    public static Command velocityCommand(Follower follower, double value) {
        Algorithm algorithm = follower.algorithm();
        if (!(algorithm instanceof Foresight)) return null;
        Foresight foresight = (Foresight) algorithm;
        double fraction = speedFraction(value);
        return Commands.instant(() -> foresight.config.maxPathSpeed.set(fraction));
    }

    // Instant command that removes any path speed cap (Foresight only).
    public static Command resetSpeedCommand(Follower follower) {
        Algorithm algorithm = follower.algorithm();
        if (!(algorithm instanceof Foresight)) return null;
        Foresight foresight = (Foresight) algorithm;
        return Commands.instant(() -> foresight.config.maxPathSpeed.set(ForesightConfig.Constraint.NONE));
    }

    // Publishes the follower's Foresight limits to PathServer (see PathServer.setRobotLimits) so
    // the web app can preview timing. Does nothing for followers that do not run Foresight.
    public static void publishRobotLimits(Follower follower) {
        Algorithm algorithm = follower.algorithm();
        if (!(algorithm instanceof Foresight)) return;
        publishRobotLimits(((Foresight) algorithm).config);
    }

    public static void publishRobotLimits(ForesightConfig c) {
        double pathSpeed = c.maxPathSpeed.get();
        if (!Double.isFinite(pathSpeed) || pathSpeed <= 0) pathSpeed = 1.0;   // Constraint.NONE
        pathSpeed = Math.min(1.0, pathSpeed);

        double maxSpeed = c.maxAchievableForwardVelocity.get() * pathSpeed;
        double velocityConstraint = c.maxVelocityConstraint.get();
        if (Double.isFinite(velocityConstraint)) maxSpeed = Math.min(maxSpeed, velocityConstraint);

        double maxStrafe = c.maxAchievableStrafeVelocity.get() * pathSpeed;
        double maxDecel = c.naturalForwardDeceleration.get();
        double accel = c.maxAccelerationConstraint.get();
        Double maxAccel = Double.isFinite(accel) ? accel : null;

        PathServer.setRobotLimits(maxSpeed, maxStrafe, maxDecel, maxAccel);
    }

    // Converts a planner velocity tag value into a 0..1 fraction of max speed.
    public static double speedFraction(double value) {
        double f = value <= 1.0 ? value : value / 100.0;
        if (f <= 0) return 1.0;
        return Math.min(1.0, f);
    }
}
