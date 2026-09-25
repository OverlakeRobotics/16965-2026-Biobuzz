package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.util.ElapsedTime;

import android.util.Log;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.system.PathPlanRunner;
import org.firstinspires.ftc.teamcode.system.PathRoute;
import org.firstinspires.ftc.teamcode.system.PathServer;
import org.firstinspires.ftc.teamcode.system.PedroPathBuilder;

import java.util.List;


// This is an example OpMode designed to interface with the Overlake Robotics Path Planner.
// It lets you upload a path from the planner to the robot and follows it with Pedro Pathing,
// sequencing everything with Ivy commands. Two tags are handled for you by PathPlanRunner:
//      - velocity: Caps the robot's speed. The value is a percent of max speed (100 = full speed);
//                  a value of 1 or less is taken as a fraction.
//      - pause: Stops the robot for a number of seconds equal to the tag's value.
// Add your own tags in commandFor() below.
//
// Robot setup: this example uses the Follower from pedro/Constants.java (Pinpoint localizer +
// Mecanum drivetrain + Foresight). If you copy this OpMode to another robot, copy
// pedro/Constants.java as well and run the Pedro tuners (pedro/Tuning.java) to fill it in.
//@Disabled
@Config
@Autonomous(name = "Path Planning Example", group = "Autonomous")
public class PathPlanExample extends OpMode implements PathPlanRunner.TagHandler {
    private Follower follower;
    private Command routeCommand;
    private List<PedroPathBuilder.Chunk> chunks;
    private final ElapsedTime runtime = new ElapsedTime();
    private double finishedAt = -1;

    @Override
    public void init() {
        Scheduler.reset();   // the scheduler is static, so clear anything left from a previous OpMode
        follower = Constants.create(hardwareMap);
        PathPlanRunner.publishRobotLimits(follower);   // lets the planner preview timing with real limits
        PathServer.startServer();
    }

    @Override
    public void init_loop() {
        // Show the planner where the robot thinks it is (the planner works in degrees).
        follower.update();
        PathServer.setRobotPose(toPose2D(follower.pose()));
        telemetry.addData("Route", PathServer.getRoute().segments.size() + " segments, "
                + PathServer.getRoute().tags.size() + " tags");
        telemetry.update();
    }

    @Override
    public void start() {
        PathRoute route = PathServer.getRoute();
        follower.setPose(PedroPathBuilder.toPose(route.start));

        chunks = PedroPathBuilder.build(route);                       // split at "pause" tags
        routeCommand = PathPlanRunner.build(follower, chunks, this);  // one command for the whole route
        Scheduler.schedule(routeCommand);
        finishedAt = -1;
        runtime.reset();
    }

    @Override
    public void loop() {
        follower.update();      // localizes and drives the motors
        Scheduler.execute();    // advances the route command and any tag commands

        PathServer.setRobotPose(toPose2D(follower.pose()));
        boolean routeDone = routeCommand != null && !routeCommand.isScheduled();
        if (routeDone && finishedAt < 0) {
            finishedAt = runtime.seconds();
            Log.d("PathPlanExample", "Finished route in " + finishedAt + " s");
        }
        Pose p = follower.pose();
        telemetry.addData("Follower", "Pedro Foresight");
        telemetry.addData("Elapsed", "%.2f s", runtime.seconds());
        telemetry.addData("Following", follower.following());
        telemetry.addData("Sub-path", follower.pathIndex());
        telemetry.addData("Pose", "x %.1f  y %.1f  h %.1f", p.x(), p.y(), Math.toDegrees(p.heading()));
        telemetry.addData("Route done", routeDone);
        if (finishedAt >= 0) telemetry.addData("Finished in", "%.2f s", finishedAt);
        telemetry.update();
    }

    @Override
    public void stop() {
        Scheduler.reset();
        follower.stop();
        PathServer.stopServer();
    }

    // Called once per tag while the route command is built. Return the command to run when the
    // robot reaches the tag, or null to ignore the tag. "velocity" and "pause" never get here.
    @Override
    public Command commandFor(String name, double value) {
        switch (name) {
            case "log":
                return Commands.instant(() -> telemetry.log().add("Reached log tag " + value));
            // Add more cases here for your own custom tags! Long running behaviors can be built
            // with Command.build().setStart(...).setExecute(...).setDone(...).
            default:
                return null;
        }
    }

    private static Pose2D toPose2D(Pose p) {
        return new Pose2D(DistanceUnit.INCH, p.x(), p.y(), AngleUnit.DEGREES, Math.toDegrees(p.heading()));
    }
}
