package org.firstinspires.ftc.teamcode.code.helpers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.code.RobotHardware;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.system.FollowerOdometryModule;
import org.firstinspires.ftc.teamcode.system.PathPlanRunner;
import org.firstinspires.ftc.teamcode.system.PathRoute;
import org.firstinspires.ftc.teamcode.system.PathServer;
import org.firstinspires.ftc.teamcode.system.PedroPathBuilder;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.Reader;
import java.util.List;

// Base autonomous: follows a Path Planner route with Pedro Pathing (Follower from pedro/Constants)
// and runs the robot-specific tags as Ivy commands. Subclasses either point jsonFilename at an asset
// (readJson = true) or, like PathPlanner, hand in the route uploaded to PathServer.
//
// Tags:
//   velocity, pause            built into PathPlanRunner / PedroPathBuilder
//   intake                     > 0 runs intake + transfer in, < 0 reverses, 0 stops
//   autoAim                    face the CELL and keep both shooters aimed on this segment
//   pollenVelocity, nectarVelocity   set that flywheel's velocity (ticks/s)
//   pollenHood, nectarHood           set that hood's servo position (0 to 1)
//   launchArtifacts            stop at the node and shoot for <value> seconds
//   startLaunch / endLaunch    shoot while moving between the two tags
@Config
public abstract class BaseAuto extends OpMode implements PathPlanRunner.TagHandler {
    // Where the robot was when Auto ended, so TeleOp starts from there instead of a guessed pose
    public static Pose2D lastPose = null;

    public static double launchHeadingRetargetDeg = 0.5;

    protected Reader jsonReader;
    protected boolean readJson;
    protected String jsonFilename;
    public double shooterDelay = 0.0;
    public String alliance;

    protected Follower follower;
    protected FollowerOdometryModule odometry;
    protected RobotHardware robot;
    protected AutoAimer autoAimer;

    // The route to run. PathPlanner sets this from PathServer before calling start(); asset based
    // autos fill it in start() from jsonFilename.
    protected PathRoute route;
    protected List<PedroPathBuilder.Chunk> chunks;
    private Command routeCommand;
    private Command shootOnMoveService;

    private boolean isShooting;
    private double intakePower = 0;   // what the intake tag last asked for, restored after shooting

    // Reads a route from jsonReader (planner export, version 1 or 2).
    public void parseJsonFromString() throws IOException, JSONException {
        BufferedReader br = new BufferedReader(jsonReader);
        StringBuilder sb = new StringBuilder();
        String line;
        while ((line = br.readLine()) != null) sb.append(line);
        br.close();
        route = PathRoute.fromJson(new JSONObject(sb.toString()));
        alliance = route.alliance;
    }

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        PathPlanRunner.publishRobotLimits(follower);   // lets the planner preview timing with real limits
        odometry = new FollowerOdometryModule(follower);

        robot = new RobotHardware(hardwareMap);
        autoAimer = new AutoAimer(robot, odometry::getPosition, false);
    }

    @Override
    public void init_loop() {
        follower.update();
    }

    @Override
    public void start() {
        if (readJson) {
            try {
                if (jsonFilename != null && !jsonFilename.isEmpty()) {
                    InputStream is = hardwareMap.appContext.getAssets().open(jsonFilename);
                    jsonReader = new InputStreamReader(is);
                }
                parseJsonFromString();
            } catch (IOException | JSONException e) {
                throw new RuntimeException(e);
            }
        }
        if (route == null) route = PathRoute.empty();
        if (alliance == null) alliance = route.alliance;
        autoAimer.setRed("red".equals(alliance));

        follower.setPose(PedroPathBuilder.toPose(route.start));

        PedroPathBuilder.Options options = new PedroPathBuilder.Options();
        options.policy = (segmentIndex, segment, tags) -> {
            if (PedroPathBuilder.hasTag(tags, "pause")) return true;
            for (PathServer.Tag t : tags) if ("launchArtifacts".equals(t.name) && t.value > 0) return true;
            return false;
        };
        // Segments tagged autoAim keep the shooters (left side) pointed at the CELL
        options.headings = (segmentIndex, segment, tags, startH, endH) ->
                PedroPathBuilder.hasTag(tags, "autoAim") ? autoAimInterpolator() : null;

        chunks = PedroPathBuilder.build(route, options);
        routeCommand = PathPlanRunner.build(follower, chunks, this);

        // Runs for the whole auto and shoots whenever startLaunch turned shooting on.
        shootOnMoveService = Command.build()
                .setExecute(() -> { if (isShooting) shootLoop(0); })
                .setDone(() -> false);

        robot.closeBlockers();
        Scheduler.schedule(routeCommand, shootOnMoveService);
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        Pose2D pos = odometry.getPosition();
        telemetry.addData("Pose", "%.1f, %.1f, %.1f", pos.getX(DistanceUnit.INCH),
                pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Following", follower.following());
        telemetry.addData("Sub-path", follower.pathIndex());
        telemetry.addData("Route done", routeCommand != null && !routeCommand.isScheduled());
        telemetry.addData("Target side", autoAimer.getActiveSide().name);
        telemetry.addData("Ready to shoot", autoAimer.readyToShoot());
        robot.pollen.addTelemetry(telemetry, "Pollen");
        robot.nectar.addTelemetry(telemetry, "Nectar");
        telemetry.update();
    }

    @Override
    public void stop() {
        lastPose = odometry.getPosition();
        Scheduler.reset();
        if (follower != null) follower.stop();
    }

    // ---- aiming helpers -------------------------------------------------------------------

    // Heading interpolator that always returns the heading AutoAimer wants (radians).
    private Interpolator autoAimInterpolator() {
        return (curve, t) -> Math.toRadians(autoAimer.getAimHeading());
    }

    // One loop of shooting: aim, and feed when both shooters are ready and the delay has passed.
    private void shootLoop(double delayLeft) {
        robot.openBlockers();
        autoAimer.aimShooters();
        if (delayLeft <= 0 && autoAimer.readyToShoot()) {
            robot.intake.in();
        } else {
            robot.intake.stop();
        }
    }

    private void setIntake(double power) {
        intakePower = power;
        if (power > 0) robot.intake.in();
        else if (power < 0) robot.intake.reverse();
        else robot.intake.stop();
    }

    // ---- tags ------------------------------------------------------------------------------

    @Override
    public Command commandFor(String name, double value) {
        return commandFor(new PathServer.Tag(name, value, -1), null);
    }

    @Override
    public Command commandFor(PathServer.Tag tag, PathPlanRunner.TagContext ctx) {
        switch (tag.name) {
            case "intake":
                return Commands.instant(() -> setIntake(tag.value));

            case "autoAim":
                // Heading is handled by the interpolator; keep both shooters aimed while this
                // segment is being followed.
                return Command.build()
                        .setExecute(autoAimer::aimShooters)
                        .setDone(() -> ctx == null || ctx.segmentPassed());

            case "pollenVelocity":
                return Commands.instant(() -> robot.pollen.setVelocity(tag.value));

            case "nectarVelocity":
                return Commands.instant(() -> robot.nectar.setVelocity(tag.value));

            case "pollenHood":
                return Commands.instant(() -> robot.pollen.setHoodPosition(tag.value));

            case "nectarHood":
                return Commands.instant(() -> robot.nectar.setHoodPosition(tag.value));

            case "launchArtifacts":
                if (tag.value <= 0) return null;
                return launchArtifacts(tag.value, ctx);

            case "startLaunch":
                return Commands.instant(() -> {
                    robot.openBlockers();
                    isShooting = true;
                });

            case "endLaunch":
                return Commands.instant(() -> {
                    isShooting = false;
                    robot.closeBlockers();
                    setIntake(intakePower);
                });

            default:
                return null;
        }
    }

    // Stopped shooting sequence: hold the node turned toward the CELL, spin up, feed once ready, and
    // close again after <seconds>.
    private Command launchArtifacts(double seconds, PathPlanRunner.TagContext ctx) {
        final ElapsedTime timer = new ElapsedTime();
        final Pose[] holdPose = new Pose[1];
        return Command.build()
                .setStart(() -> {
                    timer.reset();
                    robot.openBlockers();
                    holdPose[0] = ctx != null ? ctx.chunk.startPose : follower.pose();
                })
                .setExecute(() -> {
                    shootLoop(shooterDelay - timer.seconds());
                    // The shooters are fixed to the robot, so the whole robot turns to aim
                    double wantedDeg = autoAimer.getAimHeading();
                    double currentDeg = Math.toDegrees(holdPose[0].heading());
                    if (Math.abs(normalize(wantedDeg - currentDeg)) > launchHeadingRetargetDeg) {
                        holdPose[0] = holdPose[0].withHeading(Math.toRadians(wantedDeg));
                        follower.hold(holdPose[0]);
                    }
                })
                .setDone(() -> timer.seconds() >= seconds)
                .setEnd(end -> {
                    robot.closeBlockers();
                    setIntake(intakePower);
                });
    }

    private static double normalize(double degrees) {
        double a = degrees;
        while (a > 180) a -= 360;
        while (a <= -180) a += 360;
        return a;
    }
}
