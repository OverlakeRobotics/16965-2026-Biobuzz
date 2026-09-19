package org.firstinspires.ftc.teamcode.code.helpers;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.code.helpers.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.code.parts.Intake;
import org.firstinspires.ftc.teamcode.code.parts.LEDIndicator;
import org.firstinspires.ftc.teamcode.code.parts.Turret;
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
import java.util.Comparator;
import java.util.List;
import java.util.stream.Stream;

// Base autonomous: follows a Path Planner route with Pedro Pathing (Follower from
// pedro/Constants) and runs the robot-specific tags as Ivy commands. Subclasses either point
// jsonFilename at an asset (readJson = true) or, like PathPlanner, hand in the route uploaded to
// PathServer.
//
// Tags:
//   velocity, pause        built into PathPlanRunner / PedroPathBuilder
//   intake                 set intake velocity (<= 0 stops it)
//   autoAim                aim the drivetrain heading, hood and shooter at the goal on this segment
//   shooterVelocity        set shooter velocity
//   hoodAngle              set hood angle
//   launchArtifacts        stop at the node and shoot for <value> seconds
//   startLaunch/endLaunch  shoot while moving between the two tags
//   shootWhileMove         enable (1) / disable (0) the moving-shot compensation
//   autoArtifactPickup     drive this segment with the HuskyLens steering onto artifacts
@Config
public abstract class BaseAuto extends OpMode implements PathPlanRunner.TagHandler {
    protected Reader jsonReader;
    protected boolean readJson;
    protected String jsonFilename;

    protected Follower follower;
    protected FollowerOdometryModule odometry;
    private Intake intake;
    private LEDIndicator ledIndicator;
    private Turret turret;
    private AutoAligner autoAligner;
    private Limelight3A limelight;
    private HuskyLens huskyLens;

    // The route to run. PathPlanner sets this from PathServer before calling start(); asset based
    // autos fill it in start() from jsonFilename.
    protected PathRoute route;
    protected List<PedroPathBuilder.Chunk> chunks;
    private Command routeCommand;
    private Command shootOnMoveService;

    public static double hoodAngleVelScale = 0.5;
    public static double turretPreturnConstant = 0; //0.1;

    public static double huskyLensXP = 0.1;
    public static int huskyLensCenter = 160;
    public static double huskyLensTargetY = 67;
    public static double huskyLensRetargetInches = 1.0;
    public static double shootingIntakeVelocity = 2800;
    public static double launchHeadingRetargetDeg = 0.5;

    private double wantedShooterVelocity = 0;
    private boolean isShooting;
    public double shooterDelay = 0.0; // 0.5;
    public String alliance;
    private int intakeVelocity;

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
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        PathPlanRunner.publishRobotLimits(follower);   // lets the planner preview timing with real limits
        odometry = new FollowerOdometryModule(follower);

        turret = new Turret(
                hardwareMap.get(DcMotorEx.class, "shooterTop"),
                hardwareMap.get(DcMotorEx.class, "shooterBottom"),
                hardwareMap.get(DcMotorEx.class, "turret"),
                hardwareMap.get(Servo.class, "hood"),
                hardwareMap.get(Servo.class, "blocker"),
                hardwareMap.get(AnalogInput.class, "potentiometer")
        );
        turret.resetTurretEncoder();

        intake = new Intake(
                hardwareMap.get(DcMotorEx.class, "intake"),
                hardwareMap.get(DistanceSensor.class, "lowerDistanceSensor"),
                hardwareMap.get(DistanceSensor.class, "middleDistanceSensor"),
                hardwareMap.get(NormalizedColorSensor.class, "upperColorSensor")
        );
        ledIndicator = new LEDIndicator(hardwareMap.get(GoBildaPrismDriver.class, "prism"));
        ledIndicator.setState(Intake.IntakeState.AMBIENT);
        turret.setEncoderOffset();

        telemetry.addData("Turret Angle", turret.getTurretCurrentAngle());
        telemetry.update();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        autoAligner = new AutoAligner(odometry, turret, limelight, false);

        AutoAligner.farShooterTolerance = 80;
        AutoAligner.farTurretTolerance = 6;

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
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
        if ("red".equals(alliance)) autoAligner.setRed(); else autoAligner.setBlue();

        follower.setPose(PedroPathBuilder.toPose(route.start));

        PedroPathBuilder.Options options = new PedroPathBuilder.Options();
        options.policy = new PedroPathBuilder.SegmentPolicy() {
            @Override
            public boolean stopsBefore(int segmentIndex, PathRoute.Segment segment, List<PathServer.Tag> tags) {
                if (PedroPathBuilder.hasTag(tags, "pause")) return true;
                for (PathServer.Tag t : tags) if ("launchArtifacts".equals(t.name) && t.value > 0) return true;
                return false;
            }

            @Override
            public boolean isCustom(int segmentIndex, PathRoute.Segment segment, List<PathServer.Tag> tags) {
                return PedroPathBuilder.hasTag(tags, "autoArtifactPickup");
            }
        };
        // Segments tagged autoAim keep their heading pointed so the turret can reach the goal.
        options.headings = (segmentIndex, segment, tags, startH, endH) ->
                PedroPathBuilder.hasTag(tags, "autoAim") ? autoAimInterpolator() : null;

        chunks = PedroPathBuilder.build(route, options);
        routeCommand = PathPlanRunner.build(follower, chunks, this);

        // Runs for the whole auto and shoots whenever startLaunch turned shooting on.
        shootOnMoveService = Command.build()
                .setExecute(() -> { if (isShooting) shootLoop(0); })
                .setDone(() -> false);

        turret.close();
        Scheduler.schedule(routeCommand, shootOnMoveService);
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        Pose2D pos = odometry.getPosition();
        autoAligner.updateInterpolation(pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH));

        double wantedTurretAngle = autoAligner.getTurretAutoAlignAngle();
        turret.setTurretAngle(wantedTurretAngle + autoAligner.lastVelPerp * turretPreturnConstant);

        telemetry.addData("Pose", "%.1f, %.1f, %.1f", pos.getX(DistanceUnit.INCH),
                pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Following", follower.following());
        telemetry.addData("Sub-path", follower.pathIndex());
        telemetry.addData("Route done", routeCommand != null && !routeCommand.isScheduled());
        telemetry.update();
    }

    @Override
    public void stop() {
        Scheduler.reset();
        if (follower != null) follower.stop();
        ledIndicator.setState(Intake.IntakeState.OFF);
    }

    // ---- aiming helpers -------------------------------------------------------------------

    // Heading interpolator that always returns the drivetrain angle the aligner wants (radians).
    private Interpolator autoAimInterpolator() {
        return (curve, t) -> Math.toRadians(autoAligner.getDrivetrainAutoAlignAngleWithTurret());
    }

    // Points hood and shooter at the goal.
    public void autoAim() {
        double hoodAngle = autoAligner.getOptimalHoodAngle()
                - (autoAligner.useShootMove ? autoAligner.lastVelPar * hoodAngleVelScale : 0);
        turret.setHoodAngle(hoodAngle);
        wantedShooterVelocity = autoAligner.getShooterVelocityFromAngle(hoodAngle);
        turret.setShooterVelocity(wantedShooterVelocity);
    }

    // One loop of shooting: aim, and feed when the shooter is ready and the delay has passed.
    private void shootLoop(double delayLeft) {
        turret.open();
        autoAim();
        intake.setVelocity(delayLeft <= 0 && autoAligner.readyToShoot() ? shootingIntakeVelocity : 0);
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
                return Commands.instant(() -> {
                    if (tag.value <= 0) {
                        intakeVelocity = 0;
                        intake.stop();
                    } else {
                        intakeVelocity = (int) Math.round(tag.value);
                        intake.setVelocity(intakeVelocity);
                    }
                });

            case "autoAim":
                // Heading is handled by the interpolator; keep hood and shooter tracking the goal
                // while this segment is being followed.
                return Command.build()
                        .setStart(() -> { if ("red".equals(alliance)) autoAligner.setRed(); else autoAligner.setBlue(); })
                        .setExecute(this::autoAim)
                        .setDone(() -> ctx == null || ctx.segmentPassed());

            case "shooterVelocity":
                return Commands.instant(() -> turret.setShooterVelocity(tag.value));

            case "hoodAngle":
                return Commands.instant(() -> turret.setHoodAngle(tag.value));

            case "launchArtifacts":
                if (tag.value <= 0) return null;
                return launchArtifacts(tag.value, ctx);

            case "startLaunch":
                return Commands.instant(() -> {
                    turret.open();
                    isShooting = true;
                });

            case "endLaunch":
                return Commands.instant(() -> {
                    isShooting = false;
                    turret.close();
                    intake.setVelocity(intakeVelocity);
                });

            case "shootWhileMove":
                return Commands.instant(() -> autoAligner.useShootMove = tag.value == 1);

            case "autoArtifactPickup":
                return null;   // drives its own segment, see driveSegment()

            case "tolerance":
            default:
                return null;
        }
    }

    // Stopped shooting sequence: hold the node (turning towards the goal if the turret cannot
    // reach it), spin up, feed once ready, and close again after <seconds>.
    private Command launchArtifacts(double seconds, PathPlanRunner.TagContext ctx) {
        final ElapsedTime timer = new ElapsedTime();
        final Pose[] holdPose = new Pose[1];
        return Command.build()
                .setStart(() -> {
                    timer.reset();
                    turret.open();
                    holdPose[0] = ctx != null ? ctx.chunk.startPose : follower.pose();
                })
                .setExecute(() -> {
                    shootLoop(shooterDelay - timer.seconds());
                    // Re-aim the held pose if the turret alone cannot reach the goal.
                    double wantedDeg = autoAligner.getDrivetrainAutoAlignAngleWithTurret();
                    double currentDeg = Math.toDegrees(holdPose[0].heading());
                    double diff = Math.abs(normalize(wantedDeg - currentDeg));
                    if (diff > launchHeadingRetargetDeg) {
                        holdPose[0] = holdPose[0].withHeading(Math.toRadians(wantedDeg));
                        follower.hold(holdPose[0]);
                    }
                })
                .setDone(() -> timer.seconds() >= seconds)
                .setEnd(end -> {
                    turret.close();
                    intake.setVelocity(intakeVelocity);
                });
    }

    // Drives an autoArtifactPickup segment: follow the planned line, but once the HuskyLens sees
    // an artifact, run the intake and steer towards it (x corrected by the camera error, y to the
    // pickup line) until the follower arrives.
    @Override
    public Command driveSegment(PathServer.Tag[] tags, PathPlanRunner.TagContext ctx) {
        final Pose start = ctx.chunk.startPose;
        final Pose end = ctx.chunk.endPose;
        final boolean[] pickingUp = new boolean[1];
        final Pose[] target = new Pose[1];
        return Command.build()
                .setStart(() -> {
                    pickingUp[0] = false;
                    target[0] = end;
                    follower.follow(Paths.line(start, end).linear(start.heading(), end.heading()));
                })
                .setExecute(() -> {
                    HuskyLens.Block largestBlock = Stream.of(huskyLens.blocks())
                            .max(Comparator.comparingDouble(block -> block.width * block.height))
                            .orElse(null);
                    if (largestBlock == null) return;

                    double error = (largestBlock.x - huskyLensCenter) * huskyLensXP;
                    if (!pickingUp[0]) {
                        intake.setVelocity(shootingIntakeVelocity);
                        pickingUp[0] = true;
                    }
                    Pose here = follower.pose();
                    Pose wanted = new Pose(here.x() + error, huskyLensTargetY, here.heading());
                    if (wanted.distance(target[0]) > huskyLensRetargetInches && here.distance(wanted) > PedroPathBuilder.MIN_SEGMENT_LENGTH_IN) {
                        target[0] = wanted;
                        follower.follow(Paths.line(here, wanted).constant(here.heading()));
                    }
                })
                .setDone(() -> !follower.following())
                .setEnd(condition -> {
                    pickingUp[0] = false;
                    intake.setVelocity(0);
                });
    }

    private static double normalize(double degrees) {
        double a = degrees;
        while (a > 180) a -= 360;
        while (a <= -180) a += 360;
        return a;
    }
}
