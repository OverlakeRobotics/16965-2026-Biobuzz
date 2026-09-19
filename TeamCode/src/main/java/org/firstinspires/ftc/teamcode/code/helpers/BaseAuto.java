package org.firstinspires.ftc.teamcode.code.helpers;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
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
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointOdometry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.system.BasicHolonomicDrivetrain;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.Reader;
import java.util.Arrays;
import java.util.Comparator;
import java.util.stream.Stream;

import org.firstinspires.ftc.teamcode.system.OdometryHolonomicDrivetrain;
import org.firstinspires.ftc.teamcode.system.PathServer;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

@Config
public abstract class BaseAuto extends OpMode {
    public static final double yOffset = -156.0; // -168.0 // mm
    public static final double xOffset = 72.0; // -84.0 // mm

    protected int velocity;
    protected int intakeVelocity;
    protected double tolerance;
    protected Reader jsonReader;
    protected boolean readJson;
    protected String jsonFilename;
    protected OdometryHolonomicDrivetrain driveTrain;
    private Intake intake;
    private LEDIndicator ledIndicator;
    private Turret turret;
    private AutoAligner autoAligner;
    private Limelight3A limelight;
    private HuskyLens huskyLens;
    private final ElapsedTime runtime = new ElapsedTime();

    protected Pose2D[] positions;
    protected PathServer.Tag[] tags;

    public static double p = 30; // 19; // 30;
    public static double i = 0; // 0.1; // 0;
    public static double d = 7; // 0; // 7;
    public static double f = 0;
    public static double positionP = 3;

    public static double hoodAngleVelScale = 0.5;

    public static double turretPreturnConstant = 0; //0.1;

    public static double huskyLensXP = 0.1;
    public static int huskyLensCenter = 160;
//    public static int searchVelocity = 1500;

    private double wantedShooterVelocity = 0;
    private boolean isShooting;
    public double shooterDelay = 0.0; // 0.5;
    private double shooterTimer = 0;
    public String alliance;

    protected Pose2D startPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    private int lastTagIndex = 0;
    private double lastTime = 0;
    private double pauseTimeLeft = 0;
    private int pausedIndex = -1;
    private int autoAlignIndex = -1;
    private boolean searchingForArtifacts = false;
    private boolean pickingUp = false;
    private int searchEndIndex = -1;
    // Gets the data for a path from a json, as a string.
    public void parseJsonFromString() throws IOException, JSONException {
        BufferedReader br = new BufferedReader(jsonReader);
        StringBuilder sb = new StringBuilder();
        String line;
        while ((line = br.readLine()) != null) sb.append(line);
        JSONObject root = new JSONObject(sb.toString());

        // Velocity (inches/sec) -> encoder counts/sec
        double velInches = root.optDouble("velocity", 0);
        velocity = (int) (velInches * BasicHolonomicDrivetrain.FORWARD_COUNTS_PER_INCH);

        // Tolerance (inches)
        tolerance = root.optDouble("tolerance", 1.0);

        alliance = root.optString("alliance", alliance);
        alliance = alliance.trim().toLowerCase();

        // Start pose (support object or array formats)
        double sx = 0;
        double sy = 0;
        double sh = 0;
        Object startObj = root.opt("start");
        if (startObj instanceof JSONArray) {
            JSONArray s = (JSONArray) startObj;
            sx = s.optDouble(0, 0);
            sy = s.optDouble(1, 0);
            sh = s.optDouble(2, 0);
        } else if (startObj instanceof JSONObject) {
            JSONObject s = (JSONObject) startObj;
            sx = s.optDouble("x", 0);
            sy = s.optDouble("y", 0);
            sh = s.optDouble("h", 0);
        }
        startPose = new Pose2D(DistanceUnit.INCH, sx, sy, AngleUnit.DEGREES, sh);

        // Waypoints -> positions array
        JSONArray pts = root.getJSONArray("points");
        positions = new Pose2D[pts.length()];
        for (int i = 0; i < pts.length(); i++) {
            Object pObj = pts.get(i);
            double x = 0;
            double y = 0;
            double h = 0;
            if (pObj instanceof JSONArray) {
                JSONArray p = (JSONArray) pObj;
                x = p.optDouble(0, 0);
                y = p.optDouble(1, 0);
                h = p.optDouble(2, 0);
            } else if (pObj instanceof JSONObject) {
                JSONObject p = (JSONObject) pObj;
                x = p.optDouble("x", 0);
                y = p.optDouble("y", 0);
                h = p.optDouble("h", 0);
            }
            positions[i] = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, h);
        }

        // Tags (optional)
        JSONArray jtags = root.optJSONArray("tags");
        if (jtags != null) {
            tags = new PathServer.Tag[jtags.length()];
            for (int i = 0; i < jtags.length(); i++) {
                JSONObject t = jtags.getJSONObject(i);
                PathServer.Tag tag = new PathServer.Tag(
                        t.getString("name"),
                        t.optDouble("value", 0.0),
                        t.getInt("index")
                );
                tags[i] = tag;
            }
            Arrays.sort(tags);
        } else {
            tags = new PathServer.Tag[0];
        }
        br.close();
    }

    @Override
    public void init() {
        GoBildaPinpointDriver pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpointDriver.setOffsets(xOffset, yOffset, DistanceUnit.MM);
        pinpointDriver.resetPosAndIMU();
        driveTrain = new OdometryHolonomicDrivetrain(
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight"),
                new GoBildaPinpointOdometry(pinpointDriver)
        );
        turret = new Turret(
                hardwareMap.get(DcMotorEx.class, "shooterTop"),
                hardwareMap.get(DcMotorEx.class, "shooterBottom"),
                hardwareMap.get(DcMotorEx.class, "turret"),
                hardwareMap.get(Servo.class, "hood"),
                hardwareMap.get(Servo.class, "blocker"),
                hardwareMap.get(AnalogInput.class, "potentiometer")
        );
        turret.resetTurretEncoder();

//        driveTrain.setVelocityPIDFCoefficients(p, i, d, f);
        driveTrain.setPositionP(positionP);

        intake = new Intake(
                hardwareMap.get(DcMotorEx.class, "intake"),
                hardwareMap.get(DistanceSensor.class, "lowerDistanceSensor"),
                hardwareMap.get(DistanceSensor.class, "middleDistanceSensor"),
                hardwareMap.get(NormalizedColorSensor.class, "upperColorSensor")
        );
        ledIndicator = new LEDIndicator(hardwareMap.get(GoBildaPrismDriver.class, "prism"));
        ledIndicator.setState(Intake.IntakeState.AMBIENT);
        turret.setEncoderOffset();
//        turret.resetTurretEncoder();

        telemetry.addData("Turret Angle", turret.getTurretCurrentAngle());
        telemetry.update();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        autoAligner = new AutoAligner(driveTrain, turret, limelight, false);

        AutoAligner.farShooterTolerance = 80;
        AutoAligner.farTurretTolerance = 6;

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
    }

    @Override
    public void init_loop() {
        driveTrain.updatePosition();
    }

    @Override
    public void start() {
        if (readJson) {
            try {
                // If a filename is specified, read from assets file
                if (jsonFilename != null && !jsonFilename.isEmpty()) {
                    InputStream is = hardwareMap.appContext.getAssets().open(jsonFilename);
                    jsonReader = new InputStreamReader(is);
                }
                parseJsonFromString();
            } catch (IOException | JSONException e) {
                throw new RuntimeException(e);
            }
        }
        driveTrain.setTolerance(tolerance);
        driveTrain.setPosition(startPose);
        driveTrain.setVelocity(velocity);
        Arrays.sort(tags);
        driveTrain.setPositionDrive(positions);
        turret.close();

//        if (autoAligner.getDistanceToGoal() > AutoAligner.closeDist) {
//            shooterDelay = 0.5;
//        }
    }

    public void autoAim(int pointIndex) {
        Pose2D curTarget = positions[pointIndex];
        positions[pointIndex] = new Pose2D(DistanceUnit.INCH, curTarget.getX(DistanceUnit.INCH),
                curTarget.getY(DistanceUnit.INCH), AngleUnit.DEGREES, autoAligner.getDrivetrainAutoAlignAngleWithTurret());
        double hoodAngle = autoAligner.getOptimalHoodAngle() - (autoAligner.useShootMove ? autoAligner.lastVelPar * hoodAngleVelScale : 0);
        turret.setHoodAngle(hoodAngle);
        wantedShooterVelocity = autoAligner.getShooterVelocityFromAngle(hoodAngle);
        turret.setShooterVelocity(wantedShooterVelocity);

//        Log.d("Loop Time", "Auto Aim time: " + (runtime.time() - start));
    }

    public void shoot(int pointIndex) {
        turret.open();
        autoAim(pointIndex);
        double shootingIntakeVelocity = 0;

        if (shooterTimer <= 0 && autoAligner.readyToShoot()) {
            shootingIntakeVelocity = 2800;
        }

        intake.setVelocity(shootingIntakeVelocity);
    }

    @Override
    public void loop() {
        driveTrain.updatePosition();
        Pose2D pos = driveTrain.getPosition();
        driveTrain.drive();

//        ledIndicator.setState(Intake.IntakeState.AMBIENT);

        autoAligner.updateInterpolation(pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH));

        double wantedTurretAngle = autoAligner.getTurretAutoAlignAngle();
        turret.setTurretAngle(wantedTurretAngle + autoAligner.lastVelPerp * turretPreturnConstant);
//        Log.d("Velocity", "Vel perp: " + autoAligner.lastVelPerp);

//        Log.d("Turret Angle", "Pinpoint Pos: " + pos.getX(DistanceUnit.INCH) + ", " + pos.getY(DistanceUnit.INCH));
//        double turretPos = turret.getTurretCurrentAngle();
//        Log.d("Turret Angle", "Current Turret Angle: " + turretPos);
//        Log.d("Turret Angle", "Wanted Turret Angle: " + wantedTurretAngle);
//        Log.d("Turret Angle", "Wanted Turret Angle 2: " + turret.getTurretTargetAngle());
//        Log.d("Turret Angle", "Difference: " + (wantedTurretAngle - turretPos));

        double curTime = runtime.seconds();
        double dt = curTime - lastTime;
        lastTime = curTime;
//        Log.d("Loop Time", "dt: " + dt);

        if (pauseTimeLeft <= 0) {
            if (searchingForArtifacts) {
                if (driveTrain.getNextPointIndex() > searchEndIndex) {
                    searchingForArtifacts = false;
                    pickingUp = false;
                    searchEndIndex = -1;
                    intake.setVelocity(0);
//                    driveTrain.setVelocity(velocity);
                }
                HuskyLens.Block largestBlock = Stream.of(huskyLens.blocks()).max(Comparator.comparingDouble(block -> block.width * block.height)).orElse(null);

                double error = 0;
                if (largestBlock != null) {
                    double targetX = largestBlock.x;
                    error = (targetX - huskyLensCenter) * huskyLensXP;
                    if (!pickingUp) {
                        intake.setVelocity(2800);
                        pickingUp = true;
                    }
                }

                if (pickingUp) {
                    positions[searchEndIndex] = new Pose2D(
                            DistanceUnit.INCH,
                            pos.getX(DistanceUnit.INCH) + error,
                            67,
                            AngleUnit.DEGREES,
                            pos.getHeading(AngleUnit.DEGREES)
                    );
                }
            }
            int nextPointIndex = driveTrain.getNextPointIndex();

            if (nextPointIndex == autoAlignIndex && nextPointIndex != -1) {
                autoAim(nextPointIndex);
            } else {
                autoAlignIndex = -1;
            }

            if (isShooting) {
                shoot(nextPointIndex);
            }

            while (lastTagIndex < tags.length && tags[lastTagIndex].index <= nextPointIndex) {
                PathServer.Tag currTag = tags[lastTagIndex];
                switch (currTag.name) {
                    case "velocity":
                        velocity = (int) (currTag.value * BasicHolonomicDrivetrain.FORWARD_COUNTS_PER_INCH);
                        driveTrain.setVelocity(velocity);
                        break;
                    case "pause":
                        if (currTag.value <= 0) break;
                        pauseTimeLeft = currTag.value;
                        pausedIndex = nextPointIndex;
                        driveTrain.setPositionDrive(positions[nextPointIndex - 1]);
                        break;
                    case "intake":
                        if (currTag.value <= 0) {
                            intakeVelocity = 0;
                            intake.stop();
                        } else {
                            intakeVelocity = (int) Math.round(currTag.value);
                            intake.setVelocity(intakeVelocity);
                        }
                        break;
                    case "autoAim": {
                        if (alliance.equals("red")) {
                            autoAligner.setRed();
                        } else {
                            autoAligner.setBlue();
                        }
                        autoAlignIndex = nextPointIndex;
                        Pose2D curTarget2 = positions[nextPointIndex];
                        positions[nextPointIndex] = new Pose2D(DistanceUnit.INCH, curTarget2.getX(DistanceUnit.INCH), curTarget2.getY(DistanceUnit.INCH), AngleUnit.DEGREES, autoAligner.getDrivetrainAutoAlignAngleWithTurret());
                        break;
                    }
                    case "shooterVelocity": {
                        turret.setShooterVelocity(currTag.value);
                        break;
                    }
                    case "hoodAngle": {
                        turret.setHoodAngle(currTag.value);
                        break;
                    }
                    case "launchArtifacts": {
                        if (currTag.value <= 0) break;
                        turret.open();
                        pauseTimeLeft = currTag.value;
                        pausedIndex = nextPointIndex;
                        driveTrain.setVelocity(30);
                        driveTrain.setPositionDrive(positions[nextPointIndex - 1]);
                        shooterTimer = shooterDelay;
                        isShooting = true;
                        break;
                    }
                    case "startLaunch": {
                        turret.open();
                        isShooting = true;
                        break;
                    }
                    case "endLaunch": {
                        turret.close();
                        intake.setVelocity(intakeVelocity);
                        isShooting = false;
                        break;
                    }
                    case "tolerance": {
                        driveTrain.setTolerance(currTag.value);
                        break;
                    }
                    case "shootWhileMove": {
                        autoAligner.useShootMove = currTag.value == 1;
                        break;
                    }
                    case "autoArtifactPickup": {
//                        driveTrain.setVelocity(searchVelocity);
                        searchEndIndex = nextPointIndex;
                        pickingUp = false;
                        searchingForArtifacts = true;
                        break;
                    }
                }
                lastTagIndex++;
            }
        } else {
            if (isShooting) {
                shoot(pausedIndex - 1);
            }
            pauseTimeLeft -= dt;
            shooterTimer -= dt;

            if (pauseTimeLeft <= 0) {
                driveTrain.setVelocity(velocity);

                pauseTimeLeft = 0;
                driveTrain.setPositionDrive(positions, pausedIndex);
                if (isShooting) {
                    turret.close();
                    intake.setVelocity(intakeVelocity);
                    isShooting = false;
                }
            } else {
                driveTrain.setPositionDrive(positions[pausedIndex - 1]);
            }
        }
    }

    @Override
    public void stop() {
        ledIndicator.setState(Intake.IntakeState.OFF);
    }
}
