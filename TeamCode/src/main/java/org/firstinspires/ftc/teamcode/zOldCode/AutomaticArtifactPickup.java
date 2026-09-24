package org.firstinspires.ftc.teamcode.code.opmodes;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.code.helpers.AutoAligner;
import org.firstinspires.ftc.teamcode.code.parts.Intake;
import org.firstinspires.ftc.teamcode.code.parts.Turret;
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointOdometry;
import org.firstinspires.ftc.teamcode.system.OdometryHolonomicDrivetrain;

@Config
@TeleOp(name = "Automatic Artifact Pickup", group = "TeleOp")
public class AutomaticArtifactPickup extends OpMode {
    public HuskyLens huskyLens;
    public OdometryHolonomicDrivetrain driveTrain;
    public Intake intake;
    private Turret turret;
    private AutoAligner autoAligner;
    private Limelight3A limelight;
    public static double positionP = 3;

    public static double xP = 0.1;
    public static int center = 160;

    public boolean isPickingUp = false;
    private final ElapsedTime runtime = new ElapsedTime();

    private double pickupStartTime;
    private double atDestStartTime = -1;
    private double startShootTime;

    private double searchStartTime = -1;

    @Override
    public void init() {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        intake = new Intake(
                hardwareMap.get(DcMotorEx.class, "intake"),
                hardwareMap.get(DistanceSensor.class, "lowerDistanceSensor"),
                hardwareMap.get(DistanceSensor.class, "middleDistanceSensor"),
                hardwareMap.get(NormalizedColorSensor.class, "upperColorSensor")
        );
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        GoBildaPinpointDriver pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
//        pinpointDriver.resetPosAndIMU();
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
        turret.setEncoderOffset();

        turret.close();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        autoAligner = new AutoAligner(driveTrain, turret, limelight, false);
        autoAligner.useShootMove = false;

        driveTrain.setVelocity(1500);
        driveTrain.setPositionP(positionP);
        driveTrain.setTolerance(8);
    }

    @Override
    public void start() {
        driveTrain.setPosition(new Pose2D(DistanceUnit.INCH, -63, 15, AngleUnit.DEGREES, 90));
        runtime.reset();
    }

    @Override
    public void loop() {
        driveTrain.updatePosition();
        driveTrain.drive();
        Pose2D pos = driveTrain.getPosition();

        double t = runtime.seconds();

        double wantedTurretAngle = autoAligner.getTurretAutoAlignAngle();
        turret.setTurretAngle(wantedTurretAngle);

        Log.d("Auto Pickup", "Pos: " + pos.getX(DistanceUnit.INCH) + ", " + pos.getY(DistanceUnit.INCH) + ", " + pos.getHeading(AngleUnit.DEGREES));

        HuskyLens.Block[] blocks = huskyLens.blocks();
        HuskyLens.Block largestBlock = null;
        int largestSize = 0;
        for (HuskyLens.Block block : blocks) {
            telemetry.addData("Block", block.toString());
            int currSize = block.height * block.width;
            if (currSize > largestSize) {
                largestSize = currSize;
                largestBlock = block;
            }
        }

        double error = 0;

        if (largestBlock != null) {
            double targetX = largestBlock.x;
            error = (targetX - center) * xP;
        }

        int numArtifacts = intake.getNumArtifactsInRamp();

        if (isPickingUp) {
            double dt = (t - pickupStartTime);
            boolean atDest = driveTrain.getDistanceToDestination() < 6;
            if (atDest && atDestStartTime <= 0) {
                atDestStartTime = t;
            }
            boolean endEarly = dt >= 2.5 || numArtifacts >= 3;
            if ((atDest && (t - atDestStartTime) >= 0.4) || endEarly) {
                atDestStartTime = -1;
//                intake.setVelocity(0);
                isPickingUp = false;
            } else {
                driveTrain.setPositionDrive(
                        new Pose2D(
                                DistanceUnit.INCH,
                                pos.getX(DistanceUnit.INCH) + error,
                                67,
                                AngleUnit.DEGREES,
                                90 + 25 * Math.sin(dt * 8)
                        )
                );
                return;
            }
        }

        if (largestBlock != null && numArtifacts <= 1/* && (t - startShootTime) < 5*/) {
            Log.d("Husky Cam", "ID: " + largestBlock.id);
            Log.d("Center Pos", "X: " + largestBlock.x);

            double nextX = pos.getX(DistanceUnit.INCH) + error;
            double maxNextX = Math.max(nextX, -65);

            turret.close();

            if (Math.abs(error) < 2.5 || nextX <= -65) {
                searchStartTime = -1;
                isPickingUp = true;
                pickupStartTime = t;

                intake.setVelocity(2800);

                driveTrain.setPositionDrive(
                    new Pose2D(
                            DistanceUnit.INCH,
                            maxNextX,
                            67,
                            AngleUnit.DEGREES,
                            90
                    )
                );
            } else {
                intake.setVelocity(0);
                driveTrain.setPositionDrive(new Pose2D(
                        DistanceUnit.INCH,
                        maxNextX,
                        15,
                        AngleUnit.DEGREES,
                        90
                ));
            }
        } else {
//            turret.close();
            boolean atDest = driveTrain.getDistanceToDestination() < 5;
            boolean isShooting = false;
            if (atDest) {
                if (startShootTime <= 0) {
                    startShootTime = t;
                }
                autoAligner.updateInterpolation(pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH));
                double hoodAngle = autoAligner.getOptimalHoodAngle();
                turret.setHoodAngle(hoodAngle);
                double wantedShooterVelocity = autoAligner.getShooterVelocityFromAngle(hoodAngle);
                turret.setShooterVelocity(wantedShooterVelocity);

                double dt = t - startShootTime;
                isShooting = numArtifacts >= 1 || dt < 1.5;
                if (autoAligner.readyToShoot() && dt >= 0.4) {
                    turret.open();
                    intake.setVelocity(2800);
                } else {
                    intake.setVelocity(0);
                    turret.close();
                }
            } else {
                turret.close();
            }

            boolean atYDest = Math.abs(pos.getY(DistanceUnit.INCH) - 18) <= 5;
            boolean searchForBalls = atYDest && !isShooting;

            if (searchForBalls && searchStartTime < 0) {
                searchStartTime = t;
            }

            Log.d("Search", "Search for balls: " + searchForBalls);
            Log.d("Search", "Search start time: " + searchStartTime);

            driveTrain.setPositionDrive(new Pose2D(
                    DistanceUnit.INCH,
                    -58 + (searchForBalls ? (t - searchStartTime) * 15 : 0),
                    18,
                    AngleUnit.DEGREES,
                    90
            ));
        }

        telemetry.update();
    }
}
