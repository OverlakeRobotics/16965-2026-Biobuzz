package org.firstinspires.ftc.teamcode.code.helpers;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.code.helpers.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.code.parts.Intake;
import org.firstinspires.ftc.teamcode.code.parts.LEDIndicator;
import org.firstinspires.ftc.teamcode.code.parts.Turret;
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointOdometry;
import org.firstinspires.ftc.teamcode.system.OdometryHolonomicDrivetrain;
import org.firstinspires.ftc.teamcode.system.PathServer;

@Config
public abstract class BaseTeleOp extends OpMode {
    protected Limelight3A limelight;
    public static final ElapsedTime runtime = new ElapsedTime();

    // TODO: Check if these are correct
    public static final double yOffset = -156.0; // -168.0 // mm
    public static final double xOffset = 72.0; // -84.0 // mm

    public static double hoodAngleVelScale = 0.5;

    public static double ADJUSTMENT_FACTOR = 0.021; //0.021;

    protected static final class PresetPoint {
        public final Pose2D pose;
        public final double velocity;
        public final int tolerance;

        public PresetPoint(Pose2D pose, double velocity, int tolerance) {
            this.pose = pose;
            this.velocity = velocity;
            this.tolerance = tolerance;
        }
    }

    protected static final class Preset {
        public final PresetPoint[] points;
        public final Pose2D[] poses;
        public final boolean useAutoAlignHeading;

        public Preset(PresetPoint point, boolean useAutoAlignHeading) {
            this(new PresetPoint[]{point}, useAutoAlignHeading);
        }

        public Preset(PresetPoint[] points, boolean useAutoAlignHeading) {
            this.points = points;
            this.useAutoAlignHeading = useAutoAlignHeading;
            poses = new Pose2D[points.length];
            for (int i = 0; i < points.length; i++) {
                poses[i] = points[i].pose;
            }
        }
    }

    protected Preset[] presetPositions;

    public int currentPreset = -1;

    public double velocity = 2800;
    public static double gateOpenVelocity = 1000;
    public static double positionP = 3;
    public static int initialTolerance = 8;
    public static int gateTolerance = 3;

    protected OdometryHolonomicDrivetrain driveTrain;
    protected AutoAligner autoAligner;
    protected IMU backupIMU;

    protected boolean autoLock = false;
    protected boolean autoShooter = true;

    protected Intake intake;
    protected LEDIndicator ledIndicator;
    protected boolean intakeOn = false;
    protected boolean intakeReversed = false;

    protected Turret turret;
    protected double shooterVelocity;
    protected double hoodAngle;
    protected double turretAngle;
    protected boolean autoTurret = true;
    protected boolean useBackupIMU = false;

    protected abstract boolean isRedAlliance();

    private double normalize(double angle) {
        return (angle + 180) % 360 - 180;
    }
    protected Preset[] getPresetPositions() {
        double ySign = isRedAlliance() ? -1.0 : 1.0;
        return new Preset[]{
                new Preset(
                        new PresetPoint(
                                new Pose2D(DistanceUnit.INCH, -38, 33 * ySign, AngleUnit.DEGREES, 0),
                                60,
                                2
                        ),
                        true
                ),
                new Preset(
                        new PresetPoint(
                                new Pose2D(DistanceUnit.INCH, 15, 15 * ySign, AngleUnit.DEGREES, 0),
                                velocity,
                                8
                        ),
                        true
                ),
                new Preset(
                        new PresetPoint[] {
                                new PresetPoint(new Pose2D(DistanceUnit.INCH, -12, 38 * ySign, AngleUnit.DEGREES, 60 * ySign), velocity, initialTolerance),
                                new PresetPoint(new Pose2D(DistanceUnit.INCH, -12, 60 * ySign, AngleUnit.DEGREES, 60 * ySign), gateOpenVelocity, gateTolerance),
                        },
                        false
                ),
        };
    }

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
        limelight.start();

        GoBildaPinpointDriver pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpointDriver.setOffsets(xOffset, yOffset, DistanceUnit.MM);
        // No longer doing this because found out that issue was limelight freeze frame
//        pinpointDriver.recalibrateIMU();
        driveTrain = new OdometryHolonomicDrivetrain(
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight"),
                new GoBildaPinpointOdometry(pinpointDriver)
        );

        driveTrain.setPositionP(positionP);


        turret = new Turret(
                hardwareMap.get(DcMotorEx.class, "shooterTop"),
                hardwareMap.get(DcMotorEx.class, "shooterBottom"),
                hardwareMap.get(DcMotorEx.class, "turret"),
                hardwareMap.get(Servo.class, "hood"),
                hardwareMap.get(Servo.class, "blocker"),
                hardwareMap.get(AnalogInput.class, "potentiometer")
        );
//        turret.resetTurretEncoder();
        turret.setEncoderOffset();
        intake = new Intake(
                hardwareMap.get(DcMotorEx.class, "intake"),
                hardwareMap.get(DistanceSensor.class, "lowerDistanceSensor"),
                hardwareMap.get(DistanceSensor.class, "middleDistanceSensor"),
                hardwareMap.get(NormalizedColorSensor.class, "upperColorSensor")
        );

        backupIMU = hardwareMap.get(IMU.class, "imu");
        backupIMU.resetYaw();

        ledIndicator = new LEDIndicator(hardwareMap.get(GoBildaPrismDriver.class, "prism"));
        ledIndicator.setState(Intake.IntakeState.AMBIENT);

        // Initialize AutoAligner
        autoAligner = new AutoAligner(driveTrain, turret, limelight, isRedAlliance());

        AutoAligner.farShooterTolerance = 60;
        AutoAligner.farTurretTolerance = 2;

        presetPositions = getPresetPositions();
        PathServer.startServer();
    }

    @Override
    public void init_loop() {
        PathServer.setRobotPose(driveTrain.getPosition());
    }

    @Override
    public void loop() {
        driveTrain.updatePosition();
        Pose2D currentPos = driveTrain.getPosition();
        double backupHeading = backupIMU.getRobotYawPitchRollAngles().getYaw();
        PathServer.setRobotPose(currentPos);
        autoAligner.updateInterpolation(currentPos.getX(DistanceUnit.INCH), currentPos.getY(DistanceUnit.INCH));
//        double temp = runtime.seconds();
//        Intake.IntakeState state = intake.getState();
//        Log.d("LED Latency", "Intake: " + (runtime.seconds() - temp));
//        Log.d("LED Latency", "State: " + state);
//        temp = runtime.seconds();
//        ledIndicator.setState(state);
//        Log.d("LED Latency", "LED: " + (runtime.seconds() - temp));

//        Log.d("Turret Error", "Turret Error: " + (turret.getTurretTargetAngle() - turret.getTurretCurrentAngle()));
        ledIndicator.setState(intake.getState());

        telemetry.addData("Position", "X: %.2f, Y: %.2f, H: %.2f",
                currentPos.getX(DistanceUnit.INCH),
                currentPos.getY(DistanceUnit.INCH),
                currentPos.getHeading(AngleUnit.DEGREES));

        telemetry.addData("Backup Heading", "H: %.2f", backupHeading);
        Log.d("Turret", "Turret Position: " + turret.getTurretCurrentAngle());
        Log.d("Turret", "Turret Wanted Position: " + turret.getTurretTargetAngle());
        Log.d("Turret", "Auto Aligner Turret Position: " + autoAligner.getTurretAutoAlignAngle());

        // Use AutoAligner to get wanted heading and distance
        double wantedHeading = autoTurret ? autoAligner.getDrivetrainAutoAlignAngleWithTurret() : autoAligner.getDrivetrainAutoAlignAngleNoTurret();

        if (useBackupIMU) {
            wantedHeading += normalize(backupHeading - currentPos.getHeading(AngleUnit.DEGREES));
            wantedHeading = normalize(wantedHeading);
        }

        if (currentPreset >= 0 && (Math.abs(gamepad1.left_stick_x) > 0.001 || Math.abs(gamepad1.left_stick_y) > 0.001 || Math.abs(gamepad1.right_stick_x) > 0.001)) {
            currentPreset = -1;
            autoLock = true;
        }

        double turretAdjustment = 0;

        if (currentPreset >= 0) {
            Preset preset = presetPositions[currentPreset];
            int nextPointIndex = driveTrain.getNextPointIndex();
            driveTrain.setVelocity((int) preset.points[nextPointIndex].velocity);
            driveTrain.setTolerance(preset.points[nextPointIndex].tolerance);
            driveTrain.setPositionDrive(preset.poses, nextPointIndex);
            if (preset.useAutoAlignHeading) {
                Pose2D nextPoint = preset.poses[nextPointIndex];
                preset.poses[nextPointIndex] = new Pose2D(DistanceUnit.INCH,
                        nextPoint.getX(DistanceUnit.INCH), nextPoint.getY(DistanceUnit.INCH),
                        AngleUnit.DEGREES, wantedHeading);
            }
        } else {
            double turn = -gamepad1.right_stick_x * velocity / 1.25;

            if (gamepad1.y) {
                autoLock = true;
                autoShooter = true;
            }

            if (Math.abs(turn) > 2) {
                autoLock = false;
            }

            if (autoLock) {
                driveTrain.setWantedHeading(wantedHeading);
                turn = useBackupIMU ? driveTrain.getHeadingCorrectionVelocity(backupHeading) : driveTrain.getHeadingCorrectionVelocity();
            }
            driveTrain.setVelocityDriveFieldCentric(-gamepad1.left_stick_y * velocity, -gamepad1.left_stick_x * velocity, turn, isRedAlliance() ? -90 : 90);

            turretAdjustment = -turn * ADJUSTMENT_FACTOR;
        }

        if (autoShooter) {
            // Use AutoAligner methods to calculate shooter angle and velocity
            hoodAngle = autoAligner.getOptimalHoodAngle() - autoAligner.lastVelPar * hoodAngleVelScale;
            hoodAngle = Range.clip(hoodAngle, turret.minHoodAngle, turret.maxHoodAngle);
            shooterVelocity = autoAligner.getShooterVelocityFromAngle(hoodAngle);
        }
//        turretAngle = autoAligner.getTurretAutoAlignAngle() + turretAdjustment;

        if (autoTurret) {
            turretAngle = autoAligner.getTurretAutoAlignAngle() + turretAdjustment;
        }

        // Gamepad 1 controls
        // Right Bumper: Far preset
        // Left Bumper: Close preset
        // A: Shoot artifacts (hold down)
        // X: Turn on/off intake
        // B: Reverse intake direction
        // Y: Auto aim
        // D-Pad Right: Turn off shooter

        if (gamepad1.xWasPressed()) {
            if (!intakeReversed) {
                intakeOn = !intakeOn;
            }
            intakeReversed = false;
        }
        if (gamepad1.bWasPressed()) {
            intakeReversed = true;
            intakeOn = true;
        }

        if (gamepad1.dpadRightWasPressed()) {
            autoShooter = false;
            shooterVelocity = 0;
        }
//        if (gamepad1.dpadLeftWasPressed()) {
//            autoTurret = !autoTurret;
//        }
//        if (gamepad1.dpadUpWasReleased()){
//            pinpointDriver
//        }
        double intakeVelocity = intakeOn ? (intakeReversed ? -2800 : 2800) : 0;

        if (gamepad1.a) {
            turret.open();
            intakeVelocity = 0;

            if (autoAligner.readyToShoot()) {
                intakeVelocity = 2800;
            }
        } else {
            turret.close();
        }

        // Preset for far position
        if (gamepad1.leftBumperWasPressed()) {
            currentPreset = 0;
            driveTrain.setPositionDrive(this.presetPositions[0].poses);
        }

        // Preset for gate pickup
        if (gamepad1.rightBumperWasPressed()) {
            currentPreset = 2;
            driveTrain.setPositionDrive(this.presetPositions[2].poses);
        }

        // Gamepad 2 controls (only for backup in case of robot reset/pinpoint issues)

        // LAUNCHER BACKUP
        // Following four only work when autoShooter is off
        // Right Bumper: Turn up shooter velocity by 100
        // Left Bumper: Turn down shooter velocity by 100
        // D-Pad Up: Increase hood angle by 2 degrees (More direct, 90 is straight forward)
        // D-Pad Down: Decrease hood angle by 2 degrees (More parabolic, 0 is straight up)

        // TURRET BACKUP
        // A: Toggle autoTurret
        // Following three only work when autoTurret is off
        // D-Pad Right: Increase turret angle by 5 degrees (CCW)
        // D-Pad Left: Decrease turret angle by 5 degrees (CW)
        // Y: Reset turret encoder and sync it to potentiometer, set turret to 0 degrees

        // PINPOINT BACKUP
        // B: Reset pinpoint position based on limelight
        // X: Reset pinpoint heading to 0

        // OTHER BACKUP
        // Left Bumper: Corner Reset (own human player zone; any heading)
        // Right Bumper: Toggle Limelight (if it freeze frames)

//        if (!autoShooter) {
//            if (gamepad2.rightBumperWasPressed()) {
//                shooterVelocity = Math.round(shooterVelocity / 100) * 100;
//                shooterVelocity += 100;
//            }
//            if (gamepad2.leftBumperWasPressed()) {
//                shooterVelocity = Math.round(shooterVelocity / 100) * 100;
//                shooterVelocity -= 100;
//            }
//            if (gamepad2.dpadUpWasPressed()) {
//                hoodAngle += 2;
//            }
//            if (gamepad2.dpadDownWasPressed()) {
//                hoodAngle -= 2;
//            }
//        }

        if (gamepad2.aWasPressed()) {
            autoTurret = !autoTurret;
        }

        if (!autoTurret) {
            // Will need to check values for speed/precision
            if (gamepad2.dpad_left) {
                turretAngle += 5;
            }
            if (gamepad2.dpad_right) {
                turretAngle -= 5;
            }

            if (gamepad2.yWasPressed()) {
                turret.resetTurretEncoder();
                turret.setEncoderOffset();
                turretAngle = 0;
            }
        }

//        // MAKE SURE THE ROBOT IS STATIONARY FOR AT LEAST 0.3 SECONDS AFTER USING THIS FUNCTION
//        if (gamepad2.rightBumperWasPressed()) {
//            driveTrain.odometry.recalibrate();
//        }
        if (gamepad2.rightBumperWasPressed()) {
            autoAligner.toggleLimelight();
        }
        if (gamepad2.leftBumperWasPressed()) {
//            autoAligner.resetPinpointPositionFromLimelight();
            double ySign = isRedAlliance() ? 1.0 : -1.0;
            Pose2D currPos = driveTrain.getPosition();
            driveTrain.setPosition(new Pose2D(DistanceUnit.INCH, -63, 63 * ySign, AngleUnit.DEGREES, currPos.getHeading(AngleUnit.DEGREES)));
        }
        if (gamepad2.bWasPressed()) {
            autoAligner.resetPinpointPositionFromLimelight();
        }
        if (gamepad2.xWasPressed()) {
            Pose2D pos = driveTrain.getPosition();
            double x = pos.getX(DistanceUnit.INCH);
            double y = pos.getY(DistanceUnit.INCH);
            driveTrain.setPosition(
                    new Pose2D(
                            DistanceUnit.INCH,
                            x,
                            y,
                            AngleUnit.DEGREES,
                            0
                    )
            );
        }

        // TODO: Don't use this; just a test
//        if (gamepad2.rightBumperWasPressed()) {
//            this.useBackupIMU = true;
//        } else if (gamepad2.leftTriggerWasPressed()) {
//            this.useBackupIMU = false;
//        }

//        if (gamepad2.aWasPressed()) {
//            shooterVelocity = 2000;
//        }
//        if (gamepad2.xWasPressed()) {
//            shooterVelocity = 0;
//        }

        hoodAngle = Range.clip(hoodAngle, turret.minHoodAngle, turret.maxHoodAngle);
        turretAngle = Range.clip(turretAngle, turret.MIN_ANGLE_LIMIT, turret.MAX_ANGLE_LIMIT);
        shooterVelocity = Range.clip(shooterVelocity, 0, turret.MAX_VELOCITY);

        intake.setVelocity(intakeVelocity);
        turret.setShooterVelocity(shooterVelocity);
        turret.setHoodAngle(hoodAngle);
        turret.setTurretAngle(turretAngle);

        driveTrain.drive();

//        Log.d("PID", "Current Velocity: " + turret.getShooterVelocity());
//        Log.d("PID", "Wanted Velocity: " + shooterVelocity);
//        telemetry.addData("Current Velocity", turret.getShooterVelocity());
//        telemetry.addData("Wanted Velocity", shooterVelocity);
//        telemetry.addData("Turret Given Target Angle", turretAngle);
//        telemetry.addData("Turret Current Angle", turret.getTurretCurrentAngle());
//        telemetry.addData("Turret True Target Angle", turret.getTurretTargetAngle());
//        telemetry.addData("Shooter Top Current Velocity", turret.getShooter1Velocity());
//        telemetry.addData("Shooter Bottom Current Velocity", turret.getShooter2Velocity());
//        telemetry.addData("Wanted Shooter Velocity", shooterVelocity);
        telemetry.addData("Limelight", autoAligner.limelightEnabled ? "ON" : "OFF");
        telemetry.update();
    }

    @Override
    public void stop() {
        ledIndicator.setState(Intake.IntakeState.OFF);
        PathServer.stopServer();
    }
}
