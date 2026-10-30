package org.firstinspires.ftc.teamcode.examples;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.code.helpers.AutoAligner;
import org.firstinspires.ftc.teamcode.code.parts.Shooter;


// Two parallel shooters and no turret, both firing out the same side of the robot. While aiming,
// the whole robot turns so the point halfway between the shooters faces the nearest goal. Each
// shooter gets its own hood angle and flywheel velocity for its own distance to the goal. Artifacts
// are only fed once both shooters are ready. See ShooterTeleOpBase for the controls.
@Config
public abstract class DualShooterTeleOp extends ShooterTeleOpBase {
    // Where each shooter is on the robot, in inches from the robot's center (+forward, +left).
    public static double shooter1ForwardOffset = 4;
    public static double shooter1LeftOffset = 0;
    public static double shooter2ForwardOffset = -4;
    public static double shooter2LeftOffset = 0;
    // Direction both shooters fire relative to the robot's front, in degrees (CCW positive).
    // 90 = out the left side, -90 = out the right side.
    public static double shooterFireDirection = 90;

    private Shooter shooter1;
    private Shooter shooter2;
    private AutoAligner autoAligner1;
    private AutoAligner autoAligner2;
    private final ShotState shot1 = new ShotState();
    private final ShotState shot2 = new ShotState();

    @Override
    protected void initShooters() {
        // If a shooter only has one flywheel motor, use the same name for Top and Bottom.
        shooter1 = new Shooter(
                hardwareMap.get(DcMotorEx.class, "shooter1Top"),
                hardwareMap.get(DcMotorEx.class, "shooter1Bottom"),
                hardwareMap.get(Servo.class, "hood1"),
                hardwareMap.get(Servo.class, "blocker1")
        );
        shooter2 = new Shooter(
                hardwareMap.get(DcMotorEx.class, "shooter2Top"),
                hardwareMap.get(DcMotorEx.class, "shooter2Bottom"),
                hardwareMap.get(Servo.class, "hood2"),
                hardwareMap.get(Servo.class, "blocker2")
        );
        autoAligner1 = createAutoAligner(null);
        autoAligner2 = createAutoAligner(null);
    }

    @Override
    protected boolean updateShooters(double forward, double strafe, double turn, boolean aiming, boolean firing) {
        solveShot(autoAligner1, shooter1ForwardOffset, shooter1LeftOffset, false, shot1);
        solveShot(autoAligner2, shooter2ForwardOffset, shooter2LeftOffset, false, shot2);

        double aimForward = (shooter1ForwardOffset + shooter2ForwardOffset) / 2;
        double aimLeft = (shooter1LeftOffset + shooter2LeftOffset) / 2;
        double leadAngle = (shot1.leadAngle + shot2.leadAngle) / 2;
        double aimHeading = normalize(getAimHeading(autoAligner1, aimForward, aimLeft, shooterFireDirection) + leadAngle);
        double aimError = normalize(aimHeading - driveTrain.getPosition().getHeading(AngleUnit.DEGREES));

        if (aiming) {
            turn = getAimTurnVelocity(aimHeading, getGoalBearingRate(aimForward, aimLeft));
        }
        driveRobot(forward, strafe, turn, aiming);

        applyShot(shooter1, shot1, firing);
        applyShot(shooter2, shot2, firing);
        boolean ready1 = isShotReady(shooter1, shot1, aimError);
        boolean ready2 = isShotReady(shooter2, shot2, aimError);

        telemetry.addData("Aim Heading", "%.1f", aimHeading);
        addShotTelemetry("Shooter 1", shooter1, shot1, aimError, ready1);
        addShotTelemetry("Shooter 2", shooter2, shot2, aimError, ready2);
        return ready1 && ready2;
    }
}
