package org.firstinspires.ftc.teamcode.examples;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.zOldCode.decode.helpers.AutoAligner;
import org.firstinspires.ftc.teamcode.zOldCode.decode.parts.Shooter;


// One shooter and no turret. While aiming, the whole robot turns so the shooter faces the nearest
// goal; otherwise it drives normally. See ShooterTeleOpBase for the controls.
@Config
public abstract class SingleShooterTeleOp extends ShooterTeleOpBase {
    // Where the shooter is on the robot, in inches from the robot's center (+forward, +left).
    public static double shooterForwardOffset = 0;
    public static double shooterLeftOffset = 0;
    // Direction the shooter fires relative to the robot's front, in degrees (CCW positive).
    // 180 = out the back (like the current robot), 0 = out the front, 90 = out the left side.
    public static double shooterFireDirection = 180;

    private Shooter shooter;
    private AutoAligner autoAligner;
    private final ShotState shot = new ShotState();

    @Override
    protected void initShooters() {
        shooter = new Shooter(
                hardwareMap.get(DcMotorEx.class, "shooterTop"),
                hardwareMap.get(DcMotorEx.class, "shooterBottom"),
                hardwareMap.get(Servo.class, "hood"),
                hardwareMap.get(Servo.class, "blocker")
        );
        autoAligner = createAutoAligner(null);
    }

    @Override
    protected boolean updateShooters(double forward, double strafe, double turn, boolean aiming, boolean firing) {
        solveShot(autoAligner, shooterForwardOffset, shooterLeftOffset, false, shot);

        double aimHeading = normalize(getAimHeading(autoAligner, shooterForwardOffset, shooterLeftOffset, shooterFireDirection) + shot.leadAngle);
        double aimError = normalize(aimHeading - driveTrain.getPosition().getHeading(AngleUnit.DEGREES));

        if (aiming) {
            turn = getAimTurnVelocity(aimHeading, getGoalBearingRate(shooterForwardOffset, shooterLeftOffset));
        }
        driveRobot(forward, strafe, turn, aiming);

        applyShot(shooter, shot, firing);
        boolean ready = isShotReady(shooter, shot, aimError);

        telemetry.addData("Aim Heading", "%.1f", aimHeading);
        addShotTelemetry("Shooter", shooter, shot, aimError, ready);
        return ready;
    }
}
