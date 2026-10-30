package org.firstinspires.ftc.teamcode.examples;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.zOldCode.decode.helpers.AutoAligner;
import org.firstinspires.ftc.teamcode.zOldCode.decode.parts.Turret;


// Two shooters, each on its own rotating turret. Both turrets always track the nearest goal, so
// the robot drives freely. If a goal is outside a turret's limits while aiming, the robot turns
// just enough to bring it back in range. Artifacts are only fed once both turrets are ready. See
// ShooterTeleOpBase for the controls.
@Config
public abstract class DualTurretTeleOp extends ShooterTeleOpBase {
    // Where each turret's pivot is on the robot, in inches from the robot's center (+forward, +left).
    public static double turret1ForwardOffset = 0;
    public static double turret1LeftOffset = 6;
    public static double turret2ForwardOffset = 0;
    public static double turret2LeftOffset = -6;
    // Direction each shooter fires at turret angle 0, relative to the robot's front, in degrees
    // (CCW positive). 180 = out the back, which is what Turret/AutoAligner use on the current robot.
    public static double turret1ZeroDirection = 180;
    public static double turret2ZeroDirection = 180;
    // Turret lead in degrees per tick/s of robot turn, same as BaseTeleOp.ADJUSTMENT_FACTOR.
    public static double turretTurnFeedforward = 0.021;
    // How far inside a turret limit (degrees) the robot turns to when the goal is out of range.
    public static double turretLimitBuffer = 5;
    // If true, each turret's angle is read from its potentiometer on init (Turret.setEncoderOffset).
    // If false, the turrets must be at angle 0 on init.
    public static boolean usePotentiometers = true;

    private Turret turret1;
    private Turret turret2;
    private AutoAligner autoAligner1;
    private AutoAligner autoAligner2;
    private final ShotState shot1 = new ShotState();
    private final ShotState shot2 = new ShotState();

    @Override
    protected void initShooters() {
        // If a shooter only has one flywheel motor, use the same name for Top and Bottom.
        turret1 = new Turret(
                hardwareMap.get(DcMotorEx.class, "shooter1Top"),
                hardwareMap.get(DcMotorEx.class, "shooter1Bottom"),
                hardwareMap.get(DcMotorEx.class, "turret1"),
                hardwareMap.get(Servo.class, "hood1"),
                hardwareMap.get(Servo.class, "blocker1"),
                usePotentiometers ? hardwareMap.get(AnalogInput.class, "potentiometer1") : null
        );
        turret2 = new Turret(
                hardwareMap.get(DcMotorEx.class, "shooter2Top"),
                hardwareMap.get(DcMotorEx.class, "shooter2Bottom"),
                hardwareMap.get(DcMotorEx.class, "turret2"),
                hardwareMap.get(Servo.class, "hood2"),
                hardwareMap.get(Servo.class, "blocker2"),
                usePotentiometers ? hardwareMap.get(AnalogInput.class, "potentiometer2") : null
        );
        if (usePotentiometers) {
            turret1.setEncoderOffset();
            turret2.setEncoderOffset();
        } else {
            turret1.resetTurretEncoder();
            turret2.resetTurretEncoder();
        }
        autoAligner1 = createAutoAligner(turret1);
        autoAligner2 = createAutoAligner(turret2);
    }

    @Override
    protected boolean updateShooters(double forward, double strafe, double turn, boolean aiming, boolean firing) {
        solveShot(autoAligner1, turret1ForwardOffset, turret1LeftOffset, true, shot1);
        solveShot(autoAligner2, turret2ForwardOffset, turret2LeftOffset, true, shot2);

        double aimAngle1 = normalize(getTurretAimAngle(autoAligner1, turret1ForwardOffset, turret1LeftOffset, turret1ZeroDirection) + shot1.leadAngle);
        double aimAngle2 = normalize(getTurretAimAngle(autoAligner2, turret2ForwardOffset, turret2LeftOffset, turret2ZeroDirection) + shot2.leadAngle);

        // Only turn the robot if a turret can't reach. If both are out of range (only possible if
        // their zero directions are close but not equal), fix the one that is further out.
        double correction1 = getTurretLimitCorrection(turret1, aimAngle1);
        double correction2 = getTurretLimitCorrection(turret2, aimAngle2);
        double correction = Math.abs(correction1) >= Math.abs(correction2) ? correction1 : correction2;
        boolean turningToReach = aiming && correction != 0;
        if (turningToReach) {
            turn = getAimTurnVelocity(normalize(driveTrain.getPosition().getHeading(AngleUnit.DEGREES) + correction), 0);
        }
        driveRobot(forward, strafe, turn, turningToReach);

        // Turret.setTurretAngle clips to the turret's limits.
        turret1.setTurretAngle(aimAngle1 - turn * turretTurnFeedforward);
        turret2.setTurretAngle(aimAngle2 - turn * turretTurnFeedforward);

        applyShot(turret1, shot1, firing);
        applyShot(turret2, shot2, firing);
        // Compared to the unclipped aim angle, so a turret sitting at its limit is never "aimed".
        double aimError1 = normalize(aimAngle1 - turret1.getTurretCurrentAngle());
        double aimError2 = normalize(aimAngle2 - turret2.getTurretCurrentAngle());
        boolean ready1 = isShotReady(turret1, shot1, aimError1);
        boolean ready2 = isShotReady(turret2, shot2, aimError2);

        telemetry.addData("Turret 1 Angle", "%.1f (wanted %.1f)", turret1.getTurretCurrentAngle(), aimAngle1);
        telemetry.addData("Turret 2 Angle", "%.1f (wanted %.1f)", turret2.getTurretCurrentAngle(), aimAngle2);
        addShotTelemetry("Turret 1", turret1, shot1, aimError1, ready1);
        addShotTelemetry("Turret 2", turret2, shot2, aimError2, ready2);
        return ready1 && ready2;
    }

    // Behavior: Gets how far the robot has to turn to bring a turret angle back inside the limits,
    //           like AutoAligner.getDrivetrainAutoAlignAngleWithTurret.
    // Returns: The heading change in degrees, or 0 if the angle is already in range.
    private double getTurretLimitCorrection(Turret turret, double angle) {
        if (angle > turret.MAX_ANGLE_LIMIT) {
            return angle - turret.MAX_ANGLE_LIMIT + turretLimitBuffer;
        }
        if (angle < turret.MIN_ANGLE_LIMIT) {
            return angle - turret.MIN_ANGLE_LIMIT - turretLimitBuffer;
        }
        return 0;
    }
}
