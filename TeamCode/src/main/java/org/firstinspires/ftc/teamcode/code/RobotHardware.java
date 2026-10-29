package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.code.parts.Intake;
import org.firstinspires.ftc.teamcode.code.parts.Shooter;
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointOdometry;
import org.firstinspires.ftc.teamcode.system.OdometryHolonomicDrivetrain;


// The drivetrain is built separately (createDrivetrain) because in Auto the Pedro Follower owns the
// drive motors and Pinpoint instead (see pedro/Constants).
@Config
public class RobotHardware {
    // Flywheel velocity PIDF, tunable live in FTC Dashboard. NaN = use the motor's defaults (filled in on init).
    public static PIDFCoefficients pollenPIDF = new PIDFCoefficients(Double.NaN, Double.NaN, Double.NaN, Double.NaN);
    public static PIDFCoefficients nectarPIDF = new PIDFCoefficients(Double.NaN, Double.NaN, Double.NaN, Double.NaN);

    public final Shooter pollen;
    public final Shooter nectar;
    public final Intake intake;

    private final HardwareMap hardwareMap;

    public RobotHardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        pollen = new Shooter(
                hardwareMap.get(DcMotorEx.class, "flywheelP"),
                hardwareMap.get(Servo.class, "hoodP"),
                hardwareMap.get(Servo.class, "blockerP"),
                pollenPIDF,
                16.2   // hood angle at servo position 0 (CAD)
        );

        DcMotorEx flywheelN = hardwareMap.get(DcMotorEx.class, "flywheelN");
        flywheelN.setDirection(DcMotorSimple.Direction.REVERSE);
        Servo hoodN = hardwareMap.get(Servo.class, "hoodN");
        hoodN.setDirection(Servo.Direction.REVERSE);
        nectar = new Shooter(flywheelN, hoodN, hardwareMap.get(Servo.class, "blockerN"), nectarPIDF,
                17.04);   // hood angle at servo position 0 (CAD)

        intake = new Intake(
                hardwareMap.get(DcMotorEx.class, "intake"),
                hardwareMap.get(DcMotorEx.class, "transfer")
        );
    }

    // Odometry drivetrain for TeleOp. Pinpoint offsets come from pedro/Constants via
    // GoBildaPinpointOdometry.
    public OdometryHolonomicDrivetrain createDrivetrain() {
        GoBildaPinpointDriver pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpointDriver.recalibrateIMU();
        return new OdometryHolonomicDrivetrain(
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight"),
                new GoBildaPinpointOdometry(pinpointDriver)
        );
    }

    // Shooters and both blockers together, since both shooters always fire at the same time
    public void setShooterVelocities(double pollenVelocity, double nectarVelocity) {
        pollen.setVelocity(pollenVelocity);
        nectar.setVelocity(nectarVelocity);
    }

    public void openBlockers() {
        pollen.open();
        nectar.open();
    }

    public void closeBlockers() {
        pollen.close();
        nectar.close();
    }

    public boolean shootersAtSpeed() {
        return pollen.isAtSpeed() && nectar.isAtSpeed();
    }
}
