package org.firstinspires.ftc.teamcode.code.parts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// One flywheel + hood + blocker. The robot has two of these (pollen and nectar) that are built in
// RobotHardware; motor/servo directions are set there, so this class doesn't care which side it is.
@Config
public class Shooter {
    public static double MAX_VELOCITY = 2800; // ticks/s
    // How far the flywheel can be from its target and still count as "at speed" (ticks/s)
    public static double atSpeedTolerance = 100;
    public static double blockerOpenPos = 0.16;
    public static double blockerClosedPos = 0.0;
    // Degrees the hood servo turns from position 0 to 1. TODO: confirm for the servo model used.
    public static double HOOD_SERVO_RANGE_DEGREES = 300;
    // The servo's 32T gear drives the hood's 24T gear, so the hood turns 32/24 = 4/3 degrees per servo degree
    public static final double HOOD_GEAR_RATIO = 32.0 / 24.0;

    private final DcMotorEx flywheel;
    private final Servo hood;
    private final Servo blocker;
    // Owned by RobotHardware so each shooter can be tuned separately from the dashboard
    private final PIDFCoefficients pidf;
    private final PIDFCoefficients appliedPidf = new PIDFCoefficients();
    // Hood angle (degrees) when the hood servo is at position 0
    private final double hoodZeroAngle;

    private double targetVelocity = 0;
    private double hoodPosition = 0;

    public Shooter(DcMotorEx flywheel, Servo hood, Servo blocker, PIDFCoefficients pidf, double hoodZeroAngle) {
        this.flywheel = flywheel;
        this.flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.hood = hood;
        this.blocker = blocker;
        this.pidf = pidf;
        this.hoodZeroAngle = hoodZeroAngle;
        // NaN means "not tuned yet": start from the motor's built-in coefficients so the dashboard
        // shows the real values
        if (Double.isNaN(pidf.p)) {
            PIDFCoefficients motorDefault = flywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            pidf.p = motorDefault.p;
            pidf.i = motorDefault.i;
            pidf.d = motorDefault.d;
            pidf.f = motorDefault.f;
        }
        applyPidf();
        setHoodPosition(0);
        close();
    }

    public void setVelocity(double velocity) {
        applyPidf();
        targetVelocity = Range.clip(velocity, 0, MAX_VELOCITY);
        flywheel.setVelocity(targetVelocity);
    }

    // Sends the PIDF to the motor only when it changed on the dashboard, since each send is a hub command
    private void applyPidf() {
        if (pidf.p == appliedPidf.p && pidf.i == appliedPidf.i && pidf.d == appliedPidf.d && pidf.f == appliedPidf.f) {
            return;
        }
        flywheel.setVelocityPIDFCoefficients(pidf.p, pidf.i, pidf.d, pidf.f);
        appliedPidf.p = pidf.p;
        appliedPidf.i = pidf.i;
        appliedPidf.d = pidf.d;
        appliedPidf.f = pidf.f;
    }

    public double getVelocity() {
        return flywheel.getVelocity();
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    // True when the flywheel is spinning and within atSpeedTolerance of its target
    public boolean isAtSpeed() {
        return targetVelocity > 0 && Math.abs(getVelocity() - targetVelocity) <= atSpeedTolerance;
    }

    public boolean isBelowSpeed() {
        return getVelocity() < targetVelocity - atSpeedTolerance;
    }

    // Raw servo position, 0 to 1
    public void setHoodPosition(double position) {
        hoodPosition = Range.clip(position, 0, 1);
        hood.setPosition(hoodPosition);
    }

    public double getHoodPosition() {
        return hoodPosition;
    }

    // Hood angle in degrees. Clipped to what the servo can reach (position 0 to 1).
    public void setHoodAngle(double degrees) {
        setHoodPosition((degrees - hoodZeroAngle) / (HOOD_SERVO_RANGE_DEGREES * HOOD_GEAR_RATIO));
    }

    // The angle the hood was last told to go to (a servo can't report where it actually is)
    public double getHoodAngle() {
        return hoodZeroAngle + hoodPosition * HOOD_SERVO_RANGE_DEGREES * HOOD_GEAR_RATIO;
    }

    public void open() {
        blocker.setPosition(blockerOpenPos);
    }

    public void close() {
        blocker.setPosition(blockerClosedPos);
    }

    // Numbers are added as plain values (not formatted strings) so FTC Dashboard can graph them.
    // Graph "<name> velocity" against "<name> target" to see spin-up and recovery between shots.
    public void addTelemetry(Telemetry telemetry, String name) {
        telemetry.addData(name + " target", targetVelocity);
        telemetry.addData(name + " velocity", getVelocity());
        telemetry.addData(name + " hood", hoodPosition);
        telemetry.addData(name + " hood angle", getHoodAngle());
        if (isBelowSpeed()) {
            telemetry.addLine(name.toUpperCase() + " FLYWHEEL LOW");
        }
    }
}
