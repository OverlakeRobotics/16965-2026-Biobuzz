package org.firstinspires.ftc.teamcode.system.DifferentialSwerve;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
public class DifferentialSwerveModule {
    public static final double WHEEL_RADIUS_IN = 1.5;
    public static final double TICKS_PER_REV = 103.8;
    public static final double DEGREES_PER_TICK = 360.0 / TICKS_PER_REV;
    public static final double MOTOR_TURNS_PER_MODULE_TURN = 1.0;
    public static final double MOTOR_TURNS_PER_WHEEL_TURN = 1.0;
    public static final double MAX_MOTOR_VELOCITY = 2800;
    public static double steerP = 0;
    public static double steerI = 0;
    public static double steerD = 0;
    private final DcMotorEx topMotor;
    private final DcMotorEx bottomMotor;
    private final PIDFController steerPIDF = new PIDFController(steerP, steerI, steerD, 0);

    // x is positive forward, y is positive left
    private final double xOffsetIn;
    private final double yOffsetIn;
    private double targetDriveVelocity = 0;
    private double targetSteerAngleDeg = 0;
    private boolean flipSteerAngle = false;

    // IMPORTANT: assumes positive direction for both motors are the same for their respective gears (i.e., both CW or both CCW)
    // TODO: Check everything for correctness
    // TODO: Possibly add encoder for getting initial angle
    // TODO: Actually make the wheel turn the shorter way and reverse velocity
    public DifferentialSwerveModule(DcMotorEx topMotor, DcMotorEx bottomMotor, double xOffsetIn, double yOffsetIn) {
        this.xOffsetIn = xOffsetIn;
        this.yOffsetIn = yOffsetIn;
        this.topMotor = topMotor;
        this.bottomMotor = bottomMotor;
        this.topMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.bottomMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.topMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.bottomMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.topMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.bottomMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void updatePIDFValues() {
        steerPIDF.setPIDF(steerP, steerI, steerD, 0);
    }

    public void update() {
        double steerAngleError = normalize(targetSteerAngleDeg - getSteerAngleDeg());
        if (Math.abs(steerAngleError) >= 95) flipSteerAngle = true;
        else if (Math.abs(steerAngleError) <= 85) flipSteerAngle = false;
        if (flipSteerAngle) {
            // flip wheel direction instead of rotating module a long way
            steerAngleError = normalize(normalize(this.targetSteerAngleDeg + 180.0) - getSteerAngleDeg());
        }
        double steerDegPerSec = steerPIDF.calculate(0, steerAngleError);
        double steerTicksPerSec = steerDegPerSec * MOTOR_TURNS_PER_MODULE_TURN / DEGREES_PER_TICK;
        setVelocity(targetDriveVelocity * (flipSteerAngle ? -1 : 1), steerTicksPerSec);
    }

    private void setVelocity(double driveTicksPerSec, double steerTicksPerSec) {
//        double driveVelocityTicks = (driveTicksPerSec / WHEEL_RADIUS_IN) * (360 / (2 * Math.PI)) * MOTOR_TURNS_PER_WHEEL_TURN / DEGREES_PER_TICK;
//        double steerVelocityTicks = steerTicksPerSec * MOTOR_TURNS_PER_MODULE_TURN / DEGREES_PER_TICK;
//        double topTicksPerSec = driveVelocityTicks + steerVelocityTicks;
//        double bottomTicksPerSec = -driveVelocityTicks + steerVelocityTicks;
        double topTicksPerSec = driveTicksPerSec + steerTicksPerSec;
        double bottomTicksPerSec = -driveTicksPerSec + steerTicksPerSec;
        double max = Math.max(Math.abs(topTicksPerSec), Math.abs(bottomTicksPerSec));
        if (max > MAX_MOTOR_VELOCITY) {
            double scale = MAX_MOTOR_VELOCITY / max;
            topTicksPerSec *= scale;
            bottomTicksPerSec *= scale;
        }
        topMotor.setVelocity(topTicksPerSec);
        bottomMotor.setVelocity(bottomTicksPerSec);
    }

    public void setDesiredState(double driveVelocity, double steerAngleDeg) {
        this.targetDriveVelocity = (driveVelocity / WHEEL_RADIUS_IN) * (360.0 / (2 * Math.PI)) * MOTOR_TURNS_PER_WHEEL_TURN / DEGREES_PER_TICK;
        this.targetSteerAngleDeg = steerAngleDeg;
    }

    public void stop() {
        this.targetDriveVelocity = 0;
    }

    public double getSteerAngleDeg() {
        return (getTopPosDegrees() + getBottomPosDegrees()) / (2.0 * MOTOR_TURNS_PER_MODULE_TURN);
    }

    public void resetPIDF() {
        steerPIDF.reset();
    }

    public double getNormalizedSteerAngleDeg() {
        return normalize(getSteerAngleDeg());
    }

    public double getDrivePositionInches() {
        return (getTopPosDegrees() - getBottomPosDegrees()) / (2.0 * MOTOR_TURNS_PER_WHEEL_TURN) * (2 * Math.PI * WHEEL_RADIUS_IN) / 360.0;
    }

    // In degrees per second
    public double getSteerVelocity() {
        return (getTopVelDPS() + getBottomVelDPS()) / (2.0 * MOTOR_TURNS_PER_MODULE_TURN);
    }

    // In inches per second
    public double getDriveVelocity() {
        return (getTopVelDPS() - getBottomVelDPS()) / (2.0 * MOTOR_TURNS_PER_WHEEL_TURN) * (2 * Math.PI * WHEEL_RADIUS_IN) / 360.0;
    }

    public double getXOffsetIn() {
        return xOffsetIn;
    }

    public double getYOffsetIn() {
        return yOffsetIn;
    }

    private double getTopVelDPS() {
        return topMotor.getVelocity() * DEGREES_PER_TICK;
    }

    private double getBottomVelDPS() {
        return bottomMotor.getVelocity() * DEGREES_PER_TICK;
    }

    private double getTopPosDegrees() {
        return topMotor.getCurrentPosition() * DEGREES_PER_TICK;
    }

    private double getBottomPosDegrees() {
        return bottomMotor.getCurrentPosition() * DEGREES_PER_TICK;
    }

    private double normalize(double deg) {
        deg = ((deg + 180) % 360 + 360) % 360 - 180;
        return deg;
    }
}
