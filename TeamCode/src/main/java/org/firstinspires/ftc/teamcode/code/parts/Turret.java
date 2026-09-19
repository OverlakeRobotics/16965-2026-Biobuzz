package org.firstinspires.ftc.teamcode.code.parts;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
public class Turret extends Shooter {
    public final double TICKS_PER_DEGREE = 537.7 / 360;
    public static int MAX_LIMIT_ANGLE = 155; // 115; // degrees
    public static int MIN_LIMIT_ANGLE = -155; // -115; // degrees
    public static double turretPositionP = 17;
    public static double turretP = 10; // 15;
    public static double turretI = 1; // 0;
    public static double turretD = 2;
    public static double turretF = 0;
    public final int MAX_ANGLE_LIMIT;
    public final int MIN_ANGLE_LIMIT;
    public static int TURRET_VELOCITY = 2800;
    private final DcMotorEx turretMotor;
    private final Potentiometer potentiometer;
    public double encoderAngleOffset;
    public static boolean usePID = true;
    public Turret(DcMotorEx shooterMotor1, DcMotorEx shooterMotor2, DcMotorEx turretMotor, Servo hoodServo, Servo blocker, AnalogInput potentiometer) {
        super(shooterMotor1, shooterMotor2, hoodServo, blocker);
        this.MAX_ANGLE_LIMIT = MAX_LIMIT_ANGLE;
        this.MIN_ANGLE_LIMIT = MIN_LIMIT_ANGLE;
        this.turretMotor = turretMotor;
        this.potentiometer = new Potentiometer(potentiometer);
        this.turretMotor.setTargetPosition(0);
        this.turretMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        this.turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        if (usePID) {
            this.turretMotor.setVelocityPIDFCoefficients(turretP, turretI, turretD, turretF);
        }

//        this.turretMotor.setPositionPIDFCoefficients(turretPositionP);
    }

    // CCW is positive, CW is negative
    public void setTurretAngle(double angle) {
        Log.d("Turret", "Set wanted turret angle to: " + angle + " degrees");
        int targetAngle = (int) Math.round((Range.clip(angle, MIN_ANGLE_LIMIT, MAX_ANGLE_LIMIT) - encoderAngleOffset) * 4 * TICKS_PER_DEGREE);

        turretMotor.setTargetPosition(targetAngle);
        turretMotor.setVelocity(TURRET_VELOCITY);
    }

    public double getTurretTargetAngle() {
        return turretMotor.getTargetPosition() / (TICKS_PER_DEGREE * 4) + encoderAngleOffset;
    }

    public double getTurretCurrentAngle() {
        return turretMotor.getCurrentPosition() / (TICKS_PER_DEGREE * 4) + encoderAngleOffset;
    }

    public void resetTurretEncoder() {
        turretMotor.setVelocity(0);
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        Log.d("Turret", "Turret Reset Angle: " + turretMotor.getCurrentPosition());
    }

    private double getEncoderReportedAngle() {
        return turretMotor.getCurrentPosition() / (TICKS_PER_DEGREE * 4);
    }

    public void setEncoderOffset() {
        resetTurretEncoder();
        encoderAngleOffset = potentiometer.getAngleFromVoltagePoly() - getEncoderReportedAngle();
        Log.d("Turret", "Set encoder angle offset: " + encoderAngleOffset);
    }
}
