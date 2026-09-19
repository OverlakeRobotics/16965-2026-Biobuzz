package org.firstinspires.ftc.teamcode.code.parts;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Intake {
    public enum IntakeState {
        EMPTY,
        PASSING,
        FULL,
        JAMMED,
        AMBIENT,
        OFF
    }
    private static final double LOWER_DISTANCE_THRESHOLD_MM = 200;
    private static final double MIDDLE_DISTANCE_THRESHOLD_MM = 100;
    private static final double UPPER_DISTANCE_THRESHOLD_MM = 100;
    private static final double MAX_VELOCITY = 2800;
    private final DcMotorEx intakeMotor;
    private final DistanceSensor lowerDistanceSensor;
    private final DistanceSensor middleDistanceSensor;
    private final NormalizedColorSensor upperColorSensor;
    private final ElapsedTime runtime = new ElapsedTime();
    private double middleBlockedBeginTime = -1;
    private double stallBeginTime = -1;
    private double wantedVelocity = 0;

    public Intake(DcMotorEx intakeMotor, DistanceSensor lowerDistanceSensor, DistanceSensor middleDistanceSensor, NormalizedColorSensor upperColorSensor) {
        this.intakeMotor = intakeMotor;
        this.intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.lowerDistanceSensor = lowerDistanceSensor;
        this.middleDistanceSensor = middleDistanceSensor;
        this.upperColorSensor = upperColorSensor;
    }

    public void setVelocity(double velocity) {
        wantedVelocity = Range.clip(velocity, -MAX_VELOCITY, MAX_VELOCITY);
        intakeMotor.setVelocity(wantedVelocity);
    }

    public double getVelocity() {
        return intakeMotor.getVelocity();
    }

    public double getPower() {
        return intakeMotor.getPower();
    }

    public double getCurrent() {
        return intakeMotor.getCurrent(CurrentUnit.AMPS);
    }

    public boolean isStalling() {
        boolean stalling = Math.abs(intakeMotor.getVelocity()) < Math.abs(wantedVelocity) * 0.2 && Math.abs(getCurrent()) > 3.0;
        if (!stalling) {
            stallBeginTime = -1;
            return false;
        } else if (stallBeginTime < 0) {
            stallBeginTime = runtime.seconds();
        }
        return runtime.seconds() - stallBeginTime > 0.5;
    }

    public IntakeState getState() {
        if (isStalling()) {
            return IntakeState.JAMMED;
        }

        boolean lowerBlocked = getLowerDistanceMM() < LOWER_DISTANCE_THRESHOLD_MM;
        boolean middleBlocked = getMiddleDistanceMM() < MIDDLE_DISTANCE_THRESHOLD_MM;
//        Log.d("LED Latency", "Dist: " + distanceSensor.getDistance(DistanceUnit.MM));
        if (!middleBlocked) {
            middleBlockedBeginTime = -1;
            if (!lowerBlocked) {
                return IntakeState.EMPTY;
            }
            return IntakeState.PASSING;
        } else if (middleBlockedBeginTime < 0) {
            middleBlockedBeginTime = runtime.seconds();
        }
        return runtime.seconds() - middleBlockedBeginTime > 0.3 && lowerBlocked ? IntakeState.FULL : IntakeState.PASSING;
    }

    public double getLowerDistanceMM() {
        return lowerDistanceSensor.getDistance(DistanceUnit.MM);
    }

    public double getMiddleDistanceMM() {
        return middleDistanceSensor.getDistance(DistanceUnit.MM);
    }

    public double getUpperDistanceMM() {
        if (upperColorSensor instanceof DistanceSensor) {
            return ((DistanceSensor) upperColorSensor).getDistance(DistanceUnit.MM);
        } else {
            return -1;
        }
    }

    public int getNumArtifactsInRamp() {
        int lowerBlocked = getLowerDistanceMM() < LOWER_DISTANCE_THRESHOLD_MM ? 1 : 0;
        int middleBlocked = getMiddleDistanceMM() < MIDDLE_DISTANCE_THRESHOLD_MM ? 1 : 0;
        int upperBlocked = getUpperDistanceMM() < UPPER_DISTANCE_THRESHOLD_MM ? 1 : 0;
        return lowerBlocked + middleBlocked + upperBlocked;
    }

    public void stop() {
        wantedVelocity = 0;
        this.setVelocity(wantedVelocity);
    }
}