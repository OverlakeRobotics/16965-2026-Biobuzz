package org.firstinspires.ftc.teamcode.code.parts;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

// Intake and transfer motors. They always run together: with a blocker open, running both feeds
// balls into the shooters.
public class Intake {
    // Power that pulls balls in. Reverse (spit out) is the negative of this.
    public static final double IN_POWER = -1;

    private final DcMotorEx intakeMotor;
    private final DcMotorEx transferMotor;

    public Intake(DcMotorEx intakeMotor, DcMotorEx transferMotor) {
        this.intakeMotor = intakeMotor;
        this.transferMotor = transferMotor;
        this.intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.transferMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void setPower(double power) {
        power = Range.clip(power, -1, 1);
        intakeMotor.setPower(power);
        transferMotor.setPower(power);
    }

    public void in() {
        setPower(IN_POWER);
    }

    public void reverse() {
        setPower(-IN_POWER);
    }

    public void stop() {
        setPower(0);
    }

    public double getIntakeVelocity() {
        return intakeMotor.getVelocity();
    }

    public double getTransferVelocity() {
        return transferMotor.getVelocity();
    }
}
