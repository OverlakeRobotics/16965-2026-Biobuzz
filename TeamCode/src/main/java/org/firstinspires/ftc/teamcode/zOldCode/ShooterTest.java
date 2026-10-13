package org.firstinspires.ftc.teamcode.code.opmodes;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.code.parts.Intake;

@Config
@TeleOp(name = "Shooter Test", group = "TeleOp")
@Disabled
public class ShooterTest extends OpMode {
    private DcMotorEx shooterMotor;
    private Servo hoodServo;
    private int shooterVelocity;

    private double hoodPos;

    @Override
    public void init() {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        hoodServo = hardwareMap.get(Servo.class, "hood");
        shooterVelocity = 0;

        hoodPos = 0.5;
    }

    @Override
    public void loop() {
        if (gamepad1.yWasPressed()) {
            shooterVelocity += 300;
        } else if (gamepad1.aWasPressed()) {
            shooterVelocity -= 300;
        } else if (gamepad1.xWasPressed()) {
            shooterVelocity = 0;
        }

        shooterMotor.setVelocity(shooterVelocity);

        telemetry.addData("Wanted Shooter Velocity", shooterVelocity);
        telemetry.addData("Actual Shooter Velocity", shooterMotor.getVelocity());
        telemetry.addData("Shooter Power", shooterMotor.getPower());

        if (gamepad1.dpadUpWasPressed()) {
            hoodPos -= 0.04;
        } else if (gamepad1.dpadDownWasPressed()) {
            hoodPos += 0.04;
        }


        hoodPos = Range.clip(hoodPos,0, 1);
        hoodServo.setPosition(hoodPos);

        telemetry.addData("Servo Position", hoodPos);

        telemetry.update();
    }
}
