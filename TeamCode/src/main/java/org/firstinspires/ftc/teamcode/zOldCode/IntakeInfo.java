package org.firstinspires.ftc.teamcode.code.opmodes;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.code.parts.Intake;
import org.firstinspires.ftc.teamcode.code.parts.Turret;

@Disabled
@Config
@TeleOp(name = "Intake Info", group = "TeleOp")
public class IntakeInfo extends OpMode {
    private Intake intake;
    private Turret turret;
    @Override
    public void init() {
        this.intake = new Intake(
                hardwareMap.get(DcMotorEx.class, "intake"),
                hardwareMap.get(DistanceSensor.class, "lowerDistanceSensor"),
                hardwareMap.get(DistanceSensor.class, "middleDistanceSensor"),
                hardwareMap.get(NormalizedColorSensor.class, "upperColorSensor")
        );

        this.turret = new Turret(
                hardwareMap.get(DcMotorEx.class, "shooterTop"),
                hardwareMap.get(DcMotorEx.class, "shooterBottom"),
                hardwareMap.get(DcMotorEx.class, "turret"),
                hardwareMap.get(Servo.class, "hood"),
                hardwareMap.get(Servo.class, "blocker"),
                hardwareMap.get(AnalogInput.class, "potentiometer")
        );
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            intake.setVelocity(2800);
            turret.close();
        }
        if (gamepad1.bWasPressed()) {
            intake.setVelocity(0);
            turret.open();
        }
        Log.d("Intake Info", "Distance MM" + intake.getLowerDistanceMM());
        Log.d("Intake Info", "Velocity" + intake.getVelocity());
        Log.d("Intake Info", "Power" + intake.getPower());
        Log.d("Intake Info", "Current" + intake.getCurrent());
        telemetry.addData("Distance MM", intake.getLowerDistanceMM());
        telemetry.addData("Velocity", intake.getVelocity());
        telemetry.addData("Power", intake.getPower());
        telemetry.addData("Current", intake.getCurrent());
        telemetry.addData("Middle Distance", intake.getMiddleDistanceMM());
        telemetry.addData("Upper Distance", intake.getUpperDistanceMM());
        telemetry.update();
    }
}
