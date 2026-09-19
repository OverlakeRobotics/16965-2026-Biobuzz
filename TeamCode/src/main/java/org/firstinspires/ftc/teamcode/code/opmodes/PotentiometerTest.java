package org.firstinspires.ftc.teamcode.code.opmodes;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.code.parts.Potentiometer;
import org.firstinspires.ftc.teamcode.code.parts.Turret;

@Config
@TeleOp(name = "Potentiometer Test", group = "TeleOp")
public class PotentiometerTest extends OpMode {
    public final double TICKS_PER_DEGREE = 537.7 / 360;
    Potentiometer potentiometer;
    DcMotorEx turret;

    @Override
    public void init() {
        potentiometer = new Potentiometer(hardwareMap.get(AnalogInput.class, "potentiometer"));
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        Log.d("Potentiometer", "Voltage: " + potentiometer.getVoltage());
        Log.d("Potentiometer", "Angle: " + potentiometer.getAngleFromVoltagePoly());
        Log.d("Potentiometer", "Encoder Angle: " + calculateAngle());
        Log.d("Mapping", String.format("%f, %f", calculateAngle(), potentiometer.getVoltage()));
        telemetry.addData("Potentiometer Voltage", potentiometer.getVoltage());
        telemetry.addData("Potentiometer Angle", potentiometer.getAngleFromVoltagePoly());
        telemetry.addData("Encoder Angle", calculateAngle());
        telemetry.update();
    }

    public double calculateAngle() {
        return turret.getCurrentPosition() / (TICKS_PER_DEGREE * 4);
    }
}
