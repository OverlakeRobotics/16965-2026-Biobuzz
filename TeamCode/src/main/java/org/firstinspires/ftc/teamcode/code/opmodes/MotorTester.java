package org.firstinspires.ftc.teamcode.code.opmodes;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.code.parts.Intake;
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointOdometry;
import org.firstinspires.ftc.teamcode.system.BasicHolonomicDrivetrain;
import org.firstinspires.ftc.teamcode.system.OdometryHolonomicDrivetrain;

@Config
@TeleOp(name = "Motor Tester", group = "TeleOp")
public class MotorTester extends OpMode {
    public static int cps;
    public static double power;
    public static int position;
    public static boolean doPower = false;
    public static boolean doPosition = false;
    public static String motorName = "frontLeft";
    public DcMotorEx motor;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        if (doPosition) {
            motor.setTargetPosition(0);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    @Override
    public void loop() {
        if (doPosition) {
            motor.setTargetPosition(position);
            motor.setVelocity(cps);
        } else {
            if (doPower) {
                motor.setPower(power);
            } else {
                motor.setVelocity(cps);
            }
        }
        telemetry.addData("Power", motor.getPower());
        telemetry.addData("Vel", motor.getVelocity());
        telemetry.addData("Encoder pos", motor.getCurrentPosition());
        telemetry.update();
    }
}
