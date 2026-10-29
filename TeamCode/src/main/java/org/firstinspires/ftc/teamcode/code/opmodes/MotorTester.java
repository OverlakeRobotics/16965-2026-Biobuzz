package org.firstinspires.ftc.teamcode.code.opmodes;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointOdometry;
import org.firstinspires.ftc.teamcode.system.BasicHolonomicDrivetrain;
import org.firstinspires.ftc.teamcode.system.OdometryHolonomicDrivetrain;

@Config
@TeleOp(name = "Motor Tester", group = "TeleOp")
public class MotorTester extends OpMode {
    public static int cps;
    public static double power = 1;
    public static int position;
    public static boolean doPower = true;
    public static boolean doPosition = false;
    public static String motorName = "frontLeft"; // change before INIT

    private DcMotorEx motor;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setModeIfNeeded(DcMotor.RunMode mode) {
        if (motor.getMode() != mode) motor.setMode(mode);
    }

    @Override
    public void loop() {
        if (doPosition) {
            motor.setTargetPosition(position); // target must be set before switching mode
            setModeIfNeeded(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setVelocity(cps);
        } else if (doPower) {
            setModeIfNeeded(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(power);
        } else {
            setModeIfNeeded(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setVelocity(cps);
        }

        telemetry.addData("cfg power", power);
        telemetry.addData("cfg cps", cps);
        telemetry.addData("cfg position", position);
        telemetry.addData("Mode", motor.getMode());
        telemetry.addData("Power", motor.getPower());
        telemetry.addData("Vel", motor.getVelocity());
        telemetry.addData("Encoder pos", motor.getCurrentPosition());
        telemetry.update();
    }
}