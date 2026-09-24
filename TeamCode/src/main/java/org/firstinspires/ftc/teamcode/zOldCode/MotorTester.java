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
@TeleOp(name = "Motor Tester 2", group = "TeleOp")
public class MotorTester extends OpMode {
    public static int cps;
    public static double power;
    public static int position;
    public static boolean doPower = false;
    public static boolean doPosition = false;
    public static String motorName = "frontLeft";
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;


    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        if (doPosition) {
            frontRight.setTargetPosition(0);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setTargetPosition(0);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setTargetPosition(0);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setTargetPosition(0);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    @Override
    public void loop() {
        if (doPosition) {
            frontRight.setTargetPosition(position);
            frontRight.setVelocity(cps);
            frontLeft.setTargetPosition(position);
            frontLeft.setVelocity(cps);
            backLeft.setTargetPosition(position);
            backLeft.setVelocity(cps);
            backRight.setTargetPosition(position);
            backRight.setVelocity(cps);
        } else {
            if (doPower) {
                if (gamepad1.y) {
                    frontLeft.setPower(power);
                }
                if (gamepad1.b) {
                    frontRight.setPower(power);
                }
                if (gamepad1.x) {
                    backLeft.setPower(power);
                }
                if (gamepad1.a) {
                    backRight.setPower(power);
                }
            } else {
                if (gamepad1.y) {
                    frontLeft.setVelocity(cps);
                }
                if (gamepad1.b) {
                    frontRight.setVelocity(cps);
                }
                if (gamepad1.x) {
                    backLeft.setVelocity(cps);
                }
                if (gamepad1.a) {
                    backRight.setVelocity(cps);
                }
            }
        }
//        telemetry.addData("Power", motor.getPower());
//        telemetry.addData("Vel", motor.getVelocity());
//        telemetry.addData("Encoder pos", motor.getCurrentPosition());
        telemetry.update();
    }
}
