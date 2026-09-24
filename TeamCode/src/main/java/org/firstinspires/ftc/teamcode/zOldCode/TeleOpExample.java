package org.firstinspires.ftc.teamcode.examples;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.system.BasicHolonomicDrivetrain;


// An example of a simple robot-centric TeleOp.
//@Disabled
@Config
@TeleOp(name = "TeleOp Example", group = "TeleOp")
public class TeleOpExample extends OpMode {
    public static double velocity = 2800;
    private BasicHolonomicDrivetrain driveTrain;

    @Override
    public void init() {
        driveTrain = new BasicHolonomicDrivetrain(
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight")
        );
    }

    @Override
    public void loop() {
        // Set velocity targets based on gamepad input
        driveTrain.setVelocityDrive(
                -gamepad1.left_stick_y * velocity,
                -gamepad1.left_stick_x * velocity,
                -gamepad1.right_stick_x * velocity
        );
        // Power the motors
        driveTrain.drive();
    }
}
