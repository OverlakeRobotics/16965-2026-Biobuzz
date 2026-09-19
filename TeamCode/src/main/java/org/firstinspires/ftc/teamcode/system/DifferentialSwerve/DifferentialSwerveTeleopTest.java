package org.firstinspires.ftc.teamcode.system.DifferentialSwerve;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.system.BasicHolonomicDrivetrain;

@Config
@TeleOp(name = "Diffy Swerve Teleop Test", group = "TeleOp")
public class DifferentialSwerveTeleopTest extends OpMode {
    // in/s
    public static double driveVelocity = 33;
    // deg/s
    public static double turnVelocity = 360;
    private BasicDifferentialSwerve driveTrain;

    @Override
    public void init() {
        driveTrain = new BasicDifferentialSwerve(
                new DifferentialSwerveModule[] {
                        new DifferentialSwerveModule(
                                hardwareMap.get(DcMotorEx.class, "leftTop"),
                                hardwareMap.get(DcMotorEx.class, "leftBottom"),
                                0,
                                8
                        ),
                        new DifferentialSwerveModule(
                                hardwareMap.get(DcMotorEx.class, "rightTop"),
                                hardwareMap.get(DcMotorEx.class, "rightBottom"),
                                0,
                                -8
                        )
                }
        );
    }

    @Override
    public void start() {
        driveTrain.resetPIDF();
    }

    @Override
    public void loop() {
        driveTrain.updatePIDFValues();
        // Set velocity targets based on gamepad input
        driveTrain.setVelocityDrive(
                -gamepad1.left_stick_y * driveVelocity,
                -gamepad1.left_stick_x * driveVelocity,
                -gamepad1.right_stick_x * turnVelocity);
        // Power the motors
        driveTrain.drive();
    }
}
