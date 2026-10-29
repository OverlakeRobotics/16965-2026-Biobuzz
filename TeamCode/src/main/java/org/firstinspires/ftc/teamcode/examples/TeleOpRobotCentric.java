package org.firstinspires.ftc.teamcode.examples;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointOdometry;
import org.firstinspires.ftc.teamcode.system.OdometryHolonomicDrivetrain;


// An example of a simple robot-centric TeleOp. Shooter/intake testing lives in
// code/opmodes/ManualTestTeleOp.
//@Disabled
@Config
@TeleOp(name = "TeleOp Robot Centric", group = "TeleOp")
public class TeleOpRobotCentric extends OpMode {
    public static double velocity = 2800;
    private OdometryHolonomicDrivetrain driveTrain;

    @Override
    public void init() {
        GoBildaPinpointDriver pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpointDriver.recalibrateIMU();
        driveTrain = new OdometryHolonomicDrivetrain(
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight"),
                new GoBildaPinpointOdometry(pinpointDriver)
        );
    }

    @Override
    public void start() {
        driveTrain.setPosition(new Pose2D(DistanceUnit.INCH, -64, -33, AngleUnit.DEGREES, -90));
    }

    @Override
    public void loop() {
        driveTrain.updatePosition();
        // Set velocity targets based on gamepad input (relative to the robot, not the field)
        driveTrain.setVelocityDrive(
                -gamepad1.left_stick_y * velocity,
                -gamepad1.left_stick_x * velocity,
                -gamepad1.right_stick_x * velocity
        );

        // Power the motors
        driveTrain.drive();
    }
}
