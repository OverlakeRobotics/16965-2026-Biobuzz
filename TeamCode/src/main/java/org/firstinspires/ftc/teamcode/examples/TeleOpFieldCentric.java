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
import org.firstinspires.ftc.teamcode.system.BasicHolonomicDrivetrain;
import org.firstinspires.ftc.teamcode.system.OdometryHolonomicDrivetrain;


// An example of a simple robot-centric TeleOp.
//@Disabled
@Config
@TeleOp(name = "TeleOp Field Centric", group = "TeleOp")
public class TeleOpFieldCentric extends OpMode {
    public static double velocity = 2800;
    private OdometryHolonomicDrivetrain driveTrain;

    public static final double yOffset = -156.0; // -168.0 // mm
    public static final double xOffset = 72.0; // -84.0 // mm

    @Override
    public void init() {
        GoBildaPinpointDriver pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpointDriver.setOffsets(xOffset, yOffset, DistanceUnit.MM);
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
        // Set velocity targets based on gamepad input
        driveTrain.setVelocityDriveFieldCentric(
                -gamepad1.left_stick_y * velocity,
                -gamepad1.left_stick_x * velocity,
                -gamepad1.right_stick_x * velocity,
                -90
        );
        // Power the motors
        driveTrain.drive();
    }
}
