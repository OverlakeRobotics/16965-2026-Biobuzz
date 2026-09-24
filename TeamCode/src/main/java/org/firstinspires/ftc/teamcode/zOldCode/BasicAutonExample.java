package org.firstinspires.ftc.teamcode.examples;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.system.BasicHolonomicDrivetrain;


// Example that drives 2000 counts forward then 2000 counts to the left.
@Disabled
@Config
@Autonomous(name = "Basic Auton Example", group = "Autonomous")
public class BasicAutonExample extends OpMode {

    private BasicHolonomicDrivetrain driveTrain;
    public static int velocity = 1000;

    @Override
    public void init() {
        driveTrain = new BasicHolonomicDrivetrain(
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight")
        );

        driveTrain.setVelocity(velocity);
    }

    @Override
    public void loop() {
        driveTrain.drive();

        if (!driveTrain.isDriving()) {
            driveTrain.setPositionDrive(2000, 90);
        }
    }

    @Override
    public void start() {
            driveTrain.setPositionDrive(2000, 0);
    }
}
