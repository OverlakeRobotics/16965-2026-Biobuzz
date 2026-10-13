package org.firstinspires.ftc.teamcode.examples;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointOdometry;
import org.firstinspires.ftc.teamcode.system.OdometryHolonomicDrivetrain;


// This OpMode shows a simple example of driving the bot along a set path of points using the
// driveTrain.setPositionDrive() method while passing in an array of Pose2Ds.
@Disabled
@Config
@Autonomous(name = "Odometry Auton Example", group = "Autonomous")
public class OdometryAutonExample extends OpMode {
    // Change to your actual offsets. See GoBildaPinpointDriver.setOffsets() for details on measuring offsets.
    public double yOffset = -168.0;
    public double xOffset = -84.0;

    public static int velocity = 2000;

    // How close the drivetrain has to be to a point before starting to drive to the next one.
    // This is in inches.
    public static double tolerance = 4;

    private OdometryHolonomicDrivetrain driveTrain;

    private final Pose2D[] drivePoints = {
            new Pose2D(DistanceUnit.INCH, 96, 0, AngleUnit.DEGREES, -90),
            new Pose2D(DistanceUnit.INCH, 96, -96, AngleUnit.DEGREES, 180),
            new Pose2D(DistanceUnit.INCH, 0, -96, AngleUnit.DEGREES, 90),
            new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0),
    };

    @Override
    public void init() {
        GoBildaPinpointDriver pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpointDriver.setOffsets(xOffset, yOffset, DistanceUnit.MM);
        driveTrain = new OdometryHolonomicDrivetrain(
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight"),
                new GoBildaPinpointOdometry(pinpointDriver)
        );

        driveTrain.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        driveTrain.setTolerance(tolerance);
        driveTrain.setVelocity(velocity);
    }

    @Override
    public void loop() {
        driveTrain.updatePosition();
        driveTrain.drive();
    }

    @Override
    public void start() {
        driveTrain.setPositionDrive(drivePoints);
    }
}
