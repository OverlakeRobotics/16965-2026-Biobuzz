package org.firstinspires.ftc.teamcode.examples.tuners;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointOdometry;
import org.firstinspires.ftc.teamcode.system.BasicHolonomicDrivetrain;
import org.firstinspires.ftc.teamcode.system.OdometryModule;


// This class uses a GobildaPinpoint odometry as a baseline to tune the FORWARD_COUNTS_PER_INCH
// constant automatically. It's not required to tune this, but it will make driving better.
// FORWARD_COUNTS_PER_INCH is located in the BasicHolonomicDrivetrain class and can be set there
// after running this OpMode and getting the value. Before running this tuner class, make sure you
// have the correct offsets for your x and y offsets for the pinpoint.
//@Disabled
@Config
@Autonomous(name = "Forward Counts Tuner", group = "Autonomous")
public class ForwardCountsTuner extends OpMode {
    // Change to your actual offsets. See GoBildaPinpointDriver.setOffsets() for details on measuring offsets.
    public double yOffset = -168.0;
    public double xOffset = -84.0;

    public static int velocity = 1000;

    private BasicHolonomicDrivetrain driveTrain;
    private OdometryModule odometry;

    public double forwardCountsPerInch;

    public static int driveDist = 2500;


    @Override
    public void init() {
        GoBildaPinpointDriver pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpointDriver.setOffsets(xOffset, yOffset, DistanceUnit.MM);
        odometry = new GoBildaPinpointOdometry(pinpointDriver);
        odometry.reset();

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
        odometry.updatePosition();

        if (!driveTrain.isDriving() && forwardCountsPerInch == 0) {
            Pose2D robotPos = odometry.getPosition();
            double distDrivenInches = Math.hypot(robotPos.getX(DistanceUnit.INCH), robotPos.getY(DistanceUnit.INCH));
            forwardCountsPerInch = driveDist / distDrivenInches;
        }

        if (forwardCountsPerInch != 0) {
            telemetry.addData("Forward Counts Per Inch", forwardCountsPerInch);
            Log.d("Forward Counts Per Inch", "Forward Counts Per Inch: " + forwardCountsPerInch);
        }

        telemetry.update();
    }

    @Override
    public void start() {
        driveTrain.setPositionDrive(driveDist, 0);
    }
}
