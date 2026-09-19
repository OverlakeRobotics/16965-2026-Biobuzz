package org.firstinspires.ftc.teamcode.examples.tuners;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointOdometry;
import org.firstinspires.ftc.teamcode.system.BasicHolonomicDrivetrain;
import org.firstinspires.ftc.teamcode.system.OdometryModule;


// This class uses a GobildaPinpoint odometry as a baseline to tune the COUNTS_PER_DEGREE
// constant automatically. It's not required to tune this, but it will make driving better.
// COUNTS_PER_DEGREE is located in the OdometryHolonomicDrivetrain class and can be set there
// after running this OpMode and getting the value. Before running this tuner class, make sure you
// have the correct offsets for your x and y offsets for the pinpoint.
//@Disabled
@Config
@Autonomous(name = "Turn Counts Tuner", group = "Autonomous")
public class TurnCountsTuner extends OpMode {
    // Change to your actual offsets. See GoBildaPinpointDriver.setOffsets() for details on measuring offsets.
    public double yOffset = -168.0;
    public double xOffset = -84.0;

    public static int velocity = 1000;

    private BasicHolonomicDrivetrain driveTrain;
    private OdometryModule odometry;

    public double countsPerDegree;

    public static int turnCounts = 40000;
    
    private int revolutions = 0;
    private double lastHeading = 0;


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
    
    public boolean isPos(double x) {
        return x >= 0;
    }

    @Override
    public void loop() {
        driveTrain.drive();
        odometry.updatePosition();
        Pose2D robotPos = odometry.getPosition();

        double heading = robotPos.getHeading(AngleUnit.DEGREES);
        if (isPos(heading) && !isPos(lastHeading)) {
            revolutions++;
        }

        if (!driveTrain.isDriving() && countsPerDegree == 0) {
            double totalHeading = (revolutions * 360 + (360 + robotPos.getHeading(AngleUnit.DEGREES)) % 360);
            countsPerDegree = turnCounts / totalHeading;
        }

        if (countsPerDegree != 0) {
            telemetry.addData("Counts Per Degree", countsPerDegree);
            Log.d("Counts Per Degree", "Counts Per Degree: " + countsPerDegree);
        }
        
        lastHeading = heading;
        
        telemetry.update();
    }

    @Override
    public void start() {
        driveTrain.setPositionDrive(0, 0, turnCounts);
    }
}
