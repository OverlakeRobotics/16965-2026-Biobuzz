package org.firstinspires.ftc.teamcode.examples;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointOdometry;
import org.firstinspires.ftc.teamcode.system.BasicHolonomicDrivetrain;
import org.firstinspires.ftc.teamcode.system.OdometryHolonomicDrivetrain;
import org.firstinspires.ftc.teamcode.system.PathServer;

import java.util.Arrays;


// This is an example OpMode designed to interface with the Overlake Robotics Path Planner.
// It will let you use the path planner to upload a path to the robot with movement and two tags.
// The two tags are:
//      - velocity: Changes the velocity of the robot to the tags value (this is in in/s).
//      - pause: Pauses the robot for a number of seconds equal to the tags value.
// Feel free to take this OpMode and add your own robot specific tags if you are using the path planner.
//@Disabled
@Config
@Autonomous(name = "Path Planning Example", group = "Autonomous")
public class PathPlanExample extends OpMode {
    public double yOffset = -168.0;
    public double xOffset = -84.0;

    private OdometryHolonomicDrivetrain driveTrain;
    public Pose2D[] positions;
    public PathServer.Tag[] tags;
    private int lastTagIndex = 0;
    private final ElapsedTime runtime = new ElapsedTime();
    private double lastTime = 0;
    private double pauseTimeLeft = 0;
    private int pausedIndex = -1;

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

        PathServer.startServer();
    }

    @Override
    public void init_loop() {
        driveTrain.updatePosition();
        PathServer.setRobotPose(driveTrain.getPosition());
    }

    @Override
    public void start() {
        driveTrain.setVelocity((int) (PathServer.getVelocity() * BasicHolonomicDrivetrain.FORWARD_COUNTS_PER_INCH));
        positions = PathServer.getPath();
        driveTrain.setPosition(PathServer.getStartPose());
        driveTrain.setTolerance(PathServer.getTolerance());

        tags = PathServer.getTags();
        Arrays.sort(tags);

        driveTrain.setPositionDrive(positions);
    }


    @Override
    public void loop() {
        driveTrain.updatePosition();
        Pose2D pos = driveTrain.getPosition();
        PathServer.setRobotPose(pos);

        double curTime = runtime.seconds();
        double dt = curTime - lastTime;
        lastTime = curTime;

        if (pauseTimeLeft <= 0) {
            driveTrain.drive();
            int nextPointIndex = driveTrain.getNextPointIndex();

            while (lastTagIndex < tags.length && tags[lastTagIndex].index <= nextPointIndex) {
                PathServer.Tag currTag = tags[lastTagIndex];
                switch (currTag.name) {
                    case "velocity":
                        driveTrain.setVelocity((int) (currTag.value * BasicHolonomicDrivetrain.FORWARD_COUNTS_PER_INCH));
                        break;
                    case "pause":
                        if (currTag.value <= 0) break;
                        pauseTimeLeft = currTag.value;
                        pausedIndex = nextPointIndex;
                        driveTrain.setPositionDrive(positions[nextPointIndex - 1]);
                        break;
                    // Add more cases here for your own custom tags!
                }
                lastTagIndex++;
            }
        } else {
            pauseTimeLeft -= dt;

            if (pauseTimeLeft <= 0) {
                pauseTimeLeft = 0;
                driveTrain.setPositionDrive(positions, pausedIndex);
            } else {
                driveTrain.setPositionDrive(positions[pausedIndex - 1]);
            }
        }
    }

    @Override
    public void stop() {
        PathServer.stopServer();
    }
}
