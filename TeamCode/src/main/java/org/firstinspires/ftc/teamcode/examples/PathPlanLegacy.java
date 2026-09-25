package org.firstinspires.ftc.teamcode.examples;


import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointOdometry;
import org.firstinspires.ftc.teamcode.system.BasicHolonomicDrivetrain;
import org.firstinspires.ftc.teamcode.system.OdometryHolonomicDrivetrain;
import org.firstinspires.ftc.teamcode.system.PathServer;

import java.util.Arrays;


// The ORIGINAL Overlake point-chasing follower, kept as a baseline for comparing against the
// Pedro Pathing follower in PathPlanExample. Upload the same route from the Path Planner to both
// OpModes and compare the "Finished in" time and how closely the live pose tracks the drawn path.
//
// Differences from the old example, because the planner no longer sends a global velocity or
// tolerance:
//      - velocity tags are a PERCENT of the drivetrain's max velocity (100 = flat out), matching
//        the Pedro side. A value of 1 or less is treated as a fraction.
//      - the path tolerance comes from the TOLERANCE dashboard field below.
// Pause tags still hold the robot at the previous point for the tag's value in seconds.
@Config
@Autonomous(name = "Path Planning Legacy", group = "Autonomous")
public class PathPlanLegacy extends OpMode {
    // Pinpoint pod offsets in mm. These match pedro/Constants so both followers localize the same way.
    public static double X_OFFSET_MM = -84.0;
    public static double Y_OFFSET_MM = -168.0;
    // How close (in inches) the robot has to get to a point before it advances to the next one.
    public static double TOLERANCE = 8;

    private OdometryHolonomicDrivetrain driveTrain;
    private Pose2D[] positions;
    private PathServer.Tag[] tags;
    private int lastTagIndex = 0;
    private final ElapsedTime runtime = new ElapsedTime();
    private double lastTime = 0;
    private double pauseTimeLeft = 0;
    private int pausedIndex = -1;
    private double finishedAt = -1;

    @Override
    public void init() {
        GoBildaPinpointDriver pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpointDriver.setOffsets(X_OFFSET_MM, Y_OFFSET_MM, DistanceUnit.MM);
        driveTrain = new OdometryHolonomicDrivetrain(
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight"),
                new GoBildaPinpointOdometry(pinpointDriver)
        );

        PathServer.startServer();
        telemetry.addLine("Legacy follower ready. Upload a route, then start.");
    }

    @Override
    public void init_loop() {
        driveTrain.updatePosition();
        PathServer.setRobotPose(driveTrain.getPosition());
        telemetry.addData("Points uploaded", PathServer.getPath().length);
        telemetry.update();
    }

    @Override
    public void start() {
        positions = PathServer.getPath();
        driveTrain.setPosition(PathServer.getStartPose());
        driveTrain.setTolerance(TOLERANCE);
        driveTrain.setVelocity(BasicHolonomicDrivetrain.MAX_VELOCITY);

        tags = PathServer.getTags();
        Arrays.sort(tags);
        lastTagIndex = 0;
        pauseTimeLeft = 0;
        pausedIndex = -1;
        finishedAt = -1;

        if (positions.length > 0) {
            driveTrain.setPositionDrive(positions);
        }
        runtime.reset();
        lastTime = 0;
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
                    case "velocity": {
                        double fraction = currTag.value <= 1 ? currTag.value : currTag.value / 100.0;
                        fraction = Math.max(0.05, Math.min(1.0, fraction));
                        driveTrain.setVelocity((int) (fraction * BasicHolonomicDrivetrain.MAX_VELOCITY));
                        break;
                    }
                    case "pause":
                        if (currTag.value <= 0 || nextPointIndex < 1) break;
                        pauseTimeLeft = currTag.value;
                        pausedIndex = nextPointIndex;
                        driveTrain.setPositionDrive(positions[nextPointIndex - 1]);
                        break;
                    default:
                        Log.d("PathPlanLegacy", "Ignoring tag " + currTag.name + "=" + currTag.value);
                        break;
                }
                lastTagIndex++;
            }

            boolean atLastPoint = positions.length > 0 && nextPointIndex >= positions.length - 1;
            if (finishedAt < 0 && atLastPoint && !driveTrain.isDriving()) {
                finishedAt = curTime;
                Log.d("PathPlanLegacy", "Finished route in " + finishedAt + " s");
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

        telemetry.addData("Follower", "legacy point chaser");
        telemetry.addData("Elapsed", "%.2f s", curTime);
        telemetry.addData("Next point", "%d / %d", driveTrain.getNextPointIndex(), positions.length);
        telemetry.addData("Pose", "x %.1f  y %.1f  h %.1f",
                pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
        if (finishedAt >= 0) telemetry.addData("Finished in", "%.2f s", finishedAt);
        telemetry.update();
    }

    @Override
    public void stop() {
        PathServer.stopServer();
    }
}
