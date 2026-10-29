package org.firstinspires.ftc.teamcode.code.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.code.helpers.BaseAuto;
import org.firstinspires.ftc.teamcode.system.PathServer;

@Config
@Autonomous(name = "Path Planner", group = "Autonomous")
public class PathPlanner extends BaseAuto {
    public static double sDelay = 0;
    @Override
    public void init() {
        PathServer.startServer();
        super.readJson = false;
        super.shooterDelay = sDelay;
        super.init();
    }

    @Override
    public void init_loop() {
        super.init_loop();
        PathServer.setRobotPose(super.odometry.getPosition());
    }

    @Override
    public void start() {
        super.route = PathServer.getRoute();
        super.alliance = PathServer.getAlliance();
        super.start();
    }

    @Override
    public void loop() {
        super.loop();
        PathServer.setRobotPose(super.odometry.getPosition());
    }

    @Override
    public void stop() {
        PathServer.stopServer();
        super.stop();
    }
}
