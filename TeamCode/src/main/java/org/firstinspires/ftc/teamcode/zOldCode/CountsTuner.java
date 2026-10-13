package org.firstinspires.ftc.teamcode.code.opmodes;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.code.parts.Intake;
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointOdometry;
import org.firstinspires.ftc.teamcode.system.BasicHolonomicDrivetrain;
import org.firstinspires.ftc.teamcode.system.OdometryHolonomicDrivetrain;

@Config
@TeleOp(name = "Counts Tuner", group = "TeleOp")
@Disabled
public class CountsTuner extends OpMode {
    public double yOffset = -168.0; // mm
    public double xOffset = -84.0; // mm
    public BasicHolonomicDrivetrain driveTrain;
    public static int countsForward = 3000;
    public static int velocity = 1000;
    public static double direction = 0;
    GoBildaPinpointDriver pinpoint;

    @Override
    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(xOffset, yOffset, DistanceUnit.MM);
        pinpoint.resetPosAndIMU();
        driveTrain = new BasicHolonomicDrivetrain(
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight")
        );
        driveTrain.setPositionDrive(countsForward, direction, velocity);
    }

    @Override
    public void loop() {
        pinpoint.update();
        driveTrain.drive();
        Log.d("Pinpoint", pinpoint.getPosition().toString());
        telemetry.addData("Pinpoint Pos", pinpoint.getPosition());
        telemetry.update();
    }
}
