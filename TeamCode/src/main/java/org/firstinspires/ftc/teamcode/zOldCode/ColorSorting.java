package org.firstinspires.ftc.teamcode.code.opmodes;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.code.parts.Intake;

@Config
@TeleOp(name = "Color Sorting", group = "TeleOp")
@Disabled
public class ColorSorting extends OpMode {
    public HuskyLens huskyLens;
    public Servo greenSorter;
    public Servo purpleSorter;
    public Intake intake;

    @Override
    public void init() {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        greenSorter = hardwareMap.get(Servo.class, "greenSorter");
        purpleSorter = hardwareMap.get(Servo.class, "purpleSorter");
        intake = new Intake(
                hardwareMap.get(DcMotorEx.class, "intake"),
                hardwareMap.get(DistanceSensor.class, "lowerDistanceSensor"),
                hardwareMap.get(DistanceSensor.class, "middleDistanceSensor"),
                hardwareMap.get(NormalizedColorSensor.class, "upperColorSensor")
        );
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
    }

    @Override
    public void loop() {
        intake.setVelocity(2500 * gamepad1.left_stick_y);

        if (gamepad1.aWasPressed()) {
            greenSorter.setPosition(0.8);
            purpleSorter.setPosition(0.6);
        } else if (gamepad1.bWasPressed()) {
            greenSorter.setPosition(0.6);
            purpleSorter.setPosition(0.6);
        } else if (gamepad1.yWasPressed()) {
            purpleSorter.setPosition(0.8);
            greenSorter.setPosition(0.6);
        } else if (gamepad1.xWasPressed()) {
            purpleSorter.setPosition(0.8);
            greenSorter.setPosition(0.8);
        }

        HuskyLens.Block[] blocks = huskyLens.blocks();
        HuskyLens.Block largestBlock = null;
        int largestSize = 0;
        for (HuskyLens.Block block : blocks) {
            telemetry.addData("Block", block.toString());
            int currSize = block.height * block.width;
            if (currSize > largestSize) {
                largestSize = currSize;
                largestBlock = block;
            }
        }

        if (largestBlock != null) {
            Log.d("Husky Cam", "ID: " + largestBlock.id);
        }

        telemetry.update();
    }
}
