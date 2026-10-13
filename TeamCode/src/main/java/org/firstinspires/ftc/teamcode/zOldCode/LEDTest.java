package org.firstinspires.ftc.teamcode.code.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.code.helpers.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.code.helpers.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.code.parts.Intake;
import org.firstinspires.ftc.teamcode.code.parts.LEDIndicator;

@Disabled
@Config
@TeleOp(name = "LED Test", group = "TeleOp")
public class LEDTest extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private LEDIndicator ledIndicator;
    private Intake intake;

    @Override
    public void init() {
        intake = new Intake(
                hardwareMap.get(DcMotorEx.class, "intake"),
                hardwareMap.get(DistanceSensor.class, "lowerDistanceSensor"),
                hardwareMap.get(DistanceSensor.class, "middleDistanceSensor"),
                hardwareMap.get(NormalizedColorSensor.class, "upperColorSensor")
        );
        ledIndicator = new LEDIndicator(hardwareMap.get(GoBildaPrismDriver.class, "prism"));
        ledIndicator.setState(Intake.IntakeState.AMBIENT);
    }

    @Override
    public void loop() {
        double start = runtime.seconds();
        ledIndicator.setState(intake.getState());
        telemetry.addData("Loop Time", runtime.seconds() - start);
        telemetry.update();
        if (gamepad1.aWasPressed()) {
            intake.setVelocity(2800);
        }

        if (gamepad1.bWasPressed()) {
            intake.setVelocity(-2800);
        }

        if (gamepad1.xWasPressed()) {
            intake.setVelocity(0);
        }
    }

    @Override
    public void stop() {
        ledIndicator.setState(Intake.IntakeState.OFF);
    }
}
