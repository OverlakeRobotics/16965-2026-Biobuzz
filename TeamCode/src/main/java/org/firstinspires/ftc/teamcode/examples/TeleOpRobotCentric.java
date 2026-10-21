package org.firstinspires.ftc.teamcode.examples;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointOdometry;
import org.firstinspires.ftc.teamcode.system.BasicHolonomicDrivetrain;
import org.firstinspires.ftc.teamcode.system.OdometryHolonomicDrivetrain;


// An example of a simple robot-centric TeleOp.
//@Disabled
@Config
@TeleOp(name = "TeleOp Robot Centric", group = "TeleOp")
public class TeleOpRobotCentric extends OpMode {
    public static double velocity = 2800;
    private OdometryHolonomicDrivetrain driveTrain;

    private DcMotorEx intake;
    private DcMotorEx flywheelP;
    private boolean debug = true;
    private double flywheelPPower = 0.0;
    private boolean intakeOn = false;

    private Servo hood;
    private double hoodPosition = 0.0;

    public static final double yOffset = -156.0; // -168.0 // mm
    public static final double xOffset = 72.0; // -84.0 // mm

    @Override
    public void init() {
        GoBildaPinpointDriver pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpointDriver.setOffsets(xOffset, yOffset, DistanceUnit.MM);
        pinpointDriver.recalibrateIMU();
        driveTrain = new OdometryHolonomicDrivetrain(
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight"),
                new GoBildaPinpointOdometry(pinpointDriver)
        );
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        flywheelP = hardwareMap.get(DcMotorEx.class, "flywheelP");
        flywheelP.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hood = hardwareMap.get(Servo.class, "hood");
    }

    @Override
    public void start() {
        driveTrain.setPosition(new Pose2D(DistanceUnit.INCH, -64, -33, AngleUnit.DEGREES, -90));
        hood.setPosition(hoodPosition);
    }

    @Override
    public void loop() {
        driveTrain.updatePosition();
        // Set velocity targets based on gamepad input (relative to the robot, not the field)
        driveTrain.setVelocityDrive(
                -gamepad1.left_stick_y * velocity,
                -gamepad1.left_stick_x * velocity,
                -gamepad1.right_stick_x * velocity
        );

        //intake
        // B toggles the intake on/off (-1). Holding A reverses it (1) until released.
        if (gamepad1.bWasPressed()){
            intakeOn = !intakeOn;
        }
        if (gamepad1.a) {
            intake.setPower(1);
        } else if (intakeOn) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }

        //shooter (power test)
        if (debug) {
            if (gamepad1.dpadUpWasPressed()) {
                flywheelPPower += 0.05;
                flywheelPPower = (Math.min(1.0, flywheelPPower));
            } else if (gamepad1.dpadDownWasPressed()) {
                flywheelPPower -= 0.05;
                flywheelPPower = (Math.max(0.0, flywheelPPower));
            }
            flywheelP.setPower(flywheelPPower);

            if (gamepad1.dpadRightWasPressed()) {
                hoodPosition += 0.001;
                hoodPosition = (Math.min(1.0, hoodPosition));
            } else if (gamepad1.dpadLeftWasPressed()) {
                hoodPosition -= 0.001;
                hoodPosition = (Math.max(0.0, hoodPosition));
            }
            hood.setPosition(hoodPosition);

            telemetry.addData("Flywheel power", "%.2f", flywheelPPower);
            telemetry.addData("Flywheel velocity (ticks/s)", "%.0f", flywheelP.getVelocity());
            telemetry.addData("Hood position", "%.3f", hoodPosition);
            telemetry.addData("Intake on", intakeOn);
        }


        // Power the motors
        driveTrain.drive();
    }
}
