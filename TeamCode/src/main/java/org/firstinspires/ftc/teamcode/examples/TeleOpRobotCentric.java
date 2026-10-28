package org.firstinspires.ftc.teamcode.examples;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    // D-pad up/down changes the selected flywheel target velocity by flywheelStep ticks/s per press
    public static double flywheelStep = 50;
    public static double maxFlywheelVelocity = 2800;
    private DcMotorEx flywheelP;
    private boolean debug = true;
    private double flywheelPVelocity = 0.0;
    private boolean intakeOn = false;

    private Servo hoodP;
    private double hoodPPosition = 0.0;

    // Both blockers share one position: Y moves them to blockerUpPos, X moves them back to 0
    public static double blockerUpPos = 0.16;
    private Servo blockerP;
    private double blockerPos = 0.0;

    private DcMotorEx flywheelN;
    private double flywheelNVelocity = 0.0;

    private Servo hoodN;
    private double hoodNPosition = 0.0;

    private Servo blockerN;

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
        flywheelP.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hoodP = hardwareMap.get(Servo.class, "hoodP");
        hoodP.setPosition(hoodPPosition);
        blockerP = hardwareMap.get(Servo.class, "blockerP");
        blockerP.setPosition(blockerPos);
        flywheelN = hardwareMap.get(DcMotorEx.class, "flywheelN");
        flywheelN.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelN.setDirection(DcMotorSimple.Direction.REVERSE);
        hoodN = hardwareMap.get(Servo.class, "hoodN");
        hoodN.setDirection(Servo.Direction.REVERSE);
        hoodN.setPosition(hoodNPosition);
        blockerN = hardwareMap.get(Servo.class, "blockerN");
        blockerN.setPosition(blockerPos);

    }

    @Override
    public void start() {
        driveTrain.setPosition(new Pose2D(DistanceUnit.INCH, -64, -33, AngleUnit.DEGREES, -90));

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

        //shooter (velocity test)
        if (debug) {
            // WasPressed() only returns true once per press, so read it once and share it between both flywheels
            boolean flywheelUp = gamepad1.dpadUpWasPressed();
            boolean flywheelDown = gamepad1.dpadDownWasPressed();
            // Holding LB moves the nectar flywheel/hood; otherwise the d-pad moves the pollen ones
            boolean nectarSelected = gamepad1.left_bumper;

            if (!nectarSelected) {
                if (flywheelUp) {
                    flywheelPVelocity += flywheelStep;
                    flywheelPVelocity = (Math.min(maxFlywheelVelocity, flywheelPVelocity));
                } else if (flywheelDown) {
                    flywheelPVelocity -= flywheelStep;
                    flywheelPVelocity = (Math.max(0.0, flywheelPVelocity));
                }

                if (gamepad1.dpad_right) {
                    hoodPPosition += 0.01;
                    hoodPPosition = (Math.min(1.0, hoodPPosition));
                } else if (gamepad1.dpad_left) {
                    hoodPPosition -= 0.01;
                    hoodPPosition = (Math.max(0.0, hoodPPosition));
                }
            }
            flywheelP.setVelocity(flywheelPVelocity);

            if (gamepad1.y) {
                blockerPos = blockerUpPos;
            } else if (gamepad1.x) {
                blockerPos = 0.0;
            }

            hoodP.setPosition(hoodPPosition);
            blockerP.setPosition(blockerPos);
            blockerN.setPosition(blockerPos);

            telemetry.addData("Flywheel target velocity (ticks/s)", "%.0f", flywheelPVelocity);
            telemetry.addData("Flywheel velocity (ticks/s)", "%.0f", flywheelP.getVelocity());
            telemetry.addData("Hood position", "%.3f", hoodPPosition);
            telemetry.addData("Intake on", intakeOn);
            telemetry.addData("Intake velocity (ticks/s)", "%.0f", intake.getVelocity());
            telemetry.addData("D-pad controls", nectarSelected ? "NECTAR" : "POLLEN");
            telemetry.addData("pollen blocker position: ", blockerPos);

            //nectar shooter (velocity test)
            if (nectarSelected) {
                if (flywheelUp) {
                    flywheelNVelocity += flywheelStep;
                    flywheelNVelocity = (Math.min(maxFlywheelVelocity, flywheelNVelocity));
                } else if (flywheelDown) {
                    flywheelNVelocity -= flywheelStep;
                    flywheelNVelocity = (Math.max(0.0, flywheelNVelocity));
                }

                if (gamepad1.dpad_right) {
                    hoodNPosition += 0.01;
                    hoodNPosition = (Math.min(1.0, hoodNPosition));
                } else if (gamepad1.dpad_left) {
                    hoodNPosition -= 0.01;
                    hoodNPosition = (Math.max(0.0, hoodNPosition));
                }
            }
            flywheelN.setVelocity(flywheelNVelocity);

            hoodN.setPosition(hoodNPosition);

            telemetry.addData("Nectar flywheel target velocity (ticks/s)", "%.0f", flywheelNVelocity);
            telemetry.addData("Nectar flywheel velocity (ticks/s)", "%.0f", flywheelN.getVelocity());
            telemetry.addData("Nectar hood position", "%.3f", hoodNPosition);
            telemetry.addData("nectar blocker position: ", blockerPos);
            telemetry.update();
        }


        // Power the motors
        driveTrain.drive();
    }
}
