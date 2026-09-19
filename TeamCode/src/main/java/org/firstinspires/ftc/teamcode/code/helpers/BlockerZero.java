package org.firstinspires.ftc.teamcode.code.helpers;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.code.parts.Turret;
import org.firstinspires.ftc.teamcode.system.BasicHolonomicDrivetrain;


// An example of a simple robot-centric TeleOp.
//@Disabled
@Config
@TeleOp(name = "Blocker Zero", group = "TeleOp")
public class BlockerZero extends OpMode {
    public Turret turret;

    @Override
    public void init() {
        turret = new Turret(
                hardwareMap.get(DcMotorEx.class, "shooterTop"),
                hardwareMap.get(DcMotorEx.class, "shooterBottom"),
                hardwareMap.get(DcMotorEx.class, "turret"),
                hardwareMap.get(Servo.class, "hood"),
                hardwareMap.get(Servo.class, "blocker"),
                hardwareMap.get(AnalogInput.class, "potentiometer")
        );
    }

    @Override
    public void loop() {
        turret.close();
    }
}
