package org.firstinspires.ftc.teamcode.examples;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "Red Dual Shooter TeleOp", group = "TeleOp")
public class RedDualShooterTeleOp extends DualShooterTeleOp {
    @Override
    protected boolean isRedAlliance() {
        return true;
    }
}
