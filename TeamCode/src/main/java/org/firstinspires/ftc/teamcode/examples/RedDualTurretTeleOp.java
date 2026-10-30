package org.firstinspires.ftc.teamcode.examples;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "Red Dual Turret TeleOp", group = "TeleOp")
public class RedDualTurretTeleOp extends DualTurretTeleOp {
    @Override
    protected boolean isRedAlliance() {
        return true;
    }
}
