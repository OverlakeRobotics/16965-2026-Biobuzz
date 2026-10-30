package org.firstinspires.ftc.teamcode.examples;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "Blue Dual Turret TeleOp", group = "TeleOp")
public class BlueDualTurretTeleOp extends DualTurretTeleOp {
    @Override
    protected boolean isRedAlliance() {
        return false;
    }
}
