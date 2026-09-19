package org.firstinspires.ftc.teamcode.code.opmodes.TeleOps;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.code.helpers.BaseTeleOp;

@Config
@TeleOp(name = "Red TeleOp", group = "TeleOp")
public class RedTeleOp extends BaseTeleOp {
    @Override
    protected boolean isRedAlliance() {
        return true;
    }
}
