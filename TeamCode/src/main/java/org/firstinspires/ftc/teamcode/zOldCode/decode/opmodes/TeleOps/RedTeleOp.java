package org.firstinspires.ftc.teamcode.zOldCode.decode.opmodes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.zOldCode.decode.helpers.BaseTeleOp;

@Config
@Disabled
@TeleOp(name = "Red TeleOp", group = "TeleOp")
public class RedTeleOp extends BaseTeleOp {
    @Override
    protected boolean isRedAlliance() {
        return true;
    }
}
