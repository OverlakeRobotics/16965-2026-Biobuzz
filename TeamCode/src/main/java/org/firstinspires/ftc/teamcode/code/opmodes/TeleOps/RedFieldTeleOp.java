package org.firstinspires.ftc.teamcode.code.opmodes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.code.helpers.BaseTeleOp;

@TeleOp(name = "Red TeleOp (Field-Centric)", group = "TeleOp")
public class RedFieldTeleOp extends BaseTeleOp {
    @Override
    protected boolean isRedAlliance() {
        return true;
    }

    @Override
    protected boolean isFieldCentric() {
        return true;
    }
}
