package org.firstinspires.ftc.teamcode.code.opmodes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.code.helpers.BaseTeleOp;

@TeleOp(name = "Blue TeleOp (Field-Centric)", group = "TeleOp")
public class BlueFieldTeleOp extends BaseTeleOp {
    @Override
    protected boolean isRedAlliance() {
        return false;
    }

    @Override
    protected boolean isFieldCentric() {
        return true;
    }
}
