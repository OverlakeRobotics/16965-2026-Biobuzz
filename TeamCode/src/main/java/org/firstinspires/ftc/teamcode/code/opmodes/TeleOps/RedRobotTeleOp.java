package org.firstinspires.ftc.teamcode.code.opmodes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.code.helpers.BaseTeleOp;

@TeleOp(name = "Red TeleOp (Robot-Centric)", group = "TeleOp")
public class RedRobotTeleOp extends BaseTeleOp {
    @Override
    protected boolean isRedAlliance() {
        return true;
    }

    @Override
    protected boolean isFieldCentric() {
        return false;
    }
}
