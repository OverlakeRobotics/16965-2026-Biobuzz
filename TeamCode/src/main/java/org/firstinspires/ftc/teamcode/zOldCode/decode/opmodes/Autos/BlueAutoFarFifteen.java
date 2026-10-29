package org.firstinspires.ftc.teamcode.zOldCode.decode.opmodes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.zOldCode.decode.helpers.BaseAuto;

@Config
@Disabled
@Autonomous(name = "Blue Auto Far Fifteen", group = "Autonomous")
public class BlueAutoFarFifteen extends BaseAuto {
    @Override
    public void init() {
        super.readJson = true;
        super.jsonFilename = "pathJsons/decode/FifteenArtifactFarBlue.json";
        super.shooterDelay = 0.4;
        super.init();
    }
}
