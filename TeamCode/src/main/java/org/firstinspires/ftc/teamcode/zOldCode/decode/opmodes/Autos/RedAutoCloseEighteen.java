package org.firstinspires.ftc.teamcode.zOldCode.decode.opmodes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.zOldCode.decode.helpers.BaseAuto;

@Config
@Disabled
@Autonomous(name = "Red Auto Close Eighteen", group = "Autonomous")
public class RedAutoCloseEighteen extends BaseAuto {
    @Override
    public void init() {
        super.readJson = true;
        super.jsonFilename = "pathJsons/decode/EighteenArtifactCloseRed.json";
        super.init();
    }
}
