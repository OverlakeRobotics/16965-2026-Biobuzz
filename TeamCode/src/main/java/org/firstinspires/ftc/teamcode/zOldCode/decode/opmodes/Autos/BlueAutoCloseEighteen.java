package org.firstinspires.ftc.teamcode.zOldCode.decode.opmodes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.zOldCode.decode.helpers.BaseAuto;

@Config
@Disabled
@Autonomous(name = "Blue Auto Close Eighteen", group = "Autonomous")
public class BlueAutoCloseEighteen extends BaseAuto {
    @Override
    public void init() {
        super.readJson = true;
        super.jsonFilename = "pathJsons/decode/EighteenArtifactCloseBlue.json";
        super.init();
    }
}
