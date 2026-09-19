package org.firstinspires.ftc.teamcode.code.opmodes.Autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.code.helpers.BaseAuto;

@Config
@Autonomous(name = "Blue Auto Far Fifteen", group = "Autonomous")
public class BlueAutoFarFifteen extends BaseAuto {
    @Override
    public void init() {
        super.readJson = true;
        super.jsonFilename = "pathJsons/FifteenArtifactFarBlue.json";
        super.shooterDelay = 0.4;
        super.init();
    }
}
