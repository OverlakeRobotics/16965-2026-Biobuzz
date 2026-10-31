package org.firstinspires.ftc.teamcode.code.opmodes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.code.helpers.BaseAuto;

// BLUE: starts on the far wall, waits 17 s (stays out of the partner's way), shoots the LEFT CELL, parks
@Autonomous(name = "Blue Auto Delayed Shot", group = "Autonomous")
public class BlueAutoDelayedShot extends BaseAuto {
    @Override
    public void init() {
        super.readJson = true;
        super.jsonFilename = "pathJsons/biobuzz/DelayedShotBlue.json";
        super.init();
    }
}
