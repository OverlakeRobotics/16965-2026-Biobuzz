package org.firstinspires.ftc.teamcode.code.opmodes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.code.helpers.BaseAuto;

// RED: starts on the far wall, waits 17 s (stays out of the partner's way), shoots the LEFT CELL, parks
@Autonomous(name = "Red Auto Delayed Shot", group = "Autonomous")
public class RedAutoDelayedShot extends BaseAuto {
    @Override
    public void init() {
        super.readJson = true;
        super.jsonFilename = "pathJsons/biobuzz/DelayedShotRed.json";
        super.init();
    }
}
