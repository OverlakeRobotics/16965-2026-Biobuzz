package org.firstinspires.ftc.teamcode.code.opmodes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.code.helpers.BaseAuto;

// RED: shoots the RIGHT CELL, picks up the GARDEN, shoots the RIGHT CELL again and parks
@Autonomous(name = "Red Auto One Cell", group = "Autonomous")
public class RedAutoOneCell extends BaseAuto {
    @Override
    public void init() {
        super.readJson = true;
        super.jsonFilename = "pathJsons/biobuzz/OneCellRed.json";
        super.init();
    }
}
