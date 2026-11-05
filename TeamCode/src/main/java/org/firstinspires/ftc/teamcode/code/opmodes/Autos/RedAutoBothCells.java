package org.firstinspires.ftc.teamcode.code.opmodes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.code.helpers.BaseAuto;

// RED: shoots the RIGHT CELL, picks up the GARDEN, then shoots the LEFT CELL and parks
@Autonomous(name = "Red Auto Both Cells", group = "Autonomous")
public class RedAutoBothCells extends BaseAuto {
    @Override
    public void init() {
        super.readJson = true;
        super.jsonFilename = "pathJsons/biobuzz/BothCellsRed.json";
        super.init();
    }
}
