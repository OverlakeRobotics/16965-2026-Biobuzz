package org.firstinspires.ftc.teamcode.code.opmodes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.code.helpers.BaseAuto;

// BLUE: shoots the RIGHT CELL, picks up the GARDEN, then shoots the LEFT CELL and parks
@Autonomous(name = "Blue Auto Both Cells", group = "Autonomous")
public class BlueAutoBothCells extends BaseAuto {
    @Override
    public void init() {
        super.readJson = true;
        super.jsonFilename = "pathJsons/biobuzz/BothCellsBlue.json";
        super.init();
    }
}
