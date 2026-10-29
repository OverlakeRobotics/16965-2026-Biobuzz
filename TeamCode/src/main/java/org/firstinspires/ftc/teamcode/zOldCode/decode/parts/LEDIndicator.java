package org.firstinspires.ftc.teamcode.zOldCode.decode.parts;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.zOldCode.decode.helpers.Prism.Color;
import org.firstinspires.ftc.teamcode.zOldCode.decode.helpers.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.zOldCode.decode.helpers.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.zOldCode.decode.parts.Intake.IntakeState;

@Config
public class LEDIndicator {
    private final GoBildaPrismDriver prism;
    private IntakeState currentState;

    public LEDIndicator(GoBildaPrismDriver prism) {
        this.prism = prism;
        this.currentState = null;
    }

    // The PrepareLEDIndicatorArtboards OpMode must be run before using the LEDs to program the artboards!
    public void setState(IntakeState state) {
        if (state == currentState) {
            return;
        }
        currentState = state;
        switch (state) {
            case EMPTY:
                prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
                break;
            case PASSING:
                prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_1);
                break;
            case FULL:
                prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_2);
                break;
            case JAMMED:
                prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_3);
                break;
            case AMBIENT:
                prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_4);
                break;
            case OFF:
                prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_5);
                break;
        }
    }
}
