package org.firstinspires.ftc.teamcode.code.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.code.helpers.Prism.Color;
import org.firstinspires.ftc.teamcode.code.helpers.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.code.helpers.Prism.PrismAnimations;

@Config
@Autonomous(name = "Prepare LED Indicator Artboards", group = "Autonomous")
public class PrepareLEDIndicatorArtboards extends LinearOpMode {
    public static int ANIMATION_BRIGHTNESS = 100;
    private final PrismAnimations.Solid empty = new PrismAnimations.Solid(Color.MAGENTA);
    private final PrismAnimations.Solid passing = new PrismAnimations.Solid(Color.BLUE);
    private final PrismAnimations.Solid full = new PrismAnimations.Solid(Color.GREEN);
    private final PrismAnimations.Blink jammed = new PrismAnimations.Blink(Color.RED, Color.TRANSPARENT, 500, 250);
    private final PrismAnimations.RainbowSnakes ambient = new PrismAnimations.RainbowSnakes();
    private final PrismAnimations.Solid off = new PrismAnimations.Solid(Color.TRANSPARENT);
    private GoBildaPrismDriver prism;

    @Override
    public void runOpMode() {
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");

        telemetry.addLine("Ready to program Prism Artboards.");
        telemetry.update();
        waitForStart();

        program(GoBildaPrismDriver.Artboard.ARTBOARD_0, empty, "EMPTY");
        program(GoBildaPrismDriver.Artboard.ARTBOARD_1, passing, "PASSING");
        program(GoBildaPrismDriver.Artboard.ARTBOARD_2, full, "FULL");
        program(GoBildaPrismDriver.Artboard.ARTBOARD_3, jammed, "JAMMED");
        program(GoBildaPrismDriver.Artboard.ARTBOARD_4, ambient, "AMBIENT");
        program(GoBildaPrismDriver.Artboard.ARTBOARD_5, off, "OFF");

        prism.clearAllAnimations();
        telemetry.addLine("Done programming Prism Artboards.");
        telemetry.update();
        sleep(1000);
    }

    private void program(GoBildaPrismDriver.Artboard slot,
                         PrismAnimations.AnimationBase anim,
                         String name) {
        if (!opModeIsActive()) return;

        telemetry.addLine("Programming " + name + " animation to " + slot);
        telemetry.update();

        anim.setBrightness(ANIMATION_BRIGHTNESS);

        prism.clearAllAnimations();
        sleep(50);

        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, anim);
        sleep(50);

        prism.saveCurrentAnimationsToArtboard(slot);
        sleep(400);
    }
}