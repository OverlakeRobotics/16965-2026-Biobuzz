package org.firstinspires.ftc.teamcode.code.parts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

// Flower mechanism servo (picks up from the flower). Starts open (up).
@Config
public class Flower {
    public static double openPos = 0.47;
    public static double closedPos = 0;

    private final Servo servo;
    private boolean open;

    public Flower(Servo servo) {
        this.servo = servo;
        open();
    }

    public void open() {
        open = true;
        servo.setPosition(openPos);
    }

    public void close() {
        open = false;
        servo.setPosition(closedPos);
    }

    public void toggle() {
        if (open) close();
        else open();
    }

    public boolean isOpen() {
        return open;
    }
}
