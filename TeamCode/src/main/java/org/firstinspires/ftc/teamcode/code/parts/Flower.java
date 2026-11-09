package org.firstinspires.ftc.teamcode.code.parts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

// Flower mechanism (picks up from the flower): two servos that always move together. Starts open (up).
@Config
public class Flower {
    public static double openPos = 0.47;
    public static double closedPos = 0;

    private final Servo left;
    private final Servo right;
    private boolean open;

    public Flower(Servo left, Servo right) {
        this.left = left;
        this.right = right;
        open();
    }

    public void open() {
        open = true;
        setPosition(openPos);
    }

    public void close() {
        open = false;
        setPosition(closedPos);
    }

    public void toggle() {
        if (open) close();
        else open();
    }

    public boolean isOpen() {
        return open;
    }

    private void setPosition(double position) {
        left.setPosition(position);
        right.setPosition(position);
    }
}
