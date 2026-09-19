package org.firstinspires.ftc.teamcode.code.parts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;

@Config
public class Potentiometer {
    public static double DEGREES_PER_VOLT = 3600 / 3.33;
    public static double POTENTIOMETER_VOLTAGE_AT_TURRET_ZERO = 1.202;
    private final AnalogInput potentiometer;

    public Potentiometer(AnalogInput potentiometer) {
        this.potentiometer = potentiometer;
    }

    public double getAngleDegrees() {
        return (getVoltage() - POTENTIOMETER_VOLTAGE_AT_TURRET_ZERO) * DEGREES_PER_VOLT / 4;
    }

    public double getAngleFromVoltagePoly() {
        // tiny voltage offset because D-shaft prevented 1.202 at zero degrees; now 1.206 at zero degrees.
        double v = getVoltage() - 0.004;
        return (((((((-319.82121105417582 * v
                + 3201.2248841083424) * v
                - 13591.245983901385) * v
                + 31736.271898156174) * v
                - 44018.2997319621) * v
                + 36129.7874250335) * v
                - 15726.391887437048) * v
                + 2504.8182728753814);
    }

    public double getAngleFromVoltageExp() {
        double v = getVoltage();
        return -2147304.167668383 * Math.exp(0.013128463886552253 * v)
                + 6347845.455705036 * Math.exp(0.0045468538469219345 * v)
                - 4201176.526292968;
    }

    public double getVoltage() {
        return potentiometer.getVoltage();
    }
}
