package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// Pedro Pathing robot constants. Run the tuners in Tuning.java through AutoTune
// (http://192.168.43.1:10158 while connected to the robot) and paste the generated configs here.
public class Constants {
    // Confirmed by the Mecanum tuner. Also matches BasicHolonomicDrivetrain.
    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("frontLeft");
                c.backLeftName.set("backLeft");
                c.frontRightName.set("frontRight");
                c.backRightName.set("backRight");

                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

                c.manualBrakeMode.set(true);
            }
    );

    // Pod type and directions confirmed by the Pinpoint tuner. Offsets are deliberately the same
    // -84 mm / -168 mm the existing TeleOps pass to setOffsets() rather than the tuner's estimate.
    public static PinpointConfig localizerConfig = new PinpointConfig(
            c -> {
                c.name.set("pinpoint");
                c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
                c.offsetUnits.set(DistanceUnit.MM);
                c.xPodOffset.set(-84.0);
                c.yPodOffset.set(-168.0);
                c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.globalDistanceUnit.set(DistanceUnit.INCH);
            }
    );

    // Output of the Foresight tuner, run 2026-09-18.
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.16611510630204218);
                Controller secondaryTranslationalForward = Controller.proportional(0.0613750931012159);
                Controller primaryTranslationalLateral = Controller.proportional(0.22674626686951632);
                Controller secondaryTranslationalLateral = Controller.proportional(0.08377668683644954);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01182761236141456));
                c.brake.set(Controller.proportionalFeedforward(0.010053470507202376));

                c.headingFeedback.set(Controller.proportional(3.979454495411453));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.039392084105367806, 0.007634139573452544));

                c.linearBrakeCoefficients.set(Matrix.diag(0.05852338843693274, 0.0958030527755163));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.00149977049773331, 5.644059563320923E-4));

                c.maxAchievableForwardVelocity.set(84.70241813278044);
                c.maxAchievableStrafeVelocity.set(70.63198259372948);
                c.naturalForwardDeceleration.set(30.459526302088634);
                c.naturalStrafeDeceleration.set(53.88895230691348);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
