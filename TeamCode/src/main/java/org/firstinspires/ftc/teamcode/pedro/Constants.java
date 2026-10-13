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

    // Output of the Pinpoint tuner, run 2026-10-13. Offsets are 91.2 mm / -143.8 mm.
    // TODO(offsets): code/helpers/BaseTeleOp.java, components/GoBildaPinpointOdometry.java and
    //   examples/TeleOpFieldCentric.java / TeleOpRobotCentric.java still use xOffset = 72 mm /
    //   yOffset = -156 mm. Make them agree with these before relying on auto->teleop pose handoff.
    public static PinpointConfig localizerConfig = new PinpointConfig(
            c -> {
                c.name.set("pinpoint");
                c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
                c.offsetUnits.set(DistanceUnit.INCH);
                c.xPodOffset.set(3.5896529550627463);
                c.yPodOffset.set(-5.661084032434179);
                c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.globalDistanceUnit.set(DistanceUnit.INCH);
            }
    );

    // Output of the Foresight tuner, run 2026-10-13 (after the Pinpoint offsets above).
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.23157465085344572);
                Controller secondaryTranslationalForward = Controller.proportional(0.08556064570171526);
                Controller primaryTranslationalLateral = Controller.proportional(0.34696641765619685);
                Controller secondaryTranslationalLateral = Controller.proportional(0.12819482021053644);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.012476882836785599));
                c.brake.set(Controller.proportionalFeedforward(0.010605350411267759));

                c.headingFeedback.set(Controller.proportional(4.329002029749585));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05793483976281667, 0.0045561909995304765));

                c.linearBrakeCoefficients.set(Matrix.diag(0.05763875035002074, 0.05705158631044427));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0016656482086844936, 0.0014279422522020253));

                c.maxAchievableForwardVelocity.set(80.96765866393392);
                c.maxAchievableStrafeVelocity.set(57.465887537027626);
                c.naturalForwardDeceleration.set(41.341768161796175);
                c.naturalStrafeDeceleration.set(60.88713930668517);
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
