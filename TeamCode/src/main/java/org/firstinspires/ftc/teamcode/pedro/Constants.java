package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pedro Pathing v3 hardware wiring for this robot.
 *
 * Drivetrain + localizer below are real, ported 1:1 from the v2 repo's
 * configs/pedroPathing/Constants.java (same physical robot, same Pinpoint
 * mount) — not placeholders. MecanumTuner/PinpointTuner on Saturday will
 * refine maxPower/velocity numbers but the wiring (motor names, directions,
 * pod offsets, pod type, encoder directions) is correct today. Compiled and
 * verified against the real core-3.0.0/revhub-3.0.0 jars.
 *
 * Foresight values below come from ForesightTuner run on the physical robot
 * (2026-09-19 session). Re-run the tuners if the robot's mass, wheels, or
 * battery/drive setup change materially.
 */
public class Constants {

    public static MecanumConfig mecanumConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("fl");
        c.frontRightName.set("fr");
        c.backLeftName.set("bl");
        c.backRightName.set("br");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig pinpointConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.xPodOffset.set(6.5);
        c.yPodOffset.set(0.0);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.41454502905718466);
                Controller secondaryTranslationalForward = Controller.proportional(0.1531633113894481);
                Controller primaryTranslationalLateral = Controller.proportional(0.7504284223416694);
                Controller secondaryTranslationalLateral = Controller.proportional(0.2772632502385023);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.013351306948318575));
                c.brake.set(Controller.proportionalFeedforward(0.011348610906070788));

                c.headingFeedback.set(Controller.proportional(6.809663508971605));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05526960703822035, 0.005885458596616342));

                c.linearBrakeCoefficients.set(Matrix.diag(0.11233573105665594, 0.05705758747763002));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0012886410048569288, 0.001925256664566396));

                c.maxAchievableForwardVelocity.set(62.303392162880684);
                c.maxAchievableStrafeVelocity.set(49.591811537515795);
                c.naturalForwardDeceleration.set(33.604873139279334);
                c.naturalStrafeDeceleration.set(79.21295118764837);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, pinpointConfig),
                new Mecanum(h, mecanumConfig),
                new Foresight(foresightConfig)
        );
    }
}
