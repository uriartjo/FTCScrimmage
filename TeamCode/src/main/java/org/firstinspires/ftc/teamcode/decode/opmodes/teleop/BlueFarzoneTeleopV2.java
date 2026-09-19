package org.firstinspires.ftc.teamcode.decode.opmodes.teleop;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.configs.RobotConstants;

/**
 * Pedro Pathing v3 translation. Changes: com.pedropathing.geometry.Pose ->
 * com.pedropathing.math.Pose, and the instance .mirror() call (gone in v3 —
 * see the Pedro v3 migration doc's Landmine 1) replaced with
 * RobotConstants.mirror(Pose), the exact port of the old formula.
 */
@TeleOp(name = "BLUE FAR TELEOP (v2, shared base)")
public class BlueFarzoneTeleopV2 extends ShooterTeleopBase {
    public BlueFarzoneTeleopV2() {
        super(RobotConstants.mirror(new Pose(38.99, 11.8, Math.toRadians(180))),
                RobotConstants.blueGoal, RobotConstants.blueReset, false);
    }
}
