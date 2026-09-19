package org.firstinspires.ftc.teamcode.decode.opmodes.teleop;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.configs.RobotConstants;

/**
 * Pedro Pathing v3 translation — no .mirror() call in the original (this
 * variant's start pose is used un-mirrored), so just the Pose import update.
 */
@TeleOp(name = "RED FAR TELEOP (v2, shared base)")
public class RedFarzoneTeleopV2 extends ShooterTeleopBase {
    public RedFarzoneTeleopV2() {
        super(new Pose(38.99, 11.8, Math.toRadians(180)),
                RobotConstants.redGoal, RobotConstants.redReset, true);
    }
}
