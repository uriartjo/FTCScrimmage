package org.firstinspires.ftc.teamcode.decode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.configs.RobotConstants;

/**
 * Pedro Pathing v3 translation — no Pedro/Pose API touchpoints of its own
 * (just passes RobotConstants Poses through to the base class), so this is
 * an unchanged port aside from the package/import updates.
 */
@TeleOp(name = "RED TELEOP (v2, shared base)")
public class RedTeleopV2 extends ShooterTeleopBase {
    public RedTeleopV2() {
        super(RobotConstants.redAutoEnd, RobotConstants.redGoal, RobotConstants.redReset, true);
    }
}
