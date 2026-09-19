package org.firstinspires.ftc.teamcode.decode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.configs.RobotConstants;

/**
 * Same as BlueTeleopV2 in every way except startPose: this variant assumes
 * the robot is placed by hand at the blue alliance's corner on the
 * audience-wall side before the match starts, instead of
 * RobotConstants.startPose (BlueTeleopV2's usual pose, meant for handoff
 * from an autonomous run that already updated the follower's pose).
 *
 * NOT READY TO RUN: RobotConstants.blueAudienceCornerPose_UNVERIFIED is a
 * (-1,-1) sentinel, not a measured pose. This codebase's existing data
 * contradicts itself on which physical corner is "blue" vs "red" —
 * blueGoal=(5,140) implies low-x=blue, but redReset=(9,9) implies low-x=red
 * — and that couldn't be safely resolved from source alone (see the Pedro
 * v3 migration doc's note on this). Verify the real pose against the
 * physical field (e.g. read it back from telemetry after placing the robot
 * at the corner and running PinpointTuner/odometry) before using this OpMode.
 */
@TeleOp(name = "BLUE TELEOP (audience corner start) — UNVERIFIED POSE")
public class BlueAudienceCornerTeleop extends ShooterTeleopBase {
    public BlueAudienceCornerTeleop() {
        super(RobotConstants.blueAudienceCornerPose_UNVERIFIED, RobotConstants.blueGoal, RobotConstants.blueReset, false);
    }
}
