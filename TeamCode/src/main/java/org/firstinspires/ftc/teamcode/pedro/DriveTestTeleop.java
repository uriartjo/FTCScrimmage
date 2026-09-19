package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Drivebase-only teleop for checking the Pedro v3 Follower (Mecanum + Pinpoint + Foresight)
 * with no shooter/turret/intake hardware required in the robot config.
 * Left stick = translate, right stick X = turn (left/CCW positive).
 * A = toggle robot-centric / field-centric (starts robot-centric).
 * B = zero the pose (field-centric forward becomes the robot's current facing).
 */
@TeleOp(name = "Drive Test (Pedro v3)", group = "Test")
public class DriveTestTeleop extends OpMode {
    private Follower follower;
    private boolean fieldCentric = false;
    private boolean lastA = false, lastB = false;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(0, 0, 0));
        follower.update();
        telemetry.addLine("Ready — robot-centric. A: toggle centric, B: zero pose");
        telemetry.update();
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        if (gamepad1.a && !lastA) fieldCentric = !fieldCentric;
        if (gamepad1.b && !lastB) follower.setPose(new Pose(0, 0, 0));
        lastA = gamepad1.a;
        lastB = gamepad1.b;

        if (fieldCentric) {
            follower.manual(ManualDrive.fieldCentric(forward, strafe, turn, follower.pose().heading()));
        } else {
            follower.manual(forward, strafe, turn);
        }
        follower.update();

        Pose p = follower.pose();
        telemetry.addData("Mode", fieldCentric ? "FIELD-centric" : "ROBOT-centric");
        telemetry.addData("x / y (in)", "%.1f / %.1f", p.x(), p.y());
        telemetry.addData("heading (deg)", "%.1f", Math.toDegrees(p.heading()));
        telemetry.update();
    }
}
