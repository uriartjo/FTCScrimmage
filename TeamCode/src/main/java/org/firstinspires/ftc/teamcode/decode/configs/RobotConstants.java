package org.firstinspires.ftc.teamcode.decode.configs;

import com.pedropathing.math.Pose;

/**
 * Pedro Pathing v3 translation of the original (v2.1.2) RobotConstants.java from
 * 641WinterHavenEdition-master. Only the Pedro-specific surface changed —
 * every numeric constant below is copied verbatim.
 *
 * Changes from the original:
 *   - import com.pedropathing.geometry.Pose -> com.pedropathing.math.Pose (v3 moved the package).
 *   - Pose.mirror() no longer exists in v3. Replaced with a local mirror() helper below that
 *     reproduces the EXACT v2.1.2 formula (verified against Pedro-Pathing/PedroPathing's
 *     v2.1.2 tag source), not v3's new PoseFactory.mirrorX(). This matters: v3's
 *     PoseFactory.mirrorX(axis) computes `2*axis - x`, where v2's mirror(fieldLength)
 *     computed `fieldLength - x` — v2's constant (141.5) is the FULL field length, v3's
 *     axis parameter is HALF of it (the centerline). Calling PoseFactory.mirrorX(141.5) as a
 *     drop-in replacement for the old .mirror() would silently double the mirror offset.
 *     Using our own verified-identical formula sidesteps that unit mismatch entirely.
 *   - @Configurable (com.bylazar.configurables.annotations) REMOVED. That dependency
 *     (com.bylazar:fullpanels) is not part of Pedro v3's dependency set and wasn't added to
 *     this project's build.dependencies.gradle — these are now plain static fields, not
 *     dashboard-live-tunable. Open question for the team: re-add fullpanels for the old
 *     dashboard-editing workflow, or use whatever Pedro's own new `com.pedropathing:tuning`
 *     module provides instead (unverified — its sources don't expose an @Configurable-style
 *     annotation, so it may not be a drop-in equivalent).
 */
public class RobotConstants {

    // ─── Robot Poses ────────────────────────────────────────────────────────────────
    public static Pose blueGoal = new Pose(5.0, 140.0, 0);
    public static Pose redGoal = mirror(blueGoal);

    public static Pose blueReset = new Pose(135, 9.0, 0); // robot faces red goal
    public static Pose redReset = new Pose(9.0, 9.0, 0); // robot faces opposing human player

    public static Pose blueAutoEnd = new Pose(63.9, 105.77, Math.toRadians(180));
    public static Pose redAutoEnd = mirror(new Pose(63.9, 105.77, Math.toRadians(180)));


    public static Pose startPose    = new Pose(23.36, 117.00,  Math.toRadians(180));
    public static Pose scorePose  = new Pose(52,    79.2,   Math.toRadians(180));
    public static Pose parkPose   = new Pose(63.9,  105.77, Math.toRadians(180));

    // Control points
    public static Pose control1           = new Pose(43.29, 104.73, Math.toRadians(180));
    public static Pose control2           = new Pose(54.73,  54.9);
    public static Pose control3           = new Pose(59.221, 79.143);

    public static Pose controlThirdRow    = new Pose(55.95,  31.16);

    // Row pickup endpoints
    public static Pose pickup2Pose2 = new Pose(24.9,  58, Math.toRadians(180));
    public static Pose pickup3Pose2 = new Pose(25.1,  83.3,  Math.toRadians(180));
    public static Pose pickup1Pose2 = new Pose(24.9,  34,   Math.toRadians(180));

    // Gate poses

    public static Pose gateControl       = new Pose(44,   65.23);
    public static Pose gatePoseRotated   = new Pose(10.8, 58,    Math.toRadians(158));
    public static Pose gateRotateControl = new Pose(15.8, 61.7);
    public static Pose gatePose          = new Pose(17.4,   65.9,  Math.toRadians(180));
    public static Pose goalPose = new Pose(5, 140);

    public static Pose parkPose2   = new Pose(38.99, 11.8, Math.toRadians(180));

    // ─── Hood ────────────────────────────────────────────────────────────────
    public static double hoodInitPos = 0.5;
    public static double hoodServoMax = 0.790;
    public static double hoodServoMin = 0.005;


    // ─── Flywheel / Shooter ───────────────────────────────────────────────────

    public static double flywheelSpeedTarget = 1750;

    public static double FLYWHEEL_KP = 0.045;
    public static double FLYWHEEL_KI = 0.3;
    public static double FLYWHEEL_KD = 0.001;

    public static double SHOOTER_I_MAX = 0.4;

    // ─── Turret ───────────────────────────────────────────────────────────────
    public static double ppModeAngleTolerance = 0.5; // degrees
    public static double TICKS_PER_DEGREE = 773.9 / 360.0;

    public static double TURRET_KP = 0.045;
    public static double TURRET_KI = 0.01;
    public static double TURRET_KD = 0.0015;

    public static double TURRET_I_MAX     = 0.30;
    public static double TURRET_MAX_POWER = 1.0;
    public static double TURRET_MIN_POWER = 0.0;

    // ─── Auto-tracking offsets ────────────────────────────────────────────────
    public static double flywheelOffset = 0.0;
    public static double hoodOffset     = 0.0;

    // ─── Locked / Redundancy mode ─────────────────────────────────────────────
    public static double lockedTurretDeg   = 0.0;
    public static double lockedHoodPos     = 0.37;
    public static double lockedFlywheelTPS = 1450.0; // absolute TPS target

    // ─── Fixed shooter positions (TURRET_ONLY and FULL_MANUAL modes) ─────────
    public static double fixedFlywheelTPS = 1550; // TPS
    public static double fixedHoodPos     = 0.25;

    // ─── Full-Adjustable mode step sizes ─────────────────────────────────────
    public static double adjustableHoodStep     = 0.02; // servo position per button press
    public static double adjustableFlywheelStep = 50.0; // TPS per button press

    // ─── Kicker ───────────────────────────────────────────────────────────────
    public static double kickerDownPos    = 0.35;
    public static double kickerUpPos      = 0.51;
    public static double kickerStaggerSec = 0.15;

    // ─── Shoot-on-the-move (Huskie Robotics 3061 velocity-subtraction method) ──
    // flywheelSpeed(dist)/hoodAngle(dist) in Shooter.java give raw actuator
    // commands (TPS, servo position) — SOM's vector math needs *physical*
    // launch speed (in/s) and launch angle (degrees). These four constants are
    // that translation layer. NOT YET MEASURED — placeholders below are either
    // a literature default (efficiency) or an intentionally-inert default
    // (turret offset = 0,0, which just disables the rotation correction until
    // measured, per Huskie Physics section 1.5's "less important effect").
    // See Shooter.calculateTargetsShootOnMove's javadoc for exactly how to
    // measure each one (slow-mo video, per the paper's own "Characterizing
    // the Shooter" section — a shooter-bench measurement, not a drivetrain one).

    // TODO(measure): average radius of the two flywheels' ball-contact surface, inches.
    public static double FLYWHEEL_RADIUS_IN = 1.5; // PLACEHOLDER — measure with calipers

    // TODO(measure): rotational-to-linear efficiency (slippage). 0.90 is Huskie 3061's
    // own measured value for their shooter (see the PDF, section 1.2) — a reasonable
    // starting point, not this robot's measured number.
    public static double FLYWHEEL_EFFICIENCY = 0.90;

    // TODO(measure): flywheel motor's encoder ticks per revolution (datasheet or bench
    // test) — needed to convert the flywheel's TPS (ticks/sec) into RPS (rev/sec) for
    // Equation 5's v = pi*(R1+R2)*RPS. Left at 1.0 (a clearly-wrong sentinel, not a
    // plausible-looking guess) so a forgotten TODO fails loudly instead of quietly
    // producing a wrong ball-speed estimate.
    public static double FLYWHEEL_TICKS_PER_REV = 1.0; // PLACEHOLDER — DO NOT TRUST until set

    // TODO(measure): the physical launch angle (degrees above horizontal, 0=flat,
    // 90=straight up) at the hood's two servo-range extremes (hoodServoMin/hoodServoMax
    // above). Protractor against the actual mechanism. Placeholders below assume a
    // plausible-but-unverified 20-60 degree range — replace before trusting any SOM output.
    public static double HOOD_ANGLE_AT_MIN_DEG = 20.0; // PLACEHOLDER
    public static double HOOD_ANGLE_AT_MAX_DEG = 60.0; // PLACEHOLDER

    // TODO(measure): shooter's position offset from the robot's center of rotation, in
    // robot-frame inches (x = forward, y = left). Default (0,0) makes Huskie Physics
    // section 1.5's rotation correction a clean no-op until measured — safe, just less
    // precise while the turret is spinning fast.
    public static double SHOOTER_OFFSET_X_IN = 0.0;
    public static double SHOOTER_OFFSET_Y_IN = 0.0;

    // Below this horizontal shot-vector magnitude (in/s), subtractRobotVelocity holds the
    // last turret bearing instead of recomputing azimuth from atan2 — found via a Claude/
    // ChatGPT cross-review and confirmed numerically: near a vertical shot, atan2(vShotY,
    // vShotX)'s inputs both approach zero and the recovered angle becomes noise-dominated
    // (a ~215-degree swing across 8 loop ticks under plausible velocity noise, simulated).
    // Not a measured physical constant — a soft safety margin. Too low and the turret still
    // occasionally hunts; too high and it holds slightly early. Either direction degrades
    // gracefully, unlike the other placeholders above.
    public static double SOM_AZIMUTH_HOLD_THRESHOLD_IN_PER_SEC = 10.0;


    // ─── Corner-start poses ─────────────────────────────────────────────────
    // TODO(verify against the physical field): this codebase's own data
    // contradicts itself on which corner is "blue" vs "red" — blueGoal=(5,140)
    // suggests low-x=blue, but redReset=(9,9) suggests low-x=red. Could not be
    // safely resolved from source alone (see BlueAudienceCornerTeleop.java's
    // javadoc). (-1,-1) is an intentionally out-of-bounds sentinel (valid
    // field coordinates are 0..141.5) so an unfilled value fails obviously —
    // Pedro will reject or visibly mis-track a pose outside the field, rather
    // than silently driving from a plausible-looking wrong number.
    public static Pose blueAudienceCornerPose_UNVERIFIED = new Pose(-1, -1, 0);

    // ─── Field mirroring ────────────────────────────────────────────────────
    // Verified-identical port of Pedro Pathing v2.1.2's Pose.mirror()/mirror(fieldLength):
    // https://github.com/Pedro-Pathing/PedroPathing/blob/v2.1.2/core/src/main/java/com/pedropathing/geometry/Pose.java
    private static final double DEFAULT_MIRROR_FIELD_LENGTH = 141.5;

    public static Pose mirror(Pose pose) {
        return mirror(pose, DEFAULT_MIRROR_FIELD_LENGTH);
    }

    public static Pose mirror(Pose pose, double fieldLength) {
        return new Pose(fieldLength - pose.x(), pose.y(), normalizeAngle(Math.PI - pose.heading()));
    }

    private static double normalizeAngle(double rad) {
        while (rad >  Math.PI) rad -= 2.0 * Math.PI;
        while (rad < -Math.PI) rad += 2.0 * Math.PI;
        return rad;
    }
}
