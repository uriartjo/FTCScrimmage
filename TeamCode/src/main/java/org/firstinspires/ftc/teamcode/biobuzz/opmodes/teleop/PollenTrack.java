package org.firstinspires.ftc.teamcode.biobuzz.opmodes.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.biobuzz.configs.PollenTrackConstants;
import org.firstinspires.ftc.teamcode.decode.configs.RobotConstants;
import org.firstinspires.ftc.teamcode.decode.util.PIDController;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;
import java.util.List;

/**
 * Pedro Pathing v3 translation of the original PollenTrack.java. Drive-assist
 * TeleOp: while gamepad1.right_trigger is held, the robot steers and drives
 * toward the closest detected "pollen" (yellow_pollen, class 0 of the
 * Hive-Vision Limelight 3A model — github.com/sidhuharjas/Hive-Vision)
 * instead of taking stick input. Release the trigger and manual driving
 * resumes immediately.
 *
 * Moved from teamcode.decode.opmodes.teleop to teamcode.biobuzz.opmodes.teleop
 * — this season is "BioBuzz," not "Decode." RobotConstants and PIDController
 * are unaffected imports; they still live under teamcode.decode.* along with
 * everything else in this migration that isn't Pollen-specific.
 *
 * Design converged with ChatGPT in a paired-programmer review — see
 * ai_pair_programming/transcripts/discussion_20260916_184053.md in the old
 * repo for the full discussion (closest-detection selection, sticky target
 * lock, steering via PIDController, arrival criteria, no auto-intake). None
 * of that design changed here — only the Pedro-specific surface did.
 *
 * FOUND WHILE TRANSLATING — a real v2-era bug, unrelated to the version
 * migration itself: the original's tracking-drive call used
 * follower.setTeleOpDrive(forwardPower, 0.0, turnPower, false) — the same
 * isRobotCentric=false (field-centric) flag as the manual stick-driving
 * branch. That's wrong for this branch specifically: forwardPower/turnPower
 * here come from the Limelight's tx (a robot-mounted camera's own offset
 * angle) and a steering PID driving tx toward 0 — that's an inherently
 * robot-relative command ("turn until the ball, which the camera sees
 * relative to the robot's own facing, is centered"), not a field-relative
 * one. Field-centric mode would have rotated that command by the robot's
 * current field heading before applying it — meaning the robot would drive
 * toward field-absolute "forward" instead of toward wherever the locked
 * ball actually was, the moment the robot's heading differed from 0.
 * Verified against v2.1.2 source's own javadoc for setTeleOpDrive's isRobotCentric
 * parameter (see the Pedro v3 migration doc for the full explanation) — this
 * wasn't a guess.
 * FIX: the tracking branch now calls follower.manual(forwardPower, 0.0, turnPower)
 * directly, with no ManualDrive.fieldCentric() wrapping — v3's manual() is
 * robot-centric by construction (see ShooterTeleopBase.java's javadoc), which
 * is actually exactly the semantics this branch always needed. The manual
 * stick-driving branch still gets the fieldCentric() wrapper, since raw
 * stick input in this codebase has always meant field-relative driving (see
 * ShooterTeleopBase.java's Landmine 2 translation) and that part was correct
 * in the original.
 *
 * Other translations, same as elsewhere in this migration:
 *   - Constants.createFollower -> Constants.create (stub); follower.setStartingPose -> follower.setPose
 *   - follower.startTeleopDrive() removed — no v3 equivalent, manual() needs no separate start call
 *   - follower.update() moved to the end of loop(), after the drive command is set — v3 requires
 *     manual()/follow() to run before update() for that tick's command to actually reach the wheels
 *     (see ShooterTeleopBase.java's javadoc for the verified reasoning)
 *
 * NOT yet run on hardware.
 */
@TeleOp(name = "PollenTrack")
public class PollenTrack extends OpMode {

    // ═══════════════════════════════════════════════════════════════════════════
    // PEDRO
    // ═══════════════════════════════════════════════════════════════════════════

    private Follower follower;

    // ═══════════════════════════════════════════════════════════════════════════
    // HARDWARE
    // ═══════════════════════════════════════════════════════════════════════════

    private Limelight3A limelight;

    // ═══════════════════════════════════════════════════════════════════════════
    // STEERING
    // ═══════════════════════════════════════════════════════════════════════════

    private PIDController steerPid;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double lastLoopTimeSec = 0.0;

    // ═══════════════════════════════════════════════════════════════════════════
    // TARGET LOCK STATE
    // ═══════════════════════════════════════════════════════════════════════════

    private boolean hasLock = false;
    private double lockedTx = 0.0;
    private double lockedTy = 0.0;
    private double lockedArea = 0.0;
    private double lastSeenTimeSec = 0.0;

    // Pending-switch bookkeeping: a challenger has to keep winning, in
    // roughly the same spot, for LOCK_SWITCH_FRAMES straight before it
    // actually takes over the lock.
    private boolean hasSwitchCandidate = false;
    private double switchCandidateTx = 0.0;
    private double switchCandidateTy = 0.0;
    private int switchStreak = 0;

    // Last commanded drive, reused verbatim to coast through a single
    // dropped frame during LOST_TARGET_GRACE_SEC.
    private double lastForwardPower = 0.0;
    private double lastTurnPower = 0.0;

    private boolean wasArrived = false;

    // ═══════════════════════════════════════════════════════════════════════════
    // INIT
    // ═══════════════════════════════════════════════════════════════════════════

    @Override
    public void init() {
        follower = Constants.create(hardwareMap); // NOTE: stub, returns null until AutoTune is run
        follower.setPose(RobotConstants.startPose);
        follower.update();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(PollenTrackConstants.LIMELIGHT_POLLEN_PIPELINE);
        limelight.start(); // must call start() — getLatestResult() returns null otherwise

        steerPid = new PIDController(
                PollenTrackConstants.STEER_KP,
                PollenTrackConstants.STEER_KI,
                PollenTrackConstants.STEER_KD,
                PollenTrackConstants.STEER_I_MAX,
                PollenTrackConstants.STEER_OUT_MIN,
                PollenTrackConstants.STEER_OUT_MAX);

        loopTimer.reset();
        lastLoopTimeSec = 0.0;

        telemetry.addLine("PollenTrack initialized. Hold right trigger to track.");
        telemetry.update();
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // LOOP
    // ═══════════════════════════════════════════════════════════════════════════

    @Override
    public void loop() {
        double nowSec = loopTimer.seconds();
        double dtSec = nowSec - lastLoopTimeSec;
        lastLoopTimeSec = nowSec;

        boolean trackingEngaged = gamepad1.right_trigger > 0.5;

        double forwardPower;
        double turnPower;
        boolean arrived = false;

        if (!trackingEngaged) {
            // Manual driving — normal stick control, tracking state cleared
            // so re-engaging always starts from a fresh lock. Field-centric,
            // like every other teleop in this codebase (see ShooterTeleopBase.java).
            resetLockState();
            forwardPower = gamepad1.left_stick_y;
            turnPower = -gamepad1.right_stick_x;
            follower.manual(ManualDrive.fieldCentric(forwardPower, gamepad1.left_stick_x, turnPower, follower.pose().heading()));
        } else {
            List<LLResultTypes.DetectorResult> candidates = getPollenCandidates();
            updateLock(candidates, nowSec);

            if (!hasLock) {
                forwardPower = 0.0;
                turnPower = 0.0;
            } else if (nowSec - lastSeenTimeSec > PollenTrackConstants.LOST_TARGET_GRACE_SEC) {
                // Lock aged out this loop (checked here too, in case updateLock
                // just dropped it) — stop rather than keep coasting forever.
                hasLock = false;
                forwardPower = 0.0;
                turnPower = 0.0;
            } else {
                // Push current dashboard gains into the persistent controller
                // every loop — updates the tuning live without losing its
                // integral/derivative history (see PIDController.setGains).
                steerPid.setGains(PollenTrackConstants.STEER_KP, PollenTrackConstants.STEER_KI, PollenTrackConstants.STEER_KD);
                turnPower = steerPid.update(lockedTx, Math.max(dtSec, 1e-4));

                double turnAlignmentFactor = rampToZero(Math.abs(lockedTx),
                        PollenTrackConstants.ALIGN_TX_DEG_FOR_FULL_FORWARD,
                        PollenTrackConstants.ALIGN_TX_DEG_FOR_ZERO_FORWARD);
                double approachSlowdownStart =
                        PollenTrackConstants.ARRIVED_AREA * PollenTrackConstants.APPROACH_SLOWDOWN_START_AREA_FRACTION;
                double approachFactor = rampToZero(lockedArea, approachSlowdownStart, PollenTrackConstants.ARRIVED_AREA);

                forwardPower = PollenTrackConstants.FORWARD_MAX_POWER * turnAlignmentFactor * approachFactor;

                arrived = lockedArea >= PollenTrackConstants.ARRIVED_AREA
                        && Math.abs(lockedTx) <= PollenTrackConstants.ARRIVED_TX_TOLERANCE_DEG;
                if (arrived) {
                    forwardPower = 0.0;
                    turnPower = 0.0;
                }
            }

            lastForwardPower = forwardPower;
            lastTurnPower = turnPower;
            // No fieldCentric() wrapping here — see class javadoc's bug writeup.
            // These are robot-relative by construction (tx comes off the
            // robot-mounted camera), and v3's manual() is robot-centric by
            // default, so this is now correct rather than accidentally so.
            follower.manual(forwardPower, 0.0, turnPower);
        }

        follower.update();

        if (arrived && !wasArrived) {
            gamepad1.rumble(200);
        }
        wasArrived = arrived;

        telemetry.addData("Tracking", trackingEngaged);
        telemetry.addData("HasLock", hasLock);
        if (hasLock) {
            telemetry.addData("Locked tx/ty/area", "%.1f / %.1f / %.2f", lockedTx, lockedTy, lockedArea);
        }
        telemetry.addData("Arrived", arrived);
        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // DETECTION FILTERING
    // ═══════════════════════════════════════════════════════════════════════════

    private List<LLResultTypes.DetectorResult> getPollenCandidates() {
        List<LLResultTypes.DetectorResult> candidates = new ArrayList<>();
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return candidates;
        }
        for (LLResultTypes.DetectorResult dr : result.getDetectorResults()) {
            if (dr.getClassId() == PollenTrackConstants.POLLEN_CLASS_ID
                    && dr.getConfidence() >= PollenTrackConstants.CONFIDENCE_FLOOR) {
                candidates.add(dr);
            }
        }
        return candidates;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // TARGET LOCK
    // ═══════════════════════════════════════════════════════════════════════════

    private void resetLockState() {
        hasLock = false;
        hasSwitchCandidate = false;
        switchStreak = 0;
    }

    private void updateLock(List<LLResultTypes.DetectorResult> candidates, double nowSec) {
        if (candidates.isEmpty()) {
            if (hasLock && nowSec - lastSeenTimeSec > PollenTrackConstants.LOST_TARGET_GRACE_SEC) {
                resetLockState();
            }
            return;
        }

        // Refresh the current lock's position if the same ball is still visible.
        if (hasLock) {
            LLResultTypes.DetectorResult matched = nearestTo(candidates, lockedTx, lockedTy);
            if (matched != null) {
                lockedTx = matched.getTargetXDegrees();
                lockedTy = matched.getTargetYDegrees();
                lockedArea = matched.getTargetArea();
                lastSeenTimeSec = nowSec;
            } else if (nowSec - lastSeenTimeSec > PollenTrackConstants.LOST_TARGET_GRACE_SEC) {
                resetLockState();
            }
        }

        LLResultTypes.DetectorResult best = pickBest(candidates);

        if (!hasLock) {
            lockedTx = best.getTargetXDegrees();
            lockedTy = best.getTargetYDegrees();
            lockedArea = best.getTargetArea();
            hasLock = true;
            hasSwitchCandidate = false;
            switchStreak = 0;
            lastSeenTimeSec = nowSec;
            return;
        }

        if (withinAngleTolerance(best.getTargetXDegrees(), best.getTargetYDegrees(), lockedTx, lockedTy)) {
            // Best pick this frame IS the current lock — nothing pending.
            hasSwitchCandidate = false;
            switchStreak = 0;
            return;
        }

        // A different candidate currently ranks best — only switch once it
        // keeps winning, in roughly the same spot, for several frames straight.
        if (!hasSwitchCandidate
                || !withinAngleTolerance(best.getTargetXDegrees(), best.getTargetYDegrees(),
                        switchCandidateTx, switchCandidateTy)) {
            hasSwitchCandidate = true;
            switchCandidateTx = best.getTargetXDegrees();
            switchCandidateTy = best.getTargetYDegrees();
            switchStreak = 1;
        } else {
            switchStreak++;
        }

        if (switchStreak >= PollenTrackConstants.LOCK_SWITCH_FRAMES) {
            lockedTx = best.getTargetXDegrees();
            lockedTy = best.getTargetYDegrees();
            lockedArea = best.getTargetArea();
            lastSeenTimeSec = nowSec;
            hasSwitchCandidate = false;
            switchStreak = 0;
        }
    }

    /** Max targetArea, tie-broken by smallest |tx| among candidates within AREA_TIE_BAND of the max. */
    private LLResultTypes.DetectorResult pickBest(List<LLResultTypes.DetectorResult> candidates) {
        double maxArea = 0.0;
        for (LLResultTypes.DetectorResult dr : candidates) {
            maxArea = Math.max(maxArea, dr.getTargetArea());
        }
        double tieFloor = maxArea * (1.0 - PollenTrackConstants.AREA_TIE_BAND);

        LLResultTypes.DetectorResult best = null;
        for (LLResultTypes.DetectorResult dr : candidates) {
            if (dr.getTargetArea() < tieFloor) continue;
            if (best == null || Math.abs(dr.getTargetXDegrees()) < Math.abs(best.getTargetXDegrees())) {
                best = dr;
            }
        }
        return best;
    }

    private LLResultTypes.DetectorResult nearestTo(List<LLResultTypes.DetectorResult> candidates, double tx, double ty) {
        LLResultTypes.DetectorResult nearest = null;
        double nearestDist = Double.MAX_VALUE;
        for (LLResultTypes.DetectorResult dr : candidates) {
            double dist = angleDistance(dr.getTargetXDegrees(), dr.getTargetYDegrees(), tx, ty);
            if (dist < nearestDist) {
                nearestDist = dist;
                nearest = dr;
            }
        }
        if (nearest != null && nearestDist <= PollenTrackConstants.MATCH_ANGLE_TOLERANCE_DEG) {
            return nearest;
        }
        return null;
    }

    private boolean withinAngleTolerance(double tx1, double ty1, double tx2, double ty2) {
        return angleDistance(tx1, ty1, tx2, ty2) <= PollenTrackConstants.MATCH_ANGLE_TOLERANCE_DEG;
    }

    private static double angleDistance(double tx1, double ty1, double tx2, double ty2) {
        double dtx = tx1 - tx2;
        double dty = ty1 - ty2;
        return Math.sqrt(dtx * dtx + dty * dty);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // HELPERS
    // ═══════════════════════════════════════════════════════════════════════════

    /** 1.0 at/below fullAt, 0.0 at/above zeroAt, linear ramp between. Assumes fullAt < zeroAt. */
    private static double rampToZero(double x, double fullAt, double zeroAt) {
        if (x <= fullAt) return 1.0;
        if (x >= zeroAt) return 0.0;
        return 1.0 - (x - fullAt) / (zeroAt - fullAt);
    }
}
