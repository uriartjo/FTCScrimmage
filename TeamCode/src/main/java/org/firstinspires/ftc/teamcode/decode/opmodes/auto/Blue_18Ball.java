package org.firstinspires.ftc.teamcode.decode.opmodes.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.decode.configs.Shooter;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.decode.util.TurretLimiter;

/**
 * Pedro Pathing v3 translation of Blue_18Ball_V2.java (itself the bug-fixed
 * version of the original v2 Blue_18Ball — see that file's javadoc for the
 * flywheel-distance-tracking fix and dead-state cleanup, both carried over
 * unchanged here). Only the Pedro-specific surface changed.
 *
 * Changes from the v2 base:
 *   - Constants.createFollower(hardwareMap) -> Constants.create(hardwareMap) (stub — see
 *     ShooterTeleopBase.java's javadoc; returns null until AutoTune is run on the robot).
 *   - follower.setStartingPose(pose) -> follower.setPose(pose)
 *   - follower.getPose().getX()/getY()/getHeading() -> follower.pose().x()/y()/heading()
 *   - PathChain -> Path (v3 has no PathChain type; single-curve paths are just Path)
 *   - follower.pathBuilder().addPath(new BezierCurve(p0,p1,p2)).setLinearHeadingInterpolation(a,b).build()
 *     -> Paths.curve(p0,p1,p2).heading(Interpolator.linear(a,b))
 *   - new BezierLine(p0,p1)... -> Paths.line(p0,p1)...
 *   - import com.pedropathing.util.Timer -> com.pedropathing.utils.Timer (package renamed —
 *     found during a ChatGPT-reviewed pass over this migration, see the Pedro v3 migration doc)
 *   - timer.resetTimer() -> timer.reset(); timer.getElapsedTimeSeconds() -> timer.seconds()
 *   - THE FOLLOWPATH TRANSLATION: v2's follower.followPath(path, holdEnd) took holdEnd as a
 *     per-call boolean. v3's follower.follow(Path) takes no such argument — holdEnd moved to
 *     a Follower-level field, follower.holdEnd, of type ConfigVar<Boolean> (confirmed via
 *     javap: it has .get()/.set(T), a plain mutable box, not a one-time constructor value).
 *     So the per-call distinction the original state machine relies on (score/gate paths hold
 *     their end pose so the shot is aimed precisely; pickup paths don't bother) is reproduced
 *     by setting follower.holdEnd immediately before each follow() call:
 *         follower.holdEnd.set(true);  follower.follow(scoreFirst);   // was followPath(scoreFirst, true)
 *         follower.holdEnd.set(false); follower.follow(pickupFirst);  // was followPath(pickupFirst)
 *     This is a real, verified 1:1 mapping, not an approximation — every followPath call site
 *     below carries the same true/false the original had.
 *
 * NOT yet run on hardware — same status as every other file in this migration until
 * Constants.create() returns a real Follower (needs Saturday's AutoTune session).
 */
@Autonomous(name = "Blue 18 ball")
public class Blue_18Ball extends OpMode {

    // ── States ───────────────────────────────────────────────────────────────
    private static final int STATE_INIT = 0;
    private static final int STATE_MOVE_TO_SCORE_FIRST = 1;
    private static final int STATE_SHOOT_FIRST = 2;

    private static final int STATE_MOVE_TO_PICKUP2 = 3;
    private static final int STATE_MOVE_TO_SCORE2 = 5;
    private static final int STATE_SHOOT_SECOND = 6;

    private static final int STATE_MOVE_TO_GATE = 7;
    private static final int STATE_MOVE_TO_SCORE_GATE = 9;
    private static final int STATE_SHOOT_GATE = 10;

    private static final int STATE_MOVE_TO_GATE2 = 8;
    private static final int STATE_MOVE_TO_SCORE_GATE2 = 17;
    private static final int STATE_SHOOT_GATE2 = 18;
    private static final int STATE_MOVE_TO_GATE3 = 19;
    private static final int STATE_MOVE_TO_SCORE_GATE3 = 20;
    private static final int STATE_SHOOT_GATE3 = 21;

    private static final int STATE_MOVE_TO_PICKUP1 = 11;
    private static final int STATE_MOVE_TO_SCORE1 = 13;
    private static final int STATE_SHOOT_FIRST_PICKUP = 14;

    private static final int STATE_DONE = 16;

    // ── Hardware ─────────────────────────────────────────────────────────────
    private Shooter shooter;
    private DcMotor toggleMotor;
    private Servo servo;

    private Follower follower;

    // ── Turret safety ────────────────────────────────────────────────────────
    private final TurretLimiter turretLimiter =
            new TurretLimiter(Integer.MIN_VALUE, Integer.MAX_VALUE, 0.6, 15, 0.75);

    // ── Paths ────────────────────────────────────────────────────────────────
    private Path gateRotate;
    private Path pickupGate, scoreGate, pickupSecond;
    private Path score2, score3, scoreFirst, pickupFirst;

    // ── State machine ────────────────────────────────────────────────────────
    private int pathState;
    private boolean pathStarted = false;

    // ── Servo positions ──────────────────────────────────────────────────────
    private static final double SERVO_INTAKE = 0.35;
    private static final double SERVO_SHOOT = 0.51;

    // ── Gate pickup helpers ──────────────────────────────────────────────────
    private boolean atGate = false;
    private Timer shakeTimer;

    // ── Timers ───────────────────────────────────────────────────────────────
    private Timer pathTimer, opmodeTimer;

    // ── Poses (kept identical to the v2 file's literals) ──────────────────────
    private final Pose startPose = new Pose(24.86, 121.8, Math.toRadians(180));
    private final Pose control1 = new Pose(43.29, 104.73, Math.toRadians(180));
    private final Pose scorePose = new Pose(52, 79.2, Math.toRadians(180));

    private final Pose control3 = new Pose(59.221, 79.143);
    private final Pose pickup3Pose2 = new Pose(25.2, 83, Math.toRadians(180));

    private final Pose control2 = new Pose(58, 57.6);
    private final Pose pickup2Pose2 = new Pose(24.7, 58, Math.toRadians(180));

    private final Pose gateControl = new Pose(44, 65.23);
    private final Pose gatePoseRotated = new Pose(12.5, 58, Math.toRadians(165));
    private final Pose gateRotateControl = new Pose(15.8, 61.7);
    private final Pose gatePose = new Pose(17.2, 65.9, Math.toRadians(180));

    private final Pose parkPose = new Pose(63.9, 105.77, Math.toRadians(180));

    private final Pose goalPose = new Pose(5, 140);

    // ═══════════════════════════════════════════════════════════════════════════
    // OpMode lifecycle
    // ═══════════════════════════════════════════════════════════════════════════

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        shakeTimer = new Timer();

        toggleMotor = hardwareMap.get(DcMotor.class, "intake");
        servo = hardwareMap.get(Servo.class, "servo");

        follower = Constants.create(hardwareMap); // NOTE: stub, returns null until AutoTune is run
        follower.setPose(startPose);

        // Shooter owns shooterR/shooterL/turretMotor/hoodServo; no separate
        // motor references or PID state needed here.
        shooter = new Shooter(hardwareMap, follower, telemetry);
        shooter.resetTurretEncoder();

        buildPaths();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.reset();
        setPathState(STATE_INIT);
    }

    @Override
    public void loop() {
        follower.update();

        // calculateTargets sets turretTargetTicks, targetFlywheelTPS, and
        // targetHoodServoPos together from the distance-based curves in
        // Shooter — driveFlywheels() then tracks that same target, so all
        // three axes stay in sync as the robot moves.
        shooter.calculateTargets(goalPose);
        shooter.driveTurret();
        turretStallGuard();
        shooter.driveFlywheels(true); // flywheel always spinning during auto
        shooter.driveHood();

        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.update();
    }

    @Override
    public void stop() {}

    private void turretStallGuard() {
        boolean stalled = turretLimiter.checkStall(
                shooter.turretMotor.getPower(),
                shooter.turretMotor.getCurrentPosition(),
                opmodeTimer.seconds());
        if (stalled) {
            shooter.turretMotor.setPower(0.0);
            telemetry.addLine("*** TURRET STALL — power cut ***");
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Path building
    // ═══════════════════════════════════════════════════════════════════════════

    private void buildPaths() {
        gateRotate = Paths.curve(gatePose, gateRotateControl, gatePoseRotated)
                .heading(Interpolator.linear(gatePose.heading(), gatePoseRotated.heading()));

        scoreFirst = Paths.curve(startPose, control1, scorePose)
                .heading(Interpolator.linear(startPose.heading(), scorePose.heading()));

        pickupFirst = Paths.curve(scorePose, control2, pickup2Pose2)
                .heading(Interpolator.linear(scorePose.heading(), pickup2Pose2.heading()));

        score2 = Paths.line(pickup2Pose2, scorePose)
                .heading(Interpolator.linear(pickup2Pose2.heading(), scorePose.heading()));

        pickupGate = Paths.curve(scorePose, gateControl, gatePose)
                .heading(Interpolator.linear(scorePose.heading(), gatePose.heading()));

        scoreGate = Paths.curve(gatePoseRotated, gateControl, scorePose)
                .heading(Interpolator.linear(gatePose.heading(), scorePose.heading()));

        pickupSecond = Paths.curve(scorePose, control3, pickup3Pose2)
                .heading(Interpolator.linear(scorePose.heading(), pickup3Pose2.heading()));

        score3 = Paths.line(pickup3Pose2, parkPose)
                .heading(Interpolator.linear(pickup3Pose2.heading(), parkPose.heading()));
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Shoot helper
    // ═══════════════════════════════════════════════════════════════════════════

    private void shoot(Timer timer, int nextState) {
        if (timer.seconds() < 0.8) {
            servo.setPosition(SERVO_SHOOT);
        } else if (timer.seconds() < 1) {
            servo.setPosition(SERVO_INTAKE);
        } else if (timer.seconds() < 1.5) {
            servo.setPosition(SERVO_SHOOT);
        } else {
            servo.setPosition(SERVO_INTAKE);
            setPathState(nextState);
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Main state machine
    // ═══════════════════════════════════════════════════════════════════════════

    public void autonomousPathUpdate() {
        switch (pathState) {
            case STATE_INIT:
                toggleMotor.setPower(1);
                setPathState(STATE_MOVE_TO_SCORE_FIRST);
                break;

            case STATE_MOVE_TO_SCORE_FIRST:
                if (!pathStarted) {
                    follower.holdEnd.set(true);
                    follower.follow(scoreFirst);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(STATE_SHOOT_FIRST);
                }
                break;

            case STATE_SHOOT_FIRST:
                shoot(pathTimer, STATE_MOVE_TO_PICKUP2);
                break;

            case STATE_MOVE_TO_PICKUP2:
                if (!pathStarted) {
                    follower.holdEnd.set(false);
                    follower.follow(pickupFirst);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(STATE_MOVE_TO_SCORE2);
                }
                break;

            case STATE_MOVE_TO_SCORE2:
                if (!pathStarted) {
                    follower.holdEnd.set(true);
                    follower.follow(score2);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(STATE_SHOOT_SECOND);
                }
                break;

            case STATE_SHOOT_SECOND:
                shoot(pathTimer, STATE_MOVE_TO_GATE);
                break;

            case STATE_MOVE_TO_GATE:
                doGatePickup(STATE_MOVE_TO_SCORE_GATE);
                break;

            case STATE_MOVE_TO_SCORE_GATE:
                if (!pathStarted) {
                    follower.holdEnd.set(true);
                    follower.follow(scoreGate);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(STATE_SHOOT_GATE);
                }
                break;

            case STATE_SHOOT_GATE:
                shoot(pathTimer, STATE_MOVE_TO_GATE2);
                break;

            case STATE_MOVE_TO_GATE2:
                doGatePickup(STATE_MOVE_TO_SCORE_GATE2);
                break;

            case STATE_MOVE_TO_SCORE_GATE2:
                if (!pathStarted) {
                    follower.holdEnd.set(true);
                    follower.follow(scoreGate);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(STATE_SHOOT_GATE2);
                }
                break;

            case STATE_SHOOT_GATE2:
                shoot(pathTimer, STATE_MOVE_TO_GATE3);
                break;

            case STATE_MOVE_TO_GATE3:
                doGatePickup(STATE_MOVE_TO_SCORE_GATE3);
                break;

            case STATE_MOVE_TO_SCORE_GATE3:
                if (!pathStarted) {
                    follower.holdEnd.set(true);
                    follower.follow(scoreGate);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(STATE_SHOOT_GATE3);
                }
                break;

            case STATE_SHOOT_GATE3:
                shoot(pathTimer, STATE_MOVE_TO_PICKUP1);
                break;

            case STATE_MOVE_TO_PICKUP1:
                if (!pathStarted) {
                    follower.holdEnd.set(false);
                    follower.follow(pickupSecond);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(STATE_MOVE_TO_SCORE1);
                }
                break;

            case STATE_MOVE_TO_SCORE1:
                if (!pathStarted) {
                    follower.holdEnd.set(true);
                    follower.follow(score3);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(STATE_SHOOT_FIRST_PICKUP);
                }
                break;

            case STATE_SHOOT_FIRST_PICKUP:
                shoot(pathTimer, STATE_DONE);
                break;
        }
    }

    private void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
        pathStarted = false;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Gate pickup
    // ═══════════════════════════════════════════════════════════════════════════

    private void doGatePickup(int nextState) {
        if (!atGate) {
            if (!pathStarted) {
                follower.holdEnd.set(false);
                follower.follow(pickupGate);
                pathStarted = true;
            }
            if (!follower.isBusy()) {
                atGate = true;
                pathStarted = false;
                shakeTimer.reset();
            }
            return;
        }

        if (!pathStarted) {
            follower.holdEnd.set(false);
            follower.follow(gateRotate);
            pathStarted = true;
        }
        if (!follower.isBusy() && shakeTimer.seconds() > 1.6) {
            atGate = false;
            setPathState(nextState);
        }
    }
}
