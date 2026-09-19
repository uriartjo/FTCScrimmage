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

import org.firstinspires.ftc.teamcode.decode.configs.RobotConstants;
import org.firstinspires.ftc.teamcode.decode.configs.Shooter;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Pedro Pathing v3 translation of the original Red_18Ball.java. Faithful
 * 1:1 port — this file did NOT have the flywheel-distance-tracking bug that
 * Blue_18Ball had (it already calls shooter.driveFlywheels(true) directly),
 * so no behavior fix is bundled in here; see Blue_18Ball.java's javadoc for
 * that unrelated fix and why it doesn't apply to this file. Same reasoning
 * for TurretLimiter: the original Red_18Ball didn't have it, so this port
 * doesn't add it — migration should reproduce existing behavior exactly,
 * not bundle in improvements from a sibling file.
 *
 * Changes from the v2 base (see Blue_18Ball.java's javadoc for the fuller
 * explanation of each — identical translation, applied here):
 *   - Constants.createFollower -> Constants.create (stub)
 *   - follower.setStartingPose -> follower.setPose; follower.getPose().getX() -> follower.pose().x() etc.
 *   - PathChain -> Path; pathBuilder()/addPath()/setLinearHeadingInterpolation()/build()
 *     -> Paths.curve(...)/Paths.line(...).heading(Interpolator.linear(...))
 *   - com.pedropathing.util.Timer -> com.pedropathing.utils.Timer; resetTimer()->reset();
 *     getElapsedTimeSeconds()->seconds()
 *   - follower.followPath(path, true/omitted) -> follower.holdEnd.set(true/false) immediately
 *     before follower.follow(path) — see Blue_18Ball.java's javadoc for why this is a verified
 *     1:1 mapping, not an approximation.
 *   - RobotConstants.xxx.mirror() -> RobotConstants.mirror(RobotConstants.xxx) — v3 has no
 *     instance Pose.mirror() method (see the Pedro v3 migration doc's Landmine 1); the new
 *     RobotConstants.java exposes a static mirror(Pose) helper that reproduces the exact v2
 *     formula instead.
 *
 * NOT yet run on hardware.
 */
@Autonomous(name = "Red 18 ball normal")
public class Red_18Ball extends OpMode {

    // ── States ───────────────────────────────────────────────────────────────────
    private static final int STATE_INIT                 = 0;
    private static final int STATE_MOVE_TO_SCORE_FIRST  = 1;
    private static final int STATE_SHOOT_FIRST          = 2;

    // Pickup 3 cycle
    private static final int STATE_MOVE_TO_PICKUP2      = 3;
    private static final int STATE_MOVE_TO_SCORE2       = 5;
    private static final int STATE_SHOOT_SECOND         = 6;

    // Pickup 2 cycle (gate)
    private static final int STATE_MOVE_TO_GATE         = 7;
    private static final int STATE_MOVE_TO_SCORE_GATE   = 9;
    private static final int STATE_SHOOT_GATE           = 10;

    private static final int STATE_MOVE_TO_GATE2        = 8;
    private static final int STATE_MOVE_TO_SCORE_GATE2  = 17;
    private static final int STATE_SHOOT_GATE2          = 18;
    private static final int STATE_MOVE_TO_GATE3        = 19;
    private static final int STATE_MOVE_TO_SCORE_GATE3  = 20;
    private static final int STATE_SHOOT_GATE3          = 21;

    // Pickup 1 cycle
    private static final int STATE_MOVE_TO_PICKUP1      = 11;
    private static final int STATE_MOVE_TO_SCORE1       = 13;
    private static final int STATE_SHOOT_FIRST_PICKUP   = 14;

    private static final int STATE_DONE                 = 16;

    // ── Hardware ─────────────────────────────────────────────────────────────────
    private Shooter  shooter;
    private DcMotor  toggleMotor;
    private Servo    servo;
    private Follower follower;

    // ── Paths ────────────────────────────────────────────────────────────────────
    private Path gateRotate;
    private Path pickupGate, scoreGate, pickupSecond;
    private Path score2, score3, scoreFirst, pickupFirst;

    // ── State machine ────────────────────────────────────────────────────────────
    private int     pathState;
    private boolean pathStarted = false;

    // ── Servo positions ───────────────────────────────────────────────────────────
    private static final double SERVO_INTAKE = 0.35;
    private static final double SERVO_SHOOT  = 0.51;

    // ── Gate pickup helpers ───────────────────────────────────────────────────────
    private boolean atGate = false;
    private Timer   shakeTimer;

    // ── Timers ────────────────────────────────────────────────────────────────────
    private Timer pathTimer, opmodeTimer;

    // ── Poses (mirrored to red side) ──────────────────────────────────────────────
    private final Pose startPose    = RobotConstants.mirror(RobotConstants.startPose);
    private final Pose control1     = RobotConstants.mirror(RobotConstants.control1);
    private final Pose scorePose    = RobotConstants.mirror(RobotConstants.scorePose);

    private final Pose control3     = RobotConstants.mirror(RobotConstants.control3);
    private final Pose pickup3Pose2 = RobotConstants.mirror(RobotConstants.pickup3Pose2);

    private final Pose control2     = RobotConstants.mirror(RobotConstants.control2);
    private final Pose pickup2Pose2 = RobotConstants.mirror(RobotConstants.pickup2Pose2);

    private final Pose gateControl       = RobotConstants.mirror(RobotConstants.gateControl);
    private final Pose gatePoseRotated   = RobotConstants.mirror(RobotConstants.gatePoseRotated);
    private final Pose gateRotateControl = RobotConstants.mirror(RobotConstants.gateRotateControl);
    private final Pose gatePose          = RobotConstants.mirror(RobotConstants.gatePose);

    private final Pose parkPose = RobotConstants.mirror(RobotConstants.parkPose);

    /** The field goal that turret / flywheel / hood aim at. */
    private final Pose goalPose = RobotConstants.mirror(new Pose(5, 140));

    // ═══════════════════════════════════════════════════════════════════════════════
    // OpMode lifecycle
    // ═══════════════════════════════════════════════════════════════════════════════

    @Override
    public void init() {
        pathTimer   = new Timer();
        opmodeTimer = new Timer();
        shakeTimer  = new Timer();

        toggleMotor = hardwareMap.get(DcMotor.class, "intake");
        servo       = hardwareMap.get(Servo.class,   "servo");

        follower = Constants.create(hardwareMap); // NOTE: stub, returns null until AutoTune is run
        follower.setPose(startPose);

        // Shooter owns shooterR, shooterL, turretMotor, and hoodServo.
        // It handles their hardware init, direction, encoder reset, and all PID.
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

        // Compute turret angle, flywheel speed, and hood position from current pose.
        shooter.calculateTargets(goalPose);

        // Drive all three shooter axes with the freshly computed targets.
        shooter.driveTurret();
        shooter.driveFlywheels(true);   // flywheels always spinning during auto
        shooter.driveHood();

        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X",          follower.pose().x());
        telemetry.addData("Y",          follower.pose().y());
        telemetry.addData("Heading",    Math.toDegrees(follower.pose().heading()));
        telemetry.update();
    }

    @Override
    public void stop() {}

    // ═══════════════════════════════════════════════════════════════════════════════
    // Path building
    // ═══════════════════════════════════════════════════════════════════════════════

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

    // ═══════════════════════════════════════════════════════════════════════════════
    // Shoot servo sequence
    // ═══════════════════════════════════════════════════════════════════════════════

    /**
     * Runs the kicker servo sequence using pathTimer, then advances to nextState.
     */
    private void shoot(Timer timer, int nextState) {
        double t = timer.seconds();
        if (t < 0.8) {
            servo.setPosition(SERVO_SHOOT);
        } else if (t < 1.0) {
            servo.setPosition(SERVO_INTAKE);
        } else if (t < 1.5) {
            servo.setPosition(SERVO_SHOOT);
        } else {
            servo.setPosition(SERVO_INTAKE);
            setPathState(nextState);
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════════
    // Main state machine
    // ═══════════════════════════════════════════════════════════════════════════════

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
        pathState   = pState;
        pathTimer.reset();
        pathStarted = false;
    }

    // ═══════════════════════════════════════════════════════════════════════════════
    // Gate pickup
    // ═══════════════════════════════════════════════════════════════════════════════

    private void doGatePickup(int nextState) {
        if (!atGate) {
            if (!pathStarted) {
                follower.holdEnd.set(false);
                follower.follow(pickupGate);
                pathStarted = true;
            }
            if (!follower.isBusy()) {
                atGate      = true;
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
