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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.decode.configs.RobotConstants;
import org.firstinspires.ftc.teamcode.decode.configs.Shooter;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Pedro Pathing v3 translation of the original Blue_15Ball_lastrow.java.
 * Unlike the other 5 ported auto files, this one had two ACTIVE bugs, not
 * just inert dead code — both fixed here and disclosed, following the same
 * precedent set by Blue_18Ball.java's flywheel fix:
 *
 * FIX 1 — init() ordering: the original constructs `shooter = new Shooter(hardwareMap,
 * follower, telemetry)` BEFORE `follower = Constants.createFollower(hardwareMap)` runs.
 * Shooter stores the Follower reference in a `final` field, so it would have captured
 * null permanently — every subsequent shooter.calculateTargets()/driveTurret()/etc. call
 * would NPE the instant Constants.create() stops being a stub and starts returning a real
 * Follower. Fixed by reordering: follower is created and given its starting pose before
 * Shooter is constructed, matching every other ported auto file.
 *
 * FIX 2 — redundant, overriding turret/flywheel control: the original calls
 * coarseAlignTurret() (its own private turret PID) every loop, then immediately calls
 * shooter.driveTurret() afterward — the second call always overwrites the first's
 * turretMotor.setPower(), so coarseAlignTurret()'s entire computation was silently wasted
 * work with zero effect on the robot. Same story for the flywheel: shooter.driveFlywheels(true)
 * runs, then updateShooterVelocityPID()'s result immediately overwrites it via a direct
 * shooterR/shooterL.setPower() call — so the ACTUAL commanded flywheel speed came from this
 * file's own separate hardcoded-1500-TPS PID, not Shooter's distance-based target (the same
 * category of bug fixed in Blue_18Ball.java, just shaped as "compute-then-overwrite" instead
 * of "never call"). Fixed the same low-risk way two sibling files (Blue_18Ball_lastrow,
 * Red_18Ball_lastrow) already use elsewhere in this codebase: the redundant call sites are
 * commented out rather than the methods being deleted, so the team can see exactly what
 * changed and why. shooter.driveTurret()/driveFlywheels(true) are now the sole source of
 * truth for both axes.
 *
 * NOT FIXED, FLAGGED INSTEAD: shoot()'s state transition happens on the t &lt; 1.5 branch
 * (while the servo is still commanded to SERVO_SHOOT), not after returning to SERVO_INTAKE
 * like every other ported file's shoot() does. Genuinely unclear whether this is an
 * intentional tuning difference or a copy-paste slip — preserved exactly as the original
 * had it rather than silently normalized to match its siblings. Worth the team's attention.
 *
 * See Blue_18Ball.java/Red_18Ball.java for the rest of the translation notes (Constants.create,
 * Pose/Follower renames, Path/Paths/Interpolator, Timer package+methods, holdEnd.set() before
 * follow()) — identical here.
 *
 * NOT yet run on hardware.
 */
@Autonomous(name = "Blue 15 ball Auto_lastRow")
public class Blue_15Ball_lastrow extends OpMode {

    // ── Shooter velocity PID state (dead code as of FIX 2 — see class javadoc) ─
    private double pidIntegral   = 0.0;
    private double pidLastError  = 0.0;
    private int    lastEncoderPos = 0;
    private long   lastTimeNs    = 0;

    // ── Turret PID (position) (dead code as of FIX 2 — see class javadoc) ──────
    public static double TURRET_KP = 0.01;
    public static double TURRET_KI = 0.0005;
    public static double TURRET_KD = 0.0005;

    private static final double TURRET_I_MAX    = 0.30;
    private static final double TURRET_MAX_POWER = 1.0;
    private static final double TURRET_MIN_POWER = 0.0;

    private double turretIntegral  = 0.0;
    private double turretLastError = 0.0;
    private long   turretLastTimeNs = 0;

    // ── State machine constants ───────────────────────────────────────────────
    private static final int STATE_INIT                 = 0;
    private static final int STATE_MOVE_TO_SCORE_FIRST  = 1;
    private static final int STATE_SHOOT_FIRST          = 2;
    private static final int STATE_MOVE_TO_PICKUP2      = 3;
    private static final int STATE_MOVE_TO_SCORE2       = 4;
    private static final int STATE_SHOOT_SECOND         = 5;
    private static final int STATE_MOVE_TO_GATE         = 6;
    private static final int STATE_MOVE_TO_SCORE_GATE   = 7;
    private static final int STATE_SHOOT_GATE           = 8;
    private static final int STATE_MOVE_TO_GATE2        = 9;
    private static final int STATE_MOVE_TO_SCORE_GATE2  = 10;
    private static final int STATE_SHOOT_GATE2          = 11;
    private static final int STATE_MOVE_TO_PICKUP3      = 12;
    private static final int STATE_MOVE_TO_SCORE3       = 13;
    private static final int STATE_SHOOT_THIRD          = 14;
    private static final int STATE_PARK                 = 15;
    private static final int STATE_DONE                 = 16;

    // ── Servo positions ───────────────────────────────────────────────────────
    private static final double SERVO_INTAKE = 0.35;
    private static final double SERVO_SHOOT  = 0.51;

    // ── Hardware ──────────────────────────────────────────────────────────────
    private DcMotor shooterR, shooterL;
    private DcMotor toggleMotor;
    private DcMotor turretMotor;
    private Servo   servo, hoodServo;
    private Shooter shooter;


    // ── Pedro Pathing ─────────────────────────────────────────────────────────
    private Follower follower;
    private Path scoreFirst, pickupFirst, score2;
    private Path pickupGate, scoreGate, gateRotate;
    private Path pickupSecond, score3;
    private Path pickup1, score1, park;

    // ── Timers ────────────────────────────────────────────────────────────────
    private Timer pathTimer, opmodeTimer, shakeTimer;

    // ── Path state tracking ───────────────────────────────────────────────────
    private int     pathState;
    private boolean pathStarted = false;
    private boolean atGate      = false;

    // ── Turret alignment (dead code as of FIX 2 — see class javadoc) ───────────
    private int    turretOffset        = 0;
    private double coarseAngleTolerance = 5.0;

    private final Pose goalPose = new Pose(5, 140);

    // ── Poses ─────────────────────────────────────────────────────────────────
    private final Pose startPose  = new Pose(24.86, 121.8, Math.toRadians(180));
    private final Pose scorePose  = new Pose(52, 79.2,     Math.toRadians(180));
    private final Pose parkPose   = new Pose(63.9, 105.77, Math.toRadians(180));

    // Control points
    private final Pose control1 = new Pose(43.29, 104.73, Math.toRadians(180));
    private final Pose control2 = new Pose(54.73, 54.9);
    private final Pose control3 = new Pose(59.221, 79.143);
    private final Pose gateControl       = new Pose(44, 65.23);
    private final Pose gateRotateControl = new Pose(15.8, 61.7);
    private final Pose controlThirdRow   = new Pose(55.95, 31.16);

    // Row pickup endpoints
    private final Pose pickup2Pose2 = new Pose(24.9,  58.789, Math.toRadians(180));
    private final Pose pickup3Pose2 = new Pose(25.2,  83.68,  Math.toRadians(180));
    private final Pose pickup1Pose2 = new Pose(25.2,  34.8,   Math.toRadians(180));

    // Gate poses
    private final Pose gatePose        = new Pose(17,   65.9, Math.toRadians(180));
    private final Pose gatePoseRotated = new Pose(12.5, 58,   Math.toRadians(165));

    // ─────────────────────────────────────────────────────────────────────────

    public void buildPaths() {

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

        scoreGate = Paths.curve(gatePose, gateControl, scorePose)
                .heading(Interpolator.linear(gatePose.heading(), scorePose.heading()));

        pickupSecond = Paths.curve(scorePose, control3, pickup3Pose2)
                .heading(Interpolator.linear(scorePose.heading(), pickup3Pose2.heading()));

        score3 = Paths.line(pickup3Pose2, parkPose)
                .heading(Interpolator.linear(pickup3Pose2.heading(), parkPose.heading()));

        pickup1 = Paths.curve(scorePose, controlThirdRow, pickup1Pose2)
                .heading(Interpolator.linear(scorePose.heading(), pickup1Pose2.heading()));

        score1 = Paths.line(pickup1Pose2, scorePose)
                .heading(Interpolator.linear(pickup1Pose2.heading(), scorePose.heading()));

        park = Paths.line(scorePose, parkPose)
                .heading(Interpolator.linear(scorePose.heading(), parkPose.heading()));
    }

    /** Fires 2 balls with a brief reset between them, then transitions to nextState. */
    private void shoot(Timer timer, int nextState) {
        double t = timer.seconds();
        if (t < 0.8) {
            servo.setPosition(SERVO_SHOOT);
        } else if (t < 1.0) {
            servo.setPosition(SERVO_INTAKE);
        } else if (t < 1.5) {
            servo.setPosition(SERVO_SHOOT);
            setPathState(nextState);
        } else {
            servo.setPosition(SERVO_INTAKE);
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case STATE_INIT:
                setPathState(STATE_MOVE_TO_SCORE_FIRST);
                break;

            case STATE_MOVE_TO_SCORE_FIRST:
                if (!pathStarted) {
                    follower.holdEnd.set(true);
                    follower.follow(scoreFirst);
                    pathStarted = true;
                }
                if (!follower.isBusy()) setPathState(STATE_SHOOT_FIRST);
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
                if (pathTimer.seconds() > 1.5) setPathState(STATE_MOVE_TO_SCORE2);
                break;

            case STATE_MOVE_TO_SCORE2:
                if (!pathStarted) {
                    follower.holdEnd.set(true);
                    follower.follow(score2);
                    pathStarted = true;
                }
                if (!follower.isBusy()) setPathState(STATE_SHOOT_SECOND);
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
                if (!follower.isBusy()) setPathState(STATE_SHOOT_GATE);
                break;

            case STATE_SHOOT_GATE:
                shoot(pathTimer, STATE_MOVE_TO_PICKUP3);
                break;

            case STATE_MOVE_TO_PICKUP3:
                if (!pathStarted) {
                    follower.holdEnd.set(false);
                    follower.follow(pickupSecond);
                    pathStarted = true;
                }
                if (pathTimer.seconds() > 1.2) setPathState(STATE_MOVE_TO_SCORE3);
                break;

            case STATE_MOVE_TO_SCORE3:
                if (!pathStarted) {
                    follower.holdEnd.set(true);
                    follower.follow(score3);
                    pathStarted = true;
                }
                if (!follower.isBusy()) setPathState(STATE_SHOOT_THIRD);
                break;

            case STATE_SHOOT_THIRD:
                shoot(pathTimer, STATE_PARK);
                break;

            case STATE_PARK:
                if (!pathStarted) {
                    follower.holdEnd.set(false);
                    follower.follow(park);
                    pathStarted = true;
                }
                if (pathTimer.seconds() > 0.5) setPathState(STATE_DONE);
                break;
        }
    }

    /** Drives to the gate, waits for the gate rotate, then transitions to nextState. */
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
        if (!follower.isBusy() && shakeTimer.seconds() > 1.8) {
            atGate = false;
            setPathState(nextState);
        }
    }

    public void setPathState(int pState) {
        pathState   = pState;
        pathStarted = false;
        pathTimer.reset();
    }

    // ── OpMode lifecycle ──────────────────────────────────────────────────────

    @Override
    public void init() {
        pathTimer    = new Timer();
        opmodeTimer  = new Timer();
        shakeTimer   = new Timer();
        opmodeTimer.reset();

        toggleMotor = hardwareMap.get(DcMotor.class, "intake");
        servo       = hardwareMap.get(Servo.class, "servo");
        hoodServo   = hardwareMap.get(Servo.class, "hoodServo");

        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretLastTimeNs = System.nanoTime();

        shooterR = hardwareMap.get(DcMotor.class, "shooterR");
        shooterL = hardwareMap.get(DcMotor.class, "shooterL");
        shooterR.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterL.setDirection(DcMotorSimple.Direction.REVERSE);

        // FIX 1 (see class javadoc): follower must exist and have its starting pose
        // set BEFORE Shooter is constructed — the original had these two calls
        // backwards, which would have permanently captured a null Follower reference.
        follower = Constants.create(hardwareMap); // NOTE: stub, returns null until AutoTune is run
        follower.setPose(startPose);

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
        // FIX 2 (see class javadoc): coarseAlignTurret() call removed — its
        // turretMotor.setPower() was always immediately overwritten by
        // shooter.driveTurret() below, so it had zero effect and was pure
        // wasted computation. shooter.driveTurret() is now the sole source
        // of truth for the turret.
        shooter.calculateTargets(goalPose);

        // ── Drive all three shooter axes with the freshly computed targets ─────
        shooter.driveTurret();
        shooter.driveFlywheels(true);   // flywheel always spinning during auto
        shooter.driveHood();

        toggleMotor.setPower(1);

        // FIX 2 continued: updateShooterVelocityPID()'s result used to be applied
        // directly to shooterR/shooterL here, immediately overwriting
        // shooter.driveFlywheels(true)'s output above with a hardcoded 1500 TPS
        // target instead of the distance-based one — removed for the same reason
        // as Blue_18Ball.java's flywheel fix.

        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X",       "%.2f", follower.pose().x());
        telemetry.addData("Y",       "%.2f", follower.pose().y());
        telemetry.addData("Heading", "%.2f", Math.toDegrees(follower.pose().heading()));
        telemetry.update();
    }

    @Override
    public void stop() {}

    // ── Turret alignment (dead code as of FIX 2 — see class javadoc) ───────────

    private void coarseAlignTurret() {
        final double tx = 5.0;
        final double ty = 140.0;
        final double ticksPerDegree = 773.9 / 360.0;

        Pose   pose           = follower.pose();
        double robotX         = pose.x();
        double robotY         = pose.y();
        double robotHeadingRad = pose.heading();

        double dx = tx - robotX;
        double dy = ty - robotY;

        double targetFieldRad = Math.atan2(dy, dx);
        double turretRelRad   = targetFieldRad - robotHeadingRad;

        // Normalize to [-π, π]
        while (turretRelRad >  Math.PI) turretRelRad -= 2.0 * Math.PI;
        while (turretRelRad < -Math.PI) turretRelRad += 2.0 * Math.PI;

        double turretRelDeg = Math.toDegrees(turretRelRad);
        turretOffset = (int) Math.round(-turretRelDeg * ticksPerDegree);

        int    currentTicks = turretMotor.getCurrentPosition();
        int    errorTicks   = turretOffset - currentTicks;

        long   nowNs = System.nanoTime();
        double dtSec = Math.max((nowNs - turretLastTimeNs) / 1_000_000_000.0, 0.0005);
        dtSec = Math.min(dtSec, 0.05);

        double pTerm = TURRET_KP * errorTicks;

        turretIntegral += errorTicks * dtSec;
        double iTerm = clamp(TURRET_KI * turretIntegral, -TURRET_I_MAX, TURRET_I_MAX);

        double dTerm = TURRET_KD * ((errorTicks - turretLastError) / dtSec);

        double output = clamp(pTerm + iTerm + dTerm, -TURRET_MAX_POWER, TURRET_MAX_POWER);
        if (Math.abs(output) < TURRET_MIN_POWER) output = 0.0;

        double errorDegrees = Math.abs(errorTicks) / ticksPerDegree;
        boolean aligned = errorDegrees <= coarseAngleTolerance;

        if (aligned) {
            output         = 0.0;
            turretIntegral = 0.0;
            turretLastError = 0.0;
        }

        turretMotor.setPower(output);
        turretLastError  = errorTicks;
        turretLastTimeNs = nowNs;

        telemetry.addData("Target (x,y)",          "(%.1f, %.1f)", tx, ty);
        telemetry.addData("Robot (x,y)",            "(%.1f, %.1f)", robotX, robotY);
        telemetry.addData("Target Field Ang (deg)", "%.1f", Math.toDegrees(targetFieldRad));
        telemetry.addData("Robot Heading (deg)",    "%.1f", Math.toDegrees(robotHeadingRad));
        telemetry.addData("Turret Rel (deg)",       "%.1f", turretRelDeg);
        telemetry.addData("Turret Target",          "%d ticks", turretOffset);
        telemetry.addData("Turret Current",         "%d ticks", currentTicks);
        telemetry.addData("Error (deg)",            "%.2f", errorDegrees);
        telemetry.addData("Turret PID Out",         "%.3f", output);
        telemetry.addData("Coarse Status", aligned ? "ALIGNED" : "ALIGNING");
    }

    // ── Shooter velocity PID (dead code as of FIX 2 — see class javadoc) ───────

    private double updateShooterVelocityPID() {
        final double targetTps = 1500.0;

        long   nowNs = System.nanoTime();
        double dtSec = (nowNs - lastTimeNs) / 1_000_000_000.0;

        if (dtSec <= 0.001) {
            lastTimeNs    = nowNs;
            lastEncoderPos = shooterR.getCurrentPosition();
            return 0.0;
        }

        int    currentPos   = shooterR.getCurrentPosition();
        double velocityTPS  = (currentPos - lastEncoderPos) / dtSec;
        double error        = targetTps - velocityTPS;

        double pTerm = RobotConstants.FLYWHEEL_KP * error;

        pidIntegral += error * dtSec;
        double iTerm = clamp(RobotConstants.FLYWHEEL_KI * pidIntegral,
                -RobotConstants.SHOOTER_I_MAX, RobotConstants.SHOOTER_I_MAX);

        double dTerm = RobotConstants.FLYWHEEL_KD * (error - pidLastError) / dtSec;

        double power = clamp(pTerm + iTerm + dTerm, 0.0, 1.0);

        pidLastError   = error;
        lastEncoderPos = currentPos;
        lastTimeNs     = nowNs;

        telemetry.addData("ShooterTPS", "%.1f", velocityTPS);
        telemetry.addData("ShooterErr", "%.1f", error);

        return power;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
