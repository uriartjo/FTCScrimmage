package org.firstinspires.ftc.teamcode.decode.opmodes.auto;


import static org.firstinspires.ftc.teamcode.decode.configs.RobotConstants.control1;
import static org.firstinspires.ftc.teamcode.decode.configs.RobotConstants.control2;
import static org.firstinspires.ftc.teamcode.decode.configs.RobotConstants.control3;
import static org.firstinspires.ftc.teamcode.decode.configs.RobotConstants.controlThirdRow;
import static org.firstinspires.ftc.teamcode.decode.configs.RobotConstants.gateControl;
import static org.firstinspires.ftc.teamcode.decode.configs.RobotConstants.gatePose;
import static org.firstinspires.ftc.teamcode.decode.configs.RobotConstants.gatePoseRotated;
import static org.firstinspires.ftc.teamcode.decode.configs.RobotConstants.gateRotateControl;
import static org.firstinspires.ftc.teamcode.decode.configs.RobotConstants.parkPose;
import static org.firstinspires.ftc.teamcode.decode.configs.RobotConstants.pickup1Pose2;
import static org.firstinspires.ftc.teamcode.decode.configs.RobotConstants.pickup2Pose2;
import static org.firstinspires.ftc.teamcode.decode.configs.RobotConstants.pickup3Pose2;

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
 * Pedro Pathing v3 translation of the original Blue_18Ball_lastrow.java.
 * Faithful 1:1 port, dead code included on purpose: coarseAlignTurret() and
 * updateShooterVelocityPID() are defined but never called in the original
 * (the call site is commented out — "//coarseAlignTurret();" — and
 * updateShooterVelocityPID() has no call site at all), and shooterR/
 * shooterL/turretMotor/hoodServo duplicate hardware references Shooter
 * already owns internally. None of that is a Pedro-version concern, so
 * removing it isn't part of this migration — it's ported as-is, with only
 * its Pedro/Pose touchpoints translated so it still compiles. STATE_PARK is
 * similarly unreachable in the original (STATE_SHOOT_THIRD transitions
 * straight to STATE_DONE) and is kept as-is for the same reason.
 *
 * See Blue_18Ball.java and Red_18Ball.java's javadocs for the full
 * explanation of each translation applied here (Constants.create, Pose/
 * Follower renames, Path/Paths/Interpolator, Timer package+methods,
 * holdEnd.set() before follow()) — identical treatment, not repeated here.
 *
 * NOT yet run on hardware.
 */
@Autonomous(name = "Blue 18 ball Auto_lastRow")
public class Blue_18Ball_lastrow extends OpMode {


    // ── Shooter velocity PID state (dead code — see class javadoc) ─────────────
    private double pidIntegral   = 0.0;
    private double pidLastError  = 0.0;
    private int    lastEncoderPos = 0;
    private long   lastTimeNs    = 0;


    // ── Turret PID (position) (dead code — see class javadoc) ──────────────────
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


    private static final int STATE_MOVE_TO_PICKUP1      = 17;
    private static final int STATE_MOVE_TO_SCORE1       = 18;
    private static final int STATE_SHOOT_FIRST_PICKUP          = 19;


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


    // ── Turret alignment (dead code — see class javadoc) ────────────────────────
    private int    turretOffset        = 0;
    private double coarseAngleTolerance = 1.5;


    // ── Poses (mirrored from blue: x_red = 144 - x_blue, heading_red = π - heading_blue) ──
    //
    // Blue turret target was (5, 140). Red target is (144-5, 140) = (139, 140).
    private static final double TARGET_X = 5;
    private static final double TARGET_Y = 140.0;

    // ─────────────────────────────────────────────────────────────────────────


    public void buildPaths() {

        gateRotate = Paths.curve(gatePose, gateRotateControl, gatePoseRotated)
                .heading(Interpolator.linear(gatePose.heading(), gatePoseRotated.heading()));

        scoreFirst = Paths.curve(RobotConstants.startPose, control1, RobotConstants.scorePose)
                .heading(Interpolator.linear(RobotConstants.startPose.heading(), RobotConstants.scorePose.heading()));

        pickupFirst = Paths.curve(RobotConstants.scorePose, control2, pickup2Pose2)
                .heading(Interpolator.linear(RobotConstants.scorePose.heading(), pickup2Pose2.heading()));

        score2 = Paths.line(pickup2Pose2, RobotConstants.scorePose)
                .heading(Interpolator.linear(pickup2Pose2.heading(), RobotConstants.scorePose.heading()));

        pickupGate = Paths.curve(RobotConstants.scorePose, gateControl, gatePose)
                .heading(Interpolator.linear(RobotConstants.scorePose.heading(), gatePose.heading()));

        scoreGate = Paths.curve(gatePoseRotated, gateControl, RobotConstants.scorePose)
                .heading(Interpolator.linear(gatePose.heading(), RobotConstants.scorePose.heading()));

        pickupSecond = Paths.curve(RobotConstants.scorePose, control3, pickup3Pose2)
                .heading(Interpolator.linear(RobotConstants.scorePose.heading(), pickup3Pose2.heading()));

        score3 = Paths.line(pickup3Pose2, RobotConstants.scorePose)
                .heading(Interpolator.linear(pickup3Pose2.heading(), RobotConstants.scorePose.heading()));

        pickup1 = Paths.curve(RobotConstants.scorePose, controlThirdRow, pickup1Pose2)
                .heading(Interpolator.linear(RobotConstants.scorePose.heading(), pickup1Pose2.heading()));

        score1 = Paths.line(pickup1Pose2, parkPose)
                .heading(Interpolator.linear(pickup1Pose2.heading(), parkPose.heading()));

        park = Paths.line(RobotConstants.scorePose, parkPose)
                .heading(Interpolator.linear(RobotConstants.scorePose.heading(), parkPose.heading()));
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
        } else {
            servo.setPosition(SERVO_INTAKE);
            setPathState(nextState);
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
                if (!follower.isBusy()) setPathState(STATE_MOVE_TO_SCORE2);
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
                if (!follower.isBusy()) setPathState(STATE_SHOOT_GATE2);
                break;

            case STATE_SHOOT_GATE2:
                shoot(pathTimer, STATE_MOVE_TO_PICKUP1);
                break;

            case STATE_MOVE_TO_PICKUP1:
                if (!pathStarted) {
                    follower.holdEnd.set(false);
                    follower.follow(pickupSecond);
                    pathStarted = true;
                }
                if (!follower.isBusy()) setPathState(STATE_MOVE_TO_SCORE1);
                break;

            case STATE_MOVE_TO_SCORE1:
                if (!pathStarted) {
                    follower.holdEnd.set(true);
                    follower.follow(score3);
                    pathStarted = true;
                }
                if (!follower.isBusy()) setPathState(STATE_SHOOT_FIRST_PICKUP);
                break;

            case STATE_SHOOT_FIRST_PICKUP:
                shoot(pathTimer, STATE_MOVE_TO_PICKUP3);
                break;

            case STATE_MOVE_TO_PICKUP3:
                if (!pathStarted) {
                    follower.holdEnd.set(false);
                    follower.follow(pickup1);
                    pathStarted = true;
                }
                if (!follower.isBusy()) setPathState(STATE_MOVE_TO_SCORE3);
                break;

            case STATE_MOVE_TO_SCORE3:
                if (!pathStarted) {
                    follower.holdEnd.set(true);
                    follower.follow(score1);
                    pathStarted = true;
                }
                if (!follower.isBusy()) setPathState(STATE_SHOOT_THIRD);
                break;

            case STATE_SHOOT_THIRD:
                shoot(pathTimer, STATE_DONE);
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
        if (!follower.isBusy() && shakeTimer.seconds() > 1.7) {
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
        lastTimeNs = System.nanoTime();

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

        follower = Constants.create(hardwareMap); // NOTE: stub, returns null until AutoTune is run
        follower.setPose(RobotConstants.startPose);

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
        //coarseAlignTurret();
        toggleMotor.setPower(1);

        shooter.calculateTargets(RobotConstants.goalPose);

        // ── Drive all three shooter axes with the freshly computed targets ─────
        shooter.driveTurret();
        shooter.driveFlywheels(true);   // flywheel always spinning during auto
        shooter.driveHood();

//        double shooterPower = updateShooterVelocityPID();
//        shooterR.setPower(shooterPower);
//        shooterL.setPower(shooterPower);

        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X",       "%.2f", follower.pose().x());
        telemetry.addData("Y",       "%.2f", follower.pose().y());
        telemetry.addData("Heading", "%.2f", Math.toDegrees(follower.pose().heading()));
        telemetry.update();
    }


    @Override
    public void stop() {}


    // ── Turret alignment (dead code — see class javadoc) ────────────────────────


    private void coarseAlignTurret() {
        final double tx = TARGET_X;   // 139.0  (red-side mirror of blue's 5.0)
        final double ty = TARGET_Y;   // 140.0  (unchanged)
        final double ticksPerDegree = 773.9 / 360.0;

        Pose   pose            = follower.pose();
        double robotX          = pose.x();
        double robotY          = pose.y();
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
            output          = 0.0;
            turretIntegral  = 0.0;
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


    // ── Shooter velocity PID (dead code — see class javadoc) ────────────────────


    private double updateShooterVelocityPID() {
        final double targetTps = 1550.0;

        long   nowNs = System.nanoTime();
        double dtSec = (nowNs - lastTimeNs) / 1_000_000_000.0;

        if (dtSec <= 0.001) {
            lastTimeNs     = nowNs;
            lastEncoderPos = shooterR.getCurrentPosition();
            return 0.0;
        }

        int    currentPos  = shooterR.getCurrentPosition();
        double velocityTPS = (currentPos - lastEncoderPos) / dtSec;
        double error       = targetTps - velocityTPS;

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
