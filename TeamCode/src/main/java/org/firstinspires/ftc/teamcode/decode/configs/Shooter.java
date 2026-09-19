package org.firstinspires.ftc.teamcode.decode.configs;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Pedro Pathing v3 translation of the original Shooter.java from
 * 641WinterHavenEdition-master. Only the Pedro/Pose touchpoints changed —
 * all PID math, hardware wiring, and control logic is copied verbatim.
 * The inline flywheel/turret PID has NOT been migrated onto util/PIDController
 * here — that's a separate, independent change (see the old repo's handoff
 * doc to-dos) kept out of this diff on purpose so each can be reviewed alone.
 *
 * Changes from the original:
 *   - import com.pedropathing.geometry.Pose -> com.pedropathing.math.Pose
 *   - import com.pedropathing.math.MathFunctions removed — that class doesn't exist in v3
 *     core (checked: no MathFunctions.class in the v3 core-3.0.0.jar). Its two call sites
 *     here (MathFunctions.clamp(...)) now use this class's own existing private clamp()
 *     helper, which already did the identical clamp(v, lo, hi) — no new dependency needed.
 *   - follower.getPose() -> follower.pose()
 *   - pose.getX() / pose.getY() -> pose.x() / pose.y()
 *
 * NOT yet verified on hardware — depends on Constants.create() returning a real Follower,
 * which needs the MecanumTuner/PinpointTuner procedures run on the physical robot first.
 */
public class Shooter {

    // ─── Hardware ────────────────────────────────────────────────────────────

    public final DcMotor shooterR, shooterL, turretMotor;
    public final Servo   hoodServo;

    private final Follower  follower;
    private final Telemetry telemetry;

    // ─── Turret PID state ────────────────────────────────────────────────────

    private double turretIntegral  = 0.0;
    private double turretLastError = 0.0;
    private long   turretLastNs    = 0;

    // ─── Flywheel PID state ──────────────────────────────────────────────────

    private double pidIntegral      = 0.0;
    private double pidLastError     = 0.0;
    private double lastFlywheelPower = 0.0;
    private int    lastEncoderPos   = 0;
    private long   lastFlywheelNs   = 0;

    // ─── Calculated targets (written by calculateTargets / set externally) ───

    public int    turretTargetTicks  = 0;
    public double targetFlywheelTPS  = 0.0;
    public double targetHoodServoPos = 0.5;

    // ─── Telemetry outputs ───────────────────────────────────────────────────

    public double lastTurretRelDeg = 0.0;
    public double lastFlywheelTPS  = 0.0;

    // ─── Constructor ─────────────────────────────────────────────────────────

    public Shooter(HardwareMap hardwareMap, Follower follower, Telemetry telemetry) {
        this.follower  = follower;
        this.telemetry = telemetry;

        shooterR    = hardwareMap.get(DcMotor.class, "shooterR");
        shooterL    = hardwareMap.get(DcMotor.class, "shooterL");
        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        hoodServo   = hardwareMap.get(Servo.class,   "hoodServo");

        shooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterR.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterL.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lastEncoderPos = shooterR.getCurrentPosition();
        lastFlywheelNs = System.nanoTime();
        turretLastNs   = System.nanoTime();
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // TARGET CALCULATION
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Calculates all three targets from a goal pose in one call.
     * Sets turretTargetTicks, targetFlywheelTPS, and targetHoodServoPos.
     */
    public void calculateTargets(Pose gatePose) {
        double dist = getRobotDistanceToGoal(gatePose);
        calculateTurretTarget(gatePose.x(), gatePose.y());
        targetFlywheelTPS  = flywheelSpeed(dist);
        targetHoodServoPos = hoodAngle(dist);
    }

    /**
     * Calculates only the turret target from a field point.
     */
    public void calculateTurretTarget(double tx, double ty) {
        Pose   pose           = follower.pose();
        double dx             = tx - pose.x();
        double dy             = ty - pose.y();
        double targetFieldRad = Math.atan2(dy, dx);
        double turretRelRad   = wrapRad(targetFieldRad - pose.heading());

        lastTurretRelDeg  = Math.toDegrees(turretRelRad);
        turretTargetTicks = (int) Math.round(-lastTurretRelDeg * RobotConstants.TICKS_PER_DEGREE);
    }

    /**
     * Flywheel speed (TPS) from distance (inches).
     * flywheelOffset is an additive trim, applied before the upper clamp.
     */
    public static double flywheelSpeed(double goalDist) {
        double raw = -0.0300158 * goalDist * goalDist
                + 10.75769 * goalDist
                + 450;
        return clamp(raw + RobotConstants.flywheelOffset,0,3000);
    }

    /**
     * Hood servo position from distance (inches).
     */
    public static double hoodAngle(double goalDist) {
        double raw = 0.00010237 * goalDist * goalDist
                - 0.0247299 * goalDist
                + 1.54;
        return clamp(raw + RobotConstants.hoodOffset, RobotConstants.hoodServoMin, RobotConstants.hoodServoMax);
    }

    /**
     * Straight-line distance (inches) from the robot to a field pose.
     */
    public double getRobotDistanceToGoal(Pose gatePose) {
        Pose   pose = follower.pose();
        double dx   = gatePose.x() - pose.x();
        double dy   = gatePose.y() - pose.y();
        return Math.sqrt(dx * dx + dy * dy);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // SHOOT ON THE MOVE — Huskie Robotics 3061 velocity-subtraction method
    // (github reference: "Huskie Physics: Shoot-on-the-Move with Equations," 2026-07-22)
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Same as {@link #calculateTargets(Pose)}, but compensates for the robot's
     * current field-frame velocity so the ball's trajectory (in the field
     * frame) matches the static shot map regardless of whether the robot is
     * moving — the PDF's velocity-subtraction recipe, section 1.4-1.5.
     *
     * NOT SAFE TO USE YET: this depends on four calibration constants in
     * RobotConstants.java (FLYWHEEL_RADIUS_IN, FLYWHEEL_EFFICIENCY,
     * FLYWHEEL_TICKS_PER_REV, HOOD_ANGLE_AT_MIN_DEG/MAX_DEG) that are all
     * placeholders — see their javadoc there. Until they're measured (slow-mo
     * video of the flywheel/hood, per the PDF's "Characterizing the Shooter"
     * section — a shooter-bench measurement, doesn't need Saturday's
     * drivetrain tuning), this method will compute a plausible-looking but
     * physically wrong shot. {@link #calculateTargets(Pose)} — the existing,
     * already-tuned static map — is what ShooterTeleopBase should keep calling
     * until those constants are filled in.
     *
     * Sets turretTargetTicks, targetFlywheelTPS, and targetHoodServoPos, same
     * as calculateTargets.
     */
    private double  lastSomTurretRelRad = 0.0; // see subtractRobotVelocity's near-vertical-shot note
    private boolean somTurretRelRadInitialized = false;

    public void calculateTargetsShootOnMove(Pose goalPose) {
        Pose   pose = follower.pose();
        double dist = getRobotDistanceToGoal(goalPose);

        // Seed the azimuth-hold state from whatever bearing is already known
        // (e.g. from a prior calculateTurretTarget/calculateTargets call) the
        // first time this method runs, rather than trusting the field's
        // compile-time 0.0 default — otherwise a near-vertical first shot
        // would hold the turret at field-bearing 0 instead of its real
        // current bearing.
        if (!somTurretRelRadInitialized) {
            lastSomTurretRelRad = Math.toRadians(lastTurretRelDeg);
            somTurretRelRadInitialized = true;
        }

        // Step 1-2 of the PDF's recipe: the existing static shot map, converted
        // from raw actuator units into physical launch speed/angle.
        double staticSpeedInPerSec  = flywheelTpsToLaunchSpeed(flywheelSpeed(dist));
        double staticElevationDeg   = hoodPosToLaunchAngleDeg(hoodAngle(dist));
        double fieldBearingRad      = Math.atan2(goalPose.y() - pose.y(), goalPose.x() - pose.x());

        Velocity vel = follower.velocity(); // field-frame vx, vy, omega — verified via javap on the real v3 jar

        ShotVector shot = subtractRobotVelocity(
                staticSpeedInPerSec, staticElevationDeg, fieldBearingRad,
                vel.vx, vel.vy, vel.omega, pose.heading(),
                RobotConstants.SHOOTER_OFFSET_X_IN, RobotConstants.SHOOTER_OFFSET_Y_IN,
                lastSomTurretRelRad);
        lastSomTurretRelRad = shot.turretRelRad;

        lastTurretRelDeg  = Math.toDegrees(shot.turretRelRad);
        turretTargetTicks = (int) Math.round(-lastTurretRelDeg * RobotConstants.TICKS_PER_DEGREE);

        targetFlywheelTPS  = clamp(launchSpeedToFlywheelTps(shot.speedInPerSec) + RobotConstants.flywheelOffset, 0, 3000);
        targetHoodServoPos = clamp(launchAngleDegToHoodPos(shot.elevationDeg) + RobotConstants.hoodOffset,
                RobotConstants.hoodServoMin, RobotConstants.hoodServoMax);
    }

    /**
     * Result of {@link #subtractRobotVelocity}: the shot the robot must actually
     * take, in physical units, ready to convert back to actuator commands.
     */
    public static final class ShotVector {
        public final double speedInPerSec;  // physical launch speed
        public final double elevationDeg;   // launch angle above horizontal, 0=flat, 90=straight up
        public final double turretRelRad;   // robot-frame bearing, same sign convention as calculateTurretTarget

        public ShotVector(double speedInPerSec, double elevationDeg, double turretRelRad) {
            this.speedInPerSec = speedInPerSec;
            this.elevationDeg  = elevationDeg;
            this.turretRelRad  = turretRelRad;
        }
    }

    /**
     * Pure math, no hardware — this is Equations 8-19 of the Huskie Physics PDF.
     * Given a static shot (speed/elevation/field bearing) and the robot's current
     * field-frame velocity, returns the shot the robot must actually fire so the
     * ball's field-frame trajectory matches the static shot regardless of motion.
     *
     * Angle convention: this method takes/returns elevation above horizontal
     * (0=flat, 90=straight up) — NOT the PDF's own theta (0=straight up, 90=flat;
     * see the PDF's own footnote 1 admitting this is the unusual one). The
     * conversion (elevation = 90 - theta) happens internally so nobody has to
     * remember the PDF's flipped convention when calling this.
     *
     * Near-vertical-shot azimuth guard: found in a Claude/ChatGPT cross-review
     * (transcripts/discussion_20260917_135915.md in the ai_pair_programming
     * repo) and verified numerically, not just argued about. When the robot's
     * velocity nearly cancels the static shot's horizontal component (elevation
     * approaching 90 — exactly the "lay-up" case the PDF itself describes),
     * vShotX/vShotY both approach zero and phiShot = atan2(vShotY, vShotX)
     * becomes noise-dominated: a simulation with plausible per-axis velocity
     * noise on a small true horizontal vector produced a 215-degree azimuth
     * swing across 8 consecutive loop ticks. Below AZIMUTH_HOLD_THRESHOLD_IN_PER_SEC,
     * this method reuses previousTurretRelRad instead of recomputing a noisy
     * azimuth — cheap and correct, since azimuth barely matters physically once
     * the shot is nearly vertical anyway.
     *
     * @param staticSpeedInPerSec static-map launch speed, in/s
     * @param staticElevationDeg  static-map launch angle above horizontal, degrees
     * @param fieldBearingRad     field-frame bearing from robot to target, atan2(dy,dx)
     * @param robotVx             robot's field-frame x velocity, in/s
     * @param robotVy             robot's field-frame y velocity, in/s
     * @param robotOmega          robot's angular velocity, rad/s
     * @param robotHeadingRad     robot's current field-frame heading, rad
     * @param shooterOffsetXIn    shooter's forward offset from robot center, in (robot frame)
     * @param shooterOffsetYIn    shooter's left offset from robot center, in (robot frame)
     * @param previousTurretRelRad the last turretRelRad this method returned (or the current
     *                              turret bearing on the first call) — reused when the shot is
     *                              too close to vertical to trust a fresh azimuth (see below).
     */
    public static ShotVector subtractRobotVelocity(
            double staticSpeedInPerSec, double staticElevationDeg, double fieldBearingRad,
            double robotVx, double robotVy, double robotOmega, double robotHeadingRad,
            double shooterOffsetXIn, double shooterOffsetYIn, double previousTurretRelRad) {

        double thetaStar = Math.toRadians(90.0 - staticElevationDeg); // PDF's zenith-angle convention

        // Equation 1: static shot velocity, field-frame cartesian.
        double vStarX = staticSpeedInPerSec * Math.sin(thetaStar) * Math.cos(fieldBearingRad);
        double vStarY = staticSpeedInPerSec * Math.sin(thetaStar) * Math.sin(fieldBearingRad);
        double vStarZ = staticSpeedInPerSec * Math.cos(thetaStar);

        // Section 1.5: add the shooter's own velocity from robot rotation (a
        // no-op while shooterOffsetX/Y are both 0, i.e. not yet measured).
        double xsField =  Math.cos(robotHeadingRad) * shooterOffsetXIn - Math.sin(robotHeadingRad) * shooterOffsetYIn;
        double ysField =  Math.sin(robotHeadingRad) * shooterOffsetXIn + Math.cos(robotHeadingRad) * shooterOffsetYIn;
        double vRobotX = robotVx + (-ysField * robotOmega);
        double vRobotY = robotVy + ( xsField * robotOmega);

        // Equation 8-9: subtract the robot's velocity from the static shot.
        double vShotX = vStarX - vRobotX;
        double vShotY = vStarY - vRobotY;
        double vShotZ = vStarZ; // robot has no vertical velocity

        // Equations 13-18 (using atan2 per the PDF's own footnote 4, not atan).
        double vShotHoriz = Math.hypot(vShotX, vShotY);
        double vShotMag   = Math.hypot(vShotHoriz, vShotZ);
        double thetaShot  = Math.atan2(vShotHoriz, vShotZ); // well-defined everywhere; vShotHoriz>=0 always

        double turretRelRad;
        if (vShotHoriz < RobotConstants.SOM_AZIMUTH_HOLD_THRESHOLD_IN_PER_SEC) {
            // Near-vertical shot — see this method's javadoc. Hold the last stable
            // bearing instead of trusting a noise-dominated atan2(~0, ~0).
            turretRelRad = previousTurretRelRad;
        } else {
            // Equation 19: rotate the field-frame shot bearing into the robot/turret frame.
            double phiShot = Math.atan2(vShotY, vShotX);
            turretRelRad = wrapRad(phiShot - robotHeadingRad);
        }

        double elevationShotDeg = 90.0 - Math.toDegrees(thetaShot);

        return new ShotVector(vShotMag, elevationShotDeg, turretRelRad);
    }

    /**
     * Flywheel TPS -> physical ball exit speed (in/s). Equation 5:
     * v = pi * (R1+R2) * RPS * efficiency, with R1=R2=FLYWHEEL_RADIUS_IN here
     * (this robot's two flywheels are assumed equal radius, same as the PDF's
     * own simplification for matched pulleys).
     */
    public static double flywheelTpsToLaunchSpeed(double tps) {
        double rps = (tps / RobotConstants.FLYWHEEL_TICKS_PER_REV);
        return Math.PI * (2 * RobotConstants.FLYWHEEL_RADIUS_IN) * rps * RobotConstants.FLYWHEEL_EFFICIENCY;
    }

    /** Inverse of {@link #flywheelTpsToLaunchSpeed}. */
    public static double launchSpeedToFlywheelTps(double speedInPerSec) {
        double rps = speedInPerSec / (Math.PI * (2 * RobotConstants.FLYWHEEL_RADIUS_IN) * RobotConstants.FLYWHEEL_EFFICIENCY);
        return rps * RobotConstants.FLYWHEEL_TICKS_PER_REV;
    }

    /**
     * Hood servo position -> physical launch angle (degrees above horizontal),
     * linearly interpolated between the two measured endpoints in RobotConstants.
     */
    public static double hoodPosToLaunchAngleDeg(double servoPos) {
        double t = (servoPos - RobotConstants.hoodServoMin) / (RobotConstants.hoodServoMax - RobotConstants.hoodServoMin);
        return RobotConstants.HOOD_ANGLE_AT_MIN_DEG + t * (RobotConstants.HOOD_ANGLE_AT_MAX_DEG - RobotConstants.HOOD_ANGLE_AT_MIN_DEG);
    }

    /** Inverse of {@link #hoodPosToLaunchAngleDeg}. */
    public static double launchAngleDegToHoodPos(double elevationDeg) {
        double t = (elevationDeg - RobotConstants.HOOD_ANGLE_AT_MIN_DEG) / (RobotConstants.HOOD_ANGLE_AT_MAX_DEG - RobotConstants.HOOD_ANGLE_AT_MIN_DEG);
        return RobotConstants.hoodServoMin + t * (RobotConstants.hoodServoMax - RobotConstants.hoodServoMin);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // HARDWARE DRIVE
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Runs the turret PID toward turretTargetTicks.
     * turretLastNs is updated unconditionally so the D-term does not spike
     * on re-engagement after an alignment hold.
     */
    public void driveTurret() {
        int  currentTicks = turretMotor.getCurrentPosition();
        int  errorTicks   = turretTargetTicks - currentTicks;

        long   nowNs = System.nanoTime();
        double dtSec = clamp((nowNs - turretLastNs) / 1e9, 0.0005, 0.05);
        turretLastNs = nowNs; // always update — prevents D-term spike on re-engagement

        double pTerm = RobotConstants.TURRET_KP * errorTicks;

        turretIntegral += errorTicks * dtSec;
        double iTerm = clamp(
                RobotConstants.TURRET_KI * turretIntegral,
                -RobotConstants.TURRET_I_MAX, RobotConstants.TURRET_I_MAX);

        double dTerm = RobotConstants.TURRET_KD * (errorTicks - turretLastError) / dtSec;

        double output = clamp(pTerm + iTerm + dTerm,
                -RobotConstants.TURRET_MAX_POWER, RobotConstants.TURRET_MAX_POWER);

        double  errorDeg = Math.abs(errorTicks) / RobotConstants.TICKS_PER_DEGREE;
        boolean aligned  = errorDeg <= RobotConstants.ppModeAngleTolerance;

        if (aligned) {
            turretMotor.setPower(0.0);
            turretIntegral = 0.0;
        } else {
            if (Math.abs(output) < RobotConstants.TURRET_MIN_POWER) output = 0.0;
            turretMotor.setPower(output);
        }
        turretLastError = errorTicks;

        telemetry.addData("TurretRelDeg",  "%.1f", lastTurretRelDeg);
        telemetry.addData("TurretTarget",  "%d",   turretTargetTicks);
        telemetry.addData("TurretCurrent", "%d",   currentTicks);
        telemetry.addData("TurretOut",     "%.3f", aligned ? 0.0 : output);
    }

    /**
     * Manual turret override. Resets PID integrators so auto-tracking resumes
     * cleanly when switched back.
     * @param power positive = clockwise, negative = counter-clockwise.
     */
    public void driveManualTurret(double power) {
        turretMotor.setPower(power);
        turretIntegral  = 0.0;
        turretLastError = 0.0;
    }

    /**
     * Runs the flywheel velocity PID toward targetFlywheelTPS.
     * On a very fast loop (dt ≤ 1 ms) the last known power is re-applied
     * rather than sending 0, so the flywheel is never starved.
     * @return motor power applied [0..1].
     */
    public double driveFlywheels(boolean shooterOn) {
        if (!shooterOn) {
            shooterR.setPower(0.0);
            shooterL.setPower(0.0);
            pidIntegral      = 0.0;
            pidLastError     = 0.0;
            lastFlywheelPower = 0.0;
            return 0.0;
        }

        long   nowNs = System.nanoTime();
        double dtSec = (nowNs - lastFlywheelNs) / 1e9;

        if (dtSec <= 0.001) {
            shooterR.setPower(lastFlywheelPower);
            shooterL.setPower(lastFlywheelPower);
            return lastFlywheelPower;
        }

        int    currentPos  = shooterR.getCurrentPosition();
        double velocityTPS = (currentPos - lastEncoderPos) / dtSec;
        lastFlywheelTPS    = velocityTPS;

        double error = targetFlywheelTPS - velocityTPS;

        double pTerm = RobotConstants.FLYWHEEL_KP * error;

        pidIntegral += error * dtSec;
        double iTerm = clamp(
                RobotConstants.FLYWHEEL_KI * pidIntegral,
                -RobotConstants.SHOOTER_I_MAX, RobotConstants.SHOOTER_I_MAX);

        double dTerm = RobotConstants.FLYWHEEL_KD * (error - pidLastError) / dtSec;

        double power = clamp(pTerm + iTerm + dTerm, 0.0, 1.0);

        shooterR.setPower(power);
        shooterL.setPower(power);

        lastFlywheelPower = power;
        pidLastError      = error;
        lastEncoderPos    = currentPos;
        lastFlywheelNs    = nowNs;

        telemetry.addData("FlywheelTPS",    "%.1f", velocityTPS);
        telemetry.addData("FlywheelTarget", "%.1f", targetFlywheelTPS);
        telemetry.addData("FlywheelErr",    "%.1f", error);
        telemetry.addData("FlywheelPower",  "%.3f", power);

        return power;
    }

    /**
     * Writes targetHoodServoPos to the hood servo.
     */
    public void driveHood() {
        double pos = clamp(targetHoodServoPos, RobotConstants.hoodServoMin, RobotConstants.hoodServoMax);
        hoodServo.setPosition(pos);
        telemetry.addData("HoodPos", "%.3f", pos);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // LOCKED / REDUNDANCY MODE
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Locked mode: all three axes driven to fixed constants from RobotConstants.
     * Uses dedicated lockedTurretDeg / lockedHoodPos / lockedFlywheelTPS rather
     * than the tracking-offset fields, which have a different meaning.
     */
    public void setLockedMode(boolean shooterOn) {
        turretTargetTicks  = (int) Math.round(-RobotConstants.lockedTurretDeg * RobotConstants.TICKS_PER_DEGREE);
        turretIntegral     = 0.0;
        turretLastError    = 0.0;
        driveTurret();

        targetHoodServoPos = RobotConstants.lockedHoodPos;
        driveHood();

        targetFlywheelTPS = RobotConstants.lockedFlywheelTPS;
        driveFlywheels(shooterOn);
    }

    public void resetTurretEncoder() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretTargetTicks = 0;
        turretIntegral    = 0.0;
        turretLastError   = 0.0;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // HELPERS
    // ═══════════════════════════════════════════════════════════════════════════

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double wrapRad(double rad) {
        while (rad >  Math.PI) rad -= 2.0 * Math.PI;
        while (rad < -Math.PI) rad += 2.0 * Math.PI;
        return rad;
    }
}
