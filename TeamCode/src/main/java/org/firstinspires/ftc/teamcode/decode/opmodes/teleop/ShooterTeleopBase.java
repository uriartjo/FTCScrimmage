package org.firstinspires.ftc.teamcode.decode.opmodes.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.decode.configs.RobotConstants;
import org.firstinspires.ftc.teamcode.decode.configs.Shooter;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.decode.util.TurretLimiter;

/**
 * Pedro Pathing v3 translation of ShooterTeleopBase.java (itself a
 * from-this-session consolidation of the original 4 copy-pasted teleops in
 * 641WinterHavenEdition-master). Only the Pedro-specific surface changed —
 * mode dispatch, kicker sequencer, laser/RGB, adjustable mode, and the
 * turret stall guard are all copied verbatim.
 *
 * Changes from the v2 base class:
 *   - Constants.createFollower(hardwareMap) -> Constants.create(hardwareMap)
 *     (new package: org.firstinspires.ftc.teamcode.pedro.Constants; and its
 *     create() is currently a stub returning null until the team runs the
 *     v3 AutoTune procedures (MecanumTuner/PinpointTuner) on the robot).
 *   - follower.setStartingPose(pose) -> follower.setPose(pose)
 *   - follower.startTeleopDrive() removed — no v3 equivalent call exists;
 *     follower.manual(...) sets Follower.Mode.MANUAL as a side effect of
 *     being called, no separate "start" step.
 *   - follower.getPose() -> follower.pose()
 *
 *   - THE IMPORTANT ONE: follower.setTeleOpDrive(y, x, turn, false) in the
 *     original all four v2 teleops passed isRobotCentric=false — verified
 *     against v2.1.2 source, that means the robot has been driving FIELD
 *     CENTRIC this whole time. v3's follower.manual(DrivePowers) hardcodes
 *     robot-centric internally (drivetrain.drive(manualPowers, true) — see
 *     v3 Follower.java's MANUAL case), so a naive translation to
 *     follower.manual(y, x, turn) would silently make driving robot-centric
 *     instead — the stick would mean "robot's forward" instead of "away
 *     from the driver," which flips depending on which way the robot is
 *     currently facing. This translation instead pre-rotates the stick
 *     input with the verified ManualDrive.fieldCentric(...) helper before
 *     handing it to manual(), to reproduce the exact original behavior:
 *
 *         follower.manual(ManualDrive.fieldCentric(driveY, driveX, driveTurn, follower.pose().heading()));
 *
 *   - Call order (manual() then update(), not update() then manual() like
 *     the original v2 loop) is an INFERENCE from v3's source, not something
 *     run on hardware yet: manual(DrivePowers) only records mode+powers;
 *     the MANUAL-case drivetrain.drive(...) call lives inside update()'s
 *     mode switch. So update() has to run after manual() sets the intended
 *     powers for them to actually reach the wheels this loop tick. Verify
 *     this is really required once Constants.create() returns a real
 *     Follower — if driving feels one loop tick delayed, this ordering is
 *     the first thing to check.
 *
 * One deliberate behavior change kept from the original consolidation:
 * gamepad1.b pose-reset is always wired up here (it was live in BlueTeleop
 * but commented out in the other three original v2 files).
 *
 * Also wires in TurretLimiter's stall guard, unchanged from the v2 version
 * — TurretLimiter has zero Pedro dependency.
 */
public abstract class ShooterTeleopBase extends OpMode {

    // ═══════════════════════════════════════════════════════════════════════════
    // ROBOT MODE
    // ═══════════════════════════════════════════════════════════════════════════

    private enum RobotMode { AUTO, TURRET_ONLY, FULL_MANUAL, ADJUSTABLE }
    private RobotMode mode = RobotMode.AUTO;

    private boolean lastGp2Y = false, lastGp2B = false, lastGp2A = false, lastGp2X = false;
    private boolean lastGp2LB = false;
    private boolean lastGp1B = false;

    // ═══════════════════════════════════════════════════════════════════════════
    // ALLIANCE-SPECIFIC CONFIGURATION — supplied by the subclass
    // ═══════════════════════════════════════════════════════════════════════════

    private final Pose startPose;
    private final Pose goalPose;
    private final Pose resetPose;
    private final boolean invertDrive;

    protected ShooterTeleopBase(Pose startPose, Pose goalPose, Pose resetPose, boolean invertDrive) {
        this.startPose = startPose;
        this.goalPose = goalPose;
        this.resetPose = resetPose;
        this.invertDrive = invertDrive;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // PEDRO
    // ═══════════════════════════════════════════════════════════════════════════

    private Follower follower;

    // ═══════════════════════════════════════════════════════════════════════════
    // HARDWARE
    // ═══════════════════════════════════════════════════════════════════════════

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor toggleMotor;
    private Shooter shooter;
    private Servo kickerServo;

    private DigitalChannel laserInput;
    private Servo rgbLight;

    // goBILDA RGB Indicator Light color positions (Product Insight #4)
    private static final double RGB_RED = 0.277;    // 1100 us — no item
    private static final double RGB_YELLOW = 0.388; // 1300 us — 1st pass
    private static final double RGB_BLUE = 0.611;   // 1700 us — 2nd pass
    private static final double RGB_GREEN = 0.500;  // 1500 us — 3rd pass

    // ─── Turret safety ─────────────────────────────────────────────────────────
    private final TurretLimiter turretLimiter =
            new TurretLimiter(Integer.MIN_VALUE, Integer.MAX_VALUE, 0.6, 15, 0.75);
    private boolean turretStalled = false;

    // ═══════════════════════════════════════════════════════════════════════════
    // STATE
    // ═══════════════════════════════════════════════════════════════════════════

    private int passCount = 0;
    private boolean lastDetected = false;

    private boolean lastGp1A = false;
    private boolean shooterOn = true;
    private boolean intakeOn = true;

    private boolean kickerSeqActive = false;
    private int kickerSeqStep = 0;
    private final ElapsedTime kickerSeqTimer = new ElapsedTime();
    private boolean lastGp1RT = false;
    private boolean lastGp1RB = false;

    private double adjustableHoodPos = RobotConstants.fixedHoodPos;
    private double adjustableFlywheelTPS = RobotConstants.fixedFlywheelTPS;

    private boolean lastGp2DUp = false, lastGp2DDown = false, lastGp2DLeft = false, lastGp2DRight = false;

    private final ElapsedTime loopTimer = new ElapsedTime();

    // ═══════════════════════════════════════════════════════════════════════════
    // INIT
    // ═══════════════════════════════════════════════════════════════════════════

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        kickerServo = hardwareMap.get(Servo.class, "servo");

        toggleMotor = hardwareMap.get(DcMotor.class, "intake");
        toggleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        toggleMotor.setPower(0);

        follower = Constants.create(hardwareMap); // NOTE: stub, returns null until AutoTune is run
        follower.setPose(startPose);
        follower.update();

        shooter = new Shooter(hardwareMap, follower, telemetry);
        shooter.resetTurretEncoder();
        shooter.calculateTurretTarget(goalPose.x(), goalPose.y());

        laserInput = hardwareMap.get(DigitalChannel.class, "laserDigitalInput");
        laserInput.setMode(DigitalChannel.Mode.INPUT);
        rgbLight = hardwareMap.get(Servo.class, "rgbLight");
        rgbLight.setPosition(RGB_RED);

        loopTimer.reset();

        telemetry.addLine("Initialized — Mode: AUTO");
        telemetry.update();
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // LOOP
    // ═══════════════════════════════════════════════════════════════════════════

    @Override
    public void loop() {
        double driveY = invertDrive ? -gamepad1.left_stick_y : gamepad1.left_stick_y;
        double driveX = invertDrive ? -gamepad1.left_stick_x : gamepad1.left_stick_x;
        double driveTurn = -gamepad1.right_stick_x;
        // See class javadoc: fieldCentric(...) reproduces the original v2 teleops'
        // isRobotCentric=false behavior, since v3's manual() is robot-centric-only.
        follower.manual(ManualDrive.fieldCentric(driveY, driveX, driveTurn, follower.pose().heading()));
        follower.update();

        handleModeSwitch();

        boolean gp2LB = gamepad2.left_bumper;
        if (gp2LB && !lastGp2LB) shooterOn = !shooterOn;
        lastGp2LB = gp2LB;

        handleShooterMode();
        handleGamepad1();
        if (mode == RobotMode.ADJUSTABLE) handleGamepad2Adjustable();
        handleKickerSequencer();
        handleLaserAndRgb();
        handlePoseReset();

        telemetry.addData("Mode", mode.name());
        telemetry.addData("ShooterOn", shooterOn);
        telemetry.addData("TPS Target", "%.0f", shooter.targetFlywheelTPS);
        telemetry.addData("TPS Actual", "%.1f", shooter.lastFlywheelTPS);
        telemetry.addData("Hood Target", "%.3f", shooter.targetHoodServoPos);
        telemetry.addData("Hood Actual", "%.3f", shooter.hoodServo.getPosition());
        telemetry.addData("Robot Pose", follower.pose());
        telemetry.addData("Dist from Goal", "%.1f", shooter.getRobotDistanceToGoal(goalPose));
        telemetry.addData("Pass Count", passCount);
        if (turretStalled) telemetry.addLine("*** TURRET STALL — power cut ***");
        telemetry.update();
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // MODE SWITCH — gamepad2 face buttons
    // ═══════════════════════════════════════════════════════════════════════════

    private void handleModeSwitch() {
        boolean gp2Y = gamepad2.y;
        boolean gp2B = gamepad2.b;
        boolean gp2A = gamepad2.a;
        boolean gp2X = gamepad2.x;

        if (gp2Y && !lastGp2Y) mode = RobotMode.AUTO;
        if (gp2B && !lastGp2B) mode = RobotMode.TURRET_ONLY;
        if (gp2A && !lastGp2A) {
            mode = RobotMode.FULL_MANUAL;
            shooter.turretTargetTicks = (int) Math.round(-RobotConstants.lockedTurretDeg * RobotConstants.TICKS_PER_DEGREE);
        }
        if (gp2X && !lastGp2X) {
            if (mode != RobotMode.ADJUSTABLE) {
                adjustableHoodPos = RobotConstants.fixedHoodPos;
                adjustableFlywheelTPS = RobotConstants.fixedFlywheelTPS;
                shooter.turretTargetTicks = (int) Math.round(-RobotConstants.lockedTurretDeg * RobotConstants.TICKS_PER_DEGREE);
            }
            mode = RobotMode.ADJUSTABLE;
        }

        lastGp2Y = gp2Y;
        lastGp2B = gp2B;
        lastGp2A = gp2A;
        lastGp2X = gp2X;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // SHOOTER MODE DISPATCH
    // ═══════════════════════════════════════════════════════════════════════════

    private void handleShooterMode() {
        switch (mode) {

            case AUTO:
                shooter.calculateTargets(goalPose);
                driveTurretWithStallGuard();
                shooter.driveFlywheels(shooterOn);
                shooter.driveHood();
                break;

            case TURRET_ONLY:
                shooter.calculateTurretTarget(goalPose.x(), goalPose.y());
                shooter.targetFlywheelTPS = RobotConstants.fixedFlywheelTPS;
                shooter.targetHoodServoPos = RobotConstants.fixedHoodPos;
                driveTurretWithStallGuard();
                shooter.driveFlywheels(shooterOn);
                shooter.driveHood();
                break;

            case FULL_MANUAL:
                shooter.targetFlywheelTPS = RobotConstants.fixedFlywheelTPS;
                shooter.targetHoodServoPos = RobotConstants.fixedHoodPos;
                driveTurretWithStallGuard();
                shooter.driveFlywheels(shooterOn);
                shooter.driveHood();
                break;

            case ADJUSTABLE:
                shooter.calculateTurretTarget(goalPose.x(), goalPose.y());
                shooter.targetFlywheelTPS = adjustableFlywheelTPS;
                shooter.targetHoodServoPos = adjustableHoodPos;
                driveTurretWithStallGuard();
                shooter.driveFlywheels(shooterOn);
                shooter.driveHood();
                break;
        }
    }

    private void driveTurretWithStallGuard() {
        shooter.driveTurret();
        turretStalled = turretLimiter.checkStall(
                shooter.turretMotor.getPower(),
                shooter.turretMotor.getCurrentPosition(),
                loopTimer.seconds());
        if (turretStalled) {
            shooter.turretMotor.setPower(0.0);
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // GAMEPAD1 — active in ALL modes
    // ═══════════════════════════════════════════════════════════════════════════

    private void handleGamepad1() {
        boolean gp1A = gamepad1.a;
        if (gp1A && !lastGp1A) intakeOn = !intakeOn;
        lastGp1A = gp1A;

        if (gamepad1.x) {
            toggleMotor.setPower(-1.0);
        } else {
            toggleMotor.setPower(intakeOn ? 1.0 : 0.0);
        }

        if (!kickerSeqActive) {
            if (gamepad1.left_bumper) kickerServo.setPosition(RobotConstants.kickerDownPos);
            if (gamepad1.right_bumper) kickerServo.setPosition(RobotConstants.kickerUpPos);
        }

        boolean resetNow = (gamepad1.right_bumper && !lastGp1RB)
                || (gamepad1.right_trigger > 0.5 && !lastGp1RT);
        if (resetNow) passCount = 0;
        lastGp1RB = gamepad1.right_bumper;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // KICKER TRIPLE-KICK SEQUENCER
    // ═══════════════════════════════════════════════════════════════════════════

    private void handleKickerSequencer() {
        boolean gp1RT = gamepad1.right_trigger > 0.5;

        if (gp1RT && !lastGp1RT && !kickerSeqActive) {
            kickerSeqActive = true;
            kickerSeqStep = 0;
            kickerSeqTimer.reset();
            kickerServo.setPosition(RobotConstants.kickerDownPos);
        }
        lastGp1RT = gp1RT;

        if (!kickerSeqActive) return;

        if (kickerSeqTimer.seconds() >= RobotConstants.kickerStaggerSec) {
            kickerSeqStep++;
            kickerSeqTimer.reset();

            if (kickerSeqStep >= 6) {
                kickerServo.setPosition(RobotConstants.kickerDownPos);
                kickerSeqActive = false;
            } else if (kickerSeqStep % 2 == 0) {
                kickerServo.setPosition(RobotConstants.kickerDownPos);
            } else {
                kickerServo.setPosition(RobotConstants.kickerUpPos);
            }
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // GAMEPAD2 ADJUSTABLE MODE CONTROLS
    // ═══════════════════════════════════════════════════════════════════════════

    private void handleGamepad2Adjustable() {
        boolean dUp = gamepad2.dpad_up;
        boolean dDown = gamepad2.dpad_down;
        boolean dLeft = gamepad2.dpad_left;
        boolean dRight = gamepad2.dpad_right;

        if (dUp && !lastGp2DUp) adjustableHoodPos += RobotConstants.adjustableHoodStep;
        if (dDown && !lastGp2DDown) adjustableHoodPos -= RobotConstants.adjustableHoodStep;
        if (dRight && !lastGp2DRight) adjustableFlywheelTPS += RobotConstants.adjustableFlywheelStep;
        if (dLeft && !lastGp2DLeft) adjustableFlywheelTPS -= RobotConstants.adjustableFlywheelStep;

        adjustableHoodPos = clamp(adjustableHoodPos, RobotConstants.hoodServoMin, RobotConstants.hoodServoMax);
        adjustableFlywheelTPS = clamp(adjustableFlywheelTPS, 0.0, 6000.0);

        lastGp2DUp = dUp;
        lastGp2DDown = dDown;
        lastGp2DLeft = dLeft;
        lastGp2DRight = dRight;

        telemetry.addData("Adj Hood", "%.3f", adjustableHoodPos);
        telemetry.addData("Adj TPS", "%.0f", adjustableFlywheelTPS);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // LASER SENSOR & RGB INDICATOR
    // ═══════════════════════════════════════════════════════════════════════════

    private void handleLaserAndRgb() {
        boolean beamBroken = !laserInput.getState();

        if (beamBroken && !lastDetected) {
            passCount++;
            if (passCount > 3) passCount = 3;
        }
        lastDetected = beamBroken;

        if (!beamBroken && passCount == 0) rgbLight.setPosition(RGB_RED);
        else if (passCount == 1) rgbLight.setPosition(RGB_YELLOW);
        else if (passCount == 2) rgbLight.setPosition(RGB_BLUE);
        else if (passCount >= 3) rgbLight.setPosition(RGB_GREEN);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // POSE RESET — gamepad1.b resets the follower to this variant's resetPose.
    // Override to make this a no-op if a particular variant shouldn't allow it.
    // ═══════════════════════════════════════════════════════════════════════════

    protected void handlePoseReset() {
        boolean gp1B = gamepad1.b;
        if (gp1B && !lastGp1B) {
            follower.setPose(resetPose);
        }
        lastGp1B = gp1B;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // HELPERS
    // ═══════════════════════════════════════════════════════════════════════════

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
