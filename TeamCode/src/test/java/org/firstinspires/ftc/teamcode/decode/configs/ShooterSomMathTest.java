package org.firstinspires.ftc.teamcode.decode.configs;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

/**
 * Unit tests for Shooter.subtractRobotVelocity — the pure-math core of the
 * Huskie Robotics 3061 shoot-on-the-move velocity-subtraction algorithm.
 * No FTC SDK / Android dependency (unlike the rest of Shooter, which needs a
 * HardwareMap), so it's tested directly, same pattern as PIDControllerTest
 * and TurretLimiterTest.
 *
 * Expected values below were independently computed in Python from the same
 * formulas (Equations 8-19 of the Huskie Physics PDF) — not just re-deriving
 * whatever the Java happens to output.
 */
public class ShooterSomMathTest {

    private static final double EPS = 1e-6;

    @Test
    public void zeroRobotVelocity_returnsTheStaticShotUnchanged() {
        Shooter.ShotVector shot = Shooter.subtractRobotVelocity(
                200.0, 45.0, 0.0,
                0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0);

        assertEquals(200.0, shot.speedInPerSec, EPS);
        assertEquals(45.0, shot.elevationDeg, EPS);
        assertEquals(0.0, shot.turretRelRad, EPS);
    }

    @Test
    public void robotMovingTowardTarget_needsASlowerSteeperLayup() {
        // Bearing and heading both 0 (target straight ahead), robot moving
        // straight at it at 50 in/s. Per the PDF's own intuition (section
        // 1.3): moving toward the target needs a steeper, slower shot.
        Shooter.ShotVector shot = Shooter.subtractRobotVelocity(
                200.0, 45.0, 0.0,
                50.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0);

        assertEquals(168.3979345962089, shot.speedInPerSec, 1e-6);
        assertEquals(57.119489189889634, shot.elevationDeg, 1e-6);
        assertEquals(0.0, shot.turretRelRad, EPS); // purely axial motion, no turret correction needed
    }

    @Test
    public void robotDriftingSideways_needsATurretCorrection() {
        // Target straight ahead (bearing=heading=0), robot drifting sideways
        // (field +y) at 50 in/s while shooting. The ball inherits that drift,
        // so the turret has to aim off-axis to compensate.
        Shooter.ShotVector shot = Shooter.subtractRobotVelocity(
                200.0, 45.0, 0.0,
                0.0, 50.0, 0.0, 0.0,
                0.0, 0.0, 0.0);

        assertEquals(206.155281280883, shot.speedInPerSec, 1e-6);
        assertEquals(43.31385665828306, shot.elevationDeg, 1e-6);
        assertEquals(-0.339836909454122, shot.turretRelRad, 1e-6);
    }

    @Test
    public void zeroShooterOffset_makesRotationCorrectionANoOp() {
        // Same as zeroRobotVelocity, but with nonzero omega — since
        // shooterOffsetX/Y are both 0 (RobotConstants' current, unmeasured
        // default), robot spin must not affect the result at all.
        Shooter.ShotVector shot = Shooter.subtractRobotVelocity(
                200.0, 45.0, 0.0,
                0.0, 0.0, 3.0, 0.0,
                0.0, 0.0, 0.0);

        assertEquals(200.0, shot.speedInPerSec, EPS);
        assertEquals(45.0, shot.elevationDeg, EPS);
        assertEquals(0.0, shot.turretRelRad, EPS);
    }

    @Test
    public void nearVerticalShot_holdsPreviousAzimuthInsteadOfNoisyAtan2() {
        // Found via a Claude/ChatGPT cross-review, then verified numerically:
        // when vShotHoriz drops near zero (robot velocity nearly cancels the
        // static shot's horizontal component - a realistic "lay-up" case),
        // atan2(vShotY, vShotX) becomes noise-dominated (~215 degree swing
        // across 8 simulated loop ticks under plausible velocity noise). Here
        // vShotX = 200*sin(45)*cos(0) - 140 = ~1.42 in/s, well under
        // RobotConstants.SOM_AZIMUTH_HOLD_THRESHOLD_IN_PER_SEC (10.0), so the
        // method must hold the supplied previous bearing instead of
        // recomputing it.
        double heldValue = 0.789; // arbitrary, distinctive "previous" bearing
        Shooter.ShotVector shot = Shooter.subtractRobotVelocity(
                200.0, 45.0, 0.0,
                140.0, 0.0, 0.0, 0.0,
                0.0, 0.0, heldValue);

        assertEquals(heldValue, shot.turretRelRad, EPS);
    }

    @Test
    public void aboveHoldThreshold_ignoresPreviousAzimuthAndRecomputesNormally() {
        // Same scenario as robotDriftingSideways_needsATurretCorrection, but
        // passing a deliberately wrong "previous" value to confirm it's only
        // used below the hold threshold, not always.
        Shooter.ShotVector shot = Shooter.subtractRobotVelocity(
                200.0, 45.0, 0.0,
                0.0, 50.0, 0.0, 0.0,
                0.0, 0.0, 999.0);

        assertEquals(-0.339836909454122, shot.turretRelRad, 1e-6);
    }

    @Test
    public void flywheelTpsAndLaunchSpeed_conversionsAreInverses() {
        double tps = 1800.0;
        double speed = Shooter.flywheelTpsToLaunchSpeed(tps);
        double roundTrip = Shooter.launchSpeedToFlywheelTps(speed);
        assertEquals(tps, roundTrip, 1e-9);
    }

    @Test
    public void hoodPosAndLaunchAngle_conversionsAreInverses() {
        double pos = RobotConstants.hoodServoMin
                + 0.37 * (RobotConstants.hoodServoMax - RobotConstants.hoodServoMin);
        double angle = Shooter.hoodPosToLaunchAngleDeg(pos);
        double roundTrip = Shooter.launchAngleDegToHoodPos(angle);
        assertEquals(pos, roundTrip, 1e-9);
    }
}
