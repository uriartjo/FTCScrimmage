package org.firstinspires.ftc.teamcode.decode.util;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.assertEquals;

import org.junit.Test;

/**
 * Unit tests for TurretLimiter. Like PIDController, this has no FTC SDK /
 * Android dependency, so it's tested directly without a hardwareMap.
 */
public class TurretLimiterTest {

    @Test
    public void clampTarget_passesThroughValuesInsideRange() {
        TurretLimiter limiter = new TurretLimiter(-1000, 1000, 0.5, 10, 1.0);
        assertEquals(500, limiter.clampTarget(500));
    }

    @Test
    public void clampTarget_clampsBelowMinimum() {
        TurretLimiter limiter = new TurretLimiter(-1000, 1000, 0.5, 10, 1.0);
        assertEquals(-1000, limiter.clampTarget(-5000));
    }

    @Test
    public void clampTarget_clampsAboveMaximum() {
        TurretLimiter limiter = new TurretLimiter(-1000, 1000, 0.5, 10, 1.0);
        assertEquals(1000, limiter.clampTarget(5000));
    }

    @Test
    public void checkStall_neverTripsWhenPowerIsBelowThreshold() {
        TurretLimiter limiter = new TurretLimiter(-1000, 1000, 0.5, 10, 1.0);

        // Low power, encoder not moving, for well over stallTimeSec - should
        // never be flagged as a stall because the turret was never actually
        // being commanded hard in the first place.
        assertFalse(limiter.checkStall(0.1, 100, 0.0));
        assertFalse(limiter.checkStall(0.1, 100, 0.5));
        assertFalse(limiter.checkStall(0.1, 100, 2.0));
    }

    @Test
    public void checkStall_doesNotTripBeforeStallTimeElapses() {
        TurretLimiter limiter = new TurretLimiter(-1000, 1000, 0.5, 10, 1.0);

        assertFalse(limiter.checkStall(0.8, 100, 0.0)); // arms the stall window
        assertFalse(limiter.checkStall(0.8, 102, 0.3)); // barely moved, 0.3s < 1.0s
        assertFalse(limiter.checkStall(0.8, 103, 0.6)); // barely moved, 0.6s < 1.0s
    }

    @Test
    public void checkStall_tripsOnceDrivingHardAndBarelyMovingForStallTime() {
        TurretLimiter limiter = new TurretLimiter(-1000, 1000, 0.5, 10, 1.0);

        assertFalse(limiter.checkStall(0.8, 100, 0.0));
        assertFalse(limiter.checkStall(0.8, 103, 0.6));
        assertTrue(limiter.checkStall(0.8, 105, 1.1)); // still <10 ticks moved, 1.1s elapsed
    }

    @Test
    public void checkStall_resetsTheWindowWhenTheTurretActuallyMoves() {
        TurretLimiter limiter = new TurretLimiter(-1000, 1000, 0.5, 10, 1.0);

        assertFalse(limiter.checkStall(0.8, 0, 0.0));   // arms the window at tick 0
        assertFalse(limiter.checkStall(0.8, 50, 0.5));  // moved 50 ticks - genuinely turning, window resets
        assertFalse(limiter.checkStall(0.8, 55, 1.4));  // only 0.9s since the reset at t=0.5 - not stalled yet
        assertTrue(limiter.checkStall(0.8, 56, 1.6));   // now 1.1s since the reset - barely moved, trips
    }

    @Test
    public void checkStall_disarmsWhenPowerDropsBelowThreshold_andMustReArm() {
        TurretLimiter limiter = new TurretLimiter(-1000, 1000, 0.5, 10, 1.0);

        assertFalse(limiter.checkStall(0.8, 0, 0.0));   // arms the window
        assertFalse(limiter.checkStall(0.2, 0, 0.5));   // power drops below threshold - disarms
        assertFalse(limiter.checkStall(0.8, 0, 0.6));   // re-arms fresh at t=0.6
        assertTrue(limiter.checkStall(0.8, 2, 1.7));    // 1.1s since the re-arm, barely moved - trips
    }
}
