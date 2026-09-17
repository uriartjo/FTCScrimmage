package org.firstinspires.ftc.teamcode.decode.util;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/**
 * Unit tests for PIDController. This class has no FTC SDK / Android
 * dependency, so it can be tested directly without a hardwareMap or robot —
 * see build.gradle for the added testImplementation.
 */
public class PIDControllerTest {

    private static final double DELTA = 1e-9;

    @Test
    public void proportionalOnly_returnsExpectedOutput() {
        PIDController pid = new PIDController(1.0, 0.0, 0.0, 100.0, -10.0, 10.0);
        double output = pid.update(0.5, 0.02);
        assertEquals(0.5, output, DELTA);
    }

    @Test
    public void output_isClampedToOutputRange() {
        PIDController pid = new PIDController(10.0, 0.0, 0.0, 100.0, -1.0, 1.0);
        double output = pid.update(5.0, 0.02); // raw pTerm = 50, way past the +-1 clamp
        assertEquals(1.0, output, DELTA);
    }

    @Test
    public void output_isClampedToNegativeOutputRange() {
        PIDController pid = new PIDController(10.0, 0.0, 0.0, 100.0, -1.0, 1.0);
        double output = pid.update(-5.0, 0.02);
        assertEquals(-1.0, output, DELTA);
    }

    @Test
    public void integralTerm_isClampedIndependentlyOfOutputRange() {
        // kP = kD = 0 so the output is purely the clamped I-term.
        PIDController pid = new PIDController(0.0, 1.0, 0.0, 0.5, -10.0, 10.0);

        // integral accumulates 10 * 0.02 = 0.2 per call; after 3 calls raw
        // integral = 0.6, so kI * integral = 0.6 would exceed integralMax (0.5).
        pid.update(10.0, 0.02);
        pid.update(10.0, 0.02);
        double output = pid.update(10.0, 0.02);

        assertEquals(0.5, output, DELTA);
    }

    @Test
    public void derivativeTerm_isZeroOnFirstCall() {
        // No prior error to diff against yet.
        PIDController pid = new PIDController(0.0, 0.0, 1.0, 100.0, -1000.0, 1000.0);
        double output = pid.update(5.0, 0.1);
        assertEquals(0.0, output, DELTA);
    }

    @Test
    public void derivativeTerm_matchesRawErrorDeltaOverDt_whenUnfiltered() {
        // dt = 0.02 stays inside [MIN_DT_SEC, MAX_DT_SEC] so it passes through unclamped.
        PIDController pid = new PIDController(0.0, 0.0, 1.0, 100.0, -1000.0, 1000.0, 0.0);
        pid.update(1.0, 0.02);                    // seeds lastError = 1.0
        double output = pid.update(3.0, 0.02);    // raw derivative = (3 - 1) / 0.02 = 100
        assertEquals(100.0, output, DELTA);
    }

    @Test
    public void dt_isClampedToAMinimumToBoundDerivativeSpikes() {
        // Mirrors the turret loop's real dt clamp (MIN_DT_SEC = 0.0005): an
        // unrealistically tiny dt must not blow the derivative up past what
        // the clamp allows, even though the raw math would want to.
        PIDController pid = new PIDController(0.0, 0.0, 1.0, 100.0, -1_000_000.0, 1_000_000.0, 0.0);
        pid.update(0.0, 1e-9);
        double output = pid.update(1.0, 1e-9); // dt clamped up to 0.0005 -> derivative = 1 / 0.0005 = 2000
        assertEquals(2000.0, output, DELTA);
    }

    @Test
    public void dt_isClampedToAMaximum_soAStalePauseDoesNotDumpAHugeIntegralStep() {
        // kD = 0 so we isolate the integral: with an unclamped dt, a long
        // pause (e.g. the OpMode briefly stalling) would integrate error
        // over the full elapsed gap. The dt clamp caps that single step's
        // contribution to MAX_DT_SEC (0.05) worth of integration.
        PIDController pid = new PIDController(0.0, 1.0, 0.0, 1000.0, -1000.0, 1000.0);
        double output = pid.update(10.0, 5.0); // dt clamped down to 0.05 -> integral += 10 * 0.05 = 0.5
        assertEquals(0.5, output, DELTA);
    }

    @Test
    public void derivativeFilter_dampensASingleSpikeComparedToUnfiltered() {
        double kD = 1.0;
        PIDController unfiltered = new PIDController(0.0, 0.0, kD, 100.0, -1000.0, 1000.0, 0.0);
        PIDController filtered = new PIDController(0.0, 0.0, kD, 100.0, -1000.0, 1000.0, 0.5);

        // Two steady updates at error=1, then a spike to error=5. dt = 0.02
        // stays inside [MIN_DT_SEC, MAX_DT_SEC] so it passes through unclamped.
        for (PIDController pid : new PIDController[]{unfiltered, filtered}) {
            pid.update(1.0, 0.02);
            pid.update(1.0, 0.02);
        }

        double unfilteredSpikeOutput = unfiltered.update(5.0, 0.02); // raw derivative = 200
        double filteredSpikeOutput = filtered.update(5.0, 0.02);     // smoothed toward 0 -> 100

        assertEquals(200.0, unfilteredSpikeOutput, DELTA);
        assertEquals(100.0, filteredSpikeOutput, DELTA);
        assertTrue("filtered derivative should be damped relative to the raw spike",
                filteredSpikeOutput < unfilteredSpikeOutput);
    }

    @Test
    public void reset_clearsIntegralAndDerivativeHistory() {
        PIDController pid = new PIDController(0.0, 1.0, 1.0, 100.0, -1000.0, 1000.0, 0.0);

        pid.update(10.0, 0.02); // seeds lastError + accumulates integral
        pid.update(10.0, 0.02);
        pid.reset();

        // After reset, this should behave exactly like a first-ever call:
        // zero derivative (no prior error) and integral restarted from zero.
        double output = pid.update(10.0, 0.02);
        // kP=0, so output is purely iTerm (kI * 10*0.02 = 0.2) + dTerm (0, first call after reset).
        assertEquals(0.2, output, DELTA);
    }

    @Test
    public void setGains_takesEffectImmediately_withoutResettingIntegralHistory() {
        // Exercises the exact case PollenTrack relies on: a dashboard-tunable
        // gain change pushed into a persistent controller mid-run.
        PIDController pid = new PIDController(1.0, 1.0, 0.0, 100.0, -1000.0, 1000.0);

        double before = pid.update(10.0, 0.02); // pTerm = 10, integral = 0.2 -> iTerm 0.2, output 10.2
        assertEquals(10.2, before, DELTA);

        pid.setGains(0.0, 1.0, 0.0); // drop kP to 0, keep kI

        double after = pid.update(10.0, 0.02);
        // New kP=0 takes effect immediately (no more +10 term), while the
        // integral keeps accumulating from where it left off: 0.2 + 0.2 = 0.4,
        // not reset back to a fresh 0.2 as it would be after reset()/a new instance.
        assertEquals(0.4, after, DELTA);
    }
}
